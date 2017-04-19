#include "I2Cdev.h"
//#include "MPU6050.h"    //not neccessary with MotionApps include file
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "SPI.h"
#include "SdFat.h"
#include "CapacitiveSensor.h"

#define CAP_CONNECTED       //Enable Capactive pressure sensing code

#define SD_CONNECTED        //Enable for writing data to SD Card
                            //Must be diabled if there is no SD card or code will stall in void Setup()

//#define SERIAL_CONNECTED    //Enable for debugging and Serial control & output. 
                            //Must be commented out for standalone operation
                            
#ifdef SERIAL_CONNECTED     //Allows you to use DEBUG_PRINT() and DEBUG_PRINLN() to print to the serial port.  
                            //These commands will be ignroed if SERIAL_CONNECTED is not defined.
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#define OUTPUT_READABLE_QUATERNION      //Quaternion anlges, no gimbal lock, independant
//#define OUTPUT_READABLE_EULER           //Euler angles, gimbal lock, calclated from quaternions
//#define OUTPUT_READABLE_YAWPITCHROLL    //Yaw, Pitch, & Roll, gimbal lock, calculated from quaternions
//#define OUTPUT_READABLE_REALACCEL       //gravity is removed, orientation always follows chip axis'
//#define OUTPUT_READABLE_WORLDACCEL      //gravity is removed, orientation adjusted to real world frame of reference
//#define OUTPUT_READABLE_RAW_GYRO        //raw gyro values
#define OUTPUT_READABLE_RAW_ACCEL       //raw accel values

MPU6050 mpu;      //Instance of MPU6050 Class

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


int16_t ax, ay, az;     //accelerometer raw data values
int16_t gx, gy, gz;     //gyroscope raw data values

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#define FILE_BUFFER_LINES  1   //number of lines to buffer in RAM before flushing to SD card.  
 /*                                
  *  Quaternion:    ??  bytes/line                               
  *  Euler                               
  *  Yaw Pitch Roll                               
  *  Real Accel.                               
  *  World Accel                               
  *  Raw Gyro.                              
  *  Raw Accel.
  */
byte fileCounter = 0;           //Incremented counter to periodically flush SD card

SdFat SD;
File DataFile;      //File that all data will be stored in.  Faster and simpler than individual files for each datatype.

#ifdef CAP_CONNECTED
  CapacitiveSensor heelCap = CapacitiveSensor(6,9);     //Set up capsense for the heel pressure pad
  CapacitiveSensor ballCap = CapacitiveSensor(4,8);     //Set up capsense for the ball of the foot pressure pad (Base joint of big toe)
  CapacitiveSensor toesCap = CapacitiveSensor(5,7);     //Set up capsense for the toes pressure pad (base joints of all other toes)

  long heelPressure = 0;      //Arbitrary pressure value, not necessarily proportional to other pressures.
  long ballPressure = 0;      //Arbitrary pressure value, not necessarily proportional to other pressures.
  long toesPressure = 0;      //Arbitrary pressure value, not necessarily proportional to other pressures.
#endif


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      INTERRUPT SETUP                                                                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile bool mpuInterrupt = false;

void dmpDataReady(){
  mpuInterrupt = true;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      VOID SETUP                                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  #ifdef SERIAL_CONNECTED
    Serial.begin(115200);      //Begin Serial comms if SERIAL_CONNECTED is defined above
    while(!Serial);           //Wait for saftware serial comms to PC to be active
  #endif

  #ifdef CAP_CONNECTED
    DEBUG_PRINT("Init. Cap...");
    heelCap.set_CS_AutocaL_Millis(0xFFFFFFFF);
    ballCap.set_CS_AutocaL_Millis(0xFFFFFFFF);
    toesCap.set_CS_AutocaL_Millis(0xFFFFFFFF);
    pinMode(6, OUTPUT);
    DEBUG_PRINTLN("Done");
  #endif
  
  DEBUG_PRINT("Init. SD...");          //Begin SD comms and print status  
  if (!SD.begin()) {                                  
    DEBUG_PRINTLN("Fail");
    return;
  }
  DEBUG_PRINTLN("Done");

  DEBUG_PRINT("Init. I2C...");                  //Start I2C and print when done
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  DEBUG_PRINTLN("Done");
  
  DEBUG_PRINT("Init. devices...");      //Initialize MPU6050 and print when done    
  mpu.initialize();
  DEBUG_PRINTLN("Done");
  
  DEBUG_PRINT("Testing connections...");  //test MPU6050 and print result
  DEBUG_PRINTLN(mpu.testConnection() ? "Done" : "Fail");

  #ifdef SERIAL_CONNECTED
    DEBUG_PRINTLN(F("\nSend any char..."));    //wait for serial input to continue
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  #endif   

  DEBUG_PRINTLN("Init. DMP...");    //Load and configure the MPU6050 Digital Motion Processor
  devStatus = mpu.dmpInitialize();

  DEBUG_PRINT("Set Offsets...");
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  DEBUG_PRINTLN("Done");

  if (devStatus == 0) {
      DEBUG_PRINT("Enabling DMP...");   // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);      
      DEBUG_PRINTLN("Done");
      
      DEBUG_PRINT("Enabling int...");    // enable Arduino interrupt detection
      pinMode(0, INPUT);
      noInterrupts();
      attachInterrupt(digitalPinToInterrupt(0), dmpDataReady, RISING);
      interrupts();
      mpuIntStatus = mpu.getIntStatus();
      DEBUG_PRINTLN("Done");      
      
      DEBUG_PRINTLN("Ready!");   // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;
      
      packetSize = mpu.dmpGetFIFOPacketSize();      // get expected DMP packet size for later comparison
    }else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      DEBUG_PRINT(F("DMP Init. fail (code "));
      DEBUG_PRINT(devStatus);
      DEBUG_PRINTLN(F(")"));
    }
    
    mpu.setDLPFMode(MPU6050_DLPF_BW_5);               //Low pass Filter (256, 188, 98, 42, 20, 10, 5 Hz)
    
    #ifdef SD_CONNECTED
      DEBUG_PRINT("Opening file...");
      DataFile = SD.open("Data.txt", FILE_WRITE);
      if(!DataFile){
        DEBUG_PRINTLN("Failed");
        return;
      }else{
        DEBUG_PRINT("Done");
      }
      DataFile.print("Millis,");
      
      #ifdef OUTPUT_READABLE_QUATERNION
        DataFile.print("Quat[w,x,y,z],");
      #endif
      #ifdef OUTPUT_READABLE_EULER
        DataFile.print("Euler[Psi,Theta,Phi],");
      #endif
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
        DataFile.print("[Yaw,Pitch,Roll],");
      #endif
      #ifdef OUTPUT_READABLE_REALACCEL
        DataFile.print("RealAccel[x,y,z],");
      #endif
      #ifdef OUTPUT_READABLE_WORLDACCEL
        DataFile.print("WorldAccel[x,y,z],");
      #endif
      #ifdef OUTPUT_READABLE_RAW_GYRO
        DataFile.print("RawGyro[x,y,z]");
      #endif
      #ifdef OUTPUT_READABLE_RAW_ACCEL
        DataFile.print("RawAccel[x,y,z],");
      #endif
      #ifdef CAP_CONNECTED
        DataFile.print("Cap[heel,ball,toes],");
      #endif
      DataFile.print("\n");
    #endif
      
}




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//      VOID LOOP                                                                                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (!dmpReady) return;                                            // if programming failed, don't try to do anything

  while (!mpuInterrupt && fifoCount < packetSize) {                 // wait for MPU interrupt or extra packet(s) available
    #ifdef SD_CONNECTED
      if(fileCounter == FILE_BUFFER_LINES){
          DEBUG_PRINTLN("Flushed");
          DataFile.flush();
          fileCounter = 0;
        
      }
    #endif
  }

  mpuInterrupt = false;                                             // reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();
  //DEBUG_PRINT(mpuIntStatus);
  fifoCount = mpu.getFIFOCount();                                   // get current FIFO count

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {                 // check for overflow (this should never happen unless our code is too inefficient)

    mpu.resetFIFO();                                                // reset so we can continue cleanly
    DEBUG_PRINTLN(F("FIFO overflow!"));

  } else if (mpuIntStatus & 0x02) {                                 // otherwise, check for DMP data ready interrupt (this should happen frequently)

    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();  // wait for correct available data length, should be a VERY short wait
    mpu.getFIFOBytes(fifoBuffer, packetSize);                       // read a packet from FIFO
    
    fifoCount -= packetSize;                                        // track FIFO count here in case there is > 1 packet available (this lets us immediately read more without waiting for an interrupt)
    
    
    //DEBUG_PRINT(millis());
    //DEBUG_PRINT(",");
    #ifdef SD_CONNECTED
      DataFile.print(millis());
      DataFile.print(",");
    #endif

    #ifdef SD_CONNECTED
      
    #endif
    
    #ifdef OUTPUT_READABLE_QUATERNION         //display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      serialPrintQuaternion();
      #ifdef SD_CONNECTED
          SDPrintQuaternion();
      #endif
    #endif
    
    #ifdef OUTPUT_READABLE_EULER              //Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      serialPrintEuler();
      #ifdef SD_CONNECTED
        SDPrintEuler();
      #endif
    #endif
    
    #ifdef OUTPUT_READABLE_YAWPITCHROLL       //Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      serialPrintYPW();
      #ifdef SD_CONNECTED
        SDPrintYPW();
      #endif
    #endif
    
    #ifdef OUTPUT_READABLE_REALACCEL          //real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      serialPrintRealAccel();
      #ifdef SD_CONNECTED
        SDPrintRealAccel();
      #endif
    #endif
    
    #ifdef OUTPUT_READABLE_WORLDACCEL         //initial world-frame acceleration, adjusted to remove gravity and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      serialPrintWorldAccel();
      #ifdef SD_CONNECTED
        SDPrintWorldAccel();
      #endif
    #endif

    #ifdef OUTPUT_READABLE_RAW_GYRO           //raw gyro values
      mpu.getRotation(&gx, &gy, &gz);
      serialPrintRawGyro();
      #ifdef SD_CONNECTED
        SDPrintRawGyro();
      #endif
    #endif
    
    #ifdef OUTPUT_READABLE_RAW_ACCEL          //raw accel values
      mpu.getAcceleration(&ax, &ay, &az);
      serialPrintRawAccel();
      #ifdef SD_CONNECTED
        SDPrintRawAccel();
      #endif
    #endif
    //DEBUG_PRINT(millis());
    //DEBUG_PRINT(",");
    
    #ifdef CAP_CONNECTED
    
      heelPressure = heelCap.capacitiveSensorRaw(100);
      digitalWrite(6, LOW);
      delayMicroseconds(20);
      ballPressure = ballCap.capacitiveSensorRaw(100);
      digitalWrite(6, LOW);
      delayMicroseconds(20);
      toesPressure = toesCap.capacitiveSensorRaw(100);
      digitalWrite(6, LOW);
      delayMicroseconds(20);

      #ifdef CAP_CONNECTED
        DEBUG_PRINT(heelPressure);
        DEBUG_PRINT(",");
        DEBUG_PRINT(ballPressure);
        DEBUG_PRINT(",");
        DEBUG_PRINT(toesPressure);
      #endif
      
      #ifdef SD_CONNECTED
        DataFile.print(heelPressure);
        DataFile.print(",");
        DataFile.print(ballPressure);
        DataFile.print(",");
        DataFile.print(toesPressure);
      #endif
      
      //DEBUG_PRINT(millis());
      //DEBUG_PRINT(",");
    #endif
    
    //DEBUG_PRINT(millis());
    DEBUG_PRINT("\n");      //Start new line because no serialPrintXXX() functions don't print newlines
    
    #ifdef SD_CONNECTED
      DataFile.print("\n"); //Start new line because no SDPrintXXX() functions don't print newlines
      fileCounter ++;       //Increment the file counter to choose when to flush the buffered file to the SD card
    #endif
    
  }
}

