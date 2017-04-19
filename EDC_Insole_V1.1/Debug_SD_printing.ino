void serialPrintQuaternion(){
  DEBUG_PRINT("quat,");
  DEBUG_PRINT(q.w);
  DEBUG_PRINT(",");
  DEBUG_PRINT(q.x);
  DEBUG_PRINT(",");
  DEBUG_PRINT(q.y);
  DEBUG_PRINT(",");
  DEBUG_PRINT(q.z);
  DEBUG_PRINT(",");
}

#ifdef SD_CONNECTED
  void SDPrintQuaternion(){
    DataFile.print(q.w);
    DataFile.print(",");
    DataFile.print(q.x);
    DataFile.print(",");
    DataFile.print(q.y);
    DataFile.print(",");
    DataFile.print(q.z);
    DataFile.print(",");
  }
#endif

void serialPrintEuler(){
  DEBUG_PRINT("euler psi theta phi,");
  DEBUG_PRINT(euler[0] * 180/M_PI);
  DEBUG_PRINT(",");
  DEBUG_PRINT(euler[1] * 180/M_PI);
  DEBUG_PRINT(",");
  DEBUG_PRINT(euler[2] * 180/M_PI);
  DEBUG_PRINT(",");
}

#ifdef SD_CONNECTED
  void SDPrintEuler(){
    DataFile.print(euler[0] * 180/M_PI);
    DataFile.print(",");
    DataFile.print(euler[1] * 180/M_PI);
    DataFile.print(",");
    DataFile.print(euler[2] * 180/M_PI);
    DataFile.print(",");
  }
#endif

void serialPrintYPW(){
  DEBUG_PRINT("ypr,");
  DEBUG_PRINT(ypr[0] * 180/M_PI);
  DEBUG_PRINT(",");
  DEBUG_PRINT(ypr[1] * 180/M_PI);
  DEBUG_PRINT(",");
  DEBUG_PRINT(ypr[2] * 180/M_PI);
  DEBUG_PRINT(",");
}

#ifdef SD_CONNECTED
  void SDPrintYPW(){
    DataFile.print(ypr[0] * 180/M_PI);
    DataFile.print(",");
    DataFile.print(ypr[1] * 180/M_PI);
    DataFile.print(",");
    DataFile.print(ypr[2] * 180/M_PI);
    DataFile.print(",");
  }
#endif

void serialPrintRealAccel(){
  DEBUG_PRINT("areal xyz,");
  DEBUG_PRINT(aaReal.x);
  DEBUG_PRINT(",");
  DEBUG_PRINT(aaReal.y);
  DEBUG_PRINT(",");
  DEBUG_PRINT(aaReal.z);
  DEBUG_PRINT(",");
}

#ifdef SD_CONNECTED
  void SDPrintRealAccel(){
    DataFile.print(aaReal.x);
    DataFile.print(",");
    DataFile.print(aaReal.y);
    DataFile.print(",");
    DataFile.print(aaReal.z);
    DataFile.print(",");
  }
#endif

void serialPrintWorldAccel(){
  DEBUG_PRINT("aworld xyz,");
  DEBUG_PRINT(aaWorld.x);
  DEBUG_PRINT(",");
  DEBUG_PRINT(aaWorld.y);
  DEBUG_PRINT(",");
  DEBUG_PRINT(aaWorld.z);
  DEBUG_PRINT(",");
}

#ifdef SD_CONNECTED
  void SDPrintWorldAccel(){
    DataFile.print(aaWorld.x);
    DataFile.print(",");
    DataFile.print(aaWorld.y);
    DataFile.print(",");
    DataFile.print(aaWorld.z);
    DataFile.print(",");
  }
#endif

void serialPrintRawAccel(){
  DEBUG_PRINT("rawAccel,");
  DEBUG_PRINT(ax); 
  DEBUG_PRINT(",");             
  DEBUG_PRINT(ay); 
  DEBUG_PRINT(",");      
  DEBUG_PRINT(az);
  DEBUG_PRINT(","); 
}

#ifdef SD_CONNECTED
  void SDPrintRawAccel(){
    DataFile.print(ax); 
    DataFile.print(",");             
    DataFile.print(ay); 
    DataFile.print(",");      
    DataFile.print(az);
    DataFile.print(","); 
  }
#endif

void serialPrintRawGyro(){
  DEBUG_PRINT("rawGyro,");
  DEBUG_PRINT(gx); 
  DEBUG_PRINT(",");      
  DEBUG_PRINT(gy); 
  DEBUG_PRINT(",");
  DEBUG_PRINT(gz);
  DEBUG_PRINT(","); 
}

#ifdef SD_CONNECTED
  void SDPrintRawGyro(){
    DataFile.print(gx); 
    DataFile.print(",");      
    DataFile.print(gy); 
    DataFile.print(",");
    DataFile.print(gz);
    DataFile.print(","); 
  }
#endif

