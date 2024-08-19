//************************************ FUNCTIONS ******************************************
//*****************************************************************************************
// threshold function
float thresholdFucnction(float a, float thres){

  if (fabs(a) <= thres)
  {
    a = 0.0;
  }
  return a;
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_i nverse_square_root

float invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

//********************** Madgwick Filter Funct *************************
//*********************************************************************

// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;   

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * (1.0f / sampleFreq);
  q1 += qDot2 * (1.0f / sampleFreq);
  q2 += qDot3 * (1.0f / sampleFreq);
  q3 += qDot4 * (1.0f / sampleFreq);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------

/*
 returns as pointers, roll pitch and yaw from the quaternion generated in imu_filter
 Assume right hand system
 Roll is about the x axis, represented as phi
 Pitch is about the y axis, represented as theta
 Yaw is about the z axis, represented as psi (trident looking greek symbol)
 */
//void eulerAngles(float q0, float q1, float q2, float q3, float* roll, float* pitch, float* yaw

void eulerAngles(float q0, float q1, float q2, float q3, float* phi, float* theta, float* psy){


    *psy = atan2f((2*q1*q2 - 2*q0*q3), (2*q0*q0 + 2*q1*q1 -1));
    *theta = -asinf(2*q1*q3 + 2*q0*q2);
    *phi  = atan2f((2*q2*q3 - 2*q0*q1), (2*q0*q0 + 2*q3*q3 -1));

//  *theta= atan2f((2*q1*q3 + 2*q0*q2), sqrt(1-(2*q1*q3 + q0*q2)*(2*q1*q3 + q0*q2)));

//    *psy = atan2f((2*q2*q3 + 2*q0*q1), (1-2*q1*q1 + 2*q2*q2 ));
////  *theta = asinf( 2*q0*q2 -2*q1*q3);
//    *theta= (-pi/2)+ 2*(atan2f(sqrt(1+2*q0*q2 - 2*q1*q3), sqrt(1-2*q0*q2 +2*q1*q3)));
//    *phi = atan2f((2*q0*q3 + 2*q1*q2), (1-2*q2*q2 + 2*q3*q3 ));

    *phi *= (180.0f /pi);
    *theta *= (180.0f /pi);
    *psy *= (180.0f /pi);
}
//**************************** Acc calibration fucntion********************************
//*************************************************************************************

float off_set(float minValue , float maxValue){
  return (minValue+maxValue)/2;
}
float scale_factor (float maxValue, float offsetValue){
  float grav = 1.0f ;// value of gravity in G 
  return(grav/(maxValue-offsetValue));
}


void checkModuleByMAC(String macAdress){
  
  if(macAdress == AddrStrWheel1) {
      
      myMpuData.block    = 1;
      myMpuData.espID    = 1;
      hiWaikeUp.block    = 1;
      hiWaikeUp.iWaikeUp = 1; 
      esp.intID          = 1;
      esp.strID          = "wheel1";
      
    //Mean value  Esp32_S2_MINI   MAC Address: 84:FC:E6:C5:3A:6A  (1)
      mean_Gx =  1.130;  // the value of GyroErrorX
      mean_Gy =  -0.009; // the value of  GyroErrorY
      mean_Gz =  0.011;
    //pratical case for  Acc given sesor № 1 MAC Address: 84:FC:E6:C5:3A:6A  (1)
      minAx = -0.971; maxAx = 1.041;
      minAy = -1.01;  maxAy = 1.004;
      minAz = -0.908; maxAz = 1.113;} 
      
    else if(macAdress == AddrStrWheel2){

      myMpuData.block    = 1;
      myMpuData.espID    = 2;
      hiWaikeUp.block    = 1;
      hiWaikeUp.iWaikeUp = 2;
      esp.intID          = 2;
      esp.strID          = "wheel2";
      
    //Mean value   Esp32_S2_MINI   MAC Address: 84:FC:E6:C5:C6:A4  (2)
      mean_Gx = -0.034;  //  the value of GyroErrorX
      mean_Gy = 0.004 ;   //  is the value of  GyroErrorY
      mean_Gz = 0.014 ;
    //pratical case for  Acc given sesor № 2 MAC Address: 84:FC:E6:C5:C6:A4  (2)
      minAx = -0.945;  maxAx = 1.053;
      minAy = -1.027;  maxAy = 0.971;
      minAz = -1.004;  maxAz = 1.040;}
      
    else if(macAdress == AddrStrWheel3){
      
      myMpuData.block    = 2;
      myMpuData.espID    = 1;
      hiWaikeUp.block    = 2;
      hiWaikeUp.iWaikeUp = 1; 
      esp.intID          = 1;
      esp.strID          = "wheel3";
      
    //Mean value  Esp32_S2_MINI  MAC Address: 84:FC:E6:C5:C6:86  (3)
      mean_Gx = 0.033;  //  the value of GyroErrorX
      mean_Gy = -0.030; //  is the value of  GyroErrorY
      mean_Gz = 0.000;
     //pratical case for  Acc given sesor № 3 MAC Address: 84:FC:E6:C5:C6:86  (3)
      minAx = -0.948;  maxAx = 1.070;
      minAy = -1.005;  maxAy = 0.995;
      minAz = -1.152;  maxAz = 0.924;}
      
    else if(macAdress == AddrStrWheel4){
      
      myMpuData.block    = 2;
      myMpuData.espID    = 2;
      hiWaikeUp.block    = 2;
      hiWaikeUp.iWaikeUp = 2;
      esp.intID          = 2;
      esp.strID          = "wheel4"; 
      
    //Mean value  Esp32_S2_MINI  MAC Address: 84:FC:E6:C5:98:C0  (4)
      mean_Gx = 0.022;  //  the value of GyroErrorX
      mean_Gy = -0.011 ; //  is the value of  GyroErrorY
      mean_Gz = -0.003;
      //pratical case for  Acc given sesor № 4 MAC Address: 84:FC:E6:C5:98:C0  (4)
      minAx = -0.980;   maxAx =  1.022;
      minAy = -1.016;   maxAy = 0.980;   
      minAz = -0.980;   maxAz = 1.058; }
      
   else if(macAdress == AddrStrWheel5){

      myMpuData.block    = 3;
      myMpuData.espID    = 1;
      hiWaikeUp.block    = 3;
      hiWaikeUp.iWaikeUp = 1; 
      esp.intID          = 1;
      esp.strID          = "wheel5"; 


      //Mean value  Esp32_S2_MINI  MAC Address: A0:B7:65:58:59:64  (5)
      mean_Gx = 0.026; // 0.026 is the value of GyroErrorX
      mean_Gy = 0.010; 
      mean_Gz = 0.004; 
      //pratical case for  Acc given sesor № 4 MAC Address: A0:B7:65:58:59:64  (5)
      minAx = -0.968;  maxAx = 1.038;
      minAy = -1.021;  maxAy = 0.983;
      minAz = -0.827;  maxAz = 1.227;}
      
   else if(macAdress == AddrStrWheel6){

      myMpuData.block    = 3;
      myMpuData.espID    = 2;
      hiWaikeUp.block    = 3;
      hiWaikeUp.iWaikeUp = 2; 
      esp.intID          = 2;
      esp.strID          = "wheel6"; 

      //Mean value  Esp32_S2_MINI  MAC Address: A0:B7:65:5A:1F:60  (6)
      mean_Gx = 0.132; // 0.132 is the value of GyroErrorX
      mean_Gy = 0.048; 
      mean_Gz = 0.037;
      //pratical case for  Acc given sesor № 4 MAC Address: A0:B7:65:5A:1F:60  (6)
      minAx = -0.966;  maxAx = 1.048;
      minAy = -0.987;  maxAy = 1.030;
      minAz = -0.853;  maxAz = 1.200;}
  
  }



/*
void moveDataInDataContainer(float DataContainer[5][12],  float freq,  float roll,  float pitch,  float yaw){
  
  //move second line to first line
  for (int i = 0; i < 11; i++){
      for (int j = 0; j < 4; j++){ 
        DataContainer[i][j] = DataContainer[i+1][j];
      }
        DataContainer[12][0] = freq;
        DataContainer[12][1] = roll;
        DataContainer[12][2] = pitch;
        DataContainer[12][3] = yaw;
  } }


  bool macAddressCheck(uint8_t addr1[6], uint8_t addr2[6]){
    bool checkAddress = true;
      for(int i =0; i <6; i++){
        
        checkAddress &= (addr1[i] == addr2[i]);
     
        }    
    return checkAddress;
    } 

    */
