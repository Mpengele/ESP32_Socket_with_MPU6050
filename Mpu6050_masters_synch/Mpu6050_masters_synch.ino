/*
    esp32 Nodemcu 01 MAC Address: A0:B7:65:58:59:64
    Esp32_S2_MINI   MAC Address: 84:FC:E6:C5:3A:6A  (1)
    Esp32_S2_MINI   MAC Address: 84:FC:E6:C5:C6:A4  (2)


    MAC Address: 84:FC:E6:C5:3A:6A  (1)
    MAC Address: 84:FC:E6:C5:C6:A4  (2)

    MAC Address: 84:FC:E6:C5:C6:86  (3)
    MAC Address: 84:FC:E6:C5:98:C0  (4)
*/



#include <esp_task_wdt.h>
#include "soc/rtc_wdt.h"
#include <math.h>
#include <WiFi.h>
#include <Wire.h>
#define I2C_SDA 8            // I2c pin data
#define I2C_SCL 7            // I2c pin clock

#include <esp_now.h>

//*********************** USART lib ***************************************
//*************************************************************************
#include <HardwareSerial.h>
HardwareSerial SerialPort(1); // use UART1
#define LED_PIN 15
#define TX_PIN 17
#define RX_PIN 18
#define VCC_MPU_CONTROL_PIN  33  // Mpu6050 power control with 2N7002
//#define START_SIGNAL_TIME_PIN   XXXX       //define after

//***************** Parameters  related to thermistor *********************
//*************************************************************************
#define TEMP_PIN 4         // thermistor is connected to GPIO 4 (Analog ADC1_CH0) 
#define BAT_PIN 2          // battery is connected to GPIO 2 (Analog ADC1_CH2) 
float TempCel = 0.0;
float batLev = 0.0;

//******************************* CONST DEFINE *************************************
//**********************************************************************************
#define ACC_RANGE   16384.0f
#define TEMP_RANGE  340.0f
#define GYRO_RANGE  131.0f          //For a 250deg/s GYRO_RANGE we have to divide first the raw value by 131.0, according to the datasheet
#define betaDef     0.1f             // 2 * proportional gain
#define pi          3.14159265358979f
#define rad         pi/180.0f              // coverting factor from degrees to radians

//**************************** MPU address storage *********************************
//**********************************************************************************
#define MPU_I2C_ADDR          0x68  // MPU6050 I2C address
#define WHO_AM_I_REG          0x75
#define PWR_MGMT_1_REG        0x6B
#define SMPLRT_DIV_REG        0x19
#define ACCEL_CONFIG_REG      0x1C
#define GYRO_CONFIG_REG       0x1B
#define ACCEL_XOUT_H_REG      0x3B
#define GYRO_XOUT_H_REG       0x43
#define TEMP_OUT_H_REG        0x41

//************ Deep sleep mode and Wifi  parameters def ****************************
//**********************************************************************************

//Deep sleep mode
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  15     /* Time ESP32 will go to sleep (in seconds) */


float eqTimeDiff = 0.0;
int timeDatacollecting = 0;
int startCountSleep = 0;
int stopCountSleep = 0;

// time Watch Dog Timer parameters
float WDT_begin = 0;
float WDT_end = 0;
float WDT_time = 0;

float timeSleepMode =  TIME_TO_SLEEP * uS_TO_S_FACTOR; //in us

//*************************** MAC ADDRESS******************************************************//
//*********************************************************************************************//

// MAC Address of responder - edit as required (reciever)
uint8_t broadcastAddress[] = {0xA8, 0x42, 0xE3, 0xC9, 0xFB, 0x0C};  // esp 32 DEV KIT
String AddrStr = "A8:42:E3:C9:FB:0C";

// 1st Block :: Esp32_S2_MINI (1) and  Esp32_S2_MINI (2)
uint8_t broadcastAddressWheel1[] = {0x84, 0xFC, 0xE6, 0xC5, 0x3A, 0x6A};  // Esp32_S2_MINI (1)
String AddrStrWheel1 = "84:FC:E6:C5:3A:6A";
uint8_t broadcastAddressWheel2[] = {0x84, 0xFC, 0xE6, 0xC5, 0xC6, 0xA4};  // Esp32_S2_MINI (2)
String AddrStrWheel2 = "84:FC:E6:C5:C6:A4";

//  2nd Block :: Esp32_S2_MINI (3) and  Esp32_S2_MINI (4)
uint8_t broadcastAddressWheel3[] = {0x84, 0xFC, 0xE6, 0xC5, 0xC6, 0x86};   // Esp32_S2_MINI (3)
String AddrStrWheel3 = "84:FC:E6:C5:C6:86";
uint8_t broadcastAddressWheel4[] = {0x84, 0xFC, 0xE6, 0xC5, 0x98, 0xC0};   // Esp32_S2_MINI (4)
String AddrStrWheel4 = "84:FC:E6:C5:98:C0";



// 3rd Block ::  Esp32 (5) and  Esp32 (6)
uint8_t broadcastAddressWheel5[] = {0xA0, 0xB7, 0x65, 0x58, 0x59, 0x64};  // esp32 Nodemcu 01
String AddrStrWheel5= "A0:B7:65:58:59:64";            
uint8_t broadcastAddressWheel6[] = {0xA0, 0xB7, 0x65, 0x5A, 0x1F, 0x60};   // esp32 Nodemcu 02
String AddrStrWheel6 = "A0:B7:65:5A:1F:60";


//// 3rd Block ::  Esp32_S2_MINI (5) and  Esp32_S2_MINI (6)
//uint8_t broadcastAddressWheel5[] = {0x84, 0xFC, 0xE6, 0xC5, 0xC6, 0x86};   // Esp32_S2_MINI (5)
//String AddrStrWheel5 = "84:FC:E6:C5:C6:86";
//uint8_t broadcastAddressWheel6[] = {0x84, 0xFC, 0xE6, 0xC5, 0x98, 0xC0};   // Esp32_S2_MINI (6)
//String AddrStrWheel6 = "84:FC:E6:C5:98:C0";

//********************* Define angles data structure ***********************************
//**************************************************************************************
typedef struct struct_Data {
  // 4 bytes  start 0xA94D2B2B  = -4.5556586e-14 (float)
  byte idStart0 = 0x2B;  // + in Hex
  byte idStart1 = 0x2B;  // + in Hex
  byte idStart2 = 0x4D;  // M in Hex
  byte idStart3 = 0x23;  // # in Hex

  int block;
  int espID;
  float bat       = 3.3;
  float temp      = 25.5;
  float pression  = 2; // in bars /1000
  float x =0.0;
  float meanOmega = 0.0;
  float freq;
  float anglesData[17][3];

  // 4 bytes stop CR / LF
  byte stopByte0 = 0x0D;  // Carriage Return CR
  byte stopByte1 = 0x0A;  // line feed LF
} struct_Data;


// Define different of time structure
typedef struct struct_IDModule {
  String strID;
  int intID;
}struct_IDModule;

// Define different of time structure
typedef struct struct_waikUpSignal {
  short block;            // number of pair Wheel  (1 , 2 , or 3)
  short iWaikeUp;         //  1 or 2
} struct_waikUpSignal;


typedef struct struct_control {
  short workingRegime;   //  (3 режим : 1. режим ожидания, 2. режим тестирования, 3. режим работы)
  short block;           // number of pair Wheel  (1 , 2 , or 3)
  float timeSleepEsp1;
  float timeSleepEsp2;
} struct_control;

// Create a structured object
struct_Data myMpuData;
struct_control controlPar;
struct_waikUpSignal hiWaikeUp;
struct_IDModule esp;


// Peer info
esp_now_peer_info_t peerInfo;

// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  char macStr[18];
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  SerialPort.print("Packet to:    ");
  SerialPort.print(macStr);
  SerialPort.print("    send status:\t");
  SerialPort.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback function executed when data (angles) is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&controlPar, incomingData, sizeof(controlPar));

  SerialPort.print("controlPar.timeSleepEsp1 : ");
  SerialPort.print(controlPar.timeSleepEsp1);
  SerialPort.print("\t");
  SerialPort.print("controlPar.timeSleepEsp1 : ");
  SerialPort.println(controlPar.timeSleepEsp1);
  if (controlPar.block == hiWaikeUp.block) {
    if (esp.intID == 1){timeSleepMode = controlPar.timeSleepEsp1 * uS_TO_S_FACTOR;}
    if (esp.intID == 2){timeSleepMode = controlPar.timeSleepEsp2 * uS_TO_S_FACTOR;}
  }
}

//*************************** Variables def ****************************************
//**********************************************************************************

//filter variables
volatile float beta = betaDef;                // 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

float GyroX, GyroY, GyroZ, AccX, AccY, AccZ, temperature;
float mean_Gx, mean_Gy, mean_Gz = 0.0f;
float GyroErrorX, GyroErrorY, GyroErrorZ;
float roll, pitch, yaw;

//ideal case for  Acc data
float minAx = -1.0;
float maxAx = 1.0;
float minAy = -1.0;
float maxAy = 1.0;
float minAz = -1.0;
float maxAz = 1.0;

float offSetX, offSetY, offSetZ = 0.0f;
float scaleFactX, scaleFactY, scaleFactZ = 0.0f;
float dt, sampleFreq, currentTime, previousTime;

uint8_t checkMpuConnected;
float serialOutPutCount = 0.0;    // count 10 measument before after every serial print()


int sensorValue = 500;        // value read from the pot
int outputValue = 0;          // value output to the PWM (analog out)

int countDataStorage = 0;     // index of the  array anglesData where to reccord measured data (angles)
float timeStartSavingData = 0;
float phi_t0, phi_t1, dphi = 0; // values to calulate dphi (angle different)
float measN = 0.0;

void setup() {

  WDT_begin = millis();


  // Set up Serial Monitor and I2C
  SerialPort.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);   // Setup Serial Monitor
  delay(10);

  //IO pin defining
  //  pinMode(START_SIGNAL_TIME_PIN, OUTPUT);  //check up signal to get waike up impulse
  pinMode(LED_PIN, OUTPUT);
  pinMode(VCC_MPU_CONTROL_PIN, OUTPUT);    // Mpu power suply control pin
  delay(10);
  // digitalWrite(START_SIGNAL_TIME_PIN, LOW); //start signal reset
  digitalWrite(VCC_MPU_CONTROL_PIN, HIGH);
  delay(50);

  pinMode(TEMP_PIN, INPUT);                // thermistor connection pin
  pinMode(BAT_PIN, INPUT);                 // battery level pin


  //  Wire.begin(I2C_SDA, I2C_SCL);          // Wire.begin() with no default pin
  Wire.setPins(I2C_SDA, I2C_SCL);         //Call this function before begin() function to change the pins from the default ones.
  Wire.begin();                           // Wire.begin() with no default pin


  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    SerialPort.println("Error initializing ESP-NOW");
    return;
  }

  //***************************************** Espsnow setting **********************************************
  //**********************************************************************************************************

  // Register the send and  the receiv callback
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    SerialPort.println("Failed to add peer");
    return;
  }

  //********************** send waike up signal to receiver  ********************
  //*****************************************************************************
  checkModuleByMAC(WiFi.macAddress());


  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &hiWaikeUp, sizeof(hiWaikeUp));
  if (result == ESP_OK) {
    SerialPort.println("Sending waike up  confirmed");
  } else {
    SerialPort.println("Sending waike up error");
  }

  // ********* initialize digital pin VCC_MPU_CONTROL_PIN as an output ******
  //*************************************************************************

  MPU6050_Init();                 // Initialize mpu

  // ********************* Battery and Temperature  measuerement (Thermistor) *********************************
  //***********************************************************************************************

  TempCel = tempRead(TEMP_PIN); // TEMP_PIN = 4 pin
  delay(50);
  batLev = pinVoltageLevel(BAT_PIN);//  BAT_PIN : battery level
  delay(50);



  // ******************************** gyro error calculation Func *********************************
  //***********************************************************************************************
  //   gyro_IMU_error();     // gyro error calculation

  // ******************************** Automatic gyro calibration **********************************
  //***********************************************************************************************
  //   mean_Gx = GyroErrorX;
  //   mean_Gy = GyroErrorY;
  //   mean_Gz = GyroErrorZ;



  //*********************************** Acc calibration scale *************************************
  //***********************************************************************************************

  offSetX = off_set(minAx , maxAx);
  offSetY = off_set(minAy , maxAy);
  offSetZ = off_set(minAz , maxAz);

  scaleFactX = scale_factor (maxAx, offSetX);
  scaleFactY = scale_factor (maxAy, offSetY);
  scaleFactZ = scale_factor (maxAz, offSetZ);


  startCountSleep = millis();
  serialOutPutCount = millis();
  previousTime = millis();

  delay(20);
}

void loop() {

  currentTime = millis();                     // Current time actual time read
  dt = (currentTime - previousTime) / 1000;   // Divide by 1000 to get seconds
  sampleFreq = 1 / dt; // in Hz

  if (sampleFreq < 20) {
    ESP.restart(); // restart if there's a  problem with reading
  }

  //deep sleep setting
  stopCountSleep = millis();
  timeDatacollecting  = (stopCountSleep - startCountSleep) / 1000;


  //Data reading from Mpu6050
  readMpuData();


  // noise filtration
  GyroX = thresholdFucnction(GyroX, 0.014); // noise when 0.014*180 = 2.5° around zero
  GyroY = thresholdFucnction(GyroY, 0.014);
  GyroZ = thresholdFucnction(GyroZ, 0.014);

  //************************** Madgwick Filter Funct  ***********************************
  //*************************************************************************************
  phi_t0 = phi_t1;

  MadgwickAHRSupdateIMU(GyroX, GyroY, GyroZ, AccX, AccY, AccZ);
  eulerAngles(q0, q1, q2, q3, &roll, &pitch, &yaw);

  // counting dgamma (dphi or  dYaw) so that we can save in angles[][]
  phi_t1 = yaw;

  if ((timeDatacollecting <= 3) && (millis() - serialOutPutCount) / 1000 > 0.04) { //0.08s = 12.5 Hz
    serialOutPutCount = millis();
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Toggle LED_PIN
    SerialPort.print(esp.strID);
    SerialPort.print(" , ");
    SerialPort.print(sampleFreq, 3);
    SerialPort.print(" , ");
    SerialPort.print(roll, 3);
    SerialPort.print(" , ");
    SerialPort.print(pitch, 3);
    SerialPort.print(" , ");
    SerialPort.print(yaw, 3);
    SerialPort.print(" , ");
    //  SerialPort.println(temperature,1);  // with mpu5060 temperature  sensor
    SerialPort.print(TempCel, 1); // with Thermistor temperature  sensor
    SerialPort.print(" , ");
    SerialPort.println(batLev, 1); //battery level

  }


  //************************** STORAGE DATA AND SLEEP MODE  **********************************
  //*************************************************************************************

  timeStartSavingData = timeDatacollecting;

  if (timeDatacollecting > 3) {

    SerialPort.print("countDataStorage : ");
    SerialPort.println(countDataStorage);
    SerialPort.println("");

    dphi = abs(phi_t1 - phi_t0);
    measN = 17 / dphi;

    switch (countDataStorage) {
      case 0:
        //        myMpuData.temp      =  temperature;   // with mpu5060 temperature  sensor
        myMpuData.temp                =  TempCel;       // with Thermistor temperature  sensor
        myMpuData.bat                 =  batLev;
        myMpuData.freq                =  sampleFreq;
        myMpuData.anglesData[0][0]    =  roll;
        myMpuData.anglesData[0][1]    =  pitch;
        myMpuData.anglesData[0][2]    =  yaw;
        break;

      default:
        myMpuData.anglesData[countDataStorage][0] = roll;
        myMpuData.anglesData[countDataStorage][1] = pitch;
        myMpuData.anglesData[countDataStorage][2] = yaw;
        break;

    }
    myMpuData.meanOmega += GyroZ;
    if(countDataStorage == 16){
      myMpuData.meanOmega = abs(myMpuData.meanOmega/17);
      SerialPort.print("myMpuData.meanOmega :  ");
      SerialPort.println(myMpuData.meanOmega);
      }
    countDataStorage++;

    if (countDataStorage >= 17) {
      
      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myMpuData, sizeof(myMpuData));
      SerialPort.println(result == ESP_OK ? "Sending confirmed" : "Sending error");

      delay(1000);

      SerialPort.print("timeSleepMode ");
      SerialPort.print(esp.strID);
      SerialPort.print(" : ");
      SerialPort.print(timeSleepMode);
      SerialPort.println(" in us ");
      delay(100);

      //  WDT  time  (full time  for  the  program )
      WDT_time =  (millis() - WDT_begin) / 1000;
      SerialPort.print("\nWDT_time [s] :\t");
      SerialPort.print(WDT_time);

      esp_sleep_enable_timer_wakeup(timeSleepMode); // the function use time_in_us as parameter )
      esp_deep_sleep_start();
    }
  }

  //  delay(10);
  previousTime = currentTime;        // Previous time is stored before the actual time read


  ////  ESP.wdtfeed();
  //    rtc_wdt_feed();
}


void MPU6050_Init() {
  delay(100);
  uint8_t ReadfromReg;
  uint8_t WriteToReg;

  /* We need to check if the sensor is responding by reading the “WHO_AM_I_REG (0x75)” Register.
    If the sensor responds with 0x68, this means it’s available and good to go.*/

  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(WHO_AM_I_REG);
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_I2C_ADDR, (size_t)1, true);
  ReadfromReg = Wire.read(); // 0x68  = 104 will be returned by the sensor if everything goes wellvalue
  checkMpuConnected = ReadfromReg;
  SerialPort.print("WHO_AM_I_REG read value :  ");
  SerialPort.println(ReadfromReg);
  delay(100);

  if (ReadfromReg == 104)  {

    /* we will wake the sensor up and in order to do that we will write to the “PWR_MGMT_1 (0x6B)” Register
      On writing (0x00) to the PWR_MGMT_1 Register, sensor wakes up and the Clock sets up to 8 MHz*/
    WriteToReg = 0x00;
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(PWR_MGMT_1_REG);
    Wire.write(WriteToReg);
    Wire.endTransmission(true);
    delay(100);

    /*we have to set the Data output Rate or Sample Rate. This can be done by writing into “SMPLRT_DIV (0x19)” Register.
      This register specifies the divider from the gyroscope output rate used to generate the Sample Rate for the MPU6050.
      As the formula says Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). Where Gyroscope Output Rate is 8KHz,
      To get the sample rate of 1KHz, we need to use the SMPLRT_DIV as �?7’.      */
    WriteToReg = 0x07;
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(SMPLRT_DIV_REG);
    Wire.write(WriteToReg);
    Wire.endTransmission(true);
    delay(100);


    /*We need to configure the Accelerometer and Gyroscope registers and to do so,
      we need to modify “GYRO_CONFIG (0x1B)” and “ACCEL_CONFIG (0x1C)”Registers.
      Writing (0x00) to both of these registers would set the Full scale ACC_RANGE of ± 2g in ACCEL_CONFIGRegister
      and a Full scale GYRO_RANGE of ± 250 °/s in GYRO_CONFIGRegister along with Self-test disabled.*/

    WriteToReg = 0x00;
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(ACCEL_CONFIG_REG);     // Set accelerometer configuration in ACCEL_CONFIG Register
    Wire.write(WriteToReg);            // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
    Wire.endTransmission(true);
    delay(100);

    WriteToReg = 0x00;
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(GYRO_CONFIG_REG);     // Set Gyroscopic configuration in GYRO_CONFIG Register
    Wire.write(WriteToReg);  // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
    Wire.endTransmission(true);
    delay(100);
  } else {
    while (checkMpuConnected != 104) {
      SerialPort.println("Sensor on I2C is not connected, Connect the sensor please");
      MPU6050_Init();
    }
  }

}   //void MPU6050_Init (void)

void gyro_IMU_error() {


  // We can call this funtion in the setup section to calculate the gyro data error.
  // From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values

  // Read gyro values 500 times
  int n = 1000;
  int c = 0;
  while (c < n) {
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(GYRO_XOUT_H_REG);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)MPU_I2C_ADDR, (size_t)6, true);

    GyroX = ((int16_t(Wire.read() << 8 | Wire.read())) / GYRO_RANGE) * rad; // For a 250deg/s GYRO_RANGE we have to divide first the raw value by 131.0, according to the datasheet
    GyroY = ((int16_t(Wire.read() << 8 | Wire.read())) / GYRO_RANGE) * rad;
    GyroZ = ((int16_t(Wire.read() << 8 | Wire.read())) / GYRO_RANGE) * rad;

    // Sum all readings
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }

  //Divide the sum by "n" to get the error value
  GyroErrorX /= n;
  GyroErrorY /= n;
  GyroErrorZ /= n;

  // Print the error values on the Serial Monitor
  SerialPort.print("GyroErrorX: ");
  SerialPort.print(GyroErrorX, 3);
  SerialPort.print(" , ");
  SerialPort.print("GyroErrorY: ");
  SerialPort.print(GyroErrorY, 3);
  SerialPort.print(" , ");
  SerialPort.print("GyroErrorZ: ");
  SerialPort.println(GyroErrorZ, 3);
}


void readMpuData() {
  delay(5);
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(ACCEL_XOUT_H_REG); // Start with register 0x3B (ACCEL_XOUT_H_REG)
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_I2C_ADDR, (size_t)6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a ACC_RANGE of +-2g, we need to divide the raw values by 16384 (ACC_RANGE), according to the datasheetand write 0x00 in ACCEL_CONFIG_REG
  //For a ACC_RANGE of +-4g, we need to divide the raw values by 8192 (ACC_RANGE), according to the datasheet and write 0x01 in ACCEL_CONFIG_REG

  AccX = (int16_t(Wire.read() << 8 | Wire.read())) / ACC_RANGE; // X-axis value
  AccY = (int16_t(Wire.read() << 8 | Wire.read())) / ACC_RANGE; // Y-axis value
  AccZ = (int16_t(Wire.read() << 8 | Wire.read())) / ACC_RANGE; // Z-axis value

  // //apply offset and scale factor to calibrite data
  AccX = (AccX - offSetX) * scaleFactX;
  AccY = (AccY - offSetY) * scaleFactY;
  AccZ = (AccZ - offSetZ) * scaleFactZ;

  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(TEMP_OUT_H_REG); // Start with register 0x41 (TEMP_OUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_I2C_ADDR, (size_t)2, true); // Read 2 registers total, each value is stored in 2 registers
  //  Temperature in degrees C = (TEMP_OUT Register Value as a signed quantity)/340 + 36.53
  temperature = ((int16_t(Wire.read() << 8 | Wire.read())) / TEMP_RANGE) + 36.53 ; // temperature in °C

  // === Read gyroscope data === //
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(GYRO_XOUT_H_REG); // Gyro data first register address 0x43 (GYRO_XOUT_H_REG)
  Wire.endTransmission(false);
  Wire.requestFrom((uint16_t)MPU_I2C_ADDR, (size_t)6, true); // Read 4 registers total, each axis value is stored in 2 registers

  GyroX = ((int16_t(Wire.read() << 8 | Wire.read())) / GYRO_RANGE) * rad; // For a 250deg/s GYRO_RANGE we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = ((int16_t(Wire.read() << 8 | Wire.read())) / GYRO_RANGE) * rad;
  GyroZ = ((int16_t(Wire.read() << 8 | Wire.read())) / GYRO_RANGE) * rad;

  // Correct the outputs with the calculated error values
  GyroX -= mean_Gx;
  GyroY -= mean_Gy;
  GyroZ -= mean_Gz;
}


float pinVoltageLevel(int PIN) {

  float Vs = 3.3;       // supply voltage 3.3 v for esp
  float Vref = 1.1;         //Max voltage  ADC can measure
  float ADCMax = 8191.0;  // ADC resolution 13-bit (0- 8191) >> 2^13 = 8192
  float R1 = 6700.0;     // first resistor divider in ohm
  float R2 = 5000.0;      // second resistor divider in ohm
  float Vout, batLev = 0.0;

  Vout = (analogRead(PIN) / ADCMax) * Vs; //in volt
  batLev = Vout * (1 + (R1 / R2));

  //    return Vout;
  return batLev;
}



float tempRead(int  PIN) {

  float Vs = 3.3;           // supply voltage 3.3 v for esp
  float Vref = 1.1;         //Max voltage  ADC can measure
  float ADCMax = 8191.0;    // ADC resolution 13-bit (0- 8191) >> 2^13 = 8192
  float NTC_Beta = 3950.0;  // Beta : the material constant of NTC thermistor, is also called heat sensitivity index
  float R1 = 10000.0;       // voltage divider resistor value
  float R2 = 50000.0;      // voltage divider resistor value in parallet with the  thermistor
  float Ro = 10000.0;       // Resistance of Thermistor at 25 degree Celsius
  float To = 298.15;        // Temperature in Kelvin for 25 degree Celsius

  float Vout, Rtol, Rt = 0;
  float tempKel, TempCel = 0;  // temperature  in Kelvin and  celsius

  // Reading input value

  Vout = (analogRead(PIN) / ADCMax) * Vs;
  Vout = Vout - 0.4;
  Rtol = (R1 * Vout) / (Vs - Vout);     // total resistor in serie with R1   (i.)
  Rt =   (Rtol * R2) / (R2 - Rtol);     //real time thermistor resistor value in ohm

  tempKel = 1 / ((1 / To) + log(Rt / Ro) / NTC_Beta);  // Temperature in Kelvin
  TempCel = tempKel - 273.15;                          // Temperature in Celsius

  //   return (Vout-0.35);
  return TempCel;
}
