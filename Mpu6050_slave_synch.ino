 /*



*/
 
// Include Libraries

#include <WiFi.h>
#include <esp_now.h>

/*
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>

const char* ssid = "Keenetic-9461";
const char* password = "VAsbNoxP";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
  <p>{"BR" : 12,"TR": 13, "WR" : 15, "BL" : 12,"TL": 13, "WL" : 15}</p>
</body>  
</html>)rawliteral";
*/

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10       /* Time ESP32 will go to sleep (in seconds) */
#define DISYNCH_ONE_TWO  0.1f  /* time of disychronous between esp1 and esp2*/ 
float int2floatScale = 1000.0;

float timeCounter =0.0;



//*************************** MAC ADDRESS******************************************************//
//*********************************************************************************************//

// MAC Address of responder - edit as required (reciever)
uint8_t broadcastAddress[] = {0xA8, 0x42, 0xE3, 0xC9, 0xFB, 0x0C};  // esp 32 DEV KIT
String AddrStr = "A8:42:E3:C9:FB:0C";

  // 1st Block :: Esp32_S2_MINI (1) and  Esp32_S2_MINI (2)
 uint8_t broadcastAddressWheel1[] = {0x84, 0xFC, 0xE6, 0xC5, 0x3A, 0x6A};  // Esp32_S2_MINI (1)
 String AddrStrWheel1= "84:FC:E6:C5:3A:6A";
 uint8_t broadcastAddressWheel2[] = {0x84, 0xFC, 0xE6, 0xC5, 0xC6, 0xA4};  // Esp32_S2_MINI (2)
 String AddrStrWheel2 = "84:FC:E6:C5:C6:A4";

//  2nd Block :: Esp32_S2_MINI (3) and  Esp32_S2_MINI (4)
uint8_t broadcastAddressWheel3[] = {0x84, 0xFC, 0xE6, 0xC5, 0xC6, 0x86};   // Esp32_S2_MINI (3)
String AddrStrWheel3 = "84:FC:E6:C5:C6:86";
uint8_t broadcastAddressWheel4[] = {0x84, 0xFC, 0xE6, 0xC5, 0x98, 0xC0};   // Esp32_S2_MINI (4)
String AddrStrWheel4 = "84:FC:E6:C5:98:C0";

// 3rd Block ::  Esp32_S2_MINI (5) and  Esp32_S2_MINI (6)
uint8_t broadcastAddressWheel5[] = {0x84, 0xFC, 0xE6, 0xC5, 0xC6, 0x85};   // Esp32_S2_MINI (5)
String AddrStrWheel5 = "84:FC:E6:C5:C6:86";
uint8_t broadcastAddressWheel6[] = {0x84, 0xFC, 0xE6, 0xC5, 0x98, 0xC1};   // Esp32_S2_MINI (6)
String AddrStrWheel6 = "84:FC:E6:C5:98:C0";

float timeRec11 = 0;
float timeRec12 = 0;
int esp02SendStatusFail = 0;
float dt_sleep11_12;
int equalCountEsp01 =0;
int equalCountEsp02 =0;
float sleepModeTime = TIME_TO_SLEEP*uS_TO_S_FACTOR; // in s

// stuct to jSON data

typedef struct struct_wheelData{
  
  int id;
  float bat;
  float temp;
  float freq;
  float roll[17];
  float pitch[17];
  float yaw[17];
  
  float dyaw[17];
  float meanYaw;
  float meandyaw;
  float w;
  }struct_wheelData;
  
struct_wheelData wheel1;
struct_wheelData wheel2;

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
  int bat       = int(3.3 * 1000);
  int temp      = int(25.5 * 1000);
  int pression  = int(2*1000);  // in bars /1000
  int x;
  int meanOmega;
  int freq;                                                                                                                                                                                                      
  int anglesData[17][3];

  // 4 bytes stop CR / LF
  byte stopByte0 = 0x0D;  // Carriage Return CR
  byte stopByte1 = 0x0A;  // line feed LF
} struct_Data;

float lengthData;
String ESP_ID = ""; // string for  printing Esp01 or Esp02 in serial port


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

// Peer info
esp_now_peer_info_t peerInfo;

//struct send function 
void sendStructure(byte *structurePointer, int structureLength)
{
    Serial.write(structurePointer, structureLength);
}


 // Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  /*
  char macStr[18];
 
  // Copies the sender mac address to 
  .a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Packet to:    ");
  Serial.print(macStr);
  Serial.print("    send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
 */
}

// Callback function executed when data (angles) is received  
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len <= 6){
    memcpy(&hiWaikeUp, incomingData, sizeof(hiWaikeUp));
    
    switch (hiWaikeUp.block) {
        case 1:
            
            break;

        case 2:
            synch();
            break;

        case 3:
 
            break;
    }
    
  } else {
    
   memcpy(&myMpuData, incomingData, sizeof(myMpuData));

    switch (myMpuData.block) {
        case 1:
            
            break;

        case 2:
              spreadData();
            break;

        case 3:
 
            break;
    } 
  }
}


void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

   // Print MAC Address to Serial monitor
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

   // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  // Register callback function (angles)
  esp_now_register_recv_cb(OnDataRecv);

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddressWheel1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 01");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddressWheel2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 02");
    return;
  }

 // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddressWheel3, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 03");
    return;
  }

  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddressWheel4, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 04");
    return;
  }

  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddressWheel5, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 05");
    return;
  }

    // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddressWheel6, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer 06");
    return;
  }
  
/*
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println();
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());   //  192.168.14.78


  // Route for root / web page

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });


    // Start server
  server.begin();


*/


}
 
void loop() {

//  timeCounter = millis();
//  if (( millis()-timeCounter)/1000.0 > 1){
//
//     Serial.print(wheel1.bat);
//     Serial.print(",");
//     Serial.print(wheel1.temp);
//     Serial.print(",");
//     Serial.print(wheel1.w);
//     Serial.print(",");
//     
//     Serial.print(wheel2.bat);
//     Serial.print(",");
//     Serial.print(wheel2.temp);
//     Serial.print(",");
//     Serial.println(wheel2.w);
//     timeCounter = 0.0;
//  }
 

}

void synch(){
      
    switch (hiWaikeUp.iWaikeUp){
      case 1:
        timeRec11 = millis();
        equalCountEsp01++;
        break; 
        
      case 2:
        timeRec12 = millis(); 
        equalCountEsp02++;
        break;
        }
    
    if ((equalCountEsp01==1) && (equalCountEsp02==1)){
      
      controlPar.workingRegime = 1;
      controlPar.block = hiWaikeUp.block;
   
      dt_sleep11_12 = (timeRec12-timeRec11)/1000; // calcutation dt in s
//      /*
      Serial.print("\ntimeRec11 :\t");
      Serial.print(timeRec11);
      Serial.print("\ttimeRec12 :\t");
      Serial.println( timeRec12);
      Serial.print("dt_sleep11_12  [s] :\t");
      Serial.println( dt_sleep11_12);
//       */   
      // format time  control sleep time to send to masters
      if (dt_sleep11_12 >= 0 ){

        controlPar.timeSleepEsp1 = TIME_TO_SLEEP;
        controlPar.timeSleepEsp2 = TIME_TO_SLEEP -(dt_sleep11_12-DISYNCH_ONE_TWO);

      } else{
        controlPar.timeSleepEsp1 = TIME_TO_SLEEP - ( abs(dt_sleep11_12 - DISYNCH_ONE_TWO));
        controlPar.timeSleepEsp2 = TIME_TO_SLEEP;

      }
      if (controlPar.timeSleepEsp1  < 0 ){ controlPar.timeSleepEsp1  = 0;}
      if (controlPar.timeSleepEsp2  < 0 ){ controlPar.timeSleepEsp2  = 0;}
      
//      /*
      Serial.print("\ntimeSleepEsp1 before Sent [s]:\t");
      Serial.print( controlPar.timeSleepEsp1);
      Serial.print("\ttimeSleepEsp2 before Sent [s]:\t");
      Serial.println( controlPar.timeSleepEsp2);
//      */
      
//      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &controlPar, sizeof(controlPar));
      esp_err_t result = esp_now_send(0, (uint8_t *) &controlPar, sizeof(controlPar));
      /*
       Serial.println(result == ESP_OK ? "Sending confirmed" : "Sending error");
      
       */
      delay(1000);

//     /*
      Serial.print("timeSleepEsp1 after sent [s] :\t");
      Serial.print( controlPar.timeSleepEsp1);
      Serial.print("\ttimeSleepEsp2 after sent [s] :\t");
      Serial.println( controlPar.timeSleepEsp2);
      Serial.println("");
//      */

      equalCountEsp01 = 0;
      equalCountEsp02 = 0;
      
      } else {
        if(equalCountEsp01 >= 2){equalCountEsp01 = 0;}
        if(equalCountEsp02 >= 2){equalCountEsp02 = 0;}
      }
}

void spreadData(){

  if(myMpuData.espID == 1){
    
  /*
  wheel1.id      = myMpuData.ESP_ID;
  wheel1.bat     = myMpuData.bat/int2floatScale;
  wheel1.temp    = myMpuData.temp/int2floatScale;
  wheel1.freq    = myMpuData.freq/int2floatScale;

  wheel1.meanYaw = 0.0;
  for (int i = 0; i <18; i++){
    
    wheel1.roll[i]   = (myMpuData.anglesData[i][0])/int2floatScale;
    wheel1.pitch[i]  = (myMpuData.anglesData[i][1])/int2floatScale;
    wheel1.yaw[i]    = (myMpuData.anglesData[i][2])/int2floatScale;
    wheel1.meanYaw +=wheel1.yaw[i];
    } 
  wheel1.meanYaw/=18;

  wheel1.meandyaw =0.0;
  for(int i =0; i <17; i++){
    
    wheel1.dyaw[i] = (wheel1.yaw[i+1]- wheel1.yaw[i]);
    wheel1.meandyaw +=abs(wheel1.dyaw[i]);
    }
  wheel1.meandyaw/=17;
  wheel1.w = wheel1.meandyaw*wheel1.freq;
  */
  

  // Send a  Data structure on the  COM
  sendStructure((byte*)&myMpuData, sizeof(myMpuData));
  Serial.println();
//  Serial.println(wheel1.meanYaw);
//  Serial.println(wheel1.w);

/*
// transfert  battery , temperature, w
     Serial.print(wheel1.bat);
     Serial.print(",");
     Serial.print(wheel1.temp);
     Serial.print(",");
     Serial.print(wheel1.w);
     Serial.print(",");
     
     Serial.print(wheel2.bat);
     Serial.print(",");
     Serial.print(wheel2.temp);
     Serial.print(",");
     Serial.println(wheel2.w);

*/
//    /*
      ESP_ID = "Esp 01";
      lengthData = sizeof(hiWaikeUp);
      Serial.print(ESP_ID); Serial.print(" , ");
      Serial.print(lengthData); Serial.print(" , ");
      Serial.print(myMpuData.freq); Serial.print(" , ");
      Serial.print(myMpuData.anglesData[16][0]); Serial.print(" , ");
      Serial.print(myMpuData.anglesData[16][1]); Serial.print(" , ");
      Serial.print(myMpuData.anglesData[16][2]); Serial.print(" , ");
      Serial.println(myMpuData.temp);

//    */
  
  
  }
  else if(myMpuData.espID == 2){


/*
  wheel2.id      = myMpuData.ESP_ID;
  wheel2.bat     = myMpuData.bat/int2floatScale;
  wheel2.temp    = myMpuData.temp/int2floatScale;
  wheel2.freq    = myMpuData.freq/int2floatScale;

  wheel2.meanYaw = 0.0;
  for (int i = 0; i <18; i++){
    
    wheel2.roll[i]   = (myMpuData.anglesData[i][0])/int2floatScale;
    wheel2.pitch[i]  = (myMpuData.anglesData[i][1])/int2floatScale;
    wheel2.yaw[i]    = (myMpuData.anglesData[i][2])/int2floatScale;
    wheel2.meanYaw +=wheel2.yaw[i];
    } 
  wheel2.meanYaw/=18;

  wheel2.meandyaw =0.0;
  for(int i =0; i <17; i++){
    
    wheel2.dyaw[i] = (wheel1.yaw[i+1]- wheel1.yaw[i]);
    wheel2.meandyaw +=abs(wheel2.dyaw[i]);
    }
  wheel2.meandyaw/=17;
  wheel2.w = wheel2.meandyaw*wheel2.freq;
*/

// Send a  Data structure on the  COM
  Serial.print("\t\t\t\t\t\t\t");
  sendStructure((byte*)&myMpuData, sizeof(myMpuData));
  Serial.println();
//  Serial.println(wheel2.meanYaw);
//  Serial.println(wheel2.w);


/*
// transfert  battery , temperature, w

     Serial.print(wheel1.bat);
     Serial.print(",");
     Serial.print(wheel1.temp);
     Serial.print(",");
     Serial.print(wheel1.w);
     Serial.print(",");
     
     Serial.print(wheel2.bat);
     Serial.print(",");
     Serial.print(wheel2.temp);
     Serial.print(",");
     Serial.println(wheel2.w);
*/

//    /*
    
      ESP_ID = "Esp 02";
      lengthData = sizeof(hiWaikeUp);
      Serial.print("\t\t\t\t\t\t\t");
      Serial.print(ESP_ID);Serial.print(" , ");
      Serial.print(lengthData); Serial.print(" , ");
      Serial.print(myMpuData.freq); Serial.print(" , ");
      Serial.print(myMpuData.anglesData[16][0]);Serial.print(" , ");
      Serial.print(myMpuData.anglesData[16][1]); Serial.print(" , ");
      Serial.print(myMpuData.anglesData[16][2]); Serial.print(" , ");
      Serial.println(myMpuData.temp);

//      */

        

      
      }
}
