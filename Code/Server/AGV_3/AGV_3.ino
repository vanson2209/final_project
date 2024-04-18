#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsClient.h> 
#include <SoftwareSerial.h>
#include <Arduino.h>

#define RX2 13
#define TX2 15
SoftwareSerial mySerial(RX2, TX2);

WebSocketsClient webSocket;

const char* WIFI_SSID = "abc";
const char* WIFI_PASS = "tamsotam";

const char* ip_host = "192.168.61.10"; 
const uint16_t port = 81; 

String energy = "N/A";
String status = "Unknow";
uint8_t Power;
//String ipAddress;
char receivedChar;
uint8_t Check_P = 0;
uint32_t t = 0x0FFFF0;
uint16_t ADC;

//const char* serverIP = "192.168.2.1";
//const int serverPort = 80;

ESP8266WebServer server(80);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  
  String payloadString = (const char *)payload;
  Serial.println(payloadString);
  char AGV_num = payloadString[0];
  char AGV_in4 = payloadString[1];
  payloadString = payloadString.substring(2);
  if (AGV_num == '3'){
    if(AGV_in4 == 'P'){
      if (payloadString == "ON") {
        if(Check_P == 0){
          mySerial.write('0');
        }
        Check_P = 1;
      }
      else if (payloadString == "OFF"){
        if(Check_P == 1){
          mySerial.write('1');
        }
        Check_P = 0;
      } 
    }
    else if(AGV_in4 == 'D'){
      mySerial.write('7');
      delay(100);
      if (payloadString == "RS1") {
        mySerial.write('3');
 //         Serial.println("Destination is A1");
      }
      else if (payloadString == "RS2"){
        mySerial.write('4');
          //Serial.println("Destination is A2");
      }
      else if (payloadString == "RS3"){
        mySerial.write('5');
          //Serial.println("Destination is B1");
      }
      else if (payloadString == "RS4"){
        mySerial.write('6');
          Serial.println("Destination is B2");
      }
    }
    else if(AGV_in4 == 'A'){
      if (payloadString == "On") {
          mySerial.write('8');
          //Serial.println("AGV1 ON");
      }
      else if (payloadString == "Of"){
          mySerial.write('2');
          //Serial.println("AGV1 OFF");
      }
    }
    else if(AGV_in4 == 'C'){
      if (payloadString == "S") {
          mySerial.write('E');
          //Serial.println("AGV1 ON");
      }
      else if (payloadString == "U"){
          mySerial.write('A');
          //Serial.println("AGV1 OFF");
      }
      else if (payloadString == "D"){
          mySerial.write('B');
          //Serial.println("AGV1 OFF");
      }
      else if (payloadString == "L"){
          mySerial.write('C');
          //Serial.println("AGV1 OFF");
      }
      else if (payloadString == "R"){
          mySerial.write('D');
          //Serial.println("AGV1 OFF");
      }
    }
  }
} 

void  setup(){
  mySerial.begin(115200);
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  //  Serial.print("http://");
 //  Serial.println(WiFi.localIP());
 // IPAddress localIP = WiFi.localIP(); 
 // ipAddress = String(localIP[0]) + "." + String(localIP[1]) + "." + String(localIP[2]) + "." + String(localIP[3]);
 
 
  server.begin();
  webSocket.begin(ip_host, port);
  webSocket.onEvent(webSocketEvent);
  //Serial.println("Done");
}

void loop()
{
  webSocket.loop();
  server.handleClient();
  if (mySerial.available()){
    receivedChar = mySerial.read();
    switch (receivedChar) {
      case '0':
        webSocket.sendTXT("3S9Ready");
        break;
      case '1':
        webSocket.sendTXT("3S9Starting");
        break;
      case '2':
        webSocket.sendTXT("3S9Coming home");
        break;
      case '3':
        webSocket.sendTXT("3S9Going to the RP");
        break;
      case '4':
        webSocket.sendTXT("3S9Coming Back");
        break;
      case '5':
        webSocket.sendTXT("3S9Shipping");
        break;
        break;  
      case '7':
        webSocket.sendTXT("3S9Sampling");
        break;
      case '8':
        webSocket.sendTXT("3S0Picking Up");
        break;
      case '9':
        webSocket.sendTXT("3S1Picking Up");
        break;
      case 'A':
        webSocket.sendTXT("3S2Putting Down");
        break;
      case 'B':
        webSocket.sendTXT("3S3Putting Down");
        break;
      case 'C':
        webSocket.sendTXT("3S4Putting Down");
        break;
      case 'D':
        webSocket.sendTXT("3S5Putting Down");
        break;
    }
  }
  if(t == 0x0FFFF0){
    ADC = analogRead(A0);
    Power = map(ADC, 558, 689, 0, 100);
    if(Power < 20) mySerial.write('1');
      String data = "3E"+(String)Power+'%';
    //  Serial.println(data);
      webSocket.sendTXT(data);
    t = 0;
  }
  t = t + 1;
}
