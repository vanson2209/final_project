#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsClient.h> 
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Ticker.h>
Ticker systick;
volatile uint8_t systick_count, systick_count_pre;

#define RX2 13
#define TX2 15
SoftwareSerial mySerial(RX2, TX2);

WebSocketsClient webSocket;

const char* WIFI_SSID = "abc";
const char* WIFI_PASS = "tamsotam";

IPAddress ip_host; 
const uint16_t port = 81; 

String energy = "N/A";
String status = "Unknow";
uint8_t Power;

char receivedChar;

uint16_t ADC;

ESP8266WebServer server(80);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  
  String payloadString = (const char *)payload;
  Serial.println(payloadString);
  char AGV_num = payloadString[0];
  char AGV_in4 = payloadString[1];
  payloadString = payloadString.substring(2);
  if (AGV_num == '1'){
    if(AGV_in4 == 'P'){
      if (payloadString == "ON") {
        if((systick_count - systick_count_pre) > 2){
          mySerial.write('0');
        }
      }
      else if (payloadString == "OFF"){
        if((systick_count - systick_count_pre) > 2){
          mySerial.write('1');
        }
      }
      systick_count_pre = systick_count; 
    }
    else if(AGV_in4 == 'D'){
      if(payloadString == "Done"){
        mySerial.write('4');
      }
      else if (payloadString == "RS1") {
        mySerial.write('5');
 //         Serial.println("Destination is A1");
      }
      else if (payloadString == "RS2"){
        mySerial.write('6');
          //Serial.println("Destination is A2");
      }
      else if (payloadString == "RS3"){
        mySerial.write('7');
          //Serial.println("Destination is B1");
      }
      else if (payloadString == "RS4"){
        mySerial.write('8');
         // Serial.println("Destination is B2");
      }
    }
    else if(AGV_in4 == 'A'){
      if (payloadString == "On") {
          mySerial.write('3');
          //Serial.println("AGV1 ON");
      }
      else if (payloadString == "Of"){
          mySerial.write('2');
          //Serial.println("AGV1 OFF");
      }
    }
    else if(AGV_in4 == 'C'){
      if (payloadString == "S") {
          mySerial.write('D');
          //Serial.println("AGV1 ON");
      }
      else if (payloadString == "U"){
          mySerial.write('9');
          //Serial.println("AGV1 OFF");
      }
      else if (payloadString == "D"){
          mySerial.write('A');
          //Serial.println("AGV1 OFF");
      }
      else if (payloadString == "L"){
          mySerial.write('B');
          //Serial.println("AGV1 OFF");
      }
      else if (payloadString == "R"){
          mySerial.write('C');
          //Serial.println("AGV1 OFF");
      }
    }
  }
} 
void Timer_Call_Back(void)
{
  systick_count++;
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
  // Serial.println(WiFi.localIP());
 // IPAddress localIP = WiFi.localIP(); 
 // ipAddress = String(localIP[0]) + "." + String(localIP[1]) + "." + String(localIP[2]) + "." + String(localIP[3]);
 
  ip_host = WiFi.localIP();
  ip_host[3] = 10;
 // Serial.println(ip_host);
  server.begin();
  webSocket.begin(ip_host, port);
  webSocket.onEvent(webSocketEvent);
  //Serial.println("Done");
  systick_count = 0;
  systick.attach(1, Timer_Call_Back);
}

void loop()
{
  webSocket.loop();
  server.handleClient();
  if (mySerial.available()){
    receivedChar = mySerial.read();
    switch (receivedChar) {
      case '0':
        webSocket.sendTXT("1S9Ready");
        break;
      case '1':
        webSocket.sendTXT("1S9Sampling");
        break;
      case '2':
        webSocket.sendTXT("1S9Going to the RP");
        break;
      case '3':
        webSocket.sendTXT("1S0Picking Up");
        break;
      case '4':
        webSocket.sendTXT("1S1Picking Up");
        break;
      case '5':
        webSocket.sendTXT("1S2Putting Down");
        break; 
      case '6':
        webSocket.sendTXT("1S3Putting Down");
        break;
      case '7':
        webSocket.sendTXT("1S4Putting Down");
        break;
      case '8':
        webSocket.sendTXT("1S5Putting Down");
        break;
      case '9':
        webSocket.sendTXT("1S9Coming Back");
        break;
      case 'A':
        webSocket.sendTXT("1S9Coming Home");
        break;
      case 'D':
        webSocket.sendTXT("1S9Shipping");
        break;
    }
  }
  if((systick_count - systick_count_pre) > 10){
    ADC = analogRead(A0);
    Power = map(ADC, 824, 994, 0, 100);
    if(Power < 20) mySerial.write('1');
      String data = "1E"+(String)Power+'%';
    //  Serial.println(data);
      webSocket.sendTXT(data);
    systick_count_pre = systick_count;
  }
}
