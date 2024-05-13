#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsClient.h> 
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Ticker.h>
Ticker systick;
volatile uint8_t systick_count, systick_count_pre;

//#define RX2 13
//#define TX2 15
//SoftwareSerial mySerial(RX2, TX2);

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
 // Serial.println(payloadString);
  char AGV_num = payloadString[0];
  char AGV_in4 = payloadString[1];
  payloadString = payloadString.substring(2);
  if ((AGV_num == '1') || (AGV_num == 'Y')){
    if(AGV_in4 == 'P'){
      if (payloadString == "ON") {
        if((systick_count - systick_count_pre) > 2){
          Serial.write('0');
        }
      }
      else if (payloadString == "OFF"){
        if((systick_count - systick_count_pre) > 2){
          Serial.write('1');
        }
      }
      systick_count_pre = 0; 
      systick_count = 0;
    }
    else if(AGV_in4 == 'G'){
      if(payloadString == "Done"){
        Serial.write('4');
      }
      else if(payloadString == "1OutOf"){
        Serial.write('E');
      }
      else if(payloadString == "2OutOf"){
        Serial.write('F');
      }
      else if(payloadString == "1FullOf"){
        Serial.write('G');
      }
      else if(payloadString == "2FullOf"){
        Serial.write('H');
      }
      else if (payloadString == "RS1") {
        Serial.write('5');
      }
      else if (payloadString == "RS2"){
        Serial.write('6');
      }
      else if (payloadString == "RS3"){
        Serial.write('7');
      }
      else if (payloadString == "RS4"){
        Serial.write('8');
      }
    }
    else if(AGV_in4 == 'A'){
      if (payloadString == "On") {
          Serial.write('3');
      }
      else if (payloadString == "Of"){
          Serial.write('2');
      }
    }
    else if(AGV_in4 == 'C'){
      if (payloadString == "S") {
          Serial.write('D');
      }
      else if (payloadString == "U"){
          Serial.write('9');
      }
      else if (payloadString == "D"){
          Serial.write('A');
      }
      else if (payloadString == "L"){
          Serial.write('B');
      }
      else if (payloadString == "R"){
          Serial.write('C');
      }
    }
  }
} 
void Timer_Call_Back(void)
{
  systick_count++;
}
void  setup(){
 // mySerial.begin(115200);
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  ip_host = WiFi.localIP();
  ip_host[3] = 10;
  server.begin();
  webSocket.begin(ip_host, port);
  webSocket.onEvent(webSocketEvent);
  //Serial.println("Done");
  systick_count = 0;
  systick_count_pre = 0;
  systick.attach(1, Timer_Call_Back);
}

void loop()
{
  webSocket.loop();
  server.handleClient();
  if (Serial.available()){
    receivedChar = Serial.read();
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
      case 'E':
        webSocket.sendTXT("1PON");
        break;
      case 'F':
        webSocket.sendTXT("1POFF");
        break;
      case 'G':
        webSocket.sendTXT("1DPS1");
        break;
      case 'H':
        webSocket.sendTXT("1DTS1");
        break;
      case 'I':
        webSocket.sendTXT("1DTS2");
        break;
      case 'K':
        webSocket.sendTXT("1DRS1");
        break;
      case 'L':
        webSocket.sendTXT("1DRS2");
        break;
      case 'M':
        webSocket.sendTXT("1DRS3");
        break;
      case 'N':
        webSocket.sendTXT("1DRS4");
        break;
    }
  }
  if((systick_count - systick_count_pre) > 20){
    ADC = analogRead(A0);
    Power = map(ADC, 824, 994, 0, 100);
    if(Power < 20) Serial.write('1');
      String data = "1E"+(String)Power+'%';
      webSocket.sendTXT(data);
    systick_count_pre = systick_count;
  }
}
