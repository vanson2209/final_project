#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsClient.h> 
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <stdlib.h>

WebSocketsClient webSocket;

const char* WIFI_SSID = "abc";
const char* WIFI_PASS = "tamsotam";

const char* ip_host = "192.168.61.10"; 
const uint16_t port = 81; 

String status;
String agv_NUM;
int pos = -1;

uint16_t Quantity = 100;
String Quantity_S;

volatile uint8_t state = 0;
uint8_t state_pre;
uint8_t state_p;


String data;
uint8_t count = 0;

ESP8266WebServer server(80);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  String payloadString = (const char *)payload;
  Serial.println(payloadString);
  char TRA_num = payloadString[0];
  char TRA_in4 = payloadString[1];
  char data_rx = payloadString[2];
  payloadString = payloadString.substring(2);
  if (TRA_num == '5'){ 
    if(TRA_in4 == 'D'){
      webSocket.sendTXT("5S9Operating");
      state = 1;
      agv_NUM = payloadString;
      Serial.println("Destination");
    }
    else if(TRA_in4 == 'Q'){
      String Data_To_Send;
      Quantity_S = payloadString;
      Quantity =strtol(Quantity_S.c_str(), NULL, 10);
      Data_To_Send = "5Q" + String(Quantity);
      Serial.println(Data_To_Send);
      webSocket.sendTXT(Data_To_Send);
    }
   /* else if(TRA_in4 == 'R'){
      if(data_rx == 'S')
        if(state > 1){
          state_pre = state;
          state = 4;
        }
      else if(data_rx == 'N') state = 0;
      else if(data_rx == 'Y') state = 4;
    }*/
  }
} 
void V_Read_Button(void){
    if(digitalRead(D0) == LOW)
      pos = 0;
    else  if(digitalRead(D1) == LOW)
      pos = 1;
    else  if(digitalRead(D2) == LOW)
      pos = 2;
   else  if(digitalRead(D3) == LOW)
      pos = 3;
    if(pos > -1)
      state = 2;
    else
      state = 1;
}
void V_Data_Button(void){
  if(pos == 0)
    data = "RS1";
  else if(pos == 1)
    data = "RS2";
  else if(pos == 2)
    data = "RS3";
  else if(pos == 3)
    data = "RS4";
  pos = -1;
  state = 3;
}
void  setup(){
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D5, INPUT);
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
    Serial.print("http://");
   Serial.println(WiFi.localIP());

  server.begin();
  webSocket.begin(ip_host, port);
  webSocket.onEvent(webSocketEvent);
  Serial.println("Done");
}

void loop()
{
  webSocket.loop();
  server.handleClient();
  switch(state){
    case 0:
      break;
    case 1:
      V_Read_Button();
      break;
    case 2:
      V_Data_Button();
      break;
    case 3:
    {
     // webSocket.sendTXT("4RS");
      String Data_to_Send =agv_NUM + "D" + data;
      webSocket.sendTXT(Data_to_Send);
      webSocket.sendTXT("5S9Resting");
      Quantity--;
      Data_to_Send = "5Q"+String(Quantity);
      webSocket.sendTXT(Data_to_Send);
       state = 0;
      break;
    }
    /*case 4:
      switch(state_p){
        case 0:
          if(digitalRead(D5) == LOW){
            state_p = 1;
          }
          break;
        case 1:
          if(digitalRead(D5) == HIGH){
            state_p = 0;
            webSocket.sendTXT("4RC");
            state = state_pre;
          }
          break;
      }
      break;*/
  }
  


}
