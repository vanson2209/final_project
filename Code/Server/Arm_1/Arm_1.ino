#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsClient.h> 
#include <Arduino.h>
#include <stdlib.h>

typedef enum {CHECK_PRESS = 0, CHECK_RELEASE, PREPARE_DATA} status_button;

WebSocketsClient webSocket;

const char* WIFI_SSID = "abc";
const char* WIFI_PASS = "tamsotam";

IPAddress ip_host;
const uint16_t port = 81; 

status_button button = CHECK_PRESS;

String status;
String agv_NUM;
int pos = -1;
String Data_to_Send;
uint16_t Quantity = 100;
String Quantity_S;

volatile uint8_t state = 0;

uint8_t goods_count = 0;

String data;
char TRA_num;
char TRA_in4;

ESP8266WebServer server(80);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  String payloadString = (const char *)payload;
  TRA_num = payloadString[0];
  TRA_in4 = payloadString[1];
  payloadString = payloadString.substring(2);
  if (TRA_num == '4'){ 
    if(TRA_in4 == 'D'){
      webSocket.sendTXT("4S9Operating");
      state = 1;
      agv_NUM = payloadString;
    }
    else if(TRA_in4 == 'Q'){
      Quantity_S = payloadString;
      Quantity =strtol(Quantity_S.c_str(), NULL, 10);
      Data_to_Send = "4Q" + String(Quantity);
      webSocket.sendTXT(Data_to_Send);
    }
  }
} 
uint8_t V_Check_Button(void) {
  if(digitalRead(D0) == LOW){
    pos = 0;
    return 1;
  }
  else  if(digitalRead(D5) == LOW){
    pos = 1;
    return 1;
  }
  else  if(digitalRead(D6) == LOW){
    pos = 2;
    return 1;
  }
  else  if(digitalRead(D7) == LOW){
    pos = 3;
    return 1;
  }
  else  if(digitalRead(D8) == LOW){
    pos = 4;
    return 1;
  }
  else return 0;
}
void V_Read_Button(void){
    switch(button){
      case CHECK_PRESS:
        if(V_Check_Button() == 1)
          button = CHECK_RELEASE;
        break;
      case CHECK_RELEASE:
        if(V_Check_Button() == 0)
            button = PREPARE_DATA;
        break;
      case PREPARE_DATA:
        if(pos == 4)
          state = 4;
        else
        {
          state = 2;
          goods_count++;
        }
        button = CHECK_PRESS;
        break;
    }
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
void V_Process_Data_Button(void){
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
      Data_to_Send = agv_NUM + "D" + data;
      webSocket.sendTXT(Data_to_Send);
      webSocket.sendTXT("4S9Resting");
      Quantity--;
      Data_to_Send = "4Q"+String(Quantity);
      webSocket.sendTXT(Data_to_Send);
      if(goods_count == 4)
        state = 4;
      else
        state = 1;
      break;
    }
    case 4:
      Data_to_Send = agv_NUM + "DDone";
      webSocket.sendTXT(Data_to_Send);
      goods_count = 0;
      state = 0;
      break;
  }
}
void  setup(){
  pinMode(D0, INPUT);
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  pinMode(D7, INPUT);
  pinMode(D8, INPUT);
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
    Serial.print("http://");
   Serial.println(WiFi.localIP());
  ip_host = WiFi.localIP();
  ip_host[3] = 10;
  Serial.println(ip_host);
  server.begin();
  webSocket.begin(ip_host, port);
  webSocket.onEvent(webSocketEvent);
  Serial.println("Done");
}

void loop()
{
  webSocket.loop();
  server.handleClient();
  V_Process_Data_Button();
}
