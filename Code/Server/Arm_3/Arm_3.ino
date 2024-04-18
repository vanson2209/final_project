#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsClient.h> 
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <stdlib.h>

typedef enum {CHECK_PRESS = 0, CHECK_RELEASE} status_button_t;

WebSocketsClient webSocket;

const char* WIFI_SSID = "abc";
const char* WIFI_PASS = "tamsotam";

IPAddress ip_host;
const uint16_t port = 81; 


String agv_NUM;

uint16_t Quantity = 0;
String Quantity_S;

volatile uint8_t state = 0;
uint8_t state_p = 0;
uint8_t status = 0;


uint8_t data = 0;
uint8_t count = 0;

status_button_t status_button;

ESP8266WebServer server(80);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  String payloadString = (const char *)payload;
  Serial.println(payloadString);
  char TRA_num = payloadString[0];
  char TRA_in4 = payloadString[1];
  char rx_data = payloadString[2];
  payloadString = payloadString.substring(2);
  if ((TRA_num >= '6') && (TRA_num <= '9')){            //if (TRA_num == '6')
    if(TRA_in4 == 'D'){
      webSocket.sendTXT("6S9Operating");
      state = 1;
      agv_NUM = payloadString;
     // Serial.println("Destination");
    }
    else if(TRA_in4 == 'Q'){
      String Data_To_Send;
      Quantity_S = payloadString;
      Quantity =strtol(Quantity_S.c_str(), NULL, 10);
      Data_To_Send = "6Q" + String(Quantity);
      Serial.println(Data_To_Send);
      webSocket.sendTXT(Data_To_Send);
    }
/*    else if(TRA_in4 == 'R'){
       status = 1;

    }*/
  }
} 
void V_Read_Button(void){
  switch(status_button){
    case CHECK_PRESS:
      if(digitalRead(D2) == LOW)
        status_button = CHECK_RELEASE;
      break;
    case CHECK_RELEASE:
      if(digitalRead(D2) == HIGH){
        status_button = CHECK_PRESS;
        state = 2;
      }
      break;
  }
}
void  setup(){
  pinMode(D2, INPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);
  pinMode(D9, OUTPUT);
  pinMode(D10, OUTPUT);
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
  switch(state){
    case 0:
      break;
    case 1:
      V_Read_Button();
      break;
    case 2:
      String Data_to_Send = agv_NUM + "DDone";
      webSocket.sendTXT(Data_to_Send);
      webSocket.sendTXT("6S9Resting");
      Quantity++;
      Data_to_Send = "6Q"+String(Quantity);
      webSocket.sendTXT(Data_to_Send);
      state = 0;
      break;
  }
 /* switch(status){
    case 0:
      break;
    case 1:
      if(state == 3)
        state = 2;
      break;
  }
  switch(state_p){
    case 0:
      if(digitalRead(D6) == LOW){
        status = 0;
        state_p = 1;
      }
      break;
    case 1:
      if(digitalRead(D6) == HIGH){
        state_p = 0;
      }
      break;
  }*/



}
