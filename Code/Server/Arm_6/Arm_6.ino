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


String agv_NUM;
int tmp = 0;

uint16_t Quantity = 0;
String Quantity_S;

volatile uint8_t state = 0;
uint8_t state_p = 0;
uint8_t status = 0;


uint8_t data = 0;
uint8_t count = 0;

ESP8266WebServer server(80);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  String payloadString = (const char *)payload;
  Serial.println(payloadString);
  char TRA_num = payloadString[0];
  char TRA_in4 = payloadString[1];
  char rx_data = payloadString[2];
  payloadString = payloadString.substring(2);
  if (TRA_num == '9'){ 
    if(TRA_in4 == 'D'){
      webSocket.sendTXT("9S9Operating");
      state = 1;
      agv_NUM = payloadString;
     // Serial.println("Destination");
    }
    else if(TRA_in4 == 'Q'){
      String Data_To_Send;
      Quantity_S = payloadString;
      Quantity =strtol(Quantity_S.c_str(), NULL, 10);
      Data_To_Send = "9Q" + String(Quantity);
      Serial.println(Data_To_Send);
      webSocket.sendTXT(Data_To_Send);
    }
/*    else if(TRA_in4 == 'R'){
       status = 1;

    }*/
  }
} 
void V_Read_Button(void){
  if(digitalRead(D4) == LOW)
      tmp = 1;
  if(tmp == 1)
    state = 2;
  else
    state = 1;
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
      data = 1;
      state = 3;
      break;
    case 3:
      tmp = 0;
      String Data_to_Send = agv_NUM + "DTS2";
      webSocket.sendTXT(Data_to_Send);
      webSocket.sendTXT("9S9Resting");
      Quantity++;
      Data_to_Send = "9Q"+String(Quantity);
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
