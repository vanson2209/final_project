#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsClient.h> 
#include <Arduino.h>
#include <stdlib.h>

typedef enum {CHECK_PRESS = 0, CHECK_RELEASE, PREPARE_DATA} status_button;
typedef enum {WAIT_START = 0, PRE_PROCESS, READ_BUTTON, PREPARE_DATA_TO_SEND, SEND_DATA, SEND_DONE, WAIT_DONE} state_feedback_data_t;
typedef enum {WAIT_OUT, WAIT_DATA, UPDATE_DATA} state_data_t;

WebSocketsClient webSocket;

const char* WIFI_SSID = "abc";
const char* WIFI_PASS = "tamsotam";

IPAddress ip_host;
const uint16_t port = 81; 

status_button button = CHECK_PRESS;
volatile state_feedback_data_t state_feedback_data;
volatile state_data_t state_data, state_data_pre;

String status;
String agv_NUM;
String agv_NUM_tmp;
int pos = -1;
String Data_to_Send;
uint16_t Quantity = 100;
String Quantity_S;



uint8_t goods_count = 0;
volatile uint8_t agv_quantity = 0;

String data;
char TRA_num;
char TRA_in4;

ESP8266WebServer server(80);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  String payloadString = (const char *)payload;
  TRA_num = payloadString[0];
  TRA_in4 = payloadString[1];
  payloadString = payloadString.substring(2);
  if (TRA_num == '5'){ 
    if(TRA_in4 == 'G'){
      agv_quantity++;
      webSocket.sendTXT("5S9Operating");
      state_feedback_data = PRE_PROCESS;
      if(agv_quantity > 1)
        agv_NUM_tmp = payloadString;
      else
        agv_NUM = payloadString;
    }
    else if(TRA_in4 == 'Q'){
      Quantity_S = payloadString;
      Quantity =strtol(Quantity_S.c_str(), NULL, 10);
      Data_to_Send = "5Q" + String(Quantity);
      webSocket.sendTXT(Data_to_Send);
    }
    else if(TRA_in4 == 'U'){
      state_data_pre = state_data;
      state_data = UPDATE_DATA;
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
          state_feedback_data = SEND_DONE;
        else
        {
          state_feedback_data = PREPARE_DATA_TO_SEND;
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
  state_feedback_data = SEND_DATA;
}
void V_Process_Data_Button(void){
  switch(state_feedback_data){
    case WAIT_START:
      break;
    case PRE_PROCESS:
      if(Quantity == 0){
        webSocket.sendTXT("YG2OutOf");
        state_feedback_data = WAIT_START;
        state_data = WAIT_DATA;
      }
      else
        state_feedback_data = READ_BUTTON;
      break;
    case READ_BUTTON:
      V_Read_Button();
      break;
    case PREPARE_DATA_TO_SEND:
      V_Data_Button();
      break;
    case SEND_DATA:
      Data_to_Send = agv_NUM + "G" + data;
      webSocket.sendTXT(Data_to_Send);
      Quantity--;
      Data_to_Send = "5Q"+String(Quantity);
      webSocket.sendTXT(Data_to_Send);
      if(goods_count == 4)
        state_feedback_data = SEND_DONE;
      else
        state_feedback_data = READ_BUTTON;
      if(Quantity == 0){
        webSocket.sendTXT("YG2OutOf");
        state_feedback_data = SEND_DONE;
        state_data = WAIT_DATA;
      }
      break;
    case SEND_DONE:
      Data_to_Send = agv_NUM + "GDone";
      webSocket.sendTXT(Data_to_Send);
      goods_count = 0;
      agv_quantity--;
      state_feedback_data = WAIT_DONE;
      break;
    case WAIT_DONE:
      if(agv_quantity == 0){
        webSocket.sendTXT("5S9Resting");
        state_feedback_data = WAIT_START;
      }
      else {
        agv_NUM = agv_NUM_tmp;
        state_feedback_data = READ_BUTTON;
      }
      break;
  }
}
void V_Process_Data_Quantity(void){
  switch(state_data)
  {
    case WAIT_OUT:
      break;
    case WAIT_DATA:
      if(Quantity > 0){
        webSocket.sendTXT("YG2FullOf");
        state_data = WAIT_OUT;
      }
      break;
    case UPDATE_DATA:
      if(Quantity > 0)
        webSocket.sendTXT("YG2FullOf");
      else
       webSocket.sendTXT("YGOutOf");
      state_data = state_data_pre;
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
  //  Serial.print("http://");
  // Serial.println(WiFi.localIP());
  ip_host = WiFi.localIP();
  ip_host[3] = 10;
  //Serial.println(ip_host);
  server.begin();
  webSocket.begin(ip_host, port);
  webSocket.onEvent(webSocketEvent);
  //Serial.println("Done");
}

void loop()
{
  webSocket.loop();
  server.handleClient();
  V_Process_Data_Button();
  V_Process_Data_Quantity();
}
