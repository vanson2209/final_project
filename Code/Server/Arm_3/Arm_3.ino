#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsClient.h> 
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <stdlib.h>
#include <Ticker.h>

Ticker systick;

typedef enum {CHECK_PRESS = 0, CHECK_RELEASE} status_button_t;
typedef enum{RED = 0, WAIT_BLUE, BLUE, WAIT_YELLOW, YELLOW, WAIT_RED} state_signal_t;
typedef enum{START = 0, PROCESS_SIGNAL, PRE_SEND_STRAIGHT, PRE_SEND_TURN} update_data_t;
typedef enum{START_RE = 0, BLUE_RE, YELLOW_RE, WAIT_DONE_BLUE, WAIT_DONE_YELLOW} state_process_request_t;

WebSocketsClient webSocket;

const char* WIFI_SSID = "abc";
const char* WIFI_PASS = "tamsotam";

IPAddress ip_host;
const uint16_t port = 81; 


String agv_NUM;

uint16_t Quantity = 0;
String Quantity_S;
String Data_To_Send;

volatile uint8_t state = 0;

char TRA_num;
char TRA_in4;

status_button_t status_button = CHECK_PRESS;
volatile state_signal_t  state_signal = RED;
volatile update_data_t update_data = START;
volatile state_process_request_t state_process_request = START_RE;

volatile uint8_t systick_count;

ESP8266WebServer server(80);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  String payloadString = (const char *)payload;
  Serial.println(payloadString);
  TRA_num = payloadString[0];
  if ((TRA_num == '6') || (TRA_num == 'Y')){    
    TRA_in4 = payloadString[1];
    payloadString = payloadString.substring(2);        //if (TRA_num == '6')
    if(TRA_in4 == 'D'){
      webSocket.sendTXT("6S9Operating");
      state = 1;
      agv_NUM = payloadString;
     // Serial.println("Destination");
    }
    else if(TRA_in4 == 'Q'){
      Quantity_S = payloadString;
      Quantity =strtol(Quantity_S.c_str(), NULL, 10);
      Data_To_Send = "6Q" + String(Quantity);
      Serial.println(Data_To_Send);
      webSocket.sendTXT(Data_To_Send);
    }
    else if(TRA_in4 == 'S'){
      if(payloadString == "BLUE")
        state_signal = BLUE;
      else if(payloadString == "YELLOW")
        state_signal = YELLOW;
      else if(payloadString == "RED")
        state_signal = RED;
    }
    else if(TRA_in4 == 'R'){
      if(payloadString == "BLUE")
        state_process_request = BLUE_RE;
      else if(payloadString == "YELLOW")
        state_process_request = YELLOW_RE;
    }
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
void V_Process_Data_Button(void){
  switch(state){
    case 0:
      break;
    case 1:
      V_Read_Button();
      break;
    case 2:
      Data_To_Send = agv_NUM + "DDone";
      webSocket.sendTXT(Data_To_Send);
      webSocket.sendTXT("6S9Resting");
      Quantity++;
      Data_To_Send = "6Q"+String(Quantity);
      webSocket.sendTXT(Data_To_Send);
      state = 0;
      break;
  }
}
void V_Process_Signal(void){
  switch(state_signal)
  {
    case RED:
      digitalWrite(D3, LOW);
      digitalWrite(D4, LOW);
      break;
    case WAIT_BLUE:
      break;
    case BLUE:
      digitalWrite(D3, LOW);
      digitalWrite(D4, HIGH);
      break;
    case WAIT_YELLOW:
      break;
    case YELLOW:
      digitalWrite(D3, HIGH);
      digitalWrite(D4, LOW);
      break;
    case WAIT_RED:
      break;
  }
}
void V_Update_Signal(void){
  switch(update_data){
    case START:
      break;
    case PROCESS_SIGNAL:
      if(digitalRead(D5) == HIGH){
        if(state_signal == YELLOW){
          Data_To_Send = "6U1";
          update_data = PRE_SEND_TURN;
          systick_count = 0;
        }
        else{
          Data_To_Send = "6R1";
          update_data = START;
        }
      }
      else if(digitalRead(D6) == HIGH){
        if(state_signal == BLUE){
          Data_To_Send = "6U0";
          update_data = PRE_SEND_STRAIGHT;
          systick_count = 0;
        }
        else{
          Data_To_Send = "6R0";
          update_data = START;
        }
      }
      webSocket.sendTXT(Data_To_Send);
      break;
    case PRE_SEND_TURN:
      if(systick_count >= 2){
        webSocket.sendTXT("6U4");
        update_data = START;
      }
      break;
    case PRE_SEND_STRAIGHT:
      if(systick_count >= 2){
        webSocket.sendTXT("6U3");
        update_data = START;
      }
      break;
  }
}
void V_Process_Signal_Pro(void){
  switch(state_process_request){
    case START_RE:
      break;
    case BLUE_RE:
      digitalWrite(D4, HIGH);
      systick_count = 0;
      state_process_request = WAIT_DONE_BLUE;
      break;
    case YELLOW_RE:
      digitalWrite(D3, HIGH);
      systick_count = 0;
      state_process_request = WAIT_DONE_YELLOW;
      break;
    case WAIT_DONE_BLUE:
      if(systick_count >= 1){
        digitalWrite(D4, LOW);
        state_process_request = START_RE;
      }
      break;
    case WAIT_DONE_YELLOW:
      if(systick_count >= 1){
        digitalWrite(D3, LOW);
        state_process_request = START_RE;
      }
      break;
  }

}
ICACHE_RAM_ATTR void IST_TURN (void){
  update_data = PROCESS_SIGNAL;
}
ICACHE_RAM_ATTR void IST_STRAIGHT (void){
  update_data = PROCESS_SIGNAL;
}
void Timer_Call_Back (void){
  systick_count ++;
}
void  setup(){
  pinMode(D2, INPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, INPUT_PULLDOWN_16);
  pinMode(D6, INPUT_PULLDOWN_16);
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
  attachInterrupt(D5, IST_TURN, RISING);
  attachInterrupt(D6, IST_STRAIGHT, RISING);
  systick.attach(1, Timer_Call_Back);
  digitalWrite(D3, LOW);
  digitalWrite(D4, LOW);
}

void loop()
{
  webSocket.loop();
  server.handleClient();
  V_Process_Data_Button();
  V_Process_Signal();
  V_Update_Signal();
  V_Process_Signal_Pro();
}
