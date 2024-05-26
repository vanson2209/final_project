#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsClient.h> 
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <stdlib.h>
#include <Ticker.h>

//Ticker systick_re;
Ticker systick;

//volatile uint8_t systick_re_count;
volatile uint32_t systick_count;
void Timer_Call_Back(void);


typedef enum {Idle = 0, Wait_Expire} status_time_t;
typedef enum {WAIT_START = 0, CHECK_TIME, SEND_DONE, WAIT_DONE} state_feedback_data_t;
//typedef enum{RED = 0, WAIT_BLUE, BLUE, WAIT_YELLOW, YELLOW, WAIT_RED} state_signal_t;
//typedef enum{WAIT = 0, START_PRO, OPERATE, STOP} state_process_signal_t;
//typedef enum{START = 0, PROCESS_SIGNAL, PRE_SEND_STRAIGHT, PRE_SEND_TURN} update_data_t;
//typedef enum{START_RE = 0, BLUE_RE, YELLOW_RE, WAIT_DONE_BLUE, WAIT_DONE_YELLOW} state_process_request_t;

WebSocketsClient webSocket;

const char* WIFI_SSID = "abc";
const char* WIFI_PASS = "tamsotam";

IPAddress ip_host;
const uint16_t port = 81; 


String agv_NUM;
String agv_NUM_tmp;
volatile uint8_t agv_quantity;
uint16_t Quantity = 0;
String Quantity_S;
String Data_To_Send;


char TRA_num;
char TRA_in4;

status_time_t status_time = Idle;
state_feedback_data_t state_feedback_data = WAIT_START;
/*volatile state_signal_t  state_signal;
volatile state_process_signal_t state_process_signal;
volatile update_data_t update_data = START;
volatile state_process_request_t state_process_request = START_RE;*/



ESP8266WebServer server(80);

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  String payloadString = (const char *)payload;
 // Serial.println(payloadString);
  TRA_num = payloadString[0];
  if (TRA_num == '7'){// || (TRA_num == 'X')){    
    TRA_in4 = payloadString[1];
    payloadString = payloadString.substring(2);        //if (TRA_num == '7')
    if(TRA_in4 == 'G'){
      agv_quantity++;
      webSocket.sendTXT("7S9Operating");
      state_feedback_data = CHECK_TIME;
      if(agv_quantity > 1)
        agv_NUM_tmp = payloadString;
      else
        agv_NUM = payloadString;
     // Serial.println("Destination");
    }
    else if(TRA_in4 == 'Q'){
      Quantity_S = payloadString;
      Quantity =strtol(Quantity_S.c_str(), NULL, 10);
      Data_To_Send = "7Q" + String(Quantity);
      //Serial.println(Data_To_Send);
      webSocket.sendTXT(Data_To_Send);
    }
/*    else if(TRA_in4 == 'S'){
      if(payloadString == "START")
        state_process_signal = START_PRO;
      else if(payloadString == "STOP")
        state_process_signal = STOP;
    }
    else if(TRA_in4 == 'R'){
      if(payloadString == "BLUE")
        state_process_request = BLUE_RE;
      else if(payloadString == "YELLOW")
        state_process_request = YELLOW_RE;
    }*/
  }
} 
void V_Check_Time(void){
  switch(status_time){
    case Idle:
      systick.attach_ms(100, Timer_Call_Back);
      systick_count = 50;
      status_time = Wait_Expire;
      break;
    case Wait_Expire:
      if(systick_count == 0){
        state_feedback_data = SEND_DONE;
        status_time = Idle;
      }
      break;
  }
}
void V_Process_Data_Button(void){
  switch(state_feedback_data){
    case WAIT_START:
      break;
    case CHECK_TIME:
      V_Check_Time();
      break;
    case SEND_DONE:
      Data_To_Send = agv_NUM + "GDone";
      webSocket.sendTXT(Data_To_Send);
      Quantity++;
      Data_To_Send = "7Q"+String(Quantity);
      webSocket.sendTXT(Data_To_Send);
      agv_quantity--;
      state_feedback_data = WAIT_DONE;
      break;
    case  WAIT_DONE:
      if(agv_quantity == 0){
        systick.detach();
        webSocket.sendTXT("7S9Resting");
        state_feedback_data = WAIT_START;
      }
      else {
        agv_NUM = agv_NUM_tmp;
        state_feedback_data = CHECK_TIME;
      }
      break;
  }
}
/*void V_Process_Signal(void){
  switch(state_process_signal){
    case WAIT:
      break;
    case START_PRO:
      systick_count = 0;
      systick.attach_ms(1, Timer_Call_Back);
      state_signal = RED;
      state_process_signal = OPERATE;
      break;
    case OPERATE:
      switch(state_signal){
        case RED:
          digitalWrite(D3, LOW);
          digitalWrite(D4, LOW);
          state_signal = WAIT_BLUE;
          break;
        case WAIT_BLUE:
          if(systick_count >= 24000)
            state_signal = BLUE;
          break;
        case BLUE:
          digitalWrite(D3, LOW);
          digitalWrite(D4, HIGH);
          state_signal = WAIT_YELLOW;
          break;
        case WAIT_YELLOW:
          if(systick_count >= 32000)
            state_signal = YELLOW;
          break;
        case YELLOW:
          digitalWrite(D3, HIGH);
          digitalWrite(D4, LOW);
          state_signal = WAIT_RED;
          break;
        case WAIT_RED:
          if(systick_count >= 40000){
            systick_count = 0;
            state_signal = RED;
          }
          break;
      }
      break;
    case STOP:
      digitalWrite(D3, HIGH);
      digitalWrite(D4, HIGH);
      systick.detach();
      state_process_signal = WAIT;
      break;
  }
}
void Timer_Call_Back(void)
{
  systick_count ++;
}
void V_Update_Signal(void){
  switch(update_data){
    case START:
      break;
    case PROCESS_SIGNAL:
      if(digitalRead(D5) == HIGH){
        if(state_signal == YELLOW){
          Data_To_Send = "7U1";
          update_data = PRE_SEND_TURN;
          systick_count = 0;
        }
        else{
          Data_To_Send = "7R1";
          update_data = START;
        }
      }
      else if(digitalRead(D6) == HIGH){
        if(state_signal == BLUE){
          Data_To_Send = "7U0";
          update_data = PRE_SEND_STRAIGHT;
          systick_count = 0;
        }
        else{
          Data_To_Send = "7R0";
          update_data = START;
        }
      }
      webSocket.sendTXT(Data_To_Send);
      break;
    case PRE_SEND_TURN:
      if(systick_count >= 2){
        webSocket.sendTXT("7U4");
        update_data = START;
      }
      break;
    case PRE_SEND_STRAIGHT:
      if(systick_count >= 2){
        webSocket.sendTXT("7U3");
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

}*/
/*ICACHE_RAM_ATTR void IST_TURN (void){
  update_data = PROCESS_SIGNAL;
}
ICACHE_RAM_ATTR void IST_STRAIGHT (void){
  update_data = PROCESS_SIGNAL;
}
void Timer_re_Call_Back (void){
  systick_count ++;
}*/
void Timer_Call_Back(void)
{
  if(systick_count != 0)
    systick_count --;
}
void  setup(){
  pinMode(D2, INPUT);
// pinMode(D3, OUTPUT);
//  pinMode(D4, OUTPUT);
//  pinMode(D5, INPUT_PULLDOWN_16);
//  pinMode(D6, INPUT_PULLDOWN_16);
  Serial.begin(115200);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
   // Serial.print("http://");
   //Serial.println(WiFi.localIP());
  ip_host = WiFi.localIP();
  ip_host[3] = 10;
  //Serial.println(ip_host);
  server.begin();
  webSocket.begin(ip_host, port);
  webSocket.onEvent(webSocketEvent);
  //Serial.println("Done");
//  attachInterrupt(D5, IST_TURN, RISING);
//  attachInterrupt(D6, IST_STRAIGHT, RISING);
//  systick_re.attach(1, Timer_re_Call_Back);
//  digitalWrite(D3, HIGH);
//  digitalWrite(D4, HIGH);
}

void loop()
{
  webSocket.loop();
  server.handleClient();
  V_Process_Data_Button();
//  V_Process_Signal();
//  V_Update_Signal();
//  V_Process_Signal_Pro();
}
