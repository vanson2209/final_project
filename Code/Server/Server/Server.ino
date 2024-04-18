#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>

const char* WIFI_SSID = "abc";
const char* WIFI_PASS = "tamsotam";
struct AGVInfo {
  String status;
  String energy;
  String destination;
  String goods_list;
  String quantity;
};
uint8_t state = 0;

AGVInfo agv[9];

uint8_t check_p[3] = { 0, 0, 0 };


IPAddress ip;     //(192, 168, 61, 10);
IPAddress gateway; //(192, 168, 61, 1);
IPAddress subnet(255, 255, 255, 0);

WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266WebServer webServer(80);

const char MainPage[] PROGMEM = R"=====(
<!DOCTYPE html> 
<html>
 <head> 
     <title>HOME PAGE</title> 
     <style> 
        body {text-align:center;}
        h1 {color:#003399;}
        h2 {color:#009994;}
        table{width: 100%;}
        table, td, th{border: 1px solid #333; border-collapse: collapse;}
        table, td{height: 70px;}
        .thead{text-align: left; width: 120px}
        :root{--transition-effect: 0.25s cubic-bezier(0.25, -0.59, 0.82, 1.66);}
        .sw_toggle{width: 90px; height: 50px; appearance: none; background: #83d8ff; border-radius: 26px; position: relative; cursor: pointer;
                  box-shadow: inset 0px 0px 16px rgba(0, 0, 0, 0.3); transition: var(--transition-effect);}
        .sw_toggle::before{content: ""; width: 44px; height: 44px; position: absolute; top: 3px; left: 3px; background: #efefef; border-radius: 50%;
                           box-shadow: 0px 0px 6px rgba(0, 0, 0, 0.3); transition: var(--transition-effect);}
        .sw_toggle:checked{background: #749dd6;}
        .sw_toggle:checked:before{left: 40px;}
        .bt_write {height:20px; width:70px; margin:10px 0;background-color:#00FF00;border-radius:5px;}
        .bt_ON{height: 30px; width: 60px; margin: 10px; background-color: #09ff00; border-radius: 10px;}
        .bt_OFF{height: 30px; width: 60px; margin: 10px; background-color: #ff0000; border-radius: 10px;}
        .bt_stop{height: 38px; width: 38px; background-color: #ff0000; border-radius: 50%;text-align: center}
        .bt_direct{height: 30px; width: 60px; margin: 10px; background-color: #09ff00; border-radius: 10px;}
     </style>
     <meta name="viewport" content="width=device-width,user-scalable=0" charset="UTF-8">
 </head>
 <body> 
      <h1>ESP8266 Web Server</h1> 
      <h2>AGV Information</h2>
      <table>
          <thead>
              <th class="thead"></th>
              <th>AGV 1</th>
              <th>AGV 2</th>
          </thead>
          <tbody>
            <tr>
              <td class="thead"><b>ON/OFF</b></td>
              <td><input type="checkbox" class="sw_toggle" id="Power_AGV1"></td>
              <td><input type="checkbox" class="sw_toggle" id="Power_AGV2"></td> 
            </tr>              
            <tr>
              <td class="thead"><b>Status</b></td>
              <td><b><span id="SAGV1"></span></b></td>
              <td><b><span id="SAGV2"></span></b></td> 
            </tr>
            <tr>
              <td class="thead"><b>Energy</b></td>
              <td><b><span id="EAGV1"></span></b></td>
              <td><b><span id="EAGV2"></span></b></td>  
            </tr>   
            <tr>
              <td class="thead"><b>Goods List</b></td>
              <td><b><span id="GAGV1"></span></b></td>
              <td><b><span id="GAGV2"></span></b></td> 
            </tr>             
            <tr>
              <td class="thead"><b>Destination</b></td>
              <td><b><span id="DAGV1"></span></b></td>
              <td><b><span id="DAGV2"></span></b></td> 
            </tr>
            <tr>
              <td class="thead"><b>Automation</b></td>
              <td>
                <button class="bt_ON" onclick="request('OnAAGV1')">ON</button>
                <button class="bt_OFF" onclick="request('OfAAGV1')">OFF</button>
              </td>
              <td>
                <button class="bt_ON" onclick="request('OnAAGV2')">ON</button>
                <button class="bt_OFF" onclick="request('OfAAGV2')">OFF</button>
              </td>
            </tr>
            <tr>
              <td class="thead"><b>Control</b></td>
              <td>
                <div><button class="bt_direct" onclick="request('UpAGV1')">^</button></div>
                <div>
                  <button class="bt_direct" onclick="request('LeftAGV1')"><</button>
                  <button class="bt_stop" onclick="request('StopAGV1')">X</button>
                  <button class="bt_direct" onclick="request('RightAGV1')">></button>
                </div>
                <div><button class="bt_direct" onclick="request('DownAGV1')">v</button></div>
              </td>
              <td>
                <div><button class="bt_direct" onclick="request('UpAGV2')">^</button></div>
                <div>
                  <button class="bt_direct" onclick="request('LeftAGV2')"><</button>
                  <button class="bt_stop" onclick="request('StopAGV2')">X</button>
                  <button class="bt_direct" onclick="request('RightAGV2')">></button>
                </div>
                <div><button class="bt_direct" onclick="request('DownAGV2')">v</button></div>
              </td>
            </tr>
          </tbody>
      </table> 
      <h2>TRA Information</h2> 
      <table>
        <thead>
            <th class="thead"></th>
            <th>TRA 1</th>
            <th>TRA 2</th>
        </thead>
        <tbody>             
              <tr>
                <td class="thead"><b>Status</b></td>
                <td><b><span id="STRA1"></span></b></td>
                <td><b><span id="STRA2"></span></b></td>
              </tr>
              <tr>
                <td class="thead"><b>Quantity</b></td>
                <td><b><span id="QTRA1"></span></b></td>
                <td><b><span id="QTRA2"></span></b></td>
              </tr>
              <tr>
                <td class="thead"><b>Type</b></td>
                <td><input id="DTRA1"/> <button class="bt_write" onclick="V_WriteTRA('1')">WRITE</button></td>
                <td><input id="DTRA2"/> <button class="bt_write" onclick="V_WriteTRA('2')">WRITE</button></td>
              </tr>
        </tbody>
      </table>
      <h2>RRA Information</h2> 
      <table>
        <thead>
            <th class="thead"></th>
            <th>RRA 1</th>
            <th>RRA 2</th>
            <th>RRA 3</th>
            <th>RRA 4</th>
        </thead>
        <tbody>              
              <tr>
                <td class="thead"><b>Status</b></td>
                <td><b><span id="SRRA1"></span></b></td>
                <td><b><span id="SRRA2"></span></b></td>
                <td><b><span id="SRRA3"></span></b></td>
                <td><b><span id="SRRA4"></span></b></td>
              </tr>
              <tr>
                <td class="thead"><b>Quantity</b></td>
                <td><b><span id="QRRA1"></span></b></td>
                <td><b><span id="QRRA2"></span></b></td>
                <td><b><span id="QRRA3"></span></b></td>
                <td><b><span id="QRRA4"></span></b></td>
              </tr>
              <tr>
                <td class="thead"><b>Type</b></td>
                <td><input id="DRRA1"/> <button class="bt_write" onclick="V_WriteRRA('1')">WRITE</button></td>
                <td><input id="DRRA2"/> <button class="bt_write" onclick="V_WriteRRA('2')">WRITE</button></td>
                <td><input id="DRRA3"/> <button class="bt_write" onclick="V_WriteRRA('3')">WRITE</button></td>
                <td><input id="DRRA4"/> <button class="bt_write" onclick="V_WriteRRA('4')">WRITE</button></td>
              </tr>
        </tbody>
      </table>
      <div id="reponsetext"></div>
      <script>
          //AGV ON/OFF
          function handleAGVChange(agvNUM){
            var xhttp = new XMLHttpRequest();
            var powerCheckbox = document.getElementById("Power_AGV" + agvNUM);
            var prevChecked = powerCheckbox.checked;
            powerCheckbox.addEventListener("change", function(i){
              var stt;
              if (prevChecked && !this.checked) {
                stt = "offAGV" + agvNUM; // Gửi "offAGV" khi tắt từ bật
              } else if (!prevChecked && this.checked) {
                stt = "onAGV" + agvNUM; // Gửi "onAGV" khi bật từ tắt
              }
              xhttp.open ("GET","/"+stt, true); 
              xhttp.send(); 
              prevChecked = this.checked;
            })
          }
          function powersAGV(){     
            for (var i = 1; i <= 2; i++){
               handleAGVChange(i);
            }
          }
          function getdata(url, element, NUM){
            var xhttp = new XMLHttpRequest();
            xhttp.onreadystatechange = function() {
               if (this.readyState === 4 && this.status === 200) {
                document.getElementById(element + NUM).innerText = this.responseText;
               }
            };
            xhttp.open("GET",url + NUM, true); 
            xhttp.send(); 
          }
          function getstatusAGV(agvNUM){
            getdata("statusAGV", "SAGV", agvNUM);
          }
          function getstatusTRA(traNUM){
            getdata("statusTRA", "STRA", traNUM);
          }
          function getstatusRRA(rraNUM){
            getdata("statusRRA", "SRRA", rraNUM);
          }
          function getenergyAGV(agvNUM){
            getdata("energyAGV", "EAGV", agvNUM);
          }
          function getdestinationAGV(agvNUM){
            getdata("DeAGV", "DAGV", agvNUM);
          }
          function getgoodslistAGV(agvNUM){
            getdata("GlAGV", "GAGV", agvNUM); 
          }
          function getquantityTRA(traNUM){
            getdata("QuTRA", "QTRA", traNUM); 
          }
          function getquantityRRA(rraNUM){
            getdata("QuRRA", "QRRA", rraNUM);
          }
          function updatestatusAGV(){
            for(var i = 1; i < 3; i++){
              getstatusAGV(i);
            }
          }
          function updatestatusTRA(){
            for(var i = 1; i < 3; i++){
              getstatusTRA(i);
            }
          }
          function updatestatusRRA(){
            for(var i = 1; i < 5; i++){
              getstatusRRA(i);
            }
          }
          function updateenergyAGV(){
            for(var i = 1; i < 3; i++){
              getenergyAGV(i);
            }
          } 
          function updatedestinationAGV(){
            for(var i = 1; i < 3; i++){
              getdestinationAGV(i);
            }
          } 
          function updategoodslistAGV(){
            for(var i = 1; i < 3; i++){
              getgoodslistAGV(i);
            }
          } 
          function updatequantityTRA(){
            for(var i = 1; i < 3; i++){
              getquantityTRA(i);
            }
          }
          function updatequantityRRA(){
            for(var i = 1; i < 5; i++){
              getquantityRRA(i);
            }
          }
          function request(url){
            var xhttp = new XMLHttpRequest();
            xhttp.open("GET","/"+url, true);
            xhttp.send();
          }
          function V_WriteTRA(traNUM){
            var xhttp = new XMLHttpRequest(); 
            var Data = document.getElementById("DTRA"+traNUM).value;
            xhttp.open("GET","/writetra"+traNUM+"?DTRA"+traNUM+"="+Data, true);
            xhttp.send();
          }
          function V_WriteRRA(rraNUM){
            var xhttp = new XMLHttpRequest(); 
            var Data = document.getElementById("DRRA"+rraNUM).value;
            xhttp.open("GET","/writerra"+rraNUM+"?DRRA"+rraNUM+"="+Data, true);
            xhttp.send();
          }
          function process(){
            if(xhttp.readyState == 4 && xhttp.status == 200){
              //------Updat data sử dụng javascript----------
              ketqua = xhttp.responseText; 
              document.getElementById("reponsetext").innerHTML=ketqua;       
            }
          }
          setInterval(function(){
            updatestatusAGV();
            updatestatusTRA();
            updatestatusRRA();
            updateenergyAGV();
            updatedestinationAGV();
            updategoodslistAGV();
            powersAGV();
            updatequantityTRA();
            updatequantityRRA();
          }, 5000);  
                         
      </script>
    </body>
</html>
)=====";

void webServerStart() {
  webServer.on("/", mainpage);
  //on-off
  webServer.on("/onAGV1", on_AGV1);
  webServer.on("/offAGV1", off_AGV1);
  webServer.on("/onAGV2", on_AGV2);
  webServer.on("/offAGV2", off_AGV2);
  //status
  webServer.on("/statusAGV1", status_AGV1);
  webServer.on("/statusAGV2", status_AGV2);

  webServer.on("/statusTRA1", status_TRA1);
  webServer.on("/statusTRA2", status_TRA2);

  webServer.on("/statusRRA1", status_RRA1);
  webServer.on("/statusRRA2", status_RRA2);
  webServer.on("/statusRRA3", status_RRA3);
  webServer.on("/statusRRA4", status_RRA4);
  //energy
  webServer.on("/energyAGV1", energy_AGV1);
  webServer.on("/energyAGV2", energy_AGV2);
  //Destination
  webServer.on("/DeAGV1", De_AGV1);
  webServer.on("/DeAGV2", De_AGV2);
  //Goods_lít
  webServer.on("/GlAGV1", Gl_AGV1);
  webServer.on("/GlAGV2", Gl_AGV2);
  //Quantity
  webServer.on("/QuTRA1", QU_TRA1);
  webServer.on("/QuTRA2", QU_TRA2);

  webServer.on("/QuRRA1", QU_RRA1);
  webServer.on("/QuRRA2", QU_RRA2);
  webServer.on("/QuRRA3", QU_RRA3);
  webServer.on("/QuRRA4", QU_RRA4);
  //writedata
  webServer.on("/writetra1", Wr_TRA1);
  webServer.on("/writetra2", Wr_TRA2);
  webServer.on("/writerra1", Wr_RRA1);
  webServer.on("/writerra2", Wr_RRA2);
  webServer.on("/writerra3", Wr_RRA3);
  webServer.on("/writerra4", Wr_RRA4);
  //Automation
  webServer.on("/OnAAGV1", ONA_AGV1);
  webServer.on("/OfAAGV1", OFFA_AGV1);
  webServer.on("/OnAAGV2", ONA_AGV2);
  webServer.on("/OfAAGV2", OFFA_AGV2);
  //Control
  webServer.on("/StopAGV1", stop_AGV1);
  webServer.on("/UpAGV1", up_AGV1);
  webServer.on("/DownAGV1", down_AGV1);
  webServer.on("/LeftAGV1", left_AGV1);
  webServer.on("/RightAGV1", right_AGV1);
  webServer.on("/StopAGV2", stop_AGV2);
  webServer.on("/UpAGV2", up_AGV2);
  webServer.on("/DownAGV2", down_AGV2);
  webServer.on("/LeftAGV2", left_AGV2);
  webServer.on("/RightAGV2", right_AGV2);
  webServer.begin();
}
void mainpage() {
  String s = FPSTR(MainPage);
  webServer.send(200, "text/html", s);
}
//ON-OFF
void on_AGV1(void) {
  if (check_p[0] == 0){
    webSocket.broadcastTXT("1PON");
    Serial.println("TX: 1PON");
  }
  check_p[0] = 1;
}
void off_AGV1(void) {
  if (check_p[0] == 1){
    webSocket.broadcastTXT("1POFF");
    Serial.println("TX: 1POFF");
  }
  check_p[0] = 0;
}
void on_AGV2(void) {
  if (check_p[1] == 0)
    webSocket.broadcastTXT("2PON");
  check_p[1] = 1;
}
void off_AGV2(void) {
  if (check_p[1] == 1)
    webSocket.broadcastTXT("2POFF");
  check_p[1] = 0;
}

//status
void status_AGV1(void) {
  webServer.send(200, "text/plain", agv[0].status);
}
void status_AGV2(void) {
  webServer.send(200, "text/plain", agv[1].status);
}

void status_TRA1(void) {
  webServer.send(200, "text/plain", agv[3].status);
}
void status_TRA2(void) {
  webServer.send(200, "text/plain", agv[4].status);
}

void status_RRA1(void) {
  webServer.send(200, "text/plain", agv[5].status);
}
void status_RRA2(void) {
  webServer.send(200, "text/plain", agv[6].status);
}
void status_RRA3(void) {
  webServer.send(200, "text/plain", agv[7].status);
}
void status_RRA4(void) {
  webServer.send(200, "text/plain", agv[8].status);
}
//energy
void energy_AGV1(void) {
  webServer.send(200, "text/plain", agv[0].energy);
}
void energy_AGV2(void) {
  webServer.send(200, "text/plain", agv[1].energy);
}

void De_AGV1(void) {
  webServer.send(200, "text/plain", agv[0].destination);
}
void De_AGV2(void) {
  webServer.send(200, "text/plain", agv[1].destination);
}
void Gl_AGV1(void) {
  webServer.send(200, "text/plain", agv[0].goods_list);
}
void Gl_AGV2(void) {
  webServer.send(200, "text/plain", agv[1].goods_list);
}
void QU_TRA1(void) {
  webServer.send(200, "text/plain", agv[3].quantity);
}
void QU_TRA2(void) {
  webServer.send(200, "text/plain", agv[4].quantity);
}
void QU_RRA1(void) {
  webServer.send(200, "text/plain", agv[5].quantity);
}
void QU_RRA2(void) {
  webServer.send(200, "text/plain", agv[6].quantity);
}
void QU_RRA3(void) {
  webServer.send(200, "text/plain", agv[7].quantity);
}
void QU_RRA4(void) {
  webServer.send(200, "text/plain", agv[8].quantity);
}
//WriteDta
void Wr_TRA1(void) {
  agv[3].quantity = webServer.arg("DTRA1");
  String Q_To_Send;
  Q_To_Send = "4Q" + agv[3].quantity;
  webSocket.broadcastTXT(Q_To_Send);
  Serial.println(Q_To_Send);
}
void Wr_TRA2(void) {
  agv[4].quantity = webServer.arg("DTRA2");
  String Q_To_Send;
  Q_To_Send = "5Q" + agv[4].quantity;
  webSocket.broadcastTXT(Q_To_Send);
  Serial.println(Q_To_Send);
}
void Wr_RRA1(void) {
  agv[5].quantity = webServer.arg("DRRA1");
  String Q_To_Send;
  Q_To_Send = "6Q" + agv[5].quantity;
  webSocket.broadcastTXT(Q_To_Send);
  Serial.println(Q_To_Send);
}
void Wr_RRA2(void) {
  agv[6].quantity = webServer.arg("DRRA2");
  String Q_To_Send;
  Q_To_Send = "7Q" + agv[6].quantity;
  webSocket.broadcastTXT(Q_To_Send);
  Serial.println(Q_To_Send);
}
void Wr_RRA3(void) {
  agv[7].quantity = webServer.arg("DRRA3");
  String Q_To_Send;
  Q_To_Send = "8Q" + agv[7].quantity;
  webSocket.broadcastTXT(Q_To_Send);
  Serial.println(Q_To_Send);
}
void Wr_RRA4(void) {
  agv[8].quantity = webServer.arg("DRRA4");
  String Q_To_Send;
  Q_To_Send = "9Q" + agv[8].quantity;
  webSocket.broadcastTXT(Q_To_Send);
  Serial.println(Q_To_Send);
}
//Automation
void ONA_AGV1(void) {
  webSocket.broadcastTXT("1AOn");
}
void OFFA_AGV1(void) {
  webSocket.broadcastTXT("1AOf");
}
void ONA_AGV2(void) {
  webSocket.broadcastTXT("2AOn");
}
void OFFA_AGV2(void) {
  webSocket.broadcastTXT("2AOf");
}
//control
void stop_AGV1(void) {
  webSocket.broadcastTXT("1CS");
}
void up_AGV1(void) {
  webSocket.broadcastTXT("1CU");
}
void down_AGV1(void) {
  webSocket.broadcastTXT("1CD");
}
void left_AGV1(void) {
  webSocket.broadcastTXT("1CL");
}
void right_AGV1(void) {
  webSocket.broadcastTXT("1CR");
}

void stop_AGV2(void) {
  webSocket.broadcastTXT("2CS");
}
void up_AGV2(void) {
  webSocket.broadcastTXT("2CU");
}
void down_AGV2(void) {
  webSocket.broadcastTXT("2CD");
}
void left_AGV2(void) {
  webSocket.broadcastTXT("2CL");
}
void right_AGV2(void) {
  webSocket.broadcastTXT("2CR");
}

void webSocketEvent(uint8_t num, WStype_t type,
                    uint8_t* payload,
                    size_t length) {
  String payloadString = (const char*)payload;
  char AGV_num = payloadString[0];
  char AGV_in4 = payloadString[1];
  Serial.print("Data RX: ");
  Serial.println(payloadString);
  String broadcastPayload;
  payloadString = payloadString.substring(2);
  if (AGV_num >= '1' && AGV_num <= '9') {
    int agvIndex = AGV_num - '1';
    if (AGV_in4 == 'S') {
      char side = payloadString[0];

      agv[agvIndex].status = payloadString.substring(1);
      Serial.println(side);
      //  Serial.println(agv[agvIndex].status);
      if (side == '0') {
        broadcastPayload = "4D" + String(AGV_num);
        webSocket.broadcastTXT(broadcastPayload);
      } else if (side == '1') {
        Serial.println("Side = 1");
        broadcastPayload = "5D" + String(AGV_num);
        webSocket.broadcastTXT(broadcastPayload);
      } else if (side == '2') {
        broadcastPayload = "6D" + String(AGV_num);
        webSocket.broadcastTXT(broadcastPayload);
      } else if (side == '3') {
        broadcastPayload = "7D" + String(AGV_num);
        webSocket.broadcastTXT(broadcastPayload);
      } else if (side == '4') {
        broadcastPayload = "8D" + String(AGV_num);
        webSocket.broadcastTXT(broadcastPayload);
      } else if (side == '5') {
        broadcastPayload = "9D" + String(AGV_num);
        webSocket.broadcastTXT(broadcastPayload);
      }
      Serial.println("TX: " + broadcastPayload);
    } else if (AGV_in4 == 'E') {
      agv[agvIndex].energy = payloadString;
    } else if (AGV_in4 == 'D') {
      if(payloadString != "Done")
        agv[agvIndex].destination = payloadString;
      broadcastPayload = String(AGV_num) + 'D' + payloadString;
      webSocket.broadcastTXT(broadcastPayload);
      Serial.println("TX: " + broadcastPayload);
    } else if (AGV_in4 == 'G') {
      agv[agvIndex].goods_list = payloadString;
    } else if (AGV_in4 == 'Q') {
      agv[agvIndex].quantity = payloadString;
      Serial.println(agv[agvIndex].quantity);
    } 
  }
}
void setup(void) {
  Serial.begin(115200);
  // Connect to Wi-Fi
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("...");
    delay(500);
  }
  ip = WiFi.localIP();
  ip[3] = 10;
  gateway = WiFi.localIP();
  gateway[3] = 1;
  WiFi.config(ip, gateway, subnet);
  Serial.println(WiFi.localIP());

  // Start the server
  Serial.println("ESP8266 operating in the access point mode");
  webServerStart();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("Done");
}

void loop(void) {
  webServer.handleClient();
  webSocket.loop();
}
