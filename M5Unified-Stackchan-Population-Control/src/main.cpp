#define USE_SERVO

#define FS90MG
//#define SCS0009

//#define Master
#define Slave

#include <M5Unified.h>
#include <Avatar.h>
#include <ServoEasing.hpp> // https://github.com/ArminJo/ServoEasing 
#include <esp_now.h>
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <bitset>


#ifdef SCS0009
#include <SCServo.h>
#endif
#include <iostream>

using namespace std;


uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t stackchan1[] = {0x78, 0x21, 0x84, 0x93, 0xB9, 0x34};
esp_now_peer_info_t peerInfo;
using namespace m5avatar;
boolean flag = false; 
Avatar avatar;
int temp = 0;

#ifdef Master
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
WebServer server(80);

const char *ssid = "StackChan";
const char *pass = "hori12345";

#endif
#ifdef Slave
float masterGaze[2];
#endif
float gazeX;
float gazeY;
float lastGazeX;
float lastGazeY;

#ifdef USE_SERVO
#if defined(ARDUINO_M5STACK_Core2)
//  #define SERVO_PIN_X 13  //Core2 PORT C
//  #define SERVO_PIN_Y 14
  #define SERVO_PIN_X 33  //Core2 PORT A
  #define SERVO_PIN_Y 32
#elif defined( ARDUINO_M5STACK_FIRE )
  #define SERVO_PIN_X 21
  #define SERVO_PIN_Y 22
#elif defined( ARDUINO_M5Stack_Core_ESP32 )
  #define SERVO_PIN_X 21
  #define SERVO_PIN_Y 22
#endif
#endif

#ifdef USE_SERVO

#ifdef FS90MG
#define START_DEGREE_VALUE_X 100
#define START_DEGREE_VALUE_Y 90
ServoEasing servo_x;
ServoEasing servo_y;
#endif
#ifdef SCS0009
SCSCL sc;
#define START_DEGREE_VALUE_X 536
#define START_DEGREE_VALUE_Y 540
#define X 1
#define Y 2
#endif
#endif

enum mode{
  Standby,
  Random,
  Synchronization
};
static mode now_mode;

bool servo_home = false;
void servo(void *args)
{
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
  for (;;)
  {
#ifdef USE_SERVO
#ifdef FS90MG
    if(!servo_home)
    {
    servo_x.setEaseTo(START_DEGREE_VALUE_X + (int)(50.0 * gazeX));
    if(gazeY < 0) {
      int tmp = (int)(10.0 * gazeY);
      if(tmp > 10) tmp = 10;
      servo_y.setEaseTo(START_DEGREE_VALUE_Y + tmp);
    } else {
      servo_y.setEaseTo(START_DEGREE_VALUE_Y + (int)(10.0 * gazeY));
    }
    } else {
//     avatar->setRotation(gazeX * 5);
//     float b = avatar->getBreath();
       servo_x.setEaseTo(START_DEGREE_VALUE_X); 
//     servo_y.setEaseTo(START_DEGREE_VALUE_Y + b * 5);
       servo_y.setEaseTo(START_DEGREE_VALUE_Y);
    }
    synchronizeAllServosStartAndWaitForAllServosToStop();
#endif
#ifdef SCS0009
    if(!servo_home)
    {
      avatar->getGaze(&gazeY, &gazeX);
      sc.WritePos(X, START_DEGREE_VALUE_X + (int)(70.0 * gazeX), 3000);
      if(gazeY < 0) {
        int tmp = (int)(10.0 * gazeY);
        if(tmp > 10) tmp = 10;
        sc.WritePos(Y, START_DEGREE_VALUE_Y + tmp, 3000);
      } else {
        sc.WritePos(Y, START_DEGREE_VALUE_Y + + (int)(20.0 * gazeY), 3000);
      }
    }else{
      sc.WritePos(X, START_DEGREE_VALUE_X, 3000);
      sc.WritePos(Y, START_DEGREE_VALUE_Y, 3000);
    }
#endif
#endif
    delay(50);
  }
}

void Servo_setup() {
#ifdef USE_SERVO
#ifdef FS90MG
  if (servo_x.attach(SERVO_PIN_X, START_DEGREE_VALUE_X, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE)) {
    Serial.print("Error attaching servo X");
  }
  if (servo_y.attach(SERVO_PIN_Y, START_DEGREE_VALUE_Y, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE)) {
    Serial.print("Error attaching servo Y");
  }
  servo_x.setEasingType(EASE_QUADRATIC_IN_OUT);
  servo_y.setEasingType(EASE_QUADRATIC_IN_OUT);
  setSpeedForAllServos(30);

  servo_x.setEaseTo(START_DEGREE_VALUE_X); 
  servo_y.setEaseTo(START_DEGREE_VALUE_Y);
  synchronizeAllServosStartAndWaitForAllServosToStop();
#endif
#ifdef SCS0009
  Serial2.begin(1000000, SERIAL_8N1, 33, 32);
  sc.pSerial = &Serial2;
  sc.WritePos(X, START_DEGREE_VALUE_X, 0, 1500);
  sc.WritePos(Y, START_DEGREE_VALUE_Y, 0, 1500);
#endif
#endif

#ifdef SCS0009


#endif
}

struct box_t
{
  int x;
  int y;
  int w;
  int h;
  int touch_id = -1;

  void setupBox(int x, int y, int w, int h) {
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
  }
  bool contain(int x, int y)
  {
    return this->x <= x && x < (this->x + this->w)
        && this->y <= y && y < (this->y + this->h);
  }
};
static box_t box_servo;



void sendMessage(uint8_t *buf, int data_num){

  esp_err_t result = esp_now_send(broadcastAddress, buf, data_num);
  if (result == ESP_OK) {//送信が成功したら
    Serial.println("success");
    Serial.printf("message : %s\n", buf);
  }
  else {
    Serial.println("Error");
  }
}

//送信が完了した時の処理
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Deli_Success" : "Deli_Fail");
}

//受信データ完了した時の処理
void OnDataRecv(const uint8_t *mac, const uint8_t *recvData, int len) {
  char buf[128];
  memcpy(&buf[0], recvData, len);
  #ifdef Slave
  if(buf[0] == 0xA0 && len == 2){
    if(buf[1] == 0x00){
      now_mode = Standby;
      Serial.printf("stanby mode\n");
    }else if(buf[1] == 0x01){
      now_mode = Random;
      Serial.printf("random mode\n");
    }else if(buf[1] == 0x02){
      now_mode = Synchronization;
      Serial.printf("Synchronization mode\n");
    }
  }
  #endif
#ifdef Slave
  // if(now_mode == Synchronization){

#endif
  

}

#ifdef Master
void handleNotFound() {
  String message = "File Not Found\n\n"
                   + server.uri() + " "
                   + ((server.method() == HTTP_GET) ? "GET" : "POST")
                   + "\nArguments: " + server.args() + "\n";
  for (int i = 0; i < server.args(); i++)
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  server.send(404, "text/plain", message);
}

void handleRoot(){
  uint8_t buf[2];
  buf[0] = 0xA0;
  buf[1] = 0x00;
  sendMessage(buf, 2);

  String html;
  html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta name=viewport content=width=100>";
  html += "</head>";
  html += "<body>";
  html += "<h1>Hello Stack Chan</h1>";
  html += "now mode Standby";
  html += "<form action=Stanby><input type=submit value=Stanby></form>";
  html += "<form action=Random><input type=submit value=Random></form>";
  html += "<form action=Synchronization><input type=submit value=Synchronization></form>";

  html += "</body>";
  html += "</html>";
  server.send(200, "text/html", html);
}

void handleRandom(){
  uint8_t buf[2];
  buf[0] = 0xA0;
  buf[1] = 0x01;
  sendMessage(buf, 2);

  String html;
  html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta name=viewport content=width=100>";
  html += "</head>";
  html += "<body>";
  html += "<h1>Hello Stack Chan</h1>";
  html += "now mode Random";
  html += "<form action=Stanby><input type=submit value=Stanby></form>";
  html += "<form action=Random><input type=submit value=Random></form>";
  html += "<form action=Synchronization><input type=submit value=Synchronization></form>";

  html += "</body>";
  html += "</html>";

  now_mode = Random;
  server.send(200, "text/html", html);
}
void handleSynchronization(){
  uint8_t buf[2];
  buf[0] = 0xA0;
  buf[1] = 0x02;
  sendMessage(buf, 2);

  String html;
  html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta name=viewport content=width=100>";
  html += "</head>";
  html += "<body>";
  html += "<h1>Hello Stack Chan</h1>";
  html += "now mode Synchronization";
  html += "<form action=Stanby><input type=submit value=Stanby></form>";
  html += "<form action=Random><input type=submit value=Random></form>";
  html += "<form action=Synchronization><input type=submit value=Synchronization></form>";

  html += "</body>";
  html += "</html>";

  now_mode = Synchronization;
  server.send(200, "text/html", html);
}
#endif

void setup()
{

  auto cfg = M5.config();

  M5.begin(cfg);

  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);   //ESP-NOWでデータ受信した時のコールバック関数を登録

  avatar.init(); // start drawing

  // adjust position
  const auto r = avatar.getFace()->getBoundingRect();
  const auto scale_w = M5.Display.width() / (float)r->getWidth();
  const auto scale_h = M5.Display.height() / (float)r->getHeight();
  const auto scale = std::min(scale_w, scale_h);
  avatar.setScale(scale);
  const auto offs_x = (r->getWidth() - M5.Display.width()) / 2;
  const auto offs_y = (r->getHeight() - M5.Display.height()) / 2;
  avatar.setPosition(-offs_y, -offs_x);

  #ifdef Master
  WiFi.softAP(ssid, pass);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP address: ");  Serial.println(myIP);

  server.on("/", handleRoot);
  server.on("/Random", handleRandom);
  server.on("/Synchronization", handleSynchronization);

  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
  #endif

  
  now_mode = Standby;

  #ifdef USE_SERVO
  Servo_setup();
  avatar.addTask(servo, "servo");
  #endif

  avatar.getGaze(&gazeX, &gazeY);
  lastGazeX = gazeX;
  lastGazeY = gazeY;
}

void loop()
{
  // avatar's face updates in another thread
  // so no need to loop-by-loop rendering
  //M5.update();
#ifdef Master
  server.handleClient();
#endif

  switch (now_mode)
  {
  case Standby:
    avatar.setExpression(Expression::Neutral);
    servo_home = true;
    break;
  
  case Random:
    avatar.setExpression(Expression::Happy);
    servo_home = false;
    break;

  case Synchronization:
    servo_home = false;
#ifdef Master
    avatar.getGaze(&gazeX, &gazeY);
    if(lastGazeX != gazeX || lastGazeY != gazeY){
      char message[20];
      sprintf(message,"%f,%f",gazeX,gazeY);
      sendMessage((uint8_t *)message,20);
      lastGazeX = gazeX;
      lastGazeY = gazeY;
    }
    delay(100);
#endif
    avatar.setExpression(Expression::Doubt);
    break;

  default:
    break;
  }
  
  delay(10);
}