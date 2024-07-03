#include <M5Unified.h>
#include <Avatar.h>
#include <ServoEasing.hpp> // https://github.com/ArminJo/ServoEasing 
#include <esp_now.h>
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <SCServo.h>

#define USE_SERVO

// #define FS90MG
#define SCS0009

char buf[2];
uint8_t LED[2];
uint8_t broadcastAddress[] = {0x78, 0x21, 0x84, 0x93, 0xB9, 0x34};
esp_now_peer_info_t peerInfo;
using namespace m5avatar;
boolean flag = false; 
Avatar avatar;
int temp = 0;


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

bool servo_home = false;

void servo(void *args)
{
  float gazeX, gazeY;
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
  for (;;)
  {
#ifdef USE_SERVO
#ifdef FS90MG
    if(!servo_home)
    {
    avatar->getGaze(&gazeY, &gazeX);
    servo_x.setEaseTo(START_DEGREE_VALUE_X + (int)(30.0 * gazeX));
    if(gazeY < 0) {
      int tmp = (int)(10.0 * gazeY);
      if(tmp > 10) tmp = 10;
      servo_y.setEaseTo(START_DEGREE_VALUE_Y + tmp);
    } else {
      servo_y.setEaseTo(START_DEGREE_VALUE_Y + (int)(20.0 * gazeY));
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
      sc.WritePos(X, START_DEGREE_VALUE_X + (int)(50.0 * gazeX), 1500);
      if(gazeY < 0) {
        int tmp = (int)(10.0 * gazeY);
        if(tmp > 10) tmp = 10;
        sc.WritePos(Y, START_DEGREE_VALUE_Y + tmp, 1500);
      } else {
        sc.WritePos(Y, START_DEGREE_VALUE_Y + + (int)(20.0 * gazeY), 1500);
      }

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
    Serial.print("Error attaching servo x");
  }
  if (servo_y.attach(SERVO_PIN_Y, START_DEGREE_VALUE_Y, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE)) {
    Serial.print("Error attaching servo y");
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

//送信が完了した時の処理
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Deli_Success" : "Deli_Fail");
}

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

  #ifdef USE_SERVO
  Servo_setup();
  avatar.addTask(servo, "servo");
  #endif
}

void loop()
{
  // avatar's face updates in another thread
  // so no need to loop-by-loop rendering
  M5.update();

  if(M5.BtnA.wasPressed()){
    Serial.println("push Btn b");
    if(flag==false){
      flag = true;
      temp = 1;
    }else{
      flag = false;
      temp = 0;
    }
    sprintf(buf,"%d",temp);
    memcpy(LED,buf,strlen(buf));
    esp_err_t result = esp_now_send(broadcastAddress, LED, sizeof(buf));

    if (result == ESP_OK) {//送信が成功したら
      Serial.println("success");
      Serial.print("temp : ");
      Serial.println(temp);
    }
    else {
      Serial.println("Error");
    }
    delay(500);
  }
}