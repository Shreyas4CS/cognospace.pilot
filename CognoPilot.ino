/*
  =====================================================================
   COGNO PILOT  -  AI Autonomous Vehicle  (Vehicle firmware)
   by Cognospace
  =====================================================================
   ESP32 + L298N + 2 BO motors + 20x4 I2C LCD, controlled over BLE
   by the Cogno Pilot web app running on the phone mounted on the car.

   The phone does all the vision (MediaPipe). The ESP32 just:
     - receives drive commands over BLE and runs the motors
     - shows status text sent by the phone on the 20x4 LCD
     - stops the motors by itself if commands stop arriving (safety)

   Libraries (Library Manager):
     - LiquidCrystal_I2C  (Frank de Brabander)
   BLE comes with the ESP32 board package (works on core 2.x and 3.x).

   Board: "ESP32 Dev Module"   Partition: "Huge APP (3MB No OTA)"
          (BLE is big - pick Huge APP if the sketch is too large)

   BLE name : CognoPilot-Car
   Service  : Nordic UART  6E400001-B5A3-F393-E0A9-E50E24DCCA9E

   WIRING  (same as Welcome Robot)
   LCD I2C : SDA->GPIO21, SCL->GPIO22, VCC->5V, GND->GND
             (address 0x27, try 0x3F if blank)
   L298N   : ENA->GPIO25, IN1->GPIO26, IN2->GPIO27   (left motor)
             ENB->GPIO13, IN3->GPIO32, IN4->GPIO33   (right motor)
             Remove ENA/ENB jumpers. L298N GND must join ESP32 GND.

   COMMANDS (text, each ends with '\n', may arrive in 20-byte pieces)
     D,<left>,<right>   drive, each -100..100
     S                  stop
     T,<row>,<text>     write LCD row 0..3
     C                  clear LCD
     P                  ping (keeps the watchdog happy)
  =====================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- Settings ----------
#define DEVICE_NAME   "CognoPilot-Car"
#define SVC_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define RX_UUID       "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   // phone -> ESP32
#define TX_UUID       "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   // ESP32 -> phone

#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

// Motor pins (from Welcome Robot)
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 13
#define IN3 32
#define IN4 33

#define PWM_FREQ 1000
#define PWM_RES  8          // 0-255
#define MIN_PWM  90         // minimum PWM so BO motors actually start

#define CMD_TIMEOUT_MS 600  // no drive command for this long -> stop (safety)

// Flip these if a motor spins the wrong way (instead of swapping wires)
#define INVERT_LEFT  false
#define INVERT_RIGHT false

// ---------- BLE ----------
BLEServer*         pServer = nullptr;
BLECharacteristic* pTx     = nullptr;
volatile bool bleConnected = false;
volatile bool connEvent    = false;    // set in callback, handled in loop()
volatile bool discEvent    = false;

struct Chunk { uint8_t len; char data[64]; };
QueueHandle_t rxQueue;

// ---------- State ----------
String lineBuf;
unsigned long lastCmd = 0;
unsigned long lastIdleAnim = 0;
bool moving = false;
int idleFrame = 0;

// =====================================================================
//  Motors
// =====================================================================
void pwmSetup() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);
#else
  ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(ENA, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(ENB, 1);
#endif
}

void pwmWrite(int ch, int pin, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

void setMotor(int a, int b, int enCh, int enPin, int speed) {   // speed -100..100
  if (abs(speed) < 8) {                    // dead-zone = stop
    digitalWrite(a, LOW); digitalWrite(b, LOW);
    pwmWrite(enCh, enPin, 0);
    return;
  }
  digitalWrite(a, speed > 0 ? HIGH : LOW);
  digitalWrite(b, speed > 0 ? LOW : HIGH);
  int duty = map(abs(speed), 0, 100, MIN_PWM, 255);
  pwmWrite(enCh, enPin, duty);
}

void setDrive(int left, int right) {
  left  = constrain(left,  -100, 100);
  right = constrain(right, -100, 100);
  if (INVERT_LEFT)  left  = -left;
  if (INVERT_RIGHT) right = -right;
  setMotor(IN1, IN2, 0, ENA, left);
  setMotor(IN3, IN4, 1, ENB, right);
  moving = (abs(left) >= 8 || abs(right) >= 8);
}

void stopMotors() { setDrive(0, 0); }

// =====================================================================
//  LCD helpers
// =====================================================================
void lcdLine(int row, String s) {
  while (s.length() < 20) s += ' ';
  lcd.setCursor(0, row);
  lcd.print(s.substring(0, 20));
}

String center(String s) {
  if (s.length() >= 20) return s.substring(0, 20);
  int pad = (20 - s.length()) / 2;
  String r = "";
  for (int i = 0; i < pad; i++) r += ' ';
  return r + s;
}

void splash() {
  lcd.clear();
  lcdLine(0, center("COGNO PILOT"));
  lcdLine(1, center("AI Autonomous Car"));
  lcdLine(2, center("by Cognospace"));
  for (int i = 0; i < 20; i++) {           // loading bar
    lcd.setCursor(i, 3);
    lcd.write((uint8_t)255);               // full block
    delay(45);
  }
}

void showWaiting() {
  lcd.clear();
  lcdLine(0, center("COGNO PILOT"));
  lcdLine(1, "BLE: " DEVICE_NAME);
  lcdLine(2, "Waiting for phone");
  lcdLine(3, "");
}

void showConnected() {
  lcd.clear();
  lcdLine(0, center("COGNO PILOT"));
  lcdLine(1, center("Phone connected"));
  lcdLine(2, center("Starting app..."));
  lcdLine(3, "");
}

// =====================================================================
//  BLE callbacks (keep them tiny - real work happens in loop())
// =====================================================================
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s)    { bleConnected = true;  connEvent = true; }
  void onDisconnect(BLEServer* s) { bleConnected = false; discEvent = true; }
};

class RxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    auto v = c->getValue();                // std::string (core 2.x) or String (core 3.x)
    size_t n = v.length();
    const char* p = v.c_str();
    size_t i = 0;
    while (i < n) {
      Chunk ch;
      size_t take = (n - i > sizeof(ch.data)) ? sizeof(ch.data) : (n - i);
      memcpy(ch.data, p + i, take);
      ch.len = (uint8_t)take;
      xQueueSend(rxQueue, &ch, 0);
      i += take;
    }
  }
};

void notify(const char* msg) {
  if (!bleConnected || !pTx) return;
  pTx->setValue((uint8_t*)msg, strlen(msg));
  pTx->notify();
}

void bleSetup() {
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(185);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  BLEService* svc = pServer->createService(SVC_UUID);

  pTx = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTx->addDescriptor(new BLE2902());

  BLECharacteristic* rx = svc->createCharacteristic(
      RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCB());

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);              // name goes in the scan response
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

// =====================================================================
//  Command parser
// =====================================================================
void handleLine(const String& s) {
  if (s.length() == 0) return;
  char k = s[0];

  if (k == 'D') {                                  // D,left,right
    int c1 = s.indexOf(',');
    int c2 = s.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) return;
    int l = s.substring(c1 + 1, c2).toInt();
    int r = s.substring(c2 + 1).toInt();
    setDrive(l, r);
    lastCmd = millis();
  }
  else if (k == 'S') {                             // stop
    stopMotors();
    lastCmd = millis();
  }
  else if (k == 'T') {                             // T,row,text
    int c1 = s.indexOf(',');
    int c2 = s.indexOf(',', c1 + 1);
    if (c1 < 0 || c2 < 0) return;
    int row = s.substring(c1 + 1, c2).toInt();
    if (row < 0 || row > 3) return;
    lcdLine(row, s.substring(c2 + 1));
  }
  else if (k == 'C') {
    lcd.clear();
  }
  else if (k == 'P') {
    lastCmd = millis();
    notify("PONG\n");
  }
}

// =====================================================================
//  Setup / Loop
// =====================================================================
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pwmSetup();
  stopMotors();

  Wire.begin(21, 22);
  Wire.setClock(100000);
  lcd.init();
  lcd.backlight();
  splash();

  rxQueue = xQueueCreate(48, sizeof(Chunk));
  bleSetup();
  Serial.println("Cogno Pilot ready - BLE: " DEVICE_NAME);

  showWaiting();
}

void loop() {
  // 1) Bytes received over BLE -> assemble lines -> run commands
  Chunk ch;
  while (xQueueReceive(rxQueue, &ch, 0) == pdTRUE) {
    for (uint8_t i = 0; i < ch.len; i++) {
      char c = ch.data[i];
      if (c == '\n') { handleLine(lineBuf); lineBuf = ""; }
      else if (c != '\r' && lineBuf.length() < 96) lineBuf += c;
    }
  }

  // 2) Connection changes
  if (connEvent) {
    connEvent = false;
    lineBuf = "";
    lastCmd = millis();
    Serial.println("Phone connected");
    showConnected();
    delay(150);
    notify("READY\n");
  }
  if (discEvent) {
    discEvent = false;
    stopMotors();
    Serial.println("Phone disconnected");
    showWaiting();
    delay(300);
    BLEDevice::startAdvertising();         // let the phone reconnect
  }

  // 3) Safety watchdog: phone app sends D every ~100 ms while driving
  if (moving && millis() - lastCmd > CMD_TIMEOUT_MS) {
    stopMotors();
    if (bleConnected) lcdLine(3, "SAFETY STOP: NO CMD");
  }

  // 4) Little "waiting" animation while no phone is connected
  if (!bleConnected && millis() - lastIdleAnim > 400) {
    lastIdleAnim = millis();
    const char* f[4] = {"   ", ".  ", ".. ", "..."};
    lcd.setCursor(17, 2);
    lcd.print(f[idleFrame++ % 4]);
  }

  delay(2);
}
