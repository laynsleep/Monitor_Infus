#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HX711.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

/*  
* Initialise Variable
*/
int D22;

/* SENSOR */
constexpr int irA0Pin=25, irD0Pin=26; // Octocoupler
constexpr int hxDout=4, hxSck=5;      // HX711 + Load Balancer

/* AKTUATOR */
constexpr int servoPin=33;            // Servo
constexpr int buzzerPin=13;           // Buzzer
// constexpr int ledGreen=10, ledYellow=11, ledRed=12
constexpr int sdaPin=21, sclPin= 22;  // LCD

/* KONFIGURASI */
constexpr int SERVO_HZ=50, SERVO_BITS=16;
constexpr uint8_t HX_MAX_FAILS=3;
constexpr uint32_t HX_INIT_TIMEOUT_MS=1500;
constexpr uint32_t HX_READ_TIMEOUT_MS=100;
constexpr float HX_TARE_MAX_G=50.0f;  // auto-tare hanya jika beban masih dekat nol
constexpr float HX_SCALE=1677.72f;
constexpr float beratPenuh=5000.0f, batasHabis=300.0f, batasPeringatan=1000.0f;
const char* ssid="wifimu";
const char* password="wifiku";
const char* mqttServer="192.168.43.131";
constexpr uint16_t mqttPort=1883;
const char* baseTopic="medical/infusion/data";
const char* topicCommand="medical/infusion/command";
const char* topicData="medical/infusion/data/room001";

/* 
* Initialise Object
*/
WiFiClient net;
PubSubClient mqtt(net);
HX711 scale;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

/*
* Konfigurasi PID
*/
float berat=0; 
float setpointDpm=20.0f, measuredDpm=0;
float kp=0.35f, ki=0.015f, kd=0.04f;
float integral=0, previousError=0;
int servoAngle=90;
String statusInfus="STARTING";
unsigned long lastSample=0, lastPublish=0, lastDropMs=0, lastEdgeMs=0;
uint32_t dropCount=0;
bool previousIr=HIGH, hxAvailable=false;
uint8_t hxFailCount=0;
bool hxNeedTare=false;

void setServo(int angle) {
  // Servo is limited to a conservative mechanical range; tune only on a test rig.
  angle=constrain(angle, 60, 120);
  servoAngle=angle;
  const uint32_t pulseUs=500UL + (uint32_t)angle*2000UL/180UL;
  const uint32_t periodUs=1000000UL/SERVO_HZ;
  const uint32_t maxDuty=(1UL<<SERVO_BITS)-1;
  ledcWrite(servoPin, (uint32_t)((uint64_t)pulseUs*maxDuty/periodUs));
}

/*  Aktifkan buzzer berdasarkan nilai alarm
*   updateIndicators() memberikan nilai true/false alarm
*   setAlarm(on) menerima argumen alarm untuk menyalakan buzzer
*/
void setAlarm(bool on) { digitalWrite(buzzerPin, on ? HIGH : LOW); }
void updateIndicators() {
  // RGB common-cathode: GPIO12=R, GPIO11=G, GPIO10=B.
  bool alarm=(statusInfus=="HABIS" || statusInfus=="SENSOR_ERROR");
  bool warning=(statusInfus=="PERINGATAN");
  // digitalWrite(ledRed,(alarm||warning)?HIGH:LOW);
  // digitalWrite(ledYellow,(statusInfus=="AMAN"||warning)?HIGH:LOW);
  // digitalWrite(ledGreen,LOW);
  setAlarm(alarm);
}

/*  Mengubah status berdasarkan berat infus
*   mengembalikan status "SENSOR_ERROR(HX711)" apabila HX711 tidak terbaca
*   mengembalikan status "HABIS" atau "PERINGATAN" tergantung berat
*/
void updateStatus() {
  statusInfus="AMAN";
  if (!hxAvailable) statusInfus="SENSOR_ERROR(HX711)";
  else if (berat<=batasHabis) statusInfus="HABIS";
  else if (berat<=batasPeringatan) statusInfus="PERINGATAN";
  updateIndicators(); 
}

void showDisplay() {
  int percent=constrain((int)(berat/beratPenuh*100.0f),0,100);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0); display.print("MONITOR INFUS");
  display.setCursor(0,14); display.print("Berat: "); display.print(berat,0); display.println(" g");
  display.setCursor(0,27); display.print("Tetes: "); display.print(measuredDpm,1); display.println(" dpm");
  display.setCursor(0,40); display.print("Target: "); display.print(setpointDpm,1); display.println(" dpm");
  display.setCursor(0,53); display.print(statusInfus); display.print(" "); display.print(percent); display.println("%");
  display.display();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (String(topic) != topicCommand || length == 0 || length > 96) return;
  
  char msg[97];
  memcpy(msg,payload,length);
  msg[length]='\0';
  float candidate=NAN;

  // Accept plain numeric DPM or JSON {"setpoint": number}.
  StaticJsonDocument<96> doc;
  if (deserializeJson(doc,msg) == DeserializationError::Ok && doc.containsKey("setpoint")) 
    candidate=doc["setpoint"].as<float>();
  else 
    candidate=atof(msg);
  if (isfinite(candidate) && candidate >= 1.0f && candidate <= 120.0f) {
    setpointDpm=candidate;
    integral=0;
    previousError=0;
    mqtt.publish((String(baseTopic)+"/ack").c_str(), String(setpointDpm,1).c_str(), true);
  } else 
    mqtt.publish((String(baseTopic)+"/error").c_str(),"setpoint harus 1..120 dpm");
}

void connectWiFi() {
  if (WiFi.status()==WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  Serial.print("Menghubungkan ke WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi berhasil terhubung!");
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (mqtt.connected()) return;
  String clientId="InfusESP32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  if (mqtt.connect(clientId.c_str())) mqtt.subscribe(topicCommand);
}

void publishData() {
  if (!mqtt.connected()) return;
  StaticJsonDocument<256> doc;

  doc["weight_g"]=berat; doc["percent"]=constrain((int)(berat/beratPenuh*100),0,100);
  doc["drops_per_minute"]=measuredDpm; doc["setpoint_dpm"]=setpointDpm;
  doc["servo_angle_deg"]=servoAngle; doc["status"]=statusInfus;

  char payload[256];
  size_t n=serializeJson(doc, payload,sizeof(payload));
  if (n) mqtt.publish(topicData,payload);
}

void setup() {
  Serial.begin(115200);

  pinMode(irD0Pin,INPUT_PULLUP);
  // pinMode(ledGreen,OUTPUT); pinMode(ledYellow,OUTPUT); pinMode(ledRed,OUTPUT); 
  pinMode(buzzerPin,OUTPUT);

  // digitalWrite(ledRed,HIGH); 
  setAlarm(false);

  Wire.begin(sdaPin,sclPin);
  if (!display.begin(SSD1306_SWITCHCAPVCC,0x3C))
    Serial.println("OLED tidak terdeteksi");
  else { 
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0,0);
    display.println("Monitor Infus");
    display.display(); 
    }

  scale.begin(hxDout, hxSck);
  scale.set_scale(HX_SCALE);
  hxAvailable=scale.wait_ready_timeout(HX_INIT_TIMEOUT_MS);
  hxFailCount=hxAvailable ? 0 : HX_MAX_FAILS;
  hxNeedTare=!hxAvailable;
  if (hxAvailable) 
    scale.tare();
  if (ledcAttach(servoPin,SERVO_HZ,SERVO_BITS))
    setServo(90);
  else { 
    statusInfus="SERVO_ERROR";
    Serial.println("PWM servo gagal diinisialisasi"); 
  }

  mqtt.setServer(mqttServer,mqttPort);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(256);
  connectWiFi();
  lastSample=millis();
}

void loop() {
  if (mqtt.connected())
    mqtt.loop();
  else
    connectMQTT();

  // Break-beam output assumed HIGH when clear and LOW when a drop interrupts it.
  bool irNow=digitalRead(irD0Pin);
  unsigned long now=millis();
  if (previousIr == HIGH && irNow == LOW && now-lastEdgeMs > 80) {
    lastEdgeMs=now;
    if (lastDropMs!=0) {
      unsigned long interval=now-lastDropMs;
      if (interval >= 250 && interval <= 10000) measuredDpm=60000.0f/interval;
    }
    lastDropMs=now;
    ++dropCount;
  }
  previousIr=irNow;

  if (now-lastSample >= 1000) {
    float dt=(now-lastSample)/1000.0f; 
    lastSample=now;

    bool hxReadOk=scale.wait_ready_timeout(HX_READ_TIMEOUT_MS);
    if (hxReadOk) {
      float raw=scale.get_units(3);
      hxReadOk=isfinite(raw);
      if (hxReadOk) {
        if (hxNeedTare && raw >= -HX_TARE_MAX_G && raw<=HX_TARE_MAX_G) {
          scale.tare();
          hxNeedTare=false;
          berat=0;
          Serial.println("HX711: tare otomatis dilakukan");
        } else {
          berat=(raw<0) ? 0 : raw;
        }
      }
    }
    
    if (hxReadOk)
      hxFailCount=0;
    else if (hxFailCount < HX_MAX_FAILS)
      ++hxFailCount;
    hxAvailable=(hxFailCount < HX_MAX_FAILS);

    // If no drop for 10 s, the measured rate is zero.
    if (lastDropMs==0 || now-lastDropMs>10000)
      measuredDpm=0;
    updateStatus();

    // Bounded incremental PID: actuator position remains inside the configured range.
    float error=setpointDpm-measuredDpm;
    integral=constrain(integral+error*dt,-100.0f,100.0f);
    float derivative=(error-previousError)/dt;
    previousError=error;
    if (hxAvailable && statusInfus != "HABIS") {
      float correction=kp*error+ki*integral+kd*derivative;
      setServo((int)constrain((float)servoAngle+correction, 60.0f, 120.0f));
    }
    showDisplay();
    Serial.printf("weight=%.1fg drops=%.1f/set=%.1f dpm status=%s angle=%d\n",berat, measuredDpm, setpointDpm, statusInfus.c_str(), servoAngle);
  }
  if (now-lastPublish>=2000) { 
    lastPublish=now;
    publishData();
  }
  delay(2);
}