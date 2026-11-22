#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HX711.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

/*
* Environment
*/
const char* ssid="wifimu";
const char* password="wifiku";
const char* mqttServer="192.168.43.131";
constexpr uint16_t mqttPort=1883;
const char* baseTopic="medical/infusion/data";
const char* topicCommand="medical/infusion/command";
const char* topicData="medical/infusion/data/room001";
const char* topicAck="medical/infusion/data/ack";
const char* topicError="medical/infusion/data/error";

/*  
* Initialise Variable
*/

/* SENSOR */
constexpr int irD0Pin=26; // Octocoupler
constexpr int hxDout=4, hxSck=5;      // HX711 + Load Balancer

/* AKTUATOR */
constexpr int servoPin=33;            // Servo
constexpr int buzzerPin=13;           // Buzzer
// constexpr int ledGreen=10, ledYellow=11, ledRed=12
constexpr int sdaPin=21, sclPin= 22;  // LCD

/* KONFIGURASI */
constexpr int SERVO_HZ=50, SERVO_BITS=16;
constexpr uint8_t SERVO_CH=0;
constexpr uint8_t HX_MAX_FAILS=3;
constexpr uint32_t HX_INIT_TIMEOUT_MS=1500;
constexpr uint32_t HX_READ_TIMEOUT_MS=100;
constexpr float HX_TARE_MAX_G=50.0f;  // auto-tare hanya jika beban masih dekat nol
constexpr float HX_SCALE=1677.72f;
constexpr uint32_t WIFI_RETRY_MS=2500;
constexpr uint32_t WIFI_RETRY_MAX_MS=40000;
enum class Status : uint8_t { Starting, Aman, Peringatan, Habis, SensorError, ServoError };
static const char* const statusText[] = {
  "STARTING", "AMAN", "PERINGATAN", "HABIS", "SENSOR_ERROR", "SERVO_ERROR"
};

/* STATE */ 
float beratPenuh=5000.0f, batasHabis=300.0f, batasPeringatan=1000.0f;
unsigned long lastWiFiAttempt=0;
Status status=Status::Starting;
uint32_t wifiRetryMs=WIFI_RETRY_MS;
uint8_t hxFailCount=0;
bool hxNeedTare=false, hxAvailable=false;
bool previousIr=HIGH;
bool oledAvailable=false;
char deviceId[23];  // UID unik dari MAC 64-bit

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
constexpr float DPM_EMA_ALPHA=0.3f;
float berat=0; 
float setpointDpm=20.0f, measuredDpm=0;
float kp=0.35f, ki=0.015f, kd=0.04f;
float integral=0, previousMeasuredDpm=0;
int servoAngle=90;
unsigned long lastSample=0, lastPublish=0, lastDropMs=0, lastEdgeMs=0;

/* UNUSED */
// constexpr int irA0Pin=25;
// uint32_t dropCount=0;

void setServo(int angle) {
  // Servo is limited to a conservative mechanical range; tune only on a test rig.
  angle=constrain(angle, 60, 120);
  servoAngle=angle;
  const uint32_t pulseUs=500UL + (uint32_t)angle*2000UL/180UL;
  const uint32_t periodUs=1000000UL/SERVO_HZ;
  const uint32_t maxDuty=(1UL<<SERVO_BITS)-1;
  ledcWrite(SERVO_CH, (uint32_t)((uint64_t)pulseUs*maxDuty/periodUs));
}

/*  Aktifkan buzzer berdasarkan nilai alarm
*   updateIndicators() memberikan nilai true/false alarm
*   setAlarm(on) menerima argumen alarm untuk menyalakan buzzer
*/
void setAlarm(bool on) { digitalWrite(buzzerPin, on ? HIGH : LOW); }
void updateIndicators() {
  // RGB common-cathode: GPIO12=R, GPIO11=G, GPIO10=B.
  bool alarm=(status==Status::Habis || status==Status::SensorError);
  bool warning=(status==Status::Peringatan);
  // digitalWrite(ledRed,(alarm||warning)?HIGH:LOW);
  // digitalWrite(ledYellow,(status==Status::Aman||warning)?HIGH:LOW);
  // digitalWrite(ledGreen,LOW);
  setAlarm(alarm);
}

/*  Mengubah status berdasarkan berat infus
*   Status::SensorError apabila HX711 tidak terbaca
*   Status::Habis atau Status::Peringatan tergantung berat
*/
void updateStatus() {
  status=Status::Aman;
  if (!hxAvailable) status=Status::SensorError;
  else if (berat<=batasHabis) status=Status::Habis;
  else if (berat<=batasPeringatan) status=Status::Peringatan;
  updateIndicators(); 
}

int getPercent(float value, float hi, float lo=0.0f) {
  float span=hi-lo;
  if (span<=0.0f) return 0;
  return constrain((int)((value-lo)/span*100.0f),0,100);
}

void showDisplay() {
  if (!oledAvailable) return;
  int percent=getPercent(berat, beratPenuh);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0,0); display.print("MONITOR INFUS");
  display.setCursor(0,14); display.print("Berat: "); display.print(berat,0); display.println(" g");
  display.setCursor(0,27); display.print("Tetes: "); display.print(measuredDpm,1); display.println(" dpm");
  display.setCursor(0,40); display.print("Target: "); display.print(setpointDpm,1); display.println(" dpm");
  display.setCursor(0,53); display.print(statusText[(int)status]); display.print(" "); display.print(percent); display.println("%");
  display.display();
}

void publishCommandError(const char* reason) {
  char body[96];
  snprintf(body,sizeof(body),"{\"id\":\"%s\",\"error\":\"%s\"}",deviceId,reason);
  mqtt.publish(topicError,body);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic,topicCommand)!=0 || length==0 || length>96) return;

  char msg[97];
  memcpy(msg,payload,length);
  msg[length]='\0';

  JsonDocument doc;
  if (deserializeJson(doc,msg)!=DeserializationError::Ok) {
    publishCommandError("format JSON tidak valid");
    return;
  }
  if (!doc["id"].is<const char*>()) return;
  const char* id=doc["id"].as<const char*>();
  if (strcmp(id,deviceId)!=0) return;

  if (!doc["setpoint"].is<float>()) {
    publishCommandError("setpoint harus angka");
    return;
  }
  float candidate=doc["setpoint"].as<float>();
  if (isfinite(candidate) && candidate>=1.0f && candidate<=120.0f) {
    setpointDpm=candidate;
    integral=0;
    char ack[96];
    snprintf(ack,sizeof(ack),"{\"id\":\"%s\",\"setpoint\":%.1f}",deviceId,setpointDpm);
    mqtt.publish(topicAck,ack,true);
  } else {
    publishCommandError("setpoint harus 1..120 dpm");
  }
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiRetryMs=WIFI_RETRY_MS;
    return;
  }
  if (millis()-lastWiFiAttempt < wifiRetryMs && lastWiFiAttempt) return;
  wifiRetryMs=min(wifiRetryMs*2, WIFI_RETRY_MAX_MS);
  lastWiFiAttempt=millis();
  WiFi.begin(ssid,password);
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) return;
  if (mqtt.connect(deviceId)) mqtt.subscribe(topicCommand);
}

void publishData() {
  if (!mqtt.connected()) return;
  JsonDocument doc;

  doc["device_id"]=deviceId;
  doc["weight_g"]=berat;
  doc["percent"]=getPercent(berat, beratPenuh);
  doc["drops_per_minute"]=measuredDpm;
  doc["setpoint_dpm"]=setpointDpm;
  doc["servo_angle_deg"]=servoAngle;
  doc["status"]=statusText[(int)status];

  char payload[256];
  size_t n=serializeJson(doc, payload,sizeof(payload));
  if (n) mqtt.publish(topicData,payload);
}

void setup() {
  Serial.begin(115200);
  snprintf(deviceId,sizeof(deviceId),"ESP32-%016llX",(unsigned long long)ESP.getEfuseMac());

  pinMode(irD0Pin,INPUT_PULLUP);
  // pinMode(ledGreen,OUTPUT); pinMode(ledYellow,OUTPUT); pinMode(ledRed,OUTPUT); 
  pinMode(buzzerPin,OUTPUT);

  // digitalWrite(ledRed,HIGH); 
  setAlarm(false);

  Wire.begin(sdaPin,sclPin);
  if (!display.begin(SSD1306_SWITCHCAPVCC,0x3C))
    Serial.println("OLED tidak terdeteksi");
  else { 
    oledAvailable=true;
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
  if (ledcSetup(SERVO_CH, SERVO_HZ, SERVO_BITS) > 0) {
    ledcAttachPin(servoPin, SERVO_CH);
    setServo(90);
  } else {
    status=Status::ServoError;
    Serial.println("PWM servo gagal diinisialisasi");
  }

  net.setTimeout(2000);
  mqtt.setSocketTimeout(2);
  mqtt.setServer(mqttServer,mqttPort);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(256);
  WiFi.mode(WIFI_STA);
  Serial.println("WiFi: menghubungkan...");
  connectWiFi();
  lastSample=millis();
}

void loop() {
  connectWiFi();
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
      if (interval >= 250 && interval <= 10000) {
        float rawDpm=60000.0f/interval;
        measuredDpm=DPM_EMA_ALPHA*rawDpm + (1.0f-DPM_EMA_ALPHA)*measuredDpm;
      }
    }
    lastDropMs=now;
    // ++dropCount;
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
    float derivative=(measuredDpm-previousMeasuredDpm)/dt;
    previousMeasuredDpm=measuredDpm;
    if (hxAvailable && status != Status::Habis) {
      integral=constrain(integral+error*dt,-100.0f,100.0f);
      float correction=kp*error+ki*integral-kd*derivative;
      setServo((int)constrain((float)servoAngle+correction, 60.0f, 120.0f));
    } else {
      integral=0.0f; 
    }
    showDisplay();
    Serial.printf("weight=%.1fg drops=%.1f/set=%.1f dpm status=%s angle=%d\n",berat, measuredDpm, setpointDpm, statusText[(int)status], servoAngle);
  }
  if (now-lastPublish>=2000) { 
    lastPublish=now;
    publishData();
  }
  delay(2);
}