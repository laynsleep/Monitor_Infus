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
const char* ssid="Crzx";
const char* password="CrzxaExe3";
const char* mqttServer="192.168.43.131";
constexpr uint16_t mqttPort=1883;
const char* baseTopic="medical/infusion/data";
const char* topicCommand="medical/infusion/command";
const char* topicData="medical/infusion/data/room001";
const char* topicAck="medical/infusion/data/ack";
const char* topicError="medical/infusion/data/error";
constexpr uint16_t MQTT_CMD_MAX=255;

/*  
* Initialise Variable
*/

/* SENSOR */
constexpr int irD0Pin=26;             // Octocoupler
constexpr int hxDout=4, hxSck=5;      // HX711 + Load Balancer

/* AKTUATOR */
constexpr int servoPin=33;            // Servo
constexpr int buzzerPin=13;           // Buzzer
constexpr int sdaPin=21, sclPin= 22;  // LCD

/* KONFIGURASI */
constexpr int SERVO_HZ=50, SERVO_BITS=16;
constexpr uint8_t SERVO_CH=0;
constexpr int SERVO_MIN_DEG=0, SERVO_MAX_DEG=180, SERVO_HOME_DEG=90;
constexpr bool SERVO_SWEEP_TEST=false;
constexpr uint32_t BUZZER_FREQ_HZ=2000;
constexpr uint8_t BUZZER_CH=2;
constexpr uint8_t BUZZER_BITS=8;
constexpr uint32_t BUZZER_DUTY=32;
constexpr uint32_t BUZZER_ON_MS=120;
constexpr uint32_t BUZZER_CYCLE_MS=2000;
constexpr uint8_t HX_MAX_FAILS=3;
constexpr uint32_t HX_INIT_TIMEOUT_MS=1500;
constexpr uint32_t HX_READ_TIMEOUT_MS=100;
constexpr float HX_TARE_MAX_G=50.0f;
constexpr float HX_SCALE=400.0f;
constexpr uint32_t WIFI_RETRY_MS=2500;
constexpr uint32_t WIFI_RETRY_MAX_MS=40000;
enum class Status : uint8_t { Starting, Aman, Peringatan, Habis, SensorError, ServoError };
static const char* const statusText[] = {
  "STARTING", "AMAN", "PERINGATAN", "HABIS", "SENSOR_ERROR", "SERVO_ERROR"
};

/* STATE */ 
float beratPenuh=500.0f, batasHabis=beratPenuh*5/100, batasPeringatan=beratPenuh*20/100;
unsigned long lastWiFiAttempt=0;
Status status=Status::Starting;
uint32_t wifiRetryMs=WIFI_RETRY_MS;
uint8_t hxFailCount=0;
bool hxNeedTare=false, hxAvailable=false;
bool servoAvailable=false;
bool previousIr=HIGH;
bool oledAvailable=false;
char deviceId[23];
bool buzzerAlarm=false, buzzerWasAlarm=false, buzzerOn=false;
unsigned long buzzerStartMs=0;

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

/* 
* Initialise Object
*/
WiFiClient net;
PubSubClient mqtt(net);
HX711 scale;
Adafruit_SSD1306 display(128, 64, &Wire, -1);

void drawBunny(int cx, int cy) {
  display.fillRoundRect(cx-15, cy-34, 11, 27, 5, SSD1306_BLACK);
  display.fillRoundRect(cx-14, cy-33,  9, 25, 4, SSD1306_WHITE);
  display.fillRoundRect(cx-12, cy-30,  5, 18, 2, SSD1306_BLACK);
  display.fillRoundRect(cx+4,  cy-34, 11, 27, 5, SSD1306_BLACK);
  display.fillRoundRect(cx+5,  cy-33,  9, 25, 4, SSD1306_WHITE);
  display.fillRoundRect(cx+7,  cy-30,  5, 18, 2, SSD1306_BLACK);

  display.fillCircle(cx, cy, 17, SSD1306_BLACK);
  display.fillCircle(cx, cy, 16, SSD1306_WHITE);

  display.fillTriangle(cx-16, cy-4,  cx-13, cy-15, cx-6, cy-7,  SSD1306_BLACK);
  display.fillTriangle(cx-7,  cy-7,  cx-1,  cy-17, cx+5, cy-8,  SSD1306_BLACK);
  display.fillTriangle(cx+4,  cy-8,  cx+10, cy-16, cx+16, cy-5, SSD1306_BLACK);
  display.fillRoundRect(cx-18, cy-6, 5, 18, 2, SSD1306_BLACK);
  display.fillRoundRect(cx+13, cy-6, 5, 18, 2, SSD1306_BLACK);

  display.fillRoundRect(cx-11, cy-3, 7, 12, 3, SSD1306_BLACK);
  display.fillRoundRect(cx+4,  cy-3, 7, 12, 3, SSD1306_BLACK);
  display.fillRect(cx-9, cy-1, 2, 2, SSD1306_WHITE);
  display.fillRect(cx+6, cy-1, 2, 2, SSD1306_WHITE);
  display.drawLine(cx-12, cy-4, cx-5, cy-5, SSD1306_BLACK);
  display.drawLine(cx+5,  cy-5, cx+12, cy-4, SSD1306_BLACK);

  display.drawPixel(cx, cy+5, SSD1306_BLACK);
  display.drawLine(cx-3, cy+9, cx,  cy+11, SSD1306_BLACK);
  display.drawLine(cx,  cy+11, cx+3, cy+9,  SSD1306_BLACK);

  display.fillTriangle(cx-1, cy+18, cx-12, cy+13, cx-12, cy+25, SSD1306_WHITE);
  display.drawTriangle(cx-1, cy+18, cx-12, cy+13, cx-12, cy+25, SSD1306_BLACK);
  display.fillTriangle(cx+1, cy+18, cx+12, cy+13, cx+12, cy+25, SSD1306_WHITE);
  display.drawTriangle(cx+1, cy+18, cx+12, cy+13, cx+12, cy+25, SSD1306_BLACK);
  display.fillCircle(cx, cy+19, 2, SSD1306_BLACK);
  display.fillTriangle(cx-5, cy+20, cx-9, cy+28, cx-2, cy+23, SSD1306_WHITE);
  display.fillTriangle(cx+5, cy+20, cx+9, cy+28, cx+2, cy+23, SSD1306_WHITE);
}

void showSplash() {
  display.clearDisplay();
  drawBunny(32, 34);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(62, 22); display.print("Monitor");
  display.setCursor(62, 34); display.print("Infus");
  display.display();
  delay(2000);
}

void resetThresholdBerat(float berat) {
  beratPenuh = berat;
  batasPeringatan = beratPenuh * 20 / 100;
  batasHabis = beratPenuh * 5 / 100;
}

void setServo(int angle) {
  angle=constrain(angle, SERVO_MIN_DEG, SERVO_MAX_DEG);
  servoAngle=angle;
  const uint32_t pulseUs=500UL + (uint32_t)angle*2000UL/180UL;
  const uint32_t periodUs=1000000UL/SERVO_HZ;
  const uint32_t maxDuty=(1UL<<SERVO_BITS)-1;
  ledcWrite(SERVO_CH, (uint32_t)((uint64_t)pulseUs*maxDuty/periodUs));
}

/*  Buzzer pasif: PWM duty rendah dengan jeda (beep 120 ms tiap 2 detik)
*   updateIndicators() memberikan nilai alarm lewat setAlarm(on)
*   updateBuzzer() dijalankan tiap loop untuk mengatur jeda dan menulis PWM
*/
void setAlarm(bool on) { buzzerAlarm=on; }
void updateBuzzer() {
  if (buzzerAlarm && !buzzerWasAlarm) buzzerStartMs=millis();
  buzzerWasAlarm=buzzerAlarm;

  unsigned long phase=buzzerAlarm ? ((millis()-buzzerStartMs) % BUZZER_CYCLE_MS) : BUZZER_CYCLE_MS;
  bool beep=buzzerAlarm && (phase < BUZZER_ON_MS);
  if (beep!=buzzerOn) {
    buzzerOn=beep;
    ledcWrite(BUZZER_CH, beep ? BUZZER_DUTY : 0);
  }
}
void updateIndicators() {
  bool alarm=(status==Status::Habis || status==Status::SensorError);
  bool warning=(status==Status::Peringatan);
  setAlarm(alarm);
}

/*  Mengubah status berdasarkan berat infus
*   Status::SensorError apabila HX711 tidak terbaca
*   Status::Habis atau Status::Peringatan tergantung berat
*/
void updateStatus() {
  status=Status::Aman;
  if (!servoAvailable) status=Status::ServoError;
  else if (!hxAvailable) status=Status::SensorError;
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
  char body[128];
  snprintf(body,sizeof(body),"{\"deviceId\":\"%s\",\"error\":\"%s\"}",deviceId,reason);
  mqtt.publish(topicError,body);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic,topicCommand)!=0 || length==0 || length>MQTT_CMD_MAX) return;

  char msg[MQTT_CMD_MAX+1];
  memcpy(msg,payload,length);
  msg[length]='\0';

  JsonDocument doc;
  if (deserializeJson(doc,msg)!=DeserializationError::Ok) {
    publishCommandError("format JSON tidak valid");
    return;
  }
  if (!doc["deviceId"].is<const char*>()) return;
  if (strcmp(doc["deviceId"].as<const char*>(),deviceId)!=0) return;

  JsonObject data=doc["data"].as<JsonObject>();
  if (!data["tpm"].is<float>()) {
    publishCommandError("tpm harus angka");
    return;
  }
  if (!data["volume"].is<float>()) {
    publishCommandError("volume harus angka");
    return;
  }

  float tpm=data["tpm"].as<float>();
  float volume=data["volume"].as<float>();
  if (!isfinite(tpm) || tpm<1.0f || tpm>120.0f) {
    publishCommandError("tpm harus 1..120 dpm");
    return;
  }
  if (!isfinite(volume) || volume<=0.0f) {
    publishCommandError("volume harus > 0");
    return;
  }

  setpointDpm=tpm;
  resetThresholdBerat(volume);
  integral=0;

  char ack[160];
  snprintf(ack,sizeof(ack),"{\"deviceId\":\"%s\",\"data\":{\"tpm\":%.1f,\"volume\":%.1f}}",
           deviceId,setpointDpm,beratPenuh);
  mqtt.publish(topicAck,ack,true);
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
  if (mqtt.connect(deviceId)) {
    mqtt.subscribe(topicCommand);
    Serial.print("MQTT Terkoneksi: ");
    Serial.println(deviceId);
  }
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
  if (n) {
    mqtt.publish(topicData, payload);
    Serial.print("Data dikirim ke topic ");
    Serial.println(topicData);
  }
}

void setup() {
  Serial.begin(115200);
  snprintf(deviceId,sizeof(deviceId),"ESP32-%016llX",(unsigned long long)ESP.getEfuseMac());

  pinMode(irD0Pin,INPUT_PULLUP);
  pinMode(buzzerPin,OUTPUT);

  ledcSetup(BUZZER_CH, BUZZER_FREQ_HZ, BUZZER_BITS);
  ledcAttachPin(buzzerPin, BUZZER_CH);
  ledcWrite(BUZZER_CH, 0);
  setAlarm(false);

  Wire.begin(sdaPin,sclPin);
  if (!display.begin(SSD1306_SWITCHCAPVCC,0x3C))
    Serial.println("OLED tidak terdeteksi");
  else { 
    oledAvailable=true;
    showSplash();
    }

  scale.begin(hxDout, hxSck);
  if (HX_SCALE > 0.0f) scale.set_scale(HX_SCALE);
  else scale.set_scale(1.0f);
  hxAvailable=scale.wait_ready_timeout(HX_INIT_TIMEOUT_MS);
  hxFailCount=hxAvailable ? 0 : HX_MAX_FAILS;
  hxNeedTare=!hxAvailable;
  if (hxAvailable) 
    scale.tare();
  if (ledcSetup(SERVO_CH, SERVO_HZ, SERVO_BITS) > 0) {
    ledcAttachPin(servoPin, SERVO_CH);
    servoAvailable=true;
    setServo(SERVO_HOME_DEG);
    if (SERVO_SWEEP_TEST) {
      static const int sweep[] = {90, 120, 90, 60, 90};
      const uint32_t periodUs=1000000UL/SERVO_HZ;
      const uint32_t maxDuty=(1UL<<SERVO_BITS)-1;
      for (int a : sweep) {
        setServo(a);
        const uint32_t pulseUs=500UL + (uint32_t)a*2000UL/180UL;
        Serial.printf("SWEEP servo=%d deg pulse=%lu us duty=%lu\n",
                      a,pulseUs,(uint32_t)((uint64_t)pulseUs*maxDuty/periodUs));
        delay(1200);
      }
    }
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
  updateBuzzer();

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

    if (lastDropMs==0 || now-lastDropMs>10000)
      measuredDpm=0;
    updateStatus();

    float error=setpointDpm-measuredDpm;
    float derivative=(measuredDpm-previousMeasuredDpm)/dt;
    previousMeasuredDpm=measuredDpm;
    bool aktif = hxAvailable
              && status != Status::Habis
              && status != Status::SensorError
              && status != Status::ServoError;
    if (aktif) {
      integral=constrain(integral+error*dt,-100.0f,100.0f);
      float correction=kp*error+ki*integral-kd*derivative;
      setServo((int)constrain((float)servoAngle+correction,
                             (float)SERVO_MIN_DEG, (float)SERVO_MAX_DEG));
    } else {
      integral=0.0f;
      setServo(SERVO_HOME_DEG);
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