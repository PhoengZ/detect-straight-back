// void setup() {
//   Serial.begin(115200);
//   // เปิด Serial2 ที่ขา 16 (RX) และ 17 (TX) ความเร็ว 115200 ให้ตรงกับ STM32
//   Serial2.begin(115200, SERIAL_8N1, 16, 17); 
// }

// void loop() {
//   if (Serial2.available()) {
//     String data = Serial2.readStringUntil('\n');
//     Serial.print("Received from STM32: ");
//     Serial.println(data);
//   }
// }
#include <WiFi.h>
#include <PubSubClient.h>

// --- 1. ตั้งค่า WiFi ---
const char* ssid = "iPhone dewwts";      // ใส่ชื่อ WiFi
const char* password = "dewwts123";  // ใส่รหัส WiFi

// --- 2. ตั้งค่า NETPIE (Hardware Credential) ---
const char* mqtt_server = "mqtt.netpie.io";
const int mqtt_port = 1883;
const char* client_id = "405b5002-2c7a-407a-a837-325085186bee"; // Client ID Hardware
const char* mqtt_token = "YfSVHXyNEreCjSpp8GgAL7DvuZwSgtyX";  // Token Hardware
const char* mqtt_secret = "7i1bh4nnbcP672TwfrXeX7EzfWLDQeKU"; // Secret Hardware

// หัวข้อที่จะคุยกัน
const char* topic_data = "gearlock/sensor/distance"; // ส่งค่าขึ้น
const char* topic_alarm = "gearlock/action/result";   // รับคำสั่งลง

#define RXD2 16
#define TXD2 17

WiFiClient espClient;
PubSubClient client(espClient);

// --- 3. ฟังก์ชันเชื่อมต่อ NetPIE ---
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting NETPIE connection...");
    if (client.connect(client_id, mqtt_token, mqtt_secret)) {
      Serial.println("connected");
      // เชื่อมติดแล้ว ให้รอฟังคำสั่ง Alarm จาก Colab ทันที
      client.subscribe(topic_alarm);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// --- 4. ฟังก์ชันเมื่อได้รับข้อความจาก Colab ---
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Msg from Cloud: ");
  Serial.println(message);

  // ส่ง Logic ต่อให้ STM32 ผ่าน Serial (UART2)
  // สมมติ: ส่ง 'A' = เปิดเสียง, 'B' = ปิดเสียง
  if(String(topic) == topic_alarm) {
    if(message == "ON") {
      Serial2.print('A'); // ส่ง A ไป STM32
    } else if (message == "OFF") {
      Serial2.print('B'); // ส่ง B ไป STM32
    }
  }
}

void setup() {
  // Serial 0 เอาไว้ Debug ดูในคอม
  Serial.begin(115200);
  
  // Serial 2 เอาไว้คุยกับ STM32 (ขา TX=17, RX=16 ของ ESP32 โดย default)
  // ตรวจสอบ Pin ของบอร์ดคุณอีกทีนะครับ
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 
  // Serial2.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // --- 5. อ่านค่าจาก STM32 แล้วส่งขึ้น NetPIE ---
  if (Serial2.available()) {
    String dataFromSTM = Serial2.readStringUntil('\n'); // อ่านจนเจอขึ้นบรรทัดใหม่
    dataFromSTM.trim(); // ตัดช่องว่างหัวท้าย
    
    if (dataFromSTM.length() > 0) {
      Serial.print("Sending to NetPIE: ");
      Serial.println(dataFromSTM);
      
      // แปลง String เป็น char array เพื่อส่ง MQTT
      char msg[50];
      dataFromSTM.toCharArray(msg, 50);
      client.publish(topic_data, msg);
    }
  }
}