#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.hpp"
#include <Adafruit_AHTX0.h>
#include <ArduinoJson.h>
// #include <ESP32Servo.h>
// Servo sg955;
WiFiClient   EspClient;
PubSubClient MqttClient(EspClient);
Adafruit_AHTX0 aht10;

void WifiConnect(){
  WiFi.begin(WifiConfig::wifi_ssid,WifiConfig::wifi_password);
  while(WiFi.status() != WL_CONNECTED){
    delay(300);
    Serial.printf(".");
  
  }
}
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  //StaticJsonDocument<200> doc;
  DynamicJsonDocument doc(1024);
  deserializeJson(doc,F(payload));
  JsonObject obj = doc.as<JsonObject>();
  const auto type_obj = obj[F("T")];
  if(!type_obj.isNull()){
    InMsgType type = obj[F("T")];
    switch (type)
    {
    case GetHumitureAndTemperature:
    {
      sensors_event_t aht_humi, aht_temp;
      aht10.getEvent(&aht_humi, &aht_temp);
      auto temp = aht_temp.temperature;
      auto hum = aht_humi.relative_humidity;
      doc.clear();
      doc[F("temp")] = temp;
      doc[F("hum")] = hum;

      String json;
      serializeJson(doc,json);
      MqttClient.publish(MqttConfig::Aht10Topic,json.c_str());
      break;
    }
    default:
      break;
    }
  }

}

void MqttConnect(){
  while(!MqttClient.connected()){
    String client_id = "Esp32-Client-";
    client_id += WiFi.macAddress();
    if(MqttClient.connect(client_id.c_str())){
      Serial.println("mqtt 连接成功!");

      MqttClient.subscribe(MqttConfig::InTopic);
    }else{
      Serial.printf("mqtt 连接失败 错误代码:%x",MqttClient.state());
      Serial.printf("五秒后尝试...");

      delay(5000);
    }
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(GPIO_NUM_2,OUTPUT);
  if(aht10.begin()){
    Serial.println("初始化成功！");
  }else{
    Serial.println("初始化失败!");
  }
  // sg955.attach(GPIO_NUM_5);
  // sg955.writeMicroseconds(500); 
  // sg955.write(90);
  delay(100);
  WifiConnect();
  Serial.println("\nWiFi 链接成功!!!");
  Serial.printf("ip 地址: %s\n",WiFi.localIP().toString().c_str());
  MqttClient.setServer(MqttConfig::Broker,MqttConfig::Port);
  MqttClient.setCallback(callback);

}
int pos =0;

void loop() {
  if(!MqttClient.connected()){
    MqttConnect();
  }

  delay(100);
  MqttClient.loop();
  // for (size_t pos = 0; pos < 360; pos++)
  // {
  //   sg955.write(pos);
  // }
  
  // sensors_event_t aht_humi, aht_temp;
  // aht10.getEvent(&aht_humi, &aht_temp);
  // auto temp = aht_temp.temperature;
  // auto humi = aht_humi.relative_humidity;

  // delay(2000);
  // Serial.printf("temparr %f\nhumiarr %f\n",temp,humi);

}