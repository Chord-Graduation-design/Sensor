#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.hpp"
#include <Adafruit_AHTX0.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <msg.pb.h>
#include <Preferences.h>
#include <MQUnifiedsensor.h>
#include <ESP32Servo.h>
// Servo mg995;
Servo sg90;

WiFiClient   EspClient;
PubSubClient MqttClient(EspClient);
Preferences preferences;
Adafruit_AHTX0 aht10;
MQUnifiedsensor MQ2("Arduino", 5, 10, GPIO_NUM_32, "MQ-2");
uint32_t tick = 1500;
void WifiConnect(){
  //从nvs中获取wifi信息
  preferences.begin("wifi");
  auto ssid = preferences.getString("s");//获取ssid
  auto password = preferences.getString("p");//获取密码
  Serial.printf("ssid %s pwd %s\n",ssid.c_str(),password.c_str());
  preferences.end();
  if(ssid.length() > 0 && password.length() > 0){
    Serial.printf("正在连接 %s\n",ssid.c_str());
    WiFi.begin(ssid.c_str(),password.c_str());

    int tick_t = 0;
    while(WiFi.status() != WL_CONNECTED){
      delay(300);
      Serial.printf(".");
      //如果连接超时,则开始配网
      if (tick_t++ > 100)
        break;
    }
    if(WiFi.status() == WL_CONNECTED){
      return;
    }
    Serial.println("连接失败,开始配网...");
    preferences.begin("wifi");
    preferences.clear();
    preferences.end();

  }
  else{
    Serial.println("无法获取到wifi信息,开始配网...");
  }
  WiFi.mode(WIFI_STA);
  WiFi.beginSmartConfig();
  while(!WiFi.smartConfigDone()){
    delay(500);
    Serial.print(".");
  }
  //这个api弃用了
  // WiFi.setAutoConnect(true);
  Serial.println("SmatConfig 配网成功!");

  preferences.begin("wifi");
  preferences.putString("s",WiFi.SSID());
  preferences.putString("p",WiFi.psk());
  preferences.end();
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] \n");
  uint8_t buffer[512];
  auto istream = pb_istream_from_buffer(payload,length);
  SensorIn sensor_in = SensorIn_init_zero;

  if(pb_decode(&istream,SensorIn_fields,&sensor_in)){
    switch (sensor_in.type)
    {
    case SensorType_SET_HUM_TEMP_TICK:
    {
      if(sensor_in.has_tick){
        tick = sensor_in.tick;
      }
      break;
    }
    case SensorType_IN_CONTROL:{
      LedSwitch led_switch = LedSwitch_init_zero;
      led_switch.on = digitalRead(GPIO_NUM_2) == HIGH;
      // digitalWrite(GPIO_NUM_2,led_switch.on ? HIGH : LOW);
      auto ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
      if(!pb_encode(&ostream,LedSwitch_fields,&led_switch)){
        Serial.println("编码失败!");
        return;
      }
      Serial.println("我tm来了!!!!");

      MqttClient.publish(MqttConfig::LEDTopic,buffer,ostream.bytes_written);

      ServoAngle servo_angle = ServoAngle_init_zero;
      servo_angle.angle = sg90.read();
      if(!pb_encode(&ostream,ServoAngle_fields,&servo_angle)){
        Serial.println("编码失败!");
        return;
      }
      MqttClient.publish(MqttConfig::ServoTopic,buffer,ostream.bytes_written);
      break;
    }
    case SensorType_LED_SWITCH:{
      LedSwitch led_switch = LedSwitch_init_zero;
      led_switch.on = digitalRead(GPIO_NUM_2) != HIGH;
      digitalWrite(GPIO_NUM_2,led_switch.on ? HIGH : LOW);
      auto ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
      if(!pb_encode(&ostream,LedSwitch_fields,&led_switch)){
        Serial.println("编码失败!");
        return;
      }
      MqttClient.publish(MqttConfig::LEDTopic,buffer,ostream.bytes_written);
      
      break;
    }
    case SensorType_SERVO:{
      if (sensor_in.has_tick){
        Serial.printf("SERVO %d\n",sensor_in.tick);

        sg90.write(sensor_in.tick);
        
        ServoAngle servo_angle = ServoAngle_init_zero;
        servo_angle.angle = sg90.read();
        auto ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
        if(!pb_encode(&ostream,ServoAngle_fields,&servo_angle)){
          Serial.println("编码失败!");
          return;
        }
        MqttClient.publish(MqttConfig::ServoTopic,buffer,ostream.bytes_written);
      }
      break;
    }
    case SensorType_BEEP:{

      digitalWrite(GPIO_NUM_33,LOW);
      delay(sensor_in.tick);
      digitalWrite(GPIO_NUM_33,HIGH);
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
      Serial.printf("mqtt 连接失败 错误代码:%x\n",MqttClient.state());
      Serial.printf("五秒后尝试...\n");

      delay(5000);
    }
  }
}
void MqttLoop(void* param){
  while(1){
    if(!MqttClient.connected()){
      MqttConnect();
    }
    MqttClient.loop();
    delay(10);
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(GPIO_NUM_2,OUTPUT);
  // pinMode(GPIO_NUM_32,OUTPUT);
  pinMode(GPIO_NUM_33,OUTPUT);
  //有源且低电平触发，初始化别bb
  digitalWrite(GPIO_NUM_33,HIGH);
  
  // pinMode(GPIO_NUM_5,OUTPUT);
  sg90.setPeriodHertz(50);
  // mg995.attach(GPIO_NUM_5,1000,2000);
  sg90.attach(GPIO_NUM_5,500, 2400);// sg90

  // mg995.write(0);
  
  WifiConnect();
  Serial.println("\nWiFi 链接成功!!!");
  Serial.printf("ip 地址: %s\n",WiFi.localIP().toString().c_str());

  //初始化mqtt
  MqttClient.setServer(MqttConfig::Broker,MqttConfig::Port);
  MqttClient.setCallback(callback);
  
  delay(100);


  if(aht10.begin()){
    Serial.println("aht10 初始化成功！");
  }else{
    Serial.println("aht10 初始化失败!");
  }


  MQ2.setRegressionMethod(1);
  MQ2.setA(574.25); MQ2.setB(-2.222); //检测天然气
  // /*
  //   气体   | a      | b
  //   H2     | 987.99 | -2.162
  //   LPG    | 574.25 | -2.222
  //   CO     | 36974  | -3.109
  //   Alcohol| 3616.1 | -2.675
  //   Propane| 658.71 | -2.168
  // */
  MQ2.init();
  Serial.print("校准模块中，请等待！");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ2.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ2.calibrate(9.83);
    Serial.print(".");
  }
  MQ2.setR0(calcR0/10);
  Serial.printf("%f  校准结束!.\n",calcR0);



  xTaskCreatePinnedToCore(
    MqttLoop,
    "MqttLoop",
    0x5000,
    NULL,
    2,
    NULL,
    0
  );
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t now = millis();
  uint8_t buffer[512];

  if(now > last_time + tick){
    last_time = now;
    sensors_event_t aht_humi, aht_temp;
    aht10.getEvent(&aht_humi, &aht_temp);
    TemperatureAndHumidity temperature_and_humidity = TemperatureAndHumidity_init_zero;
    temperature_and_humidity.temperature = aht_temp.temperature;
    temperature_and_humidity.humidity = aht_humi.relative_humidity;
    auto ostream = pb_ostream_from_buffer(buffer, sizeof(buffer));
    if(!pb_encode(&ostream,TemperatureAndHumidity_fields,&temperature_and_humidity)){
      Serial.println("编码失败!");
      return;
    }
    MqttClient.publish(MqttConfig::Aht10Topic,buffer,ostream.bytes_written);

    MQ2.update();
    auto sensor = MQ2.readSensor();
    Mq2Message mq2_message = Mq2Message_init_zero;
    mq2_message.value = sensor;
    if(!pb_encode(&ostream,Mq2Message_fields,&mq2_message)){
      Serial.println("编码失败!");
      return;
    }
    MqttClient.publish(MqttConfig::MQ2Topic,buffer,ostream.bytes_written);    
    // Serial.printf("MQ2: %f\n",sensor);
    // digitalWrite(GPIO_NUM_5,LOW);
    // delay(100);
    // digitalWrite(GPIO_NUM_5,HIGH);
      
    // delay(10);
  }
  
  delay(100);

}