namespace MqttConfig
{
    constexpr auto Broker = "47.113.151.63";
    constexpr auto Port = 1883;
    constexpr auto InTopic = "esp32/InTopic";
    constexpr auto Aht10Topic = "esp32/aht10";
    constexpr auto LEDTopic = "esp32/led";
    constexpr auto ServoTopic = "esp32/servo";

}
namespace Mq2Sensor
{
    constexpr auto Type = "MQ-2";
    constexpr auto Pin = GPIO_NUM_34;
    constexpr auto Voltage_Resolution = 5;

}