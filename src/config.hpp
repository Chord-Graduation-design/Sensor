namespace WifiConfig
{
    constexpr auto wifi_ssid = "513";
    constexpr auto wifi_password = "qwer1234..";
} // namespace name
namespace MqttConfig
{
    constexpr auto Broker = "81.68.127.168";
    constexpr auto Port = 1883;
    constexpr auto InTopic = "esp32/InTopic";
    constexpr auto Aht10Topic = "esp32/aht10";

} // namespace name
enum InMsgType : int{
    None = 0,
    GetHumitureAndTemperature = 1,
};