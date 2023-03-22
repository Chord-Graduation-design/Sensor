import msg_pb2
import base64
msg = msg_pb2.TemperatureAndHumidity();
msg.temperature = 222.2
msg.humidity = 999.1

data = msg.SerializeToString()

print("base64:{}".format(base64.encodebytes(data)))