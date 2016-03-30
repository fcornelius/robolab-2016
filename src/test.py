import paho.mqtt.client as mqtt

def msg(client, data, message):
	print('Got message "{}", topic was "{}"'.format(message.payload.decode('utf-8'), message.topic))

# configure
client = mqtt.Client(client_id='felixalex', clean_session=False, protocol=mqtt.MQTTv31) # client_id has to be unique among ALL users
client.on_message = msg # callback function
client.username_pw_set('121', password='ydpGX5bMNY') # group 001
client.connect('stream-006.inf.tu-dresden.de', port=8883)
client.subscribe('explorer/121', qos=1) # subscribe to topic explorer/001

# run
client.loop_start()

while True:
	input("Press Enter to continue...\n")

client.loop_stop()