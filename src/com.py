import paho.mqtt.client as mqtt
import PID as main

class communication:

    def __init__(self, user_, pass_, id_):
        self.client = mqtt.Client(client_id='felixalex', clean_session=False, protocol=mqtt.MQTTv31)
        self.user = user_
        self.passwd = pass_


    def connect(self):
        self.client.on_message = self.received
        self.client.username_pw_set('121', password='ydpGX5bMNY')
        self.client.connect('stream-006.inf.tu-dresden.de', port=8883)
        self.client.subscribe('explorer/121', qos=1) # subscribe to topic explorer/001
        print("subscribed to explorer")
        self.client.loop_start()
        input()


    def publish(self, topic, payload):
        self.client.publish(topic, payload=payload, qos=1)

    def subscribe(self, topic):
        self.client.subscribe(topic, qos=1)

    def received(self, client, data, message):
        payload = message.payload.decode('utf-8')
        print("\nGot Messagage '{}', topic was '{}'".format(payload, message.topic))
        if message.topic == 'explorer/121' and len(payload.split(' ')) == 3:
            planet = payload.split()[0]
            planet_topic = 'planet/{}'.format(planet)
            x = payload.split()[1]
            y = payload.split()[2]
            print("payload:", payload.split(' '))
            print("Start position is X: {}, Y: {}".format(x,y))
            print("Subscribing to", planet_topic)
            self.subscribe(planet_topic)
            main.start_x = x
            main.start_y = y

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()



