import paho.mqtt.client as mqtt
from PID import *

class communication:

    def __init__(self, user_, pass_, id_, pid):
        self.client = mqtt.Client(client_id='felixalex', clean_session=False, protocol=mqtt.MQTTv31)
        self.user = user_
        self.passwd = pass_

        self.main = pid
        self.planet_topic = ""
        self.sent = []


    def connect(self):
        self.client.on_message = self.received
        self.client.username_pw_set('126', password='ydpGX5bMNY')
        self.client.connect('stream-006.inf.tu-dresden.de', port=8883)
        self.client.subscribe('explorer/121', qos=1) # subscribe to topic explorer/001
        print("subscribed to explorer")
        self.publish('explorer/121', 'ready Gliese')
        self.client.loop_start()


    def send_path(self, path_str):
        self.publish(self.planet_topic, "path " + path_str)
        self.sent.append("path " + path_str)
        print("Sent path '{}' topic: '{}'".format("path " + path_str, self.planet_topic))

    def publish(self, topic, payload):
        self.client.publish(topic, payload=payload, qos=1)

    def subscribe(self, topic):
        self.client.subscribe(topic, qos=1)

    def received(self, client, data, message):
        payload = message.payload.decode('utf-8')
        print("\nGot Messagage '{}', topic was '{}'".format(payload, message.topic))

        if payload.split(' ')[0] == 'path':
            path = payload.split(' ')
            start_cord = ' '.join(path[1:3])
            start_leave = int(path[3])
            end_cord = ' '.join(path[4:6])
            end_enter = int(path[6])
            self.main.add_path(start_cord, start_leave, end_cord, end_enter)
            if payload not in self.sent:
                self.main.got_message = True


        elif payload.split(' ')[0] == 'target':
            target = payload.split(' ')
            target_cord = ' '.join(target[1:3])
            self.main.set_target(target_cord)

        elif len(payload.split(' ')) == 3 and message.topic == 'explorer/121':
            planet = payload.split()[0]
            x = payload.split()[1]
            y = payload.split()[2]
            self.planet_topic = 'planet/{}'.format(planet)

            print("payload:", payload.split(' '))
            print("Start position is X: {}, Y: {}".format(x,y))
            print("Subscribing to", self.planet_topic)

            self.subscribe(self.planet_topic)
            self.main.set_start(x,y)

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()



# com = communication('121', 'ydpGX5bMNY', 'felixalex')
# print("Conecting")
# com.connect()