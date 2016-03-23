import ev3dev.ev3 as ev3
import time
import math


class odometry:

    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        self.diameter = 5.6
        self.track = 11.8
        self.scale_factor = math.pi * (self.diameter/360)
        self.displacement = 0
        self.rotation = 0
        self.heading = 0
        self.l_count = 0
        self.r_count = 0

        self.motors = [ev3.LargeMotor(port) for port in ['outB', 'outC']]
        for m in self.motors: m.reset()

    def update(self):

        self.motors[0].duty_cycle_sp = 20
        self.motors[0].run_to_rel_pos(position_sp=720)

        # time.sleep(50)
        while True:

            self.l_count = motors[0]
            self.displacement = (self.motors[0].position + self.motors[1].position) * self.scale_factor/2
            self.rotation = (self.motors[0].position - self.motors[1].position)  * self.scale_factor/self.track

            self.pos_x += self.displacement * math.cos(self.heading + self.rotation / 2)
            self.pos_y += self.pos_y + self.displacement * math.sin(self.heading + self.rotation / 2)
            self.heading += self.rotation

            for m in self.motors: m.position = 0

            print("pos_x: ", self.pos_x, " pos_y: ", self.pos_y, " heading: ", self.heading)


OD = odometry()
OD.update()
