import ev3dev.ev3 as ev3
import time
import math


class odometry:

    def __init__(self):
        self.pos_x = 0
        self.pos_y = 0
        # self.diameter = 5.6
        self.diameter = 3
        # self.track = 11.8
        self.track = 11
        self.scale_factor = math.pi * (self.diameter/360)
        self.displacement = 0
        self.rotation = 0
        # self.heading = -1.5707963267948966
        self.heading = 0
        self.l_count = 0
        self.r_count = 0
        self.l_prevc = 0
        self.r_prevc = 0


    def reset(self, heading):
        self.displacement = 0
        self.rotation = 0
        self.heading = math.radians(heading)
        self.l_count = 0
        self.r_count = 0
        self.l_prevc = 0
        self.r_prevc = 0


    def update(self,l_pos, r_pos):

        # self.motors[0].duty_cycle_sp = 30
        # self.motors[1].duty_cycle_sp = 15
        # self.motors[0].run_to_rel_pos(position_sp=1420)
        # self.motors[1].run_to_rel_pos(position_sp=-720)

        # time.sleep(50)


        # l_pos = self.motors[0].position
        # r_pos = self.motors[1].position

        self.l_count = l_pos - self.l_prevc
        self.r_count = r_pos - self.r_prevc

        self.l_prevc = l_pos
        self.r_prevc = r_pos


        self.displacement = (self.l_count + self.r_count) * self.scale_factor/2
        self.rotation = (self.l_count - self.r_count)  * self.scale_factor/self.track

        self.pos_x += self.displacement * math.cos(self.heading + self.rotation / 2)
        self.pos_y += self.displacement * math.sin(self.heading + self.rotation / 2)
        self.heading += self.rotation

        # print("heading (deg):", math.degrees(self.heading), "heading: ", self.heading, "rotation:", self.rotation, "displacement:", self.displacement)
        # print(l_pos, " ", self.l_count, " prevc ", self.l_prevc)



        # print("pos_x: ", self.pos_x, " pos_y: ", self.pos_y, " l_count: ", self.l_count, " r_count: ", self.r_count, " rotation: ", self.rotation," heading: ", self.heading, " l_speed: ", self.motors[1].speed)



