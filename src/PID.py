#!/usr/bin/python3

import ev3dev.ev3 as ev3
import time
import math
import odometry as odometer

class PID:
    def __init__(self):

        self.motors = None
        self.cs = None
        self.white = 0
        self.black = 0

        self.black = 112
        self.white = 781

        self.err = 0
        self.mid = 0
        self.m = 0
        self.i = 0
        self.start_power = 26
        self.power_offset = -5

        self.init_comp()
        self.calibrate(self.cs)
        self.odm = odometer.odometry()
        self.follow()





    def init_comp(self):
        self.motors = [ev3.LargeMotor(port) for port in ['outB', 'outC']]
        for m in self.motors: m.reset()
        self.cs = ev3.ColorSensor()
        self.cs.mode = "RGB-RAW"


    def calibrate(self,cs):

        if self.white == 0:
            input("Place sensor on white area")
            self.white = sum(cs.bin_data("hhh"))
            input("Place sensor on black area")
            self.black = sum(cs.bin_data("hhh"))
            print("Calibrated to black: ", self.black, " white: ", self.white)
            input("Position at line")

        self.mid = (self.white+self.black)/2        # error = col-mid
        err_max = (self.white-self.black)/2
        self.m = -(self.start_power/err_max)*1.0      # turn = slo*error
        self.i = self.m/35
        print("White:", self.white, " Black: ", self.black, " m: ", self.m)


    def follow(self):

        for m in self.motors:
            m.duty_cycle_sp = self.start_power
            m.run_direct()

        integral = 0
        # last_err = 0
        error = 0
        d = 0           # direction (links oder rechts)
        col = ()

        while True:
            col = self.cs.bin_data("hhh")
            if col[0] > col[1] + col[2]:
                for m in self.motors:
                    m.stop()
                # ev3.Sound.beep()
                time.sleep(0.5)
                x = math.floor(self.odm.pos_x)
                y = math.floor(self.odm.pos_y)
                # ev3.Sound.speak("X is {} Y is {}".format(x,y)).wait()
                break


            self.odm.update(self.motors[0].position, self.motors[1].position)

            last_err = error
            error = sum(col) - self.mid

            if d == 0:
                d = 1 if error > 0 else -1
            elif error*d < 0:
                d *= -1
                # integral = 0
                print("\nchanged dir\n")
            elif math.fabs(error-last_err) > 200:
                # integral = 0
                print("\nstep\n")

            integral += error
            integral *= 2/3

            turn = error*self.m + integral*self.i

            l_turn = turn
            r_turn = -turn


            self.motors[0].duty_cycle_sp = self.start_power + l_turn
            self.motors[1].duty_cycle_sp = self.start_power + r_turn


            print("Error: {:6.2f} l_turn: {:5.2f} r_turn: {:5.2f} l: {:5.2f} r: {:5.2f} pos: {:6.3f} {:6.3}".format(error,l_turn,r_turn,self.motors[0].duty_cycle_sp, self.motors[1].duty_cycle_sp, self.odm.pos_x, self.odm.pos_y))


pid = PID()
