#!/usr/bin/python3

import ev3dev.ev3 as ev3
import time
import math

class PID:
    def __init__(self):

        self.motors = None
        self.cs = None
        self.white = 0
        self.black = 0

        self.black = 75
        self.white = 949

        self.err = 0
        self.mid = 0
        self.m = 0
        self.i = 0
        self.start_power = 26
        self.power_offset = -5

        self.init_comp()
        self.calibrate(self.cs)
        self.follow()


    def init_comp(self,):
        self.motors = [ev3.LargeMotor(port) for port in ['outB', 'outC']]
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
        self.m = -(self.start_power/err_max)*1.3      # turn = slo*error
        self.i = self.m/50
        print("White:", self.white, " Black: ", self.black, " m: ", self.m)


    def follow(self):

        for m in self.motors:
            m.duty_cycle_sp = self.start_power
            m.run_direct()

        integral = 0
        last_err = 0
        error = 0
        d = 0           # direction (links oder rechts)
        while True:
            last_err = error
            error = sum(self.cs.bin_data("hhh")) - self.mid



            if d == 0:
                d = 1 if error > 0 else -1
            elif error*d < 0:
                d *= -1
                integral = 0
                print("\nchanged dir\n")
            elif math.fabs(error-last_err) > 200:
                integral = 0
                print("\nstep\n")

            integral += error

            turn = error*self.m + integral*self.i

            l_turn = turn
            r_turn = -turn

            # if turn < -self.start_power+3:
            #     back_turn *= 1.2
            #     l_turn *= 1.6
            # else: back_turn = 1

            # if error < -400:
            #     # self.motors[0].stop()
            #     # self.motors[1].stop()
            #
            #     while error < 90:
            #         error = sum(self.cs.bin_data("hhh")) - self.mid
            #         self.motors[0].duty_cycle_sp = 20
            #         self.motors[1].duty_cycle_sp = -20
            #
            #         self.motors[0].run_direct()
            #         self.motors[1].run_direct()
            # elif error > 400:
            #     self.motors[0].stop()
            #     self.motors[1].stop()
            # else:
            self.motors[0].duty_cycle_sp = self.start_power + l_turn
            self.motors[1].duty_cycle_sp = self.start_power + r_turn


            print("Error: {:6.2f} l_turn: {:5.2f} r_turn: {:5.2f} l: {:5.2f} r: {:5.2f}".format(error,l_turn,r_turn,self.motors[0].duty_cycle_sp, self.motors[1].duty_cycle_sp))


pid = PID()
