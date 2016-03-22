#!/usr/bin/python3

import ev3dev.ev3 as ev3
import time

class PID:
    def __init__(self):

        self.motors = None
        self.cs = None
        self.white = 0
        self.black = 0

        self.black = 76
        self.white = 860

        self.err = 0
        self.mid = 0
        self.slo = 0
        self.start_power = 20
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
        self.slo = -((self.start_power)/err_max)      # turn = slo*error

        print("White:", self.white, " Black: ", self.black)

    def follow(self):

        for m in self.motors:
            m.duty_cycle_sp = self.start_power
            m.run_direct()
        while True:
            error = sum(self.cs.bin_data("hhh")) - self.mid
            turn = error*self.slo

            l_turn = turn
            r_turn = -turn

            if turn < self.start_power-4:
                pass

            self.motors[0].duty_cycle_sp = self.start_power + l_turn
            self.motors[1].duty_cycle_sp = self.start_power + r_turn

            for m in self.motors:
                if 4 > m.duty_cycle_sp > 0:


            print("Error: ", error, " Turn: ", turn, " l: ", self.motors[0].duty_cycle_sp, " r: ", self.motors[1].duty_cycle_sp)

pid = PID()
