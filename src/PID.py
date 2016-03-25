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

        self.black = 98
        self.white = 879

        self.err = 0
        self.mid = 0
        self.m = 0
        self.i = 0
        self.start_power = 26
        self.power_offset = -5

        self.init_comp()
        self.calibrate(self.cs)
        self.odm = odometer.odometry()

        self.correct = 5
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



                ev3.Sound.beep()
                x = math.floor(self.odm.pos_x)
                y = math.floor(self.odm.pos_y)
                print(x, y)
                # Weiter vorfahren:

                while col[0] > col[1] + col[2]:
                    col = self.cs.bin_data("hhh")
                    self.motors[0].run_direct(duty_cycle_sp = 15)
                    self.motors[1].run_direct(duty_cycle_sp = 15)
                    self.odm.update(self.motors[0].position, self.motors[1].position)


                self.motors[0].run_to_rel_pos(position_sp=20)
                self.motors[1].run_to_rel_pos(position_sp=20)
                self.odm.update(self.motors[0].position, self.motors[1].position)

                # Drehen:


                self.motors[0].run_direct(duty_cycle_sp = -15)
                self.motors[1].run_direct(duty_cycle_sp = 15)

                # for m in self.motors:
                #     m.stop()


                time.sleep(10)
                print(self.odm.pos_x, self.odm.pos_y)
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


            print("Error: {:7.2f}   l_turn: {:5.2f}   r_turn: {:5.2f}   l: {:5.2f}   r: {:5.2f}   posX: {:6.3f}   posY: {:6.3f}   heading: {:6.2f}   l_change: {:6.2f}   r_change: {:6.2f}   rotation: {:6.2f}   displacement: {:6.2f}"
                  .format(error,l_turn,r_turn,self.motors[0].duty_cycle_sp, self.motors[1].duty_cycle_sp,
                          self.odm.pos_x, self.odm.pos_y, self.odm.heading, self.odm.l_count, self.odm.r_count,
                          self.odm.rotation, self.odm.displacement))



pid = PID()
