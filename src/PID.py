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

        self.black = 117
        self.white = 879

        self.err = 0
        self.mid = 0
        self.m = 0
        self.i = 0
        self.d = 0
        self.start_power = 26
        self.power_offset = -5

        self.init_comp()
        self.calibrate(self.cs)
        self.odm = odometer.odometry()

        self.knots = [[]]
        self.follow()





    def init_comp(self):
        self.motors = [ev3.LargeMotor(port) for port in ['outB', 'outC']]
        for m in self.motors:
            m.reset()
            m.stop_command = "brake"
            # m.speed_regulation_enabled = ev3.LargeMotor.SPEED_REGULATION_ON
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

        self.mid = (self.white+self.black)/2          # error = col-mid
        err_max = (self.white-self.black)/2

        self.m = -(self.start_power/err_max)*1.0      # turn = slo*error
        self.i = self.m/45
        self.d = self.m /10
        print("White:", self.white, " Black: ", self.black, " m: ", self.m, "i:", self.i, "d:", self.d)


    def follow(self):

        for m in self.motors:
            m.duty_cycle_sp = self.start_power
            m.run_direct()

        integral = 0
        deriv = 0
        last_err = 0
        error = 0
        d = 0           # direction (links oder rechts)
        col = ()

        while True:
            col = self.cs.bin_data("hhh")
            self.odm.update(self.motors[0].position, self.motors[1].position)

            if col[0] > col[1] + col[2]:

                self.crossing_reached()
                break


            error = sum(col) - self.mid
            integral += error
            integral *= 2/3
            deriv = error - last_err

            turn = error*self.m #+ integral*self.i #+ deriv*self.d

            l_turn = turn
            r_turn = -turn


            self.motors[0].duty_cycle_sp = self.start_power + l_turn
            self.motors[1].duty_cycle_sp = self.start_power + r_turn


            print("Error: {:7.2f}   l_turn: {:5.2f}   r_turn: {:5.2f}   l: {:5.2f}   r: {:5.2f}   posX: {:6.3f}   posY: {:6.3f}   heading: {:6.2f}   l_change: {:6.2f}   r_change: {:6.2f}   rotation: {:6.2f}   displacement: {:6.2f}"
                  .format(error,l_turn,r_turn,self.motors[0].duty_cycle_sp, self.motors[1].duty_cycle_sp,
                          self.odm.pos_x, self.odm.pos_y, self.odm.heading, self.odm.l_count, self.odm.r_count,
                          self.odm.rotation, self.odm.displacement))

            last_err = error

    def crossing_reached(self):

        self.knots.append([])

        x = math.floor(self.odm.pos_x)
        y = math.floor(self.odm.pos_y)
        print(x, y)

        # Weiter vorfahren:

        for m in self.motors:
            m.stop()

        c = self.motors[0].duty_cycle_sp - self.motors[1].duty_cycle_sp
        self.motors[1].run_to_rel_pos(position_sp=c, duty_cycle_sp = 15)

        while self.motors[1].speed > 0:
            self.odm.update(self.motors[0].position, self.motors[1].position)



        self.motors[0].run_direct(duty_cycle_sp = 15)
        self.motors[1].run_direct(duty_cycle_sp = 15)


        while col[0] > col[1] + col[2]:
            col = self.cs.bin_data("hhh")
            self.odm.update(self.motors[0].position, self.motors[1].position)

        self.motors[0].run_to_rel_pos(position_sp=25)
        self.motors[1].run_to_rel_pos(position_sp=25)

        while self.motors[0].speed > 0:
            self.odm.update(self.motors[0].position, self.motors[1].position)

        # quit()

        # Drehen:

        deg = math.degrees(self.odm.heading)
        turned = 0
        lpos = self.motors[0].position

        self.motors[0].run_direct(duty_cycle_sp = 25)
        self.motors[1].run_direct(duty_cycle_sp = -25)

        while turned < 200:

            while self.motors[0].position - lpos < 60:
                self.odm.update(self.motors[0].position, self.motors[1].position)
                turned = math.degrees(self.odm.heading) - deg
                print("turned: ", turned, "lpos:", lpos)


            col = self.cs.bin_data("hhh")
            while sum(col) > self.white-300:
                self.odm.update(self.motors[0].position, self.motors[1].position)
                turned = math.degrees(self.odm.heading) - deg
                col = self.cs.bin_data("hhh")
                print("turned: ", turned, "lpos:", lpos, "col", sum(col))

            lpos = self.motors[0].position

            print("path at", turned)



        for dir in range(4):
            col = self.cs.bin_data("hhh")

            if sum(col) < 700:
                # ev3.Sound.speak("Path found").wait()
                self.knots[-1].append(dir)




            self.motors[0].run_to_rel_pos(position_sp=174, duty_cycle_sp=25)
            self.motors[1].run_to_rel_pos(position_sp=-174, duty_cycle_sp=25)

            time.sleep(2)



        # for m in self.motors:
        #     m.stop()

        print(self.knots)
        print(self.odm.pos_x, self.odm.pos_y)


        self.motors[0].run_to_rel_pos(position_sp=self.knots[-1][0] * 174, duty_cycle_sp=25)
        self.motors[1].run_to_rel_pos(position_sp=self.knots[-1][0] * -174, duty_cycle_sp=25)

        time.sleep(2)

        self.follow()



pid = PID()
