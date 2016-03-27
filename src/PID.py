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

        self.black = 90
        self.white = 766

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

        self.knots = []
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

        self.m = -(self.start_power/err_max)*1.0     # turn = slo*error
        self.i = self.m/20
        self.d = self.m * 10
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

            if col[0] > col[1] + col[2]:   # Rot erkannt

                self.crossing_reached()
                break


            error = sum(col) - self.mid
            integral += error
            integral *= 1/3
            deriv = error - last_err

            turn = error*self.m + integral*self.i # + deriv*0.04

            l_turn = turn
            r_turn = -turn


            self.motors[0].duty_cycle_sp = self.start_power + l_turn
            self.motors[1].duty_cycle_sp = self.start_power + r_turn


            print("Error: {:7.2f}   l_turn: {:5.2f}   r_turn: {:5.2f}   l: {:5.2f}   r: {:5.2f}   posX: {:6.3f}   posY: {:6.3f}   heading: {:6.2f}   l_change: {:6.2f}   r_change: {:6.2f}   rotation: {:6.2f}   displacement: {:6.2f} heading_deg: {:6.2f} l pos: {:6.2f} r pos: {:6.2f}"
                  .format(error,l_turn,r_turn,self.motors[0].duty_cycle_sp, self.motors[1].duty_cycle_sp,
                          self.odm.pos_x, self.odm.pos_y, self.odm.heading, self.odm.l_count, self.odm.r_count,
                          self.odm.rotation, self.odm.displacement, math.degrees(self.odm.heading), self.motors[0].position, self.motors[1].position))

            last_err = error

    def crossing_reached(self):
        col = self.cs.bin_data("hhh")

        self.knots.append({})
        self.knots[-1]["ways_out"] = ""

        x = math.floor(self.odm.pos_x)
        y = math.floor(self.odm.pos_y)
        print(x, y)

        # Weiter vorfahren:

        for m in self.motors:
            m.stop()

        # ev3.Sound.speak("left {} right {}".format(self.motors[0].duty_cycle_sp,self.motors[1].duty_cycle_sp)).wait()

        # Rotausgleich
        c = self.motors[0].duty_cycle_sp - self.motors[1].duty_cycle_sp
        self.motors[1].run_to_rel_pos(position_sp=1.5 * c, duty_cycle_sp = 15)

        while self.motors[1].speed > 0:
            self.odm.update(self.motors[0].position, self.motors[1].position)

        # time.sleep(2)
        # quit()

        self.motors[0].run_direct(duty_cycle_sp = 15)
        self.motors[1].run_direct(duty_cycle_sp = 15)



        while col[0] > col[1] + col[2]:
            col = self.cs.bin_data("hhh")
            self.odm.update(self.motors[0].position, self.motors[1].position)


        # quit()

        self.motors[0].run_to_rel_pos(position_sp=40)
        self.motors[1].run_to_rel_pos(position_sp=40)

        while self.motors[0].speed > 0:
            self.odm.update(self.motors[0].position, self.motors[1].position)

        # time.sleep(3)
        # self.follow()
        # quit()

        print("\n\nThis: X:", math.floor(self.odm.pos_x), "Y:", math.floor(self.odm.pos_y))

        # Knoten kord. speichern

        x = math.floor(self.odm.pos_x)
        y = math.floor(self.odm.pos_y)

        self.knots[-1]["x"] = x
        self.knots[-1]["y"] = y


        if len(self.knots) == 1:
            self.knots[0]["x_cord"] = x // 40 + ((x%40)*2) // 40
            self.knots[0]["y_cord"] = y // 40 + ((y%40)*2) // 40
        else:
            x = x - self.knots[-2]["x"]  # relative x und y zum vorigen Knoten
            y = y - self.knots[-2]["y"]

            ev3.Sound.speak("X {} Y {} ".format(x,y)).wait()
            print("X {} Y {} ".format(x,y))
            self.knots[-1]["x_cord"] = self.knots[-2]["x_cord"] + x // 40 + ((x%40)*2) // 40    # relative x und y in Kord. umrechnen, dann draufaddieren,
            self.knots[-1]["y_cord"] = self.knots[-2]["y_cord"] + y // 40 + ((y%40)*2) // 40    # sodass keine Folgefehler in der Kord. Bestimmung entstehen

        ev3.Sound.speak("X {} Y {}".format(self.knots[-1]["x_cord"], self.knots[-1]["y_cord"])).wait()
        print(self.knots)

        # quit()

        self.knots[-1]["paths"] = []
        self.knots[-1]["not_visited"] = []
        turns = 0

        ev3.Sound.speak("Enter at {}".format(self.deg_to_enter())).wait()
        enter_at = self.deg_to_enter()
        # gradeaus checken:

        col = self.cs.bin_data("hhh")
        if sum(col) < self.white-100:
            print("   heading", math.degrees(self.odm.heading), "topathnum:", self.deg_to_pathnum())
            self.knots[-1]["not_visited"].append(self.deg_to_pathnum())
            self.knots[-1]["paths"].append(self.deg_to_pathnum())
            go_straight = True
        else:
            go_straight = False



        # Drehen:

        deg = math.degrees(self.odm.heading)
        turned = 0
        lpos = self.motors[0].position

        self.motors[0].run_direct(duty_cycle_sp = 25)
        self.motors[1].run_direct(duty_cycle_sp = -25)




        while turned < 330:

            turns += 1

            while self.motors[0].position - lpos < 90 and turned < 330:
                self.odm.update(self.motors[0].position, self.motors[1].position)
                turned = math.degrees(self.odm.heading) - deg
                print("turned: ", turned, "lpos:", lpos)

            # quit()

            col = self.cs.bin_data("hhh")
            while sum(col) > self.white-300 and turned < 330:
                self.odm.update(self.motors[0].position, self.motors[1].position)
                turned = math.degrees(self.odm.heading) - deg
                col = self.cs.bin_data("hhh")
                print("turned: ", turned, "lpos:", lpos, "col", sum(col))

            # ev3.Sound.beep()

            lpos = self.motors[0].position

            print("path at", turned)

            if turned < 300:
                print("   heading", math.degrees(self.odm.heading), "topathnum:", self.deg_to_pathnum())

                if self.deg_to_pathnum() != enter_at:
                    self.knots[-1]["not_visited"].append(self.deg_to_pathnum())
                self.knots[-1]["paths"].append(self.deg_to_pathnum())


        deg = math.degrees(self.odm.heading)
        lpos = self.motors[0].position

        if not go_straight:
            for path in self.knots[-1]["paths"]:

                while self.motors[0].position - lpos < 90:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    turned = math.degrees(self.odm.heading) - deg
                    print("turned: ", turned, "lpos:", lpos)

                # quit()

                col = self.cs.bin_data("hhh")
                while sum(col) > self.white-300:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    turned = math.degrees(self.odm.heading) - deg
                    col = self.cs.bin_data("hhh")
                    print("turned: ", turned, "lpos:", lpos, "col", sum(col))

                # print(self.deg_to_pathnum())
                print("\nheading:", math.degrees(self.odm.heading))

                lpos = self.motors[0].position
                print("  pathnum", self.deg_to_pathnum(), "knots:", self.knots[-1]["not_visited"],"\n\n")

                if self.deg_to_pathnum() in self.knots[-1]["not_visited"]:
                    while self.motors[0].position - lpos < 15:
                        self.odm.update(self.motors[0].position, self.motors[1].position)
                    for m in self.motors: m.stop()
                    print("Visiting", self.deg_to_pathnum())
                    self.odm.reset(self.pathnum_to_rad(self.deg_to_pathnum()))
                    self.motors[0].reset()
                    self.motors[1].reset()

                    print("reset heading to:", self.pathnum_to_rad(self.deg_to_pathnum()))
                    print("heading now:", self.odm.heading)
                    break

        print(self.knots[-1]["not_visited"])
        self.follow()
        quit()







        # for m in self.motors:
        #     m.stop()

        print(self.knots)
        print(self.odm.pos_x, self.odm.pos_y)


        self.motors[0].run_to_rel_pos(position_sp=self.knots[-1][0] * 174, duty_cycle_sp=25)
        self.motors[1].run_to_rel_pos(position_sp=self.knots[-1][0] * -174, duty_cycle_sp=25)

        time.sleep(2)

        self.follow()



    def deg_to_pathnum(self):

        deg = math.degrees(self.odm.heading)

        if -20 < deg%360 < 30 or -20 < deg < 30:
            return 1
        elif 30 < deg%360 < 120:
            return 2
        elif 120 < deg%360 < 180:
            return 3
        elif 180 < deg%360 < 300:
            return 4

    def deg_to_enter(self):

        deg = math.degrees(self.odm.heading)

        if -20 < deg%360 < 30 or -20 < deg < 30:
            return 3
        elif 30 < deg%360 < 120:
            return 4
        elif 120 < deg%360 < 200:
            return 1
        elif 200 < deg%360 < 300:
            return 2

    def pathnum_to_rad(self, num):
        deg = 0
        if num == 1:
            deg = 0
        elif num == 2:
            deg = 90
        elif num == 3:
            deg = 180
        elif num == 4:
            deg = -90

        return math.radians(deg)





pid = PID()
