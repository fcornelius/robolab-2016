#!/usr/bin/python3

import ev3dev.ev3 as ev3
import time
import math
import odometry as odometer
from pprint import pprint
import os

class PID:
    def __init__(self):

        self.motors = None
        self.cs = None
        self.white = 0
        self.black = 0

        self.black = 118
        self.white = 838

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

        self.saved_position = {"x": 0, "y": 0}
        self.knots = []
        self.cords = []
        self.go_back = False
        self.backtrack = False
        self.backtrack_path = []

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

        print("\n" * 20)

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

        print("\n\n\n--------------------------------------------\n")
        x = math.floor(self.odm.pos_x)
        y = math.floor(self.odm.pos_y)
        print("Absolute pos:",x, y)

        # Weiter vorfahren:

        for m in self.motors:
            m.stop()

        # ev3.Sound.speak("left {} right {}".format(self.motors[0].duty_cycle_sp,self.motors[1].duty_cycle_sp)).wait()

        #Rotausgleich

        c = self.motors[0].duty_cycle_sp - self.motors[1].duty_cycle_sp
        self.motors[1].run_to_rel_pos(position_sp=1.0 * c, duty_cycle_sp = 15)

        while self.motors[1].speed > 0:
            self.odm.update(self.motors[0].position, self.motors[1].position)



        self.motors[0].run_direct(duty_cycle_sp = 15)
        self.motors[1].run_direct(duty_cycle_sp = 15)



        while col[0] > col[1] + col[2]:
            col = self.cs.bin_data("hhh")
            self.odm.update(self.motors[0].position, self.motors[1].position)




        self.motors[0].run_to_rel_pos(position_sp=40)
        self.motors[1].run_to_rel_pos(position_sp=40)

        while self.motors[0].speed > 0:
            self.odm.update(self.motors[0].position, self.motors[1].position)





        print("\n\nThis: X:", math.floor(self.odm.pos_x), "Y:", math.floor(self.odm.pos_y))

        # Knoten kord. speichern

        x = math.floor(self.odm.pos_x)
        y = math.floor(self.odm.pos_y)
        x_cord = 0
        y_cord = 0



        print("len knots: ", len(self.knots))


        if len(self.knots) == 0:

            # x_cord = x // 40 + ((x%40)*2) // 40
            # y_cord = y // 40 + ((y%40)*2) // 40

            x_cord = 0
            y_cord = 0
            x = 0
            y = 0
            self.odm.pos_x = 0
            self.odm.pos_y = 0

        else:
            x_rel = x - self.saved_position["x"]  # relative x und y zum vorigen Knoten
            y_rel = y - self.saved_position["y"]

            print("rel X {} Y {} ".format(x_rel,y_rel))

            last_knot = self.cords.index(self.backtrack_path[-1])
            print("Last_Knot: ", self.knots[last_knot]["x_cord"], self.knots[last_knot]["y_cord"])

            x_cord = self.knots[last_knot]["x_cord"] + x_rel // 40 + ((x_rel%40)*2) // 40    # relative x und y in Kord. umrechnen, dann draufaddieren,
            y_cord = self.knots[last_knot]["y_cord"] + y_rel // 40 + ((y_rel%40)*2) // 40    # sodass keine Folgefehler in der Kord. Bestimmung entstehen



        cord_str = "{} {}".format(x_cord, y_cord)

        if self.backtrack:
            self.backtrack_path.pop()
            cord_str = self.backtrack_path[-1]
            print("Coming back, cord_str is:", cord_str)
            print("Backtrack Path is", self.backtrack_path)
        else:
            self.backtrack_path.append(cord_str)                                    # Dem Pfad hinzufügen
            print("Added", cord_str, "to Backtrack Path")
            print("Backtrack Path is", self.backtrack_path)

        print("\n\nX, Y:", cord_str)


        # Fertig?

        if cord_str in self.cords and self.cords.index(cord_str) == 0:
            if self.backtrack and len(self.knots[0]["not_visited"]) == 0:
                ev3.Sound.speak("Yippeee I am finished!").wait()
                time.sleep(5)
                quit()

        # Knoten bekannt?

        if cord_str in self.cords:
            known_knot = True
            this_knot = self.cords.index(cord_str)

            print("\nKnown Knot (", this_knot,")\n\n")
        else:
            self.knots.append({})
            self.knots[-1]["x_cord"] = x_cord
            self.knots[-1]["y_cord"] = y_cord
            self.knots[-1]["x"] = x
            self.knots[-1]["y"] = y
            self.knots[-1]["paths"] = []
            self.knots[-1]["not_visited"] = []

            self.cords.append(cord_str)
            known_knot = False
            this_knot = -1
            print("\nNew Knot\n")

        # ev3.Sound.speak("X {} Y {}".format(self.knots[-1]["x_cord"], self.knots[-1]["y_cord"])).wait()


        # Enter


        enter_deg = math.degrees(self.odm.heading)
        enter_aligned = self.align_enter(enter_deg)
        enter_at = self.deg_to_enter(enter_aligned)

        if not self.backtrack: self.knots[this_knot]["last_enter"] = enter_at
        print("Last enter:", self.knots[this_knot]["last_enter"])

        print("\n\nEnter:", enter_deg,"\n")
        print("Enter aligned:", self.align_enter(enter_deg),"\n")
        # ev3.Sound.speak("Enter at {} degrees".format(self.align_enter(math.degrees(self.odm.heading)))).wait()



        # gradeaus checken:

        col = self.cs.bin_data("hhh")
        if sum(col) < self.white-100 and not known_knot:
            self.knots[this_knot]["not_visited"].append(self.deg_to_pathnum())
            self.knots[this_knot]["paths"].append(self.deg_to_pathnum())
            go_straight = True
        else:
            go_straight = False



        # Drehen:

        deg = math.degrees(self.odm.heading)
        turned = 0
        lpos = self.motors[0].position




        # Kanten Scan, falls neuer Knoten

        if not known_knot:

            self.motors[0].run_direct(duty_cycle_sp = 25)
            self.motors[1].run_direct(duty_cycle_sp = -25)

            while turned < 330:


                while self.motors[0].position - lpos < 90 and turned < 330:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    turned = math.degrees(self.odm.heading) - deg
                    # print("turned: ", turned,  "heading:", math.degrees(self.odm.heading), "lpos:", lpos)


                col = self.cs.bin_data("hhh")
                while sum(col) > self.white-300 and turned < 330:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    turned = math.degrees(self.odm.heading) - deg
                    col = self.cs.bin_data("hhh")
                    # print("turned: ", turned,  "heading:", math.degrees(self.odm.heading), "lpos:", lpos, "col", sum(col))


                lpos = self.motors[0].position

                print("path at", turned)

                if turned < 300:
                    abs_deg = enter_aligned + self.turn_to_real_deg(turned)
                    path_num = self.deg_to_pathnum(abs_deg)

                    print("Path found. Relative (real_deg):", self.turn_to_real_deg(turned), "Absolute Degrees:", abs_deg, "-> Adding Path", path_num)

                    self.knots[-1]["not_visited"].append(path_num)
                    self.knots[-1]["paths"].append(path_num)


        deg = math.degrees(self.odm.heading)
        lpos = self.motors[0].position

        if enter_at in self.knots[this_knot]["not_visited"]:
            self.knots[this_knot]["not_visited"].remove(enter_at)

        print("\n\nScan completed.")
        print("Not_visited:", self.knots[this_knot]["not_visited"])

        print("backtrack is", self.backtrack)


                                                                                                                         #  3 Fälle relevant:
                                                                                                                         #
        if known_knot and not self.backtrack:                                                                            #  1. Erreichen eines bekannten Knotens auf der Suche
            self.backtrack = True                                                                                        #
            print("Setting backtrack to", self.backtrack)                                                                #
        elif known_knot and self.backtrack and len(self.knots[this_knot]["not_visited"]) == 0:                           #  2. Nach Backracking zu bekannten Knoten zurückkommen
            self.backtrack = True                                                                                        #    und Backtracking fortsetzen, da keine unentdeckten Kanten
            print("Setting backtrack to", self.backtrack)                                                                #
        elif known_knot and self.backtrack and len(self.knots[this_knot]["not_visited"]) > 0:                            #  3. Nach Backtracking zu bekannten Knoten zurückkommen
            self.backtrack = False                                                                                       #    und Backtracking fortsetzen, da keine unentdeckten Kanten
            print("Setting backtrack to", self.backtrack)                                                                #    und Backtracking beenden, weil es noch unentdeckte Kanten gibt


        print("\n\nlen(not_visited):", len(self.knots[this_knot]["not_visited"]), "backtrack:", self.backtrack)
        print("Knots:")
        for k in self.knots: print(self.knots.index(k), ": ", k)


        # Kanten finden

        turned = 0
        print("\nIdentifying Paths....")

        if go_straight and self.deg_to_pathnum(enter_aligned) in self.knots[this_knot]["not_visited"] and not self.backtrack:     # Gradeaus möglich und noch nicht besucht

            for m in self.motors: m.stop()
            path_num = self.deg_to_pathnum(enter_aligned)
            print("\n\nGoing straight")
            print("Visiting", path_num, "enter_aligned:", enter_aligned)
            # ev3.Sound.speak("Visiting {}".format(path_num)).wait()
            self.odm.reset(enter_aligned % 360)
            self.motors[0].reset()
            self.motors[1].reset()
            self.knots[this_knot]["not_visited"].remove(path_num)
            self.saved_position["x"] = x
            self.saved_position["y"] = y

            print("reset heading to:", enter_aligned % 360)
            print("heading now:", math.degrees(self.odm.heading))
            print("saved position:", self.saved_position["x"], self.saved_position["y"])

        else:                                                                                       # Grade aus nicht möglich und/oder schon besucht

            self.motors[0].run_direct(duty_cycle_sp = 25)
            self.motors[1].run_direct(duty_cycle_sp = -25)

            for path in self.knots[this_knot]["paths"]:

                while self.motors[0].position - lpos < 90:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    turned = math.degrees(self.odm.heading) - deg
                    # print("turned: ", turned, "heading:", math.degrees(self.odm.heading), "lpos:", lpos)

                # quit()

                col = self.cs.bin_data("hhh")
                while sum(col) > self.white-300:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    turned = math.degrees(self.odm.heading) - deg
                    col = self.cs.bin_data("hhh")
                    # print("turned: ", turned,  "heading:", math.degrees(self.odm.heading),  "lpos:", lpos, "col", sum(col))



                lpos = self.motors[0].position
                abs_deg = enter_aligned + self.turn_to_real_deg(turned)
                path_num = self.deg_to_pathnum(abs_deg)

                print("\nchecking ",enter_deg,"(", enter_aligned,") +",turned,"(",self.turn_to_real_deg(turned),") =", abs_deg," for pathnum. -> Path", path_num)


                if (path_num in self.knots[this_knot]["not_visited"] and not self.backtrack) \
                    or (self.backtrack and path_num == self.knots[this_knot]["last_enter"]):            # Unbesuchter Pfad oder soll zurück oder auf Rückweg

                    if self.backtrack:
                        print("\nBacktracking. Last enter:", self.knots[this_knot]["last_enter"])
                        print("Backtrack Path:", self.backtrack_path)

                    while self.motors[0].position - lpos < 25:                                           # Stück weiterfahren um rechts an Linie zu sein
                        self.odm.update(self.motors[0].position, self.motors[1].position)
                        # print("going further...",turned)
                    for m in self.motors: m.stop()
                    print("Visiting", path_num)
                    # ev3.Sound.speak("Visiting {}".format(path_num))
                    self.odm.reset(abs_deg % 360)                                                        # Knoten ansteuern und aus not_visited löschen
                    self.motors[0].reset()
                    self.motors[1].reset()
                    self.saved_position["x"] = x
                    self.saved_position["y"] = y
                    if path_num in self.knots[this_knot]["not_visited"]:
                        self.knots[this_knot]["not_visited"].remove(path_num)


                    print("reset heading to:", abs_deg % 360)
                    print("heading now:", math.degrees(self.odm.heading))
                    print("saved position:", self.saved_position["x"], self.saved_position["y"])
                    break


        # if not go_straight:
        #     for path in self.knots[-1]["paths"]:
        #
        #         while self.motors[0].position - lpos < 90:
        #             self.odm.update(self.motors[0].position, self.motors[1].position)
        #             turned = math.degrees(self.odm.heading) - deg
        #             print("turned: ", turned, "heading:", math.degrees(self.odm.heading), "lpos:", lpos)
        #
        #         # quit()
        #
        #         col = self.cs.bin_data("hhh")
        #         while sum(col) > self.white-300:
        #             self.odm.update(self.motors[0].position, self.motors[1].position)
        #             turned = math.degrees(self.odm.heading) - deg
        #             col = self.cs.bin_data("hhh")
        #             print("turned: ", turned,  "heading:", math.degrees(self.odm.heading),  "lpos:", lpos, "col", sum(col))
        #
        #         print("\nheading:", math.degrees(self.odm.heading))
        #
        #         lpos = self.motors[0].position
        #         abs_deg = enter_aligned + self.turn_to_real_deg(turned)
        #         path_num = self.deg_to_pathnum(abs_deg)
        #
        #         print("  pathnum", path_num, "knots:", self.knots[-1]["not_visited"],"\n\n")
        #         print("entered at", enter_at," aligned:", enter_aligned, "path:", enter_at)
        #         print("\nchecking ",enter_deg,"(", enter_aligned,") +",turned,"(",self.turn_to_real_deg(turned),") =", abs_deg," for pathnum\n")
        #         print("pathnum is", path_num)
        #
        #         if path_num in self.knots[-1]["not_visited"] or (go_back and path_num == enter_at):      # Unbesuchter Pfad oder soll zurück
        #
        #             while self.motors[0].position - lpos < 25:                                           # Stück weiterfahren um rechts an Linie zu sein
        #                 self.odm.update(self.motors[0].position, self.motors[1].position)
        #                 print("going further...",turned)
        #             for m in self.motors: m.stop()
        #             print("Visiting", path_num)
        #             # ev3.Sound.speak("Visiting {}".format(path_num))
        #             self.odm.reset(abs_deg % 360)                                                        # Knoten ansteuern und aus not_visited löschen
        #             self.motors[0].reset()
        #             self.motors[1].reset()
        #             self.knots[-1]["not_visited"].remove(path_num)
        #
        #
        #             print("reset heading to:", abs_deg % 360)
        #             print("heading now:", math.degrees(self.odm.heading))
        #             break
        # else:
        #
        #     # Gradeaus
        #
        #     for m in self.motors: m.stop()
        #     path_num = self.deg_to_pathnum(enter_aligned)
        #     print("\n\nGoing straight")
        #     print("Visiting", path_num, "enter_aligned:", enter_aligned)
        #     # ev3.Sound.speak("Visiting {}".format(path_num)).wait()
        #     self.odm.reset(enter_aligned % 360)
        #     self.motors[0].reset()
        #     self.motors[1].reset()
        #
        #     print("reset heading to:", enter_aligned % 360)
        #     print("heading now:", math.degrees(self.odm.heading))



        print("Not Visited:", self.knots[this_knot]["not_visited"])
        self.follow()
        quit()







        # for m in self.motors:
        #     m.stop()

        # print(self.knots)
        # print(self.odm.pos_x, self.odm.pos_y)


        self.motors[0].run_to_rel_pos(position_sp=self.knots[-1][0] * 174, duty_cycle_sp=25)
        self.motors[1].run_to_rel_pos(position_sp=self.knots[-1][0] * -174, duty_cycle_sp=25)

        time.sleep(2)

        self.follow()



    def deg_to_pathnum(self, deg = None):

        if deg == None: deg = math.degrees(self.odm.heading)

        if -30 < deg%360 < 55 or -20 < deg < 55:
            return 1
        elif 55 < deg%360 < 145:
            return 2
        elif 145 < deg%360 < 235:
            return 3
        elif 235 < deg%360 < 325:
            return 4
        elif 325 < deg%360 < 359:
            return 1

    def deg_to_enter(self, deg):



        if -30 < deg%360 < 55 or -20 < deg < 55:
            return 3
        elif 55 < deg%360 < 145:
            return 4
        elif 145 < deg%360 < 235:
            return 1
        elif 235 < deg%360 < 325:
            return 2
        elif 325 < deg%360 < 359:
            return 3

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


    def turn_to_real_deg(self, turn):
        deg = 0

        if 40 < turn < 120:
            deg = 90
        elif 120 < turn < 200:
            deg = 180
        elif 200 < turn < 290:
            deg = 270

        return deg

    def align_enter(self, enter):
        deg = 0
        enter %= 360

        if -20 < enter < 20 or 340 < enter < 359:
            deg = 0
        elif 20 < enter < 65:
            deg = 45
        elif 65 < enter < 110:
            deg = 90
        elif 110 < enter < 155:
            deg = 135
        elif 155 < enter < 200:
            deg = 180
        elif 200 < enter < 245:
            deg = -135
        elif 245 < enter < 290:
            deg = -90
        elif 290 < enter < 330:
            deg = -45

        return deg





pid = PID()
