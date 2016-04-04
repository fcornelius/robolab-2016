#!/usr/bin/python3

import ev3dev.ev3 as ev3
import time
import math
import odometry as odometer
from pprint import pprint
# import com as mqtt

class PID:
    def __init__(self):

        self.motors = None
        self.cs = None
        self.white = 0
        self.black = 0

        # self.black = 93
        # self.white = 688

        self.err = 0
        self.mid = 0
        self.m = 0
        self.i = 0
        self.d = 0
        self.start_power = 30
        self.power_offset = -5


        self.init_comp()
        self.odm = odometer.odometry()



        self.start_x = 0
        self.start_y = 0
        self.target = None
        self.navigate = None
        self.got_message = False

        self.saved_position = {"x": 0, "y": 0}
        self.heading_sp = None
        self.knots = []
        self.cords = []
        self.map = {}
        self.last_leave = 0
        self.go_back = False
        self.backtrack = False
        self.backtrack_path = []

        self.com = None
        self.calibrate(self.cs)


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
        time.sleep(1)
        self.black = sum(cs.bin_data("hhh"))

        self.motors[0].run_direct(duty_cycle_sp = 40)
        col = sum(cs.bin_data("hhh"))
        print("black:", col)
        self.odm.update(self.motors[0].position, self.motors[1].position)
        deg = math.degrees(self.odm.heading)
        while deg < 30:
            self.odm.update(self.motors[0].position, self.motors[1].position)
            deg = math.degrees(self.odm.heading)
            col = sum(cs.bin_data("hhh"))
            print("col:",col, "deg:", deg)

        self.white = sum(cs.bin_data("hhh"))
        print("\nWhite:", self.white, " Black: ", self.black)

        self.white = sum(cs.bin_data("hhh"))
        self.mid = (self.white+self.black)/2          # error = col-mid
        err_max = (self.white-self.black)/2

        self.motors[0].run_direct(duty_cycle_sp = -40)

        while sum(cs.bin_data("hhh")) > self.mid:
            self.odm.update(self.motors[0].position, self.motors[1].position)

        for m in self.motors: m.stop()

        # quit()




        self.m = -(self.start_power/err_max) # + 0.008    # turn = slo*error
        self.i = self.m/40
        self.d = self.m -0.2




    def set_start(self,x,y):
        print("Set Start:",x,y)
        self.start_x = x
        self.start_y = y
        print("Setting start to", self.start_x, self.start_y)


    def shortest_way(self, start_cord, end_cord):
        queue = [[start_cord]]

        while queue:
            print("queue:", queue)
            path = queue.pop(0)
            print("path:", path)
            knot = path[-1]
            if knot == end_cord: return path
            for k in self.map[knot]:
                if k not in path:
                    new = list(path)
                    new.append(k)
                    print("new:", new)
                    queue.append(new)


    def way_to_dirs(self, way):
        dirs = []
        for k in range(len(way)-1):
            knot = way[k]
            dirs.append(self.map[knot][way[k+1]]['out'])
        return dirs


    def lookup_target(self):
         if self.target and self.target in self.map:
                this_knot = self.backtrack_path[-1]
                print("Target in Map. Testing if way exists")
                way = self.shortest_way(this_knot, self.target)
                print("'{}'".format(way))
                if way:
                    self.navigate = self.way_to_dirs(way)
                    print("navigate:", self.navigate)


    def set_target(self, target_str):
        self.target = target_str
        print("Got Target. Target is:", self.target)
        self.lookup_target()




    def add_path(self, start_cord, start_leave, end_cord, end_enter):

        if not start_cord in self.map:
            self.map[start_cord] = {}
        if not end_cord in self.map:
            self.map[end_cord] = {}

        self.map[start_cord][end_cord] = { 'out': start_leave, 'in': end_enter}
        self.map[end_cord][start_cord] = { 'out': end_enter, 'in': start_leave}

        self.lookup_target()



    def send_path(self, cord_last, last_leave, cord_this, enter):
        path_str = "{} {} {} {}".format(cord_last, last_leave, cord_this, enter)
        print("\n\nSend Path: '{}'\n\n".format(path_str))
        self.com.send_path(path_str)


    def wait_for_cs(self, l_duty, r_duty):
        for m in self.motors: m.stop()
        print("Color sensor failure. Waiting...")
        col = self.cs.bin_data("hhh")
        while sum(col) == 0:
            print("waiting")
            time.sleep(1)
            col = self.cs.bin_data("hhh")
        time.sleep(1)
        col = self.cs.bin_data("hhh")
        while sum(col) == 0:
            time.sleep(1)
            col = self.cs.bin_data("hhh")
        print("Color Sensor working.")
        self.motors[0].run_direct(duty_cycle_sp = l_duty)
        self.motors[1].run_direct(duty_cycle_sp = r_duty)


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
            if sum(col) == 0:
                self.wait_for_cs(self.start_power,self.start_power)

            self.odm.update(self.motors[0].position, self.motors[1].position)
            # print("heading_sp:", self.heading_sp, "diff x+y", (self.odm.pos_x - self.saved_position["x"]) + (self.odm.pos_y - self.saved_position["y"]))

            if self.heading_sp is not None and ((self.odm.pos_x - self.saved_position["x"]) + (self.odm.pos_y - self.saved_position["y"])) > 5:
                print("---  Heading Setpoint reached, setting heading to", self.heading_sp)
                self.odm.heading = math.radians(self.heading_sp)
                self.heading_sp = None

            if col[0] > col[1] + col[2]:   # Rot erkannt

                self.crossing_reached()
                break


            error = sum(col) - self.mid
            integral += error
            integral *= 1/3
            deriv = error - last_err

            turn = error*self.m #+ integral*self.i # + deriv*0.04

            l_turn = turn
            r_turn = -turn


            self.motors[0].duty_cycle_sp = self.start_power + l_turn
            self.motors[1].duty_cycle_sp = self.start_power + r_turn


            print("Error: {:7.2f}   l_turn: {:5.2f}   r_turn: {:5.2f}   l: {:5.2f}   r: {:5.2f}   posX: {:6.3f}   posY: {:6.3f}   heading: {:6.2f}   l_change: {:6.2f}   r_change: {:6.2f}   rotation: {:6.2f}   displacement: {:6.2f} heading_deg: {:6.2f} l pos: {:6.2f} r pos: {:6.2f} {}"
                  .format(error,l_turn,r_turn,self.motors[0].duty_cycle_sp, self.motors[1].duty_cycle_sp,
                          self.odm.pos_x, self.odm.pos_y, self.odm.heading, self.odm.l_count, self.odm.r_count,
                          self.odm.rotation, self.odm.displacement, math.degrees(self.odm.heading), self.motors[0].position, self.motors[1].position, col))

            last_err = error

    def crossing_reached(self):
        col = self.cs.bin_data("hhh")

        print("\n\n\n--------------------------------------------\n")
        x = math.floor(self.odm.pos_x)
        y = math.floor(self.odm.pos_y)
        # print("Absolute pos:",x, y)

        # Weiter vorfahren:

        # for m in self.motors:
        #     m.stop()

        # ev3.Sound.speak("left {} right {}".format(self.motors[0].duty_cycle_sp,self.motors[1].duty_cycle_sp)).wait()

        # #Rotausgleich
        #
        # c = self.motors[0].duty_cycle_sp - self.motors[1].duty_cycle_sp
        # self.motors[1].run_to_rel_pos(position_sp=1.0 * c, duty_cycle_sp = 30)
        #
        # while self.motors[1].speed > 0:
        #     self.odm.update(self.motors[0].position, self.motors[1].position)



        self.motors[0].run_direct(duty_cycle_sp = 30)
        self.motors[1].run_direct(duty_cycle_sp = 30)



        while col[0] > col[1] + col[2]:
            col = self.cs.bin_data("hhh")
            if sum(col) == 0:
                self.wait_for_cs(30,30)

            self.odm.update(self.motors[0].position, self.motors[1].position)

        lpos = self.motors[0].position

        while self.motors[0].position - lpos < 50:
            self.odm.update(self.motors[0].position, self.motors[1].position)


        for m in self.motors: m.stop()




        # print("\n\nThis: X:", math.floor(self.odm.pos_x), "Y:", math.floor(self.odm.pos_y))

        # Knoten kord. speichern

        x = math.floor(self.odm.pos_x)
        y = math.floor(self.odm.pos_y)
        x_cord = 0
        y_cord = 0



        # print("len knots: ", len(self.knots))
        last_knot = 0

        if len(self.knots) == 0:

            # Erster Knoten

            x_cord = int(self.start_x)
            y_cord = int(self.start_y)

            print("Starting at", x_cord, y_cord)
            x = 0
            y = 0
            self.odm.pos_x = 0
            self.odm.pos_y = 0
            self.map["{} {}".format(x_cord, y_cord)] = {}

        else:
            x_rel = x - self.saved_position["x"]  # relative x und y zum vorigen Knoten
            y_rel = y - self.saved_position["y"]

            print("\nRel: X {} Y {} \n".format(x_rel,y_rel))

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
        if self.target and cord_str == self.target:
            ev3.Sound.speak("Target reached").wait()
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



        # Enter


        enter_deg = math.degrees(self.odm.heading)
        enter_aligned = self.align_enter(enter_deg)
        enter_at = self.deg_to_enter(enter_aligned)

        print("\n\nEnter:", enter_deg,"\n")
        print("Enter aligned:", self.align_enter(enter_deg),"\n")

        if not self.backtrack: self.knots[this_knot]["last_enter"] = enter_at
        print("Last enter:", self.knots[this_knot]["last_enter"])




        # Nach Norden ausrichten:

        north = 45 if enter_aligned % 90 > 0 else 5
        print("Enter at", enter_at)
        if enter_at != 3:
            print("Going north")
            deg = math.degrees(self.odm.heading)

            if enter_aligned < 0:
                self.motors[0].run_direct(duty_cycle_sp = 35)
                self.motors[1].run_direct(duty_cycle_sp = -35)

                while enter_aligned + (math.degrees(self.odm.heading) - deg) < north-10:
                    # print(math.degrees(self.odm.heading))
                    self.odm.update(self.motors[0].position, self.motors[1].position)
            else:
                self.motors[0].run_direct(duty_cycle_sp = -35)
                self.motors[1].run_direct(duty_cycle_sp = 35)

                while enter_aligned + (math.degrees(self.odm.heading) - deg) > north:
                    # print(math.degrees(self.odm.heading))
                    self.odm.update(self.motors[0].position, self.motors[1].position)

            # for m in self.motors: m.stop()
            # print("heading",math.degrees(self.odm.heading))
            # time.sleep(4)



        # Drehen:

        deg = math.degrees(self.odm.heading)
        turned = 0
        lpos = self.motors[0].position


        # 20 Grad nach links

        self.motors[0].run_direct(duty_cycle_sp = -35)
        self.motors[1].run_direct(duty_cycle_sp = 35)

        while turned > -25:
            self.odm.update(self.motors[0].position, self.motors[1].position)
            turned = math.degrees(self.odm.heading) - deg
        # print("turned:  ", turned)
        # print("deg: ", math.degrees(self.odm.heading))

        deg = math.degrees(self.odm.heading)
        print("deg:", deg)
        # for m in self.motors: m.stop()
        # time.sleep(4)
        turned = 0

        # Kanten Scan, falls neuer Knoten

        if not known_knot:


            self.motors[0].run_direct(duty_cycle_sp = 35)
            self.motors[1].run_direct(duty_cycle_sp = -35)

            while turned < 355:



                col = self.cs.bin_data("hhh")
                if sum(col) == 0:
                    self.wait_for_cs(35,-35)

                while sum(col) > self.white-300 and turned < 355:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    turned = math.degrees(self.odm.heading) - deg
                    col = self.cs.bin_data("hhh")
                    if sum(col) == 0:
                        self.wait_for_cs(35,-35)
                    # print("turned: ", turned,  "heading:", math.degrees(self.odm.heading), "lpos:", lpos, "col", sum(col))


                lpos = self.motors[0].position



                if turned < 300:

                    corrected_turn = turned if turned < 20 else turned+10
                    abs_deg = self.turn_to_real_deg(corrected_turn)
                    path_num = self.deg_to_pathnum(abs_deg)

                    print("Path found. Turned:", corrected_turn, "to real:", abs_deg, "-> Adding Path", path_num)

                    if path_num not in self.knots[this_knot]["paths"]:
                        self.knots[this_knot]["not_visited"].append(path_num)
                        self.knots[this_knot]["paths"].append(path_num)


                while self.motors[0].position - lpos < 200 and turned < 355:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    turned = math.degrees(self.odm.heading) - deg


        for m in self.motors: m.stop()
        deg = math.degrees(self.odm.heading)
        turned = 0



        # quit()

        # print("turned:", turned, "Deg after Turn:", math.degrees(self.odm.heading))



        # deg = math.degrees(self.odm.heading)
        # lpos = self.motors[0].position

        if enter_at in self.knots[this_knot]["not_visited"]:
            self.knots[this_knot]["not_visited"].remove(enter_at)

        print("\n\nScan completed.")
        print("Not_visited:", self.knots[this_knot]["not_visited"])


        last_leave_rel = 0
        enter_at_rel = 0



        # Breitensuche Karte


        if not self.backtrack and len(self.knots) > 1:

            last_cord = self.cords[last_knot]
            print("last cord:", last_cord)

            self.add_path(last_cord,last_leave_rel,cord_str,enter_at_rel)



        # Pfad senden:

        if len(self.knots) > 1:
            last_leave_rel = self.knots[last_knot]["paths"].index(self.last_leave) + 1
            enter_at_rel   = self.knots[this_knot]["paths"].index(enter_at) + 1
            self.send_path(self.cords[last_knot], last_leave_rel, cord_str, enter_at_rel)
            time.sleep(3)
            if self.got_message:
                ev3.Sound.tone([(300, 100, 20), (300, 100, 100)])
                self.got_message = False
            else:
                ev3.Sound.tone([(100, 100, 100)])




                                                                                                                         #  3 Fälle relevant:

        if known_knot and not self.backtrack:                                                                            #  1. Erreichen eines bekannten Knotens auf der Suche
            self.backtrack = True                                                                                        #
            print("Setting backtrack to", self.backtrack)                                                                #
        elif known_knot and self.backtrack and len(self.knots[this_knot]["not_visited"]) == 0:                           #  2. Nach Backtracking zu bekannten Knoten zurückkommen
            self.backtrack = True                                                                                        #    und Backtracking fortsetzen, da keine unentdeckten Kanten
            print("Setting backtrack to", self.backtrack)                                                                #
        elif known_knot and self.backtrack and len(self.knots[this_knot]["not_visited"]) > 0:                            #  3. Nach Backtracking zu bekannten Knoten zurückkommen
            self.backtrack = False                                                                                       #    und Backtracking fortsetzen, da keine unentdeckten Kanten
            print("Setting backtrack to", self.backtrack)                                                                #    und Backtracking beenden, weil es noch unentdeckte Kanten gibt
        elif not known_knot and len(self.knots[this_knot]["not_visited"]) == 0:
            self.backtrack = True                                                                                        #    und Backtracking fortsetzen, da keine unentdeckten Kanten
            print("Setting backtrack to", self.backtrack)


        # print("\n\nlen(not_visited):", len(self.knots[this_knot]["not_visited"]), "backtrack:", self.backtrack)
        # print("Knots:")
        for k in self.knots: print(self.knots.index(k), ": ", k)


        # Kanten finden


        print("\nIdentifying Paths....")                                                                                    # Grade aus nicht möglich und/oder schon besucht

        self.motors[0].run_direct(duty_cycle_sp = 35)
        self.motors[1].run_direct(duty_cycle_sp = -35)
        turned = 0
        rel_path = 0

        for path in self.knots[this_knot]["paths"]:

            col = self.cs.bin_data("hhh")
            if sum(col) == 0:
                    self.wait_for_cs(35,-35)

            while sum(col) > self.white-300:
                self.odm.update(self.motors[0].position, self.motors[1].position)
                turned = math.degrees(self.odm.heading) - deg
                col = self.cs.bin_data("hhh")
                if sum(col) == 0:
                    self.wait_for_cs(35,-35)
                # print("turned: ", turned,  "heading:", math.degrees(self.odm.heading),  "lpos:", lpos, "col", sum(col))



            lpos = self.motors[0].position
            corrected_turn = turned if turned < 20 else turned+10
            abs_deg = self.turn_to_real_deg(corrected_turn)
            path_num = self.deg_to_pathnum(abs_deg)
            rel_path += 1

            print("\nchecking ",enter_deg,"(", enter_aligned,") +",corrected_turn,"(",abs_deg,") =", abs_deg," for pathnum. -> Path", path_num)



            if (len(self.knots[this_knot]["not_visited"]) > 0 and path_num == min(self.knots[this_knot]["not_visited"]) and not self.backtrack and not self.navigate) \
                or (self.backtrack and path_num == self.knots[this_knot]["last_enter"] and not self.navigate) \
                or self.navigate and rel_path == self.navigate[0]:                                                        # Unbesuchter Pfad oder soll zurück oder auf Rückweg oder navigiert

                if self.navigate:
                    print("Navigating to", rel_path, "popping", self.navigate.pop(0), "off Navigate List")

                if self.backtrack:
                    print("\nBacktracking. Last enter:", self.knots[this_knot]["last_enter"])
                    print("Backtrack Path:", self.backtrack_path)

                col = self.cs.bin_data("hhh")
                if sum(col) == 0:
                    self.wait_for_cs(35,-35)

                while sum(col) >= self.mid:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    col = self.cs.bin_data("hhh")
                    if sum(col) == 0:
                        self.wait_for_cs(35,-35)

                while sum(col) < self.mid:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    col = self.cs.bin_data("hhh")
                    if sum(col) == 0:
                        self.wait_for_cs(35,-35)

                for m in self.motors: m.stop()
                print("Visiting", path_num)
                # ev3.Sound.speak("Visiting {}".format(path_num))
                if enter_aligned % 90 > 0: abs_deg += 45
                self.odm.reset(abs_deg % 360 )                                                        # Knoten ansteuern und aus not_visited löschen
                self.heading_sp = abs_deg % 360
                self.motors[0].reset()
                self.motors[1].reset()
                x = math.floor(self.odm.pos_x)
                y = math.floor(self.odm.pos_y)
                self.saved_position["x"] = x
                self.saved_position["y"] = y
                self.last_leave = path_num
                if path_num in self.knots[this_knot]["not_visited"]:
                    self.knots[this_knot]["not_visited"].remove(path_num)


                print("reset heading to:", abs_deg % 360, "+", 20, "=", abs_deg % 360 + 10)
                print("heading now:", math.degrees(self.odm.heading))
                print("saved position:", self.saved_position["x"], self.saved_position["y"])
                print("Map:")
                pprint(self.map)
                print("\n\n")

                break

            else:

                while self.motors[0].position - lpos < 200:
                    self.odm.update(self.motors[0].position, self.motors[1].position)
                    turned = math.degrees(self.odm.heading) - deg
                    # print("turned: ", turned, "heading:", math.degrees(self.odm.heading), "lpos:", lpos)








        # print("Not Visited:", self.knots[this_knot]["not_visited"])
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

        if 60 < turn < 150:
            deg = 90
        elif 150 < turn < 230:
            deg = 180
        elif 230 < turn < 330:
            deg = 270

        return deg

    def align_enter(self, enter):
        deg = 0
        enter %= 350

        if -20 < enter < 20 or 340 < enter < 359:
            deg = 0
        elif 20 < enter < 65:
            deg = 45
        elif 65 < enter < 110:
            deg = 90
        elif 110 < enter < 155:
            deg = 135
        elif 155 < enter < 190:
            deg = 180
        elif 190 < enter < 245:
            deg = -135
        elif 245 < enter < 290:
            deg = -90
        elif 290 < enter < 340:
            deg = -45

        return deg




