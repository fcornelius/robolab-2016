from pprint import pprint

class shortest_way:

    def __init__(self):

        self.map = {}
        # self.map = {'0 0': {'0 1': {'in': 3, 'out': 1}},'0 1': {'0 0': {'in': 1, 'out': 3}, '1 2': {'in': 4, 'out': 1}},'1 2': {'0 1': {'in': 1, 'out': 4},'1 5': {'in': 4, 'out': 1},'4 0': {'in': 2, 'out': 3}},'1 5': {'1 2': {'in': 1, 'out': 4}},'4 0': {'1 2': {'in': 3, 'out': 2}, '6 1': {'in': 3, 'out': 1}},'6 1': {'4 0': {'in': 1, 'out': 3}}}
        # self.map = {'0 0': {'0 1': {'in': 3, 'out': 1}},
        #             '0 1': {'0 0': {'in': 1, 'out': 3},'1 2': {'in': 4, 'out': 1}},
        #             '1 2': {'0 1': {'in': 1, 'out': 4},'1 5': {'in': 4, 'out': 1},'4 0': {'in': 2, 'out': 3}},
        #             '1 5': {'1 2': {'in': 1, 'out': 4}},
        #             '4 0': {'1 2': {'in': 3, 'out': 2}, '6 1': {'in': 3, 'out': 1}},
        #             '6 1': {'4 0': {'in': 1, 'out': 3}}}

        self.map = {'0 0': {'0 1': {'in': 3, 'out': 1}},
            '0 1': {'0 0': {'in': 1, 'out': 3},'1 2': {'in': 4, 'out': 1}},
            '1 2': {'0 1': {'in': 1, 'out': 4},'1 5': {'in': 4, 'out': 1}},
            '1 5': {'1 2': {'in': 1, 'out': 4}},
            '4 0': {'6 1': {'in': 3, 'out': 1}},
            '6 1': {'4 0': {'in': 1, 'out': 3}}}

        self.target = None
        self.backtrack_path = ['0 0']

    def set_target(self, target_str):
        self.target = target_str
        print("Set target to", target_str)

        if self.target in self.map:
                this_knot = self.backtrack_path[-1]
                print("calcing shortest way...")
                way = self.shortest_way(this_knot, self.target)
                print("'{}'".format(way))
                if way: print(self.way_to_dirs(way))

    def add_path(self, start_cord, start_leave, end_cord, end_enter):

            if not start_cord in self.map:
                self.map[start_cord] = {}
            if not end_cord in self.map:
                self.map[end_cord] = {}

            self.map[start_cord][end_cord] = { 'out': start_leave, 'in': end_enter}
            self.map[end_cord][start_cord] = { 'out': end_enter, 'in': start_leave}

            if self.target and self.target in self.map:
                this_knot = self.backtrack_path[-1]
                print("calcing shortest way...")
                way = self.shortest_way(this_knot, self.target)
                print("'{}'".format(way))
                if way: print(self.way_to_dirs(way))




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

sw = shortest_way()
while True:
    inp = input("Enter path or target ")

    if inp.split(' ')[0] == 'target':
        sw.set_target(' '.join(inp.split(' ')[1:3]))
    elif len(inp.split(' ')) == 6:
        path = inp.split(' ')
        start_cord = ' '.join(path[0:2])
        start_leave = int(path[2])
        end_cord = ' '.join(path[3:5])
        end_enter = int(path[5])
        sw.add_path(start_cord, start_leave, end_cord, end_enter)
    elif inp == '':
        print("\nMap:")
        pprint(sw.map)
        print("\n")
    else:
        exec(inp)