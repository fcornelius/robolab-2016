#!/usr/bin/env python3

import time
from PID import *
from com import *


def main():

    pid = PID()
    com = communication('121', 'ydpGX5bMNY', 'felixalex', pid)
    pid.com = com
    print("Conecting")
    com.connect()


    pid.follow()

if __name__ == "__main__":
    main()
