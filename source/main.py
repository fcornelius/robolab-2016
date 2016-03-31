#!/usr/bin/env python3

import time
from PID import *
# from com import *
# test

def main():
    # com = communication('121', 'ydpGX5bMNY', 'felixalex')
    # print("Conecting")
    # com.connect()

    # time.sleep(10)
    pid = PID()
    pid.follow()

if __name__ == "__main__":
    main()
