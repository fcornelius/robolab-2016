#!/usr/bin/env python3

import ev3dev.ev3 as ev3
# test

def main():
    print("main.py started.")

    motors = [ev3.LargeMotor(port) for port in ('outB', 'outC')]
if __name__ == "__main__":
    main()
