#!/usr/bin/env python3

# Reference: https://www.cnblogs.com/dongxiaodong/p/9992083.html

import serial


def yamaha_connect(port, freq=9600, timeout=0.5):
    '''
    connect to yamaha through serial property
    '''
    try:
        ser=serial.Serial(port, freq, timeout=timeout)
        # green output
        print("\033[1;31;45m[WARN] Hyperion connected\033[0m")

    except:
        #red output
        print("\033[1;32;42m[WARN] Yamaha connection failed!\033[0m")
        sys.exit()


def yamaha_transmit(command_string):
    '''
    transmit data to yamaha through serial property
    '''
    try:
        result=ser.write(command_string.encode("utf-8")) #data layout
        print("\033[1;31;45m[WARN] Hyperion connected\033[0m")

    except:
        print("\033[1;31;45m[WARN] Yamaha data NOT sent!\033[0m")
        sys.exit()
