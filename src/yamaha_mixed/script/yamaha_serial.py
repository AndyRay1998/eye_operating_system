#!/usr/bin/env python3

# Reference: https://www.cnblogs.com/dongxiaodong/p/9992083.html
# Reference: ../RCX2 programming manual.pdf
'''
We need to share parameter serial.Serial(), and contain different functions /
for different motion, so I decide to implement OOP-style
'''
import sys
import serial

class command():
    def __init__(self, port, freq=9600, timeout=0.5):
        self.port = port
        self.freq = freq
        self.timeout = timeout
        # empty serial object
        self.ser = serial.Serial()


    def connect(self):
        '''
        connect to yamaha through serial property
        '''
        try:
            self.ser=serial.Serial(self.port, self.freq, timeout=self.timeout)
            # green output
            print("\033[1;32;42m[INFO] Serial connected\033[0m")

        except:
            #red output
            print("\033[1;31;45m[WARN] Yamaha connection failed!\033[0m")
            sys.exit()


    def where(self):
        '''
        request current location of six axis in a tuple
        '''
        self.ser.write("@?WHERE[cr/lf]")


    def jog(self, axis, direction):
        '''
        request jog motion of specific axis in main group
        @param:
            axis: 1 - 6
            direction: + / -
        '''
        # axis list
        a = ("X", "Y", "Z", "R", "A", "B")
        self.ser.write(f"@?JOG {a[axis-1]}{direction}[cr/lf]")


    def servo_state(self):
        '''
        request servo status
        response: OFF,xxxxxxxx [cr/lf] or ON,xxxxxxxx[cr/lf]
        xxxxxxxx: axis 8 to 1
                  0: Mechanical break ON + dynamic break ON
                  1: Servo ON
                  2: Mechanical break OFF + dynamic break OFF
                  9: Not applicable
        '''
        self.ser.write(f"@?SERVO[cr/lf]")


    def servo_set(self, status, *axis):
        '''
        servo status setup
        @params:
            status: ON -> Turns the servo ON. If no axis is specified,
                          the motor power supply also turns ON.
                    OFF -> Turns the servo OFF and applies the dynamic brake.
                          Axes equipped with brakes are all locked by the brake.
                          If no axis is specified, the motor power supply also turns OFF.
                    FREE -> Turns the servo OFF and releases the dynamic brake.
                            The brakes are released at all axes with brakes.
                            If no axis is specified, the motor power supply also turns OFF.
                    PWR -> Turns only the motor power supply ON.
            *axis: random number of axis; main group -> 1 to 6; is a tuple
        e.g. SERVO ON(3)
        '''
        self.ser.write(f"@SERVO {status}{axis}[cr/lf]")


    def move(self, loc, mode='P', speed=100):
        '''
        command movement to absolute location
        loc unit: pulse
        speed unit: %
        e.g.: MOVE P,10000 10000 5000 20 500 700,S=20
        @params:
            mode: P -> point to point;
                  L -> linear interpolation;
                  C -> circular interpolation.
            loc: array-like data specifying target location
        '''
        self.ser.write(f"@MOVE {mode},{loc[0]} {loc[1]} {loc[2]} {loc[3]} {loc[4]} {loc[5]}[cr/lf],S={speed}")


    def movei(self, incre, speed=100):
        '''
        Performs relative movement of all robot axes
        incer unit: pulse
        speed unit: %
        e.g. see 'move'
        @params:
            incre: array-like data specifying target increment
        '''
        self.ser.write(f"@MOVEI P,{incre[0]} {incre[1]} {incre[2]} {incre[3]} {incre[4]} {incre[5]}[cr/lf],S={speed}")


    def interrupt(self):
        '''
        Interrupts execution of the current command.
        '''
        self.ser.write(f"^C")


    def ser_close(self):
        '''
        close serial transmission
        '''
        self.ser.close()


    def ser_read(self):
        '''
        read a line from serial port
        end with /n
        '''
        # TODO: test if it works
        # may do encode or pre-process before returning
        return(self.ser.readline())
