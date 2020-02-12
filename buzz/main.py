#!/usr/bin/env python3

from serial import Serial
import struct

import helper


def main():
    s = Serial(baudrate=19200)
    s.port = '/dev/ttyS0'

    s.open()
    data = helper.construct_serial_motors_packet(20,0)
    print(data)
    print(data[0])
    s.write(data)


if __name__ == '__main__':
    main()

