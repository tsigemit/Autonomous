#!/usr/bin/python
#-- Content-Encoding: UTF-8 --

import struct

# ------------------------------------------------------------------------------

def crc16(data):
    """
    Small implementation of the CRC16 hash algorithm
    
    :param data: A string to hash
    :return: Its CRC16, as an integer
    """
    crc = 0xFFFF
    polynome = 0xA001
    parity = 0

    taille_max = len(data)

    cpt_octet = 0
    while cpt_octet < taille_max:
        crc ^= ord(data[cpt_octet])

        cpt_bit = 0
        while cpt_bit < 8:
            parity = crc % 2
            crc >>= 1

            if parity:
                crc ^= polynome

            cpt_bit += 1

        cpt_octet += 1

    print('{} {}'.format('CRC input', data))
    print('{} {}'.format('CRC output', crc))

    return crc


def hexdump(src, length=8):
    """
    Generates a string containing an hexadecimal dump
    """
    result = []
    digits = 4 if isinstance(src, unicode) else 2
    for i in xrange(0, len(src), length):
       s = src[i:i + length]
       hexa = ' '.join(["%0*X" % (digits, ord(x))  for x in s])
       text = ''.join([x if 0x20 <= ord(x) < 0x7F else '.'  for x in s])
       result.append("%04X   %-*s   %s" % (i, length * (digits + 1), hexa, text))
    return '\n'.join(result)


# ------------------------------------------------------------------------------

def normalize_speed(speed):
    """
    Normalizes the given speed between 0 and 60
    
    :return: A (speed: int, forward: bool) tuple
    """
    forward = (speed >= 0)

    if not forward:
        speed = -speed

    if speed > 60:
        speed = 60

    return speed, forward


def construct_tcp_motors_packet(left_speed, right_speed, control=True):
    """
    Constructs a 2-bytes string that can be read by the TCP server on the robot
    """
    packets = []
    for speed, forward in (normalize_speed(left_speed),
                           normalize_speed(right_speed)):
        # Ensure that the value is on 6 bits max
        packet = speed & 0x3F

        # Set up flags
        if forward:
            packet += 64

        if control:
            packet += 128

        packets.append(packet)

    return struct.pack('BB', packets[0], packets[1])


def construct_serial_motors_packet(left_speed, right_speed, control=True):
    """
    Constructs a string that can be written to the robot serial port
    """
    magic = 255
    length = 7

    left_speed, left_forward = normalize_speed(left_speed)
    right_speed, right_forward = normalize_speed(right_speed)

    left_raw_speed = 8 * (left_speed & 0x3F)
    right_raw_speed = 8 * (right_speed & 0x3F)

    # Set up flags
    flag = 8 + 1
    if left_forward:
        flag += 16

    if right_forward:
        flag += 64

    if control:
        flag += 32
        flag += 128


    # Prepare packet content
    prefix = struct.pack('>BB', magic, length)
    content = struct.pack('>HHB', left_raw_speed, right_raw_speed, flag)
    crc = struct.pack('>H', crc16(content))
    return prefix + content + crc
