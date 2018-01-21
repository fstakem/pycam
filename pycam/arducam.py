from machine import SPI, I2C, Pin
import time
from ubinascii import hexlify

from arducam_constants import *

# Notes
#   ESP32 can put the I2C SCL SDA on any GPIO -> default ports work
#   SPI ??


class Arducam(object):

    def __init__(self, scl_pin_num=32, sda_pin_num=33, cs_pin_num=2, resolution=OV2640_320x240_JPEG):
        # Set pins
        self.scl_pin_num = scl_pin_num
        self.sda_pin_num = sda_pin_num
        self.cs_pin_num = cs_pin_num
        self.cs_pin = None

        self.resolution = resolution

    def init(self):
        self.setup_io()

    def setup_io(self):
        self.doc_pins()

        self.spi = SPI(1, baudrate=80000000, polarity=0, phase=0)
        self.i2c = I2C(scl=Pin(self.scl_pin_num), sda=Pin(self.sda_pin_num), freq=1000000)

        self.spi.init(baudrate=2000000)

        # chip select -- active low
        self.cs_pin = machine.Pin(self.cs_pin_num, Pin.OUT)
        self.cs_pin.on()

        addrs = self.i2c.scan()
        print('Arducam: devices detected on on i2c:')
        for a in addrs:
            print('0x%x' % a)
   
        # select register set
        self.i2c.writeto_mem(SENSORADDR, 0xff, b'\x01')
        # initiate system reset
        self.i2c.writeto_mem(SENSORADDR, 0x12, b'\x80')
       
        # let it come up
        time.sleep_ms(100)

        # jpg init registers
        self.i2c_write_set(SENSORADDR, OV2640_JPEG_INIT)
        self.i2c_write_set(SENSORADDR, OV2640_YUV422)
        self.i2c_write_set(SENSORADDR, OV2640_JPEG)

        # select register set
        self.i2c.writeto_mem(SENSORADDR, 0xff, b'\x01')
        self.i2c.writeto_mem(SENSORADDR, 0x15, b'\x00')

        # select jpg resolution
        self.i2c_write_set(SENSORADDR, self.resolution)

        # register set select
        self.i2c.writeto_mem(SENSORADDR, 0xff, b'\x01')

        current_register = self.is_register_correct()
        correct_type = self.is_correct_type()
        
    def doc_pins(self):
        self.pin_docs = {
            'CS': str(self.cs_pin_num),
            'MOSI': '?',
            'MISO': '?',
            'SCK': '?',
            'GND': 'G',
            'VCC': '3V',
            'SDA': str(self.sda_pin_num),
            'SCL': str(self.scl_pin_num)
        }

    def i2c_write_set(self, addr, set):
        for mem_addr, value in set:
            value = bytes([value])

            if (mem_addr == 0xff and value == b'\xff'):
                return

        self.i2c.writeto_mem(addr, mem_addr, value)

    def spi_write(address, value):
        self.cs_pin.off()
        modebit = b'\x80'
        d = bytes([address[0] | modebit[0], value[0]])
        self.spi.write(d)
        self.cs_pin.on()

    def spi_read(address):
        self.cs_pin.off()
        maskbits = b'\x7f'
        wbuf = bytes([address[0] & maskbits[0]])
        self.spi.write(wbuf)
        buf = self.spi.read(1)
        self.cs_pin.on()

        return (buf)

    def is_correct_type(self):
        part_a = self.i2c.readfrom_mem(SENSORADDR, 0x0a, 1)
        part_b = self.i2c.readfrom_mem(SENSORADDR, 0x0b, 1)
        hex_a = hexlify(part_a)
        hex_b = hexlify(part_b)

        if part_a != b'\x26' or part_b != b'\x42':
            msg = "Arducam: camera is not ov2640, bytes: %s/%s" % (hex_a, hex_b)
            print(msg)
            return False
        else:
            msg = "Arducam: camera is ov2640, bytes: %s/%s" % (hex_a, hex_b)
            print(msg)
            return True

    def is_register_correct(self):
        self.spi_write(b'\x00', b'\x55')
        result = self.spi_read(b'\x00')
        hex_result = hexlify(result)
        print("Arducam: register test return bytes %s" % hex_result)

        if result == b'\x55':
            print("Arducam: register test successful")
            return True
        else:
            print("Arducam: register test failed!")
            return False

    def capture(self):
        pass