from machine import SPI, I2C, Pin
import time
from ubinascii import hexlify
import gc
import uos

from arducam_constants import *

# IO Notes
#   ESP32 can put the I2C SCL SDA on any GPIO -> default ports work
#   SPI-> (19: DC)   (23: SCLK) (18: DIN)  (5: CS) -> online docks
#   SPI-> (19: MISO) (23: SCK)  (18: MOSI) (5: CS) -> my translation
#   Other names
#       SCLK: SCK
#       MOSI: SIMO, SDI, DI, DIN, SI, MTSR.
#       MISO: SOMI, SDO, DO, DOUT, SO, MRST.
#       SS: S̅S̅, SSEL, CS, C̅S̅, CE, nSS, /SS, SS#


class Arducam(object):
    default_sck_pin_num = -1
    default_mosi_pin_num = -1
    default_miso_pin_num = -1

    def __init__(self, scl_pin_num=32, sda_pin_num=33, cs_pin_num=5, 
                 sck_pin_num=23, mosi_pin_num=18, miso_pin_num=19, 
                 resolution=OV2640_320x240_JPEG):
        # Set pins
        self.scl_pin_num = scl_pin_num
        self.sda_pin_num = sda_pin_num
        self.cs_pin_num = cs_pin_num
        self.sck_pin_num = sck_pin_num
        self.mosi_pin_num = mosi_pin_num
        self.miso_pin_num = miso_pin_num
        self.cs_pin = None

        self.resolution = resolution

    def init(self):
        self.setup_io()

    def setup_io(self):
        if self.sck_pin_num >= 0 and self.mosi_pin_num >= 0 and self.miso_pin_num >= 0:
            sck_pin = Pin(self.sck_pin_num)
            mosi_pin = Pin(self.mosi_pin_num)
            miso_pin = Pin(self.miso_pin_num)
            self.spi = SPI(1, baudrate=2000000, polarity=0, phase=0, 
                           sck=sck_pin, mosi=mosi_pin, miso=miso_pin)
        else:
            self.spi = SPI(1, baudrate=2000000, polarity=0, phase=0)

        scl_pin = Pin(self.scl_pin_num)
        sda_pin = Pin(self.sda_pin_num)
        self.i2c = I2C(scl=scl_pin, sda=sda_pin, freq=1000000)

        self.spi.init(baudrate=2000000)

        # chip select -- active low
        self.cs_pin = Pin(self.cs_pin_num, Pin.OUT)
        self.cs_pin.value(1)

        addrs = self.i2c.scan()
        print('Arducam: devices detected on i2c:')
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
        self.doc_pins()
        
    def doc_pins(self):
        self.pin_docs = {
            'CS': str(self.cs_pin_num),
            'MOSI': str(self.mosi_pin_num),
            'MISO': str(self.miso_pin_num),
            'SCK': str(self.sck_pin_num),
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

    def spi_write(self, address, value):
        self.cs_pin.value(0)
        modebit = b'\x80'
        d = bytes([address[0] | modebit[0], value[0]])
        self.spi.write(d)
        self.cs_pin.value(1)

    def spi_read(self, address):
        self.cs_pin.value(0)
        maskbits = b'\x7f'
        wbuf = bytes([address[0] & maskbits[0]])
        self.spi.write(wbuf)
        buf = self.spi.read(1)
        self.cs_pin.value(1)

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

    def capture(self, filename, overwrite=True):
        self.spi_write(b'\x04', b'\x01')
        self.spi_write(b'\x04', b'\x02')
        time.sleep_ms(10)

        _ = self.spi_read(b'\x41')
        count = 0

        # read the image from the camera fifo
        while True:
            result = self.spi_read(b'\x41')
            mask = b'\x08'

            if result[0] & mask[0]:
                break

            time.sleep_ms(10)
            count += 1

            if count % 1000 == 0:
                print('Stuck in loop')

        # read the fifo size
        b1 = self.spi_read(b'\x44')
        b2 = self.spi_read(b'\x43')
        b3 = self.spi_read(b'\x42')
        val = b1[0] << 16 | b2[0] << 8 | b3[0] 
        print("ov2640_capture: %d bytes in fifo" % val)
        gc.collect()

        if overwrite == True:
            try:
                uos.remove(filename)
            except OSError:
                pass

        bytebuf = [0, 0]
        picbuf = [b'\x00'] * PICBUFSIZE
        l = 0
        bp = 0

        while bytebuf[0] != b'\xd9' or bytebuf[1] != b'\xff':
            bytebuf[1] = bytebuf[0]

            if bp > len(picbuf) - 1:
                self.appendbuf(filename, picbuf, bp)
                bp = 0
    
            bytebuf[0] = self.spi_read(b'\x3d')
            l += 1
            picbuf[bp] = bytebuf[0]
            bp += 1

        if (bp > 0):
            self.appendbuf(filename, picbuf, bp)

        print("read %d bytes from fifo, camera said %d were available" % (l, val))
        
        return (l)

    def appendbuf(self, filename, picbuf, byte_num):
        try:
            f = open(filename, 'ab')
            c = 1

            for b in picbuf:
                if c > byte_num:
                    break

                c += 1
                f.write(bytes([b[0]]))

            f.close()
        except OSError:
            print("error writing file")

        print("wrote %d bytes from buffer" % byte_num)