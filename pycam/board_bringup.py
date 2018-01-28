from machine import SPI, I2C, Pin

scl_pin_num = 32
sda_pin_num = 33
cs_pin_num = 5 
sck_pin_num = 23
mosi_pin_num = 18
miso_pin_num = 19

scl_pin = Pin(scl_pin_num)
sda_pin = Pin(sda_pin_num)
i2c = I2C(scl=scl_pin, sda=sda_pin, freq=1000000)
addrs = i2c.scan()
print('Arducam: devices detected on on i2c:')
for a in addrs:
    print('0x%x' % a)

sck_pin = Pin(sck_pin_num)
mosi_pin = Pin(mosi_pin_num)
miso_pin = Pin(miso_pin_num)
spi = SPI(1, baudrate=2000000, polarity=0, phase=0, sck=sck_pin, mosi=mosi_pin, miso=miso_pin)
spi.init(baudrate=2000000)
