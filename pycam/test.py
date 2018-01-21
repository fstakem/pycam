import machine
pin19 = machine.Pin(19, machine.Pin.OUT)
while True:
    pin19.value(1)
    utime.sleep_ms(500)
    pin19.value(0)
    utime.sleep_ms(500)