import time

from arducam import Arducam


def main():
    camera = Arducam()
    time_diff.last_time = time.ticks_us()

    while True:
        loop(camera)

def loop(camera):
    delta = time_diff()
    #print('Time %d' % (delta / 1000000))

def time_diff():
    current_time = time.ticks_us()
    delta = time_diff.last_time - current_time
    time_diff.last_time = current_time

    return delta

main()