import machine
import time
import neopixel

# Configuration
pin = 48  # GPIO pin where the NeoPixel is connected
num_leds = 1  # Number of LEDs in the strip

# Initialize NeoPixel
np = neopixel.NeoPixel(machine.Pin(pin), num_leds)

def set_color(r, g, b):
    np[0] = (r, g, b)
    np.write()

# Main loop
while True:
    set_color(255, 0, 0)  # Red
    time.sleep(1)
    set_color(0, 255, 0)  # Green
    time.sleep(1)
    set_color(0, 0, 255)  # Blue
    time.sleep(1)