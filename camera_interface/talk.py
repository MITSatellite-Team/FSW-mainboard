import time
import board
import digitalio
import microcontroller

cs = digitalio.DigitalInOut(microcontroller.pin.GPIO7)
cs.switch_to_output(value=True)