import time
import board
import digitalio
import microcontroller

print("Start")

readPin = digitalio.DigitalInOut(microcontroller.pin.GPIO8)
readPin.switch_to_input(pull=digitalio.Pull.UP)

while True:
    print(readPin.value)

# testPin = digitalio.DigitalInOut(microcontroller.pin.GPIO8)
# testPin.switch_to_output(value=True)

# while True:
#     testPin.value = not testPin.value
#     print(testPin.value)

#     time.sleep(5)
#     pass