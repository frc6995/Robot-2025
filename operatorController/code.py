# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# You must add a gamepad HID device inside your boot.py file
# in order to use this example.
# See this Learn Guide for details:
# https://learn.adafruit.com/customizing-usb-devices-in-circuitpython/hid-devices#custom-hid-devices-3096614-9

import time

import analogio
import board
import digitalio
# from operatorController.lib import neopixel
import usb_hid
from adafruit_hid.hid_gamepad import Gamepad

gp = Gamepad(usb_hid.devices)

class Selector:
    def __init__(self, pins):
        self.buttons = [digitalio.DigitalInOut(pin) for pin in pins]
        for button in self.buttons:
            button.direction = digitalio.Direction.INPUT
            button.pull = digitalio.Pull.UP
        self.button_state = [False for j in range(len(pins))]
        self.selected = 0
    def update(self):
        
        for j, button in enumerate(self.buttons):
            
            pressed = not button.value
            if (pressed and not self.button_state[j]):
                self.selected = j
            self.button_state[j] = pressed


branch_pins = [board.GP0, board.GP1, board.GP2, board.GP3, board.GP4, board.GP5, board.GP6, board.GP7, board.GP8, board.GP9, board.GP10, board.GP11]
level_pins = [board.GP19, board.GP18, board.GP15,board.GP16]
climb_pins = [board.GP21, board.GP22, board.GP20]

climb_buttons = [digitalio.DigitalInOut(pin) for pin in climb_pins]
for button in climb_buttons:
    button.direction = digitalio.Direction.INPUT
    button.pull = digitalio.Pull.UP
# pixel_pin = board.GP20

# PIXEL_PIN = board.GP20  # pin that the NeoPixel is connected to
# ORDER = neopixel.RGB  # pixel color channel order
# COLOR = (100, 50, 150)  # color to blink
# CLEAR = (0, 0, 0)  # clear (or second color)
# DELAY = 0.25  # blink rate in seconds

# # Create the NeoPixel object
# pixel = neopixel.NeoPixel(PIXEL_PIN, 1, pixel_order=ORDER)

# Loop forever and blink the color

#1 = A, 12 = L
branch_selector = Selector(branch_pins)
level_selector = Selector(level_pins)



print('start')
gp.press_buttons(1)
gp.release_buttons(1)
while True:
    branch_selector.update ()
    level_selector.update ()
    branch_button = branch_selector.selected
    level_button = level_selector.selected

    #8 bits
    # button 1 = bit 0
    # MSB | climb (2 bits) | level (2 bits) | branch (4 bits) | LSB
    bitfield = (branch_button & 0xf) + ((level_button & 0x3) << 4)
   # send the current state
    for i in range(1,7):
        # 0 is invalid button id

        if bitfield & (1 << (i-1)):
            gp.press_buttons(i)
        else:
            gp.release_buttons(i)
    for j in range(3):
        if not climb_buttons[j].value:
            gp.press_buttons(7+j)
        else:
            gp.release_buttons(7+j)

                