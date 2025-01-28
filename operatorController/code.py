# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# You must add a gamepad HID device inside your boot.py file
# in order to use this example.
# See this Learn Guide for details:
# https://learn.adafruit.com/customizing-usb-devices-in-circuitpython/hid-devices#custom-hid-devices-3096614-9

import board
import digitalio
import analogio
import usb_hid

from adafruit_hid.hid_gamepad import Gamepad

gp = Gamepad(usb_hid.devices)

class Selector:
    def __init__(self, pins):
        self.buttons = [digitalio.DigitalInOut(pin) for pin in pins]
        for button in self.buttons:
            button.direction = digitalio.Direction.INPUT
            button.pull = digitalio.Pull.UP
        self.button_state = [True for j in range(len(pins))]
        self.selected = 0
    def update(self):
        
        for j, button in enumerate(self.buttons):
            
            pressed = not button.value
            if (pressed and not self.button_state[j]):
                self.selected = j
            self.button_state[j] = pressed


branch_pins = [board.GP0, board.GP1, board.GP2, board.GP3, board.GP4, board.GP5, board.GP6, board.GP7, board.GP8, board.GP9, board.GP10, board.GP11]
level_pins = [board.GP12, board.GP13, board.GP14,board.GP15]
climb_pins = [board.GP16, board.GP17,board.GP18]


#1 = A, 12 = L
branch_selector = Selector(branch_pins)
level_selector = Selector(level_pins)
climb_selector = Selector(climb_pins)


print('start')
gp.press_buttons(1)
gp.release_buttons(1)
while True:
    branch_selector.update ()
    level_selector.update ()
    climb_selector.update ()
    branch_button = branch_selector.selected
    level_button = level_selector.selected
    climb_button = climb_selector.selected

    bitfield = (branch_button & 0xf) + ((level_button & 0x3) << 4) + ((climb_button & 0x3) << 6)
   # send the current state
    for i in range(1,8):
        # 0 is invalid button id

        if bitfield & (1 << (i-1)):
            gp.press_buttons(i)
        else:
            gp.release_buttons(i)

                