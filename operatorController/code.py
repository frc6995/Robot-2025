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
        self.selected = 1
    def getSelected(self):
        return self.selected
    def update(self):
        for j, button in enumerate(self.buttons):
            pressed = not button.value
            if (pressed and not button_state[j]):
                self.selected = j+1
            button_state[j] = pressed


branch_pins = [board.GPIO0, board.GPIO1, board.GPIO2, board.GPIO3, board.GPIO4, board.GPIO5, board.GPIO6, board.GPIO7, board.GPIO10, board.GPIO11]
level_pins = [board.GPIO12, board.GPIO13, board.GPIO14,board.GPIO15]
climb_pins = [board.GPIO16, board.GPIO17,board.GPIO18]
# 1 = A, 12 = L
branch_selector = Selector(branch_pins)
level_selector = Selector(level_pins)
climb_selector = Selector(climb_pins)


print('start')
while True:

    branch_selector.update ()
    level_selector.update ()
    climb_selector.update ()
    branch_button = branch_selector.selected
    level_button = level_selector.selected + len(branch_pins)
    climb_button = climb_selector.selected + len(branch_pins) +len(level_pins)

   # send the current state
    for i in range(1,20):
        # 0 is invalid button id
        if i == 0:
            continue
        if i == branch_button or i == level_button or i == climb_button:
            gp.press_buttons(i)
        else:
            gp.release_buttons(i)

                