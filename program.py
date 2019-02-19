from AutoTest.testing import Test, TestStatus, TestResult, TestStep, testStep
import IQS5xx
from IQS5xx import IQS5xx
import logging
import time
import click

hexFile = "IQS550_B000_Trackpad_40_15_2_2_BL.HEX"
readyPins = [27, 23, 25, 6]
resetPins = [17, 22, 24, 5]
addresses = [0x74, 0x75, 0x76, 0x77]

controllers = []
for i in range(4):
    controllers.append(IQS5xx(resetPins[i], readyPins[i]))
# device.updateFirmware(hexFile, newDeviceAddress=0x77)

steps = []

def programController(controllerNumber):
    for i,controller in enumerate(controllers):
        if i == controllerNumber:
            continue
        controller.holdReset()

    # try to program the controller at all 4 of possible addresses
    # if this PCBA has never been programmed, it will be at the default 0x74 address
    for address in addresses:
        try:
            controllers[controllerNumber].address = address
            controllers[controllerNumber].updateFirmware(hexFile, newDeviceAddress=addresses[controllerNumber])
            break
        except:
            if address == addresses[-1]:
                raise # re-raise
    return (True, ())

@testStep(len(steps)+1, "Scan Barcode", TestResult("Serial Number"), "Scan the DUT's barcode")
def step(input):
    return (True, input)
steps.append(step)

@testStep(len(steps)+1, "Program Controller #4")
def step(input):
    return programController(3)
steps.append(step)

@testStep(len(steps)+1, "Program Controller #3")
def step(input):
    return programController(2)
steps.append(step)

@testStep(len(steps)+1, "Program Controller #2")
def step(input):
    return programController(1)
steps.append(step)

@testStep(len(steps)+1, "Program Controller #1")
def step(input):
    return programController(0)
steps.append(step)

test = Test(steps, "Everyday Calendar - Front PCB Test", version="0.0.1", identifier="")

click.clear()
while True:
    test.reset()
    test.run()
    click.echo("Next Test. ", nl=False)
