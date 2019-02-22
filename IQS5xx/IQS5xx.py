import unittest
import time
import logging
logging.basicConfig()
from intelhex import IntelHex
import Adafruit_GPIO.I2C as i2c
from gpiozero import OutputDevice
from gpiozero import DigitalInputDevice

from ctypes import c_uint8, c_uint16, c_uint32, cast, pointer, POINTER
from ctypes import create_string_buffer, Structure
from fcntl import ioctl
import struct
import Adafruit_PureIO.smbus as smbus
from Adafruit_PureIO.smbus import make_i2c_rdwr_data

from IQS5xx_Defs import *


def bytesToHexString(bytes):
    return ''.join('{:02x} '.format(ord(c)) for c in bytes)

IQS5xx_DEFAULT_ADDRESS = 0x74
IQS5xx_MAX_ADDRESS = 0x78
CHECKSUM_DESCRIPTOR_START = 0x83C0
CHECKSUM_DESCRIPTOR_END = 0x83FF
APP_START_ADDRESS = 0x8400
APP_END_ADDRESS = 0xBDFF #inclusive
NV_SETTINGS_START = 0xBE00
NV_SETTINGS_END = 0xBFFF #inclusive
FLASH_PADDING = 0x00
BLOCK_SIZE = 64
APP_SIZE_BLOCKS = (((APP_END_ADDRESS+1) - APP_START_ADDRESS) / BLOCK_SIZE)
NV_SETTINGS_SIZE_BLOCKS = (((NV_SETTINGS_END+1) - NV_SETTINGS_START) / BLOCK_SIZE)

BL_CMD_READ_VERSION = 0x00
BL_CMD_READ_64_BYTES = 0x01
BL_CMD_EXECUTE_APP = 0x02 # Write only, 0 bytes
BL_CMD_RUN_CRC = 0x03

BL_CRC_FAIL = 0x01
BL_CRC_PASS = 0x00
BL_VERSION = 0x0200



def swapEndianess(uint16):
    return ((uint16 & 0xFF) << 8) | ((uint16 & 0xFF00) >> 8)

def writeBytes(self, data):
    self._bus.write_bytes(self._address, bytes(data))
i2c.Device.writeBytes = writeBytes

def readBytes(self, data):
    return self._bus.read_bytes(self._address, data)
i2c.Device.readBytes = readBytes

def writeRawListReadRawList(self, data, readLength):
    self.writeBytes(data)
    # This isn't using a repeat start
    return self.readBytes(readLength)
i2c.Device.writeRawListReadRawList = writeRawListReadRawList

def writeBytes_16BitAddress(self, address, data):
    addressBytes = struct.pack('>H', address)
    dataBytes = bytearray(data)
    bytes = addressBytes + dataBytes
    self.writeBytes(bytes)
i2c.Device.writeBytes_16BitAddress = writeBytes_16BitAddress

def readBytes_16BitAddress(self, address, length):
    assert self._bus._device is not None, 'Bus must be opened before operations are made against it!'
    # Build ctypes values to marshall between ioctl and Python.
    reg = c_uint16(swapEndianess(address))
    result = create_string_buffer(length)
    # Build ioctl request.
    request = make_i2c_rdwr_data([
        (self._address, 0, 2, cast(pointer(reg), POINTER(c_uint8))),            # Write cmd register.
        (self._address, smbus.I2C_M_RD, length, cast(result, POINTER(c_uint8)))   # Read data.
    ])
    # Make ioctl call and return result data.
    ioctl(self._bus._device.fileno(), smbus.I2C_RDWR, request)
    return bytearray(result.raw)  # Use .raw instead of .value which will stop at a null byte!
i2c.Device.readBytes_16BitAddress = readBytes_16BitAddress

def readByte_16BitAddress(self, address):
    result = self.readBytes_16BitAddress(address, 1)
    result = struct.unpack('>B', result)[0]
    return result
i2c.Device.readByte_16BitAddress = readByte_16BitAddress

def writeByte_16BitAddress(self, address, value, mask=0xFF):
    if mask is not 0xFF:
        register = self.readByte_16BitAddress(address)
        register &= ~mask
        register |= (value & mask)
        value = register
    format = '>HB' if (value > 0) else '>Hb'
    bytes = struct.pack(format, address, value)
    self.writeBytes(bytes)
i2c.Device.writeByte_16BitAddress = writeByte_16BitAddress

class IQS5xx(object):
    def __init__(self, resetPin, readyPin, address=IQS5xx_DEFAULT_ADDRESS):
        self.address = address
        self._resetPinNum = resetPin
        self._readyPinNum = readyPin
        self._resetPin = OutputDevice(pin=self._resetPinNum, active_high=False, initial_value=True)
        self._readypin = DigitalInputDevice(pin=self._readyPinNum, active_state=True, pull_up=None)
        self.releaseReset()

    def begin(self):
        self.waitUntilReady()
        self.acknowledgeReset()
        time.sleep(0.01)
        self.acknowledgeReset()
        time.sleep(0.01)
        self.endSession()
        time.sleep(0.020)

    @property
    def address(self):
        return self.__address

    @address.setter
    def address(self, value):
        if (value < IQS5xx_DEFAULT_ADDRESS) or (value > IQS5xx_MAX_ADDRESS):
            raise ValueError("Invalid I2C Address. Use something in the range [%x, %x]" %(IQS5xx_DEFAULT_ADDRESS, IQS5xx_MAX_ADDRESS))
        self.__address = value
        self._device = i2c.get_i2c_device(value)
        self._logger = logging.getLogger('IQS5xx.Address.{0:#0X}'.format(value))

    def setupComplete(self):
        self._device.writeByte_16BitAddress(SystemConfig0_adr, SETUP_COMPLETE, SETUP_COMPLETE)

    def setManualControl(self):
        self._device.writeByte_16BitAddress(SystemConfig0_adr, MANUAL_CONTROL, MANUAL_CONTROL)
        self._device.writeByte_16BitAddress(SystemControl0_adr, 0x00, 0x07) # active mode

    def setTXPinMappings(self, pinList):
        assert isinstance(pinList, list), "TX pinList must be a list of integers"
        assert 0 <= len(pinList) <= 15, "TX pinList must be between 0 and 15 long"
        self._device.writeBytes_16BitAddress(TxMapping_adr, pinList)
        self._device.writeByte_16BitAddress(TotalTx_adr, len(pinList))

    def setRXPinMappings(self, pinList):
        assert isinstance(pinList, list), "RX pinList must be a list of integers"
        assert 0 <= len(pinList) <= 10, "RX pinList must be between 0 and 15 long"
        self._device.writeBytes_16BitAddress(RxMapping_adr, pinList)
        self._device.writeByte_16BitAddress(TotalRx_adr, len(pinList))

    def enableChannel(self, txChannel, rxChannel, enabled):
        assert 0 <= txChannel < 15, "txChannel must be less than 15"
        assert 0 <= rxChannel < 10, "rxChannel must be less than 10"
        registerAddy = ActiveChannels_adr + (txChannel * 2)
        if rxChannel >= 8:
            mask = 1 << (rxChannel - 8)
        else:
            registerAddy += 1
            mask = 1 << rxChannel

        value = mask if enabled else 0x00
        self._device.writeByte_16BitAddress(registerAddy, value)

    def setTXRXChannelCount(self, tx_count, rx_count):
        assert 0 <= txChannel <= 15, "tx_count must be less or equal tp 15"
        assert 0 <= rxChannel <= 10, "rx_count must be less than or equal to 10"
        self._device.writeByte_16BitAddress(TotalTx_adr, txChannel)
        self._device.writeByte_16BitAddress(TotalRx_adr, rxChannel)

    def swapXY(self, swapped):
        value = SWITCH_XY_AXIS if swapped else 0x00
        self._device.writeByte_16BitAddress(XYConfig0_adr, value, SWITCH_XY_AXIS)

    def setAtiGlobalC(self, globalC):
        self._device.writeByte_16BitAddress(GlobalATIC_adr, globalC)

    def setChannel_ATI_C_Adjustment(self, txChannel, rxChannel, adjustment):
        assert 0 <= txChannel < 15, "txChannel must be less than 15"
        assert 0 <= rxChannel < 10, "rxChannel must be less than 10"
        registerAddy = ATICAdjust_adr + (txChannel * 10) + rxChannel
        self._device.writeByte_16BitAddress(registerAddy, adjustment)

    def setTouchMultipliers(self, set, clear):
        self._device.writeByte_16BitAddress(GlobalTouchSet_adr, set)
        self._device.writeByte_16BitAddress(GlobalTouchClear_adr, clear)

    def rxFloat(self, floatWhenInactive):
        value = RX_FLOAT if floatWhenInactive else 0x00
        self._device.writeByte_16BitAddress(HardwareSettingsA_adr, value, RX_FLOAT)

    def runAtiAlgorithm(self):
        self._device.writeByte_16BitAddress(SystemControl0_adr, AUTO_ATI, AUTO_ATI)

    def acknowledgeReset(self):
        self._device.writeByte_16BitAddress(SystemControl0_adr, ACK_RESET, ACK_RESET)

    def atiErrorDetected(self):
        reg = self._device.readByte_16BitAddress(SystemInfo0_adr)
        return bool(reg & ATI_ERROR)

    def reseed(self):
        self._device.writeByte_16BitAddress(SystemControl0_adr, RESEED, RESEED)

    def endSession(self):
        self._device.writeByte_16BitAddress(EndWindow_adr, 0x00)
        time.sleep(0.001)

    def readVersionNumbers(self):
        bytes = self._device.readBytes_16BitAddress(ProductNumber_adr, 6)
        fields = struct.unpack(">HHBB",bytes)
        return {"product":fields[0], "project":fields[1], "major":fields[2], "minor":fields[3]}

    def bootloaderAvailable(self):
        BOOTLOADER_AVAILABLE = 0xA5
        NO_BOOTLOADER = 0xEE
        result = self._device.readByte_16BitAddress(BLStatus_adr)
        # result = ord(result)
        if result == BOOTLOADER_AVAILABLE:
            return True
        elif result == NO_BOOTLOADER:
            return False
        else:
            raise ValueError("Unexpected value returned for bootloader status: {0:#0X}".format(result))

    def holdReset(self, millis=None):
        self._resetPin.on()
        if millis != None:
            time.sleep(millis/1000.0)
            self.releaseReset()

    def releaseReset(self):
        self._resetPin.off()

    def isReady(self):
        return self._readypin.is_active

    def waitUntilReady(self, timeout=None):
        self._readypin.wait_for_active(timeout)

    def updateFirmware(self, hexFilePath, newDeviceAddress=None):
        hexFile = IntelHex(source = hexFilePath)
        hexFile.padding = FLASH_PADDING
        appBinary = hexFile.tobinarray(start=APP_START_ADDRESS, end=NV_SETTINGS_END)
        crcBinary = hexFile.tobinarray(start=CHECKSUM_DESCRIPTOR_START, end=CHECKSUM_DESCRIPTOR_END)

        if newDeviceAddress:
            self._logger.debug("Modifying the last byte in NV settings to change Device I2C Addrress to {0:#0X}".format(newDeviceAddress))
            if (newDeviceAddress < IQS5xx_DEFAULT_ADDRESS) or (newDeviceAddress > IQS5xx_MAX_ADDRESS):
                raise ValueError("Invalid I2C Address. Use something in the range [%x, %x]" %(IQS5xx_DEFAULT_ADDRESS, IQS5xx_MAX_ADDRESS))
            appBinary[-1] = newDeviceAddress

        # Step 1 - Enter Bootloader
        self._logger.debug("Entering Bootloader")
        bootloaderAddress = 0x40 ^ self.address
        bootloaderDevice = i2c.get_i2c_device(bootloaderAddress)
        self.holdReset(100)
        bootloaderEntered = False
        for i in range(10):
            try:
                version = bootloaderDevice.readU16(BL_CMD_READ_VERSION, little_endian=False)
                bootloaderEntered = True
            except:
                pass
        if not bootloaderEntered:
            raise IOError("Timeout while trying to enter bootlaoder")
        self._logger.debug("Bootloader entered successfully")

        # Step 2 - Read and verify the bootloader version number
        self._logger.debug("Reading Bootloader version")
        if version != BL_VERSION:
            raise Exception("Incompatible bootloader version detected: {0:#0X}".format(version))
        self._logger.debug("Bootloader version is compatible: 0x%02X",version)

        # Step 3 - Write the new application firmware and settings
        self._logger.debug("Starting to write Application and NV settings")
        for blockNum in range(APP_SIZE_BLOCKS + NV_SETTINGS_SIZE_BLOCKS):
            blockAddress = APP_START_ADDRESS + (blockNum * BLOCK_SIZE)
            self._logger.debug('Writing 64-byte block {0}/{1} at address {2:#0X}'.format(blockNum+1, APP_SIZE_BLOCKS + NV_SETTINGS_SIZE_BLOCKS ,blockAddress))
            data = bytearray(BLOCK_SIZE + 2)
            data[0] = (blockAddress >> 8) & 0xFF
            data[1] = blockAddress & 0xFF
            data[2:] = appBinary[blockNum*BLOCK_SIZE : (blockNum+1)*BLOCK_SIZE]
            bootloaderDevice.writeBytes(data)
            time.sleep(.010) # give the device time to write to flash

        # Step 4 - Write the checksum descriptor section
        self._logger.debug("Writing CRC section")
        blockAddress = CHECKSUM_DESCRIPTOR_START
        data = bytearray(BLOCK_SIZE + 2)
        data[0] = (blockAddress >> 8) & 0xFF
        data[1] = blockAddress & 0xFF
        data[2:] = crcBinary[0:]
        bootloaderDevice.writeBytes(data)
        time.sleep(0.010) # give the device time to write to flash

        # Step 5 - Perform CRC and read back settins section
        time.sleep(0.1)
        self._logger.debug("Performing CRC calculation")
        bootloaderDevice.writeRaw8(BL_CMD_RUN_CRC)
        time.sleep(0.2)
        crcStatus = bootloaderDevice.readRaw8()
        if crcStatus != BL_CRC_PASS:
            raise Exception("CRC Failure")
        self._logger.debug("CRC Success")

        self._logger.debug("Reading back NV settings and comparing")
        for blockNum in range(NV_SETTINGS_SIZE_BLOCKS):
            blockAddress = NV_SETTINGS_START + (blockNum * BLOCK_SIZE)
            self._logger.debug('Reading 64-byte block {0}/{1} at address {2:#0X}'.format(blockNum+1, NV_SETTINGS_SIZE_BLOCKS, blockAddress))
            data = bytearray(3)
            data[0] = BL_CMD_READ_64_BYTES
            data[1] = (blockAddress >> 8) & 0xFF
            data[2] = blockAddress & 0xFF
            reply = bootloaderDevice.writeRawListReadRawList(data, BLOCK_SIZE)
            expectedReply = appBinary[(APP_SIZE_BLOCKS+blockNum)*BLOCK_SIZE : (APP_SIZE_BLOCKS+blockNum+1)*BLOCK_SIZE].tostring()
            if reply != expectedReply:
                raise Exception("Unexpected values while reading back NV Setting: {0} \nExpected values: {1}".format(bytesToHexString(reply), bytesToHexString(expectedReply)))
        self._logger.debug("NV Settings match expected values")

        # Step 6 - Execute application
        self._logger.debug("Execute Application")
        bootloaderDevice.writeRaw8(BL_CMD_EXECUTE_APP)

        if newDeviceAddress:
            self.address = newDeviceAddress



class TestIQS5xx(unittest.TestCase):
    hexFile = "IQS550_B000_Trackpad_40_15_2_2_BL.HEX"
    possibleAddresses = [0x74, 0x75, 0x76, 0x77]
    desiredAddress = 0x74
    device = None

    def setUp(self):
        if not self.__class__.device:
            self.__class__.device = IQS5xx(17, 27)
        for address in self.__class__.possibleAddresses:
            self.__class__.device.address = address
            self.__class__.device._logger.setLevel(logging.DEBUG)
            try:
                self.__class__.device.waitUntilReady(1)
                self.__class__.device.bootloaderAvailable()
                break
            except:
                if address == self.__class__.possibleAddresses[-1]:
                    raise IOError("Couldn't communicate with the controller")

        if self.__class__.device.address != self.__class__.desiredAddress:
            self.__class__.device.updateFirmware(self.__class__.hexFile, newDeviceAddress=self.__class__.desiredAddress)

    def tearDown(self):
        if self.__class__.device.address != self.__class__.desiredAddress:
            print("Cleaning up by reprogramming the controller to the default address")
            self.__class__.device.updateFirmware(self.__class__.hexFile, newDeviceAddress=self.__class__.desiredAddress)

    def test_bootloaderAvailable(self):
        self.assertTrue(self.__class__.device.bootloaderAvailable())

    # @unittest.skip
    # def test_update(self):
    #     self.__class__.device.updateFirmware(self.__class__.hexFile)
    #
    # @unittest.skip
    # def test_update_and_changeaddress(self):
    #     newAddy = 0x77
    #     self.__class__.device.updateFirmware(self.__class__.hexFile, newDeviceAddress=newAddy)
    #     self.assertEqual(self.__class__.device.address, newAddy)
    #     time.sleep(0.1)
    #     self.assertTrue(self.__class__.device.bootloaderAvailable())


if __name__ == '__main__':
    unittest.main()
