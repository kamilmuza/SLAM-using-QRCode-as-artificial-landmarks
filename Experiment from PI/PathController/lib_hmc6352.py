#!/usr/bin/python
import smbus
import time

# ===========================================================================
# HMC6352 Class
# ===========================================================================
class I2CBase:
 
	def __init__(self, address):
		self.address = address
		self.bus = smbus.SMBus(1)

	def reverseByteOrder(self, data):
		"Reverses the byte order of an int (16-bit) or long (32-bit) value"
		if data is None:
			print "ERROR: Invlid data"
			return 0
		# Courtesy Vishal Sapre
		byteCount = len(hex(data)[2:].replace('L','')[::2])
		val = 0
		for i in range(byteCount):
			val    = (val << 8) | (data & 0xff)
			data >>= 8
		return val

	def write8(self, reg, value):
		"Writes an 8-bit value to the specified register/address"
		try:
			self.bus.write_byte_data(self.address, reg, value)
		except IOError, err:
			print "ERROR: failed to write8 to 0x%02X" % self.address

	def readU16(self, reg):
		"Reads an unsigned 16-bit value from the I2C device"
		try:
			result = self.bus.read_word_data(self.address,reg)
			return result
		except IOError, err:
			print "ERROR: failed to readU16 from 0x%02X" % self.address
			return 0

class HMC6352:
	i2c = None
  
	# HMC6352 Address
	address = 0x21
	
	# Commands
	CMD_READ_DATA = 0x41
	CMD_ENTER_USER_CALIBRATION = 0x45
	CMD_EXIT_USER_CALIBRATION = 0x4C

	# Constructor
	def __init__(self):
		self.i2c = I2CBase(self.address)
	
	def userCalibration(self):
		"Write 0x45 for calibration and write 0x4C for leave after 20s"
		self.i2c.write8(0x0, self.CMD_ENTER_USER_CALIBRATION)
		time.sleep(20)
		self.i2c.write8(0x0, self.CMD_EXIT_USER_CALIBRATION)
		
	def readData(self):
		"Read the heading data by write a 0x41 first"
		self.i2c.write8(0x0, self.CMD_READ_DATA)
		# Wait 6 ms
		time.sleep(0.006)
		# Read 2 bytes
		value = self.i2c.readU16(0x0)
		# Reverse the data byte order
		value = self.i2c.reverseByteOrder(value)
		# Convert to 360.0 range from the raw integer value
		value = float('%0.1f'%value)/10

		return value
