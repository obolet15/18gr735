# To insert delay
import time
# For serial communication over RS232
import serial
import struct
import numpy as np
# BacNet
import os
import bacpypes as bac
from   bacpypes.consolelogging import ConfigArgumentParser
import gc

# Sets up two ports for serial communication 115200 baudrate
baud   = 115200
hvac   = serial.Serial("/dev/ttyUSB0",baudrate=baud, timeout=3.0, rtscts = 1)
aircon = serial.Serial("/dev/ttyUSB1",baudrate=baud, timeout=3.0, rtscts = 1)

AC_Data   = []#[0.0, 0.0, 0.0, 0.0, 0.0]
HVAC_Data = []#[0.0, 0.0, 0.0, 0.0, 0.0]

# For reading floats send from the Real-Time simulink computers, port, bytes
def readfloatSimu(port,b):
  if   port == 1:
    dataPort  = aircon.read(b)
  elif port == 0:
    dataPort  = hvac.read(b)
  dataPortU = ()
  dataPortD = []
  x = 0
  i = 0
  for k in dataPort:
    #Unpacks data and turns it into 8 bit integers
    dataPortU=dataPortU+struct.unpack('!B',k)
    if (i+1)%4==0:
      #Forms four 8 bit unsigned integer into 1 32 bit float
      dataPortD.append( (float(( (int(dataPortU[i])-1) + ((int(dataPortU[i-1])-1)<<8) + ((int(dataPortU[i-2])-1)<<16) + ((int(dataPortU[i-3])-1)<<24) ))/1000)-100 )
      x += 1
    i += 1
#  print(dataPortD)
  return dataPortD

# For sending floats to the Real-Time simulink computers port,  data
# unused due to simulink computers crashing upon recieving data on RS232.
def writefloat(port,floater):
  integer = np.empty([1,len(floater)])
  integer = floater*1000
  tbs = []	# to be send
  for i in integer:
    bin = '{:032b}'.format(int(i))
    tbs.append(int(bin[ 0: 8],2))
    tbs.append(int(bin[ 8:16],2))
    tbs.append(int(bin[16:24],2))
    tbs.append(int(bin[24:32],2))
  tbs = tuple(tbs)
  rtbs = '' # Ready to be send
  for i in tbs:
	rtbs = rtbs + struct.pack('!B',i)
  if port == 1:
	aircon.write(rtbs)
  elif port == 0:
	hvac.write(rtbs)

class SensorValueProperty(bac.object.Property):
  #creates a subclass of the sensorobject
  def __init__(self, identifier, sensorID):
    bac.object.Property.__init__(self, identifier, bac.primitivedata.Real, default=0.0, optional=True, mutable=False)
    self.sensorID = sensorID

  def ReadProperty(self, obj, arrayIndex=None):
  # reads data from sensor into the right ID
    gc.collect()
    if self.sensorID == 4:
      # for flushing the buffer rs232 creates when receiving multiple messages before doing anything with them
      hvac.flushOutput()
      hvac.flushInput()
      hvac.flush()
      time.sleep(1)
      global HVAC_Data
      HVAC_Data = readfloatSimu(0,20)
      aircon.flushInput()
      aircon.flushOutput()
      aircon.flush()
      time.sleep(1)
      global AC_Data
      AC_Data   = readfloatSimu(1,20)
    return readSensor(self.sensorID)

  def WriteProperty(self, obj, value, arrayIndex=None, priority=None, direct=False):
    raise  bac.errors.ExecutionError(errorClass='property', errorCode='writeAccessDenied')
    # Sends the data to the gateway
class SensorValueObject(bac.object.AnalogValueObject):
  properties = []

  def __init__(self, readPropertyMethod, **kwargs):
    bac.object.AnalogValueObject.__init__(self, **kwargs)
#    self.properties[0].ReadProperty = lambda obj, arrayIndex: readPropertyMethod()

bac.object.register_object_type(SensorValueObject)

# Exchange with your method reading data from serial/RS-232
# NB: Right now, expects sensorID to be edataPortDquivalent to 'objectIdentifier'
# if not the same, you can e.g. add another 'ID' entry in 'sensors', which
# matches your sensor ID -- or you can add an entry in sensors that is a method,
# and then reference that method when creating the 'SensorValueObject'.

def readSensor(sensorID):
  # reads sensor based on ID
  global AC_Data
  global HVAC_Data
  if   sensorID == 4:
    return (HVAC_Data[0])
  elif sensorID == 5:
    return (HVAC_Data[1])
  elif sensorID == 6:
    return (HVAC_Data[2])
  elif sensorID == 7:
    return (HVAC_Data[3])
  elif sensorID == 8:
    return (HVAC_Data[4]*10)
  elif sensorID == 9:
    return (AC_Data[0])
  elif sensorID == 10:
    return (AC_Data[1])
  elif sensorID == 11:
    return (AC_Data[2]*10)
  elif sensorID == 12:
    return (AC_Data[3]*10)
  elif sensorID == 13:
    return (AC_Data[4]*10)

def main():
  # parse the command line arguments
  parser = ConfigArgumentParser()
  # parse the command line arguments
  args = parser.parse_args()
  # make a device object
  this_device = bac.local.device.LocalDeviceObject(ini=args.ini)
  # make a sample application
  this_application = bac.app.BIPSimpleApplication(this_device, args.ini.address)

  sensors = [
    {'name': 'Outside_Temperature', 'id': 4, 'units': 'degreesCelsius'},
    {'name': 'Input_Temperature', 'id': 5, 'units': 'degreesCelsius'},
    {'name': 'Room_Temperature', 'id': 6, 'units': 'degreesCelsius'},
    {'name': 'WatchDog', 'id': 7, 'units': 'degreesCelsius'},
    {'name': 'Fan_speed', 'id': 8, 'units': 'degreesCelsius'},
    {'name': 'Evaporator_Input_Temperature', 'id': 9, 'units': 'degreesCelsius'},
    {'name': 'Condenser_Pressure', 'id': 10, 'units': 'bars'},
    {'name': 'Valve_Opening_Degree', 'id': 11, 'units': 'percent'},
    {'name': 'Compressor_Onnes', 'id': 12, 'units': 'percent'},
    {'name': 'Condenser_Vent_Speed', 'id': 13, 'units': 'percent'}
  ]
  # https://github.com/JoelBender/bacpypes/blob/master/py34/bacpypes/basetypes.py
  # for different units
  #   Create sensor object
  for sensor in sensors:
    sensor_object = SensorValueObject(
      lambda: readSensor(sensor['id']),
      objectIdentifier=('analogValue', sensor['id']),
      objectName=sensor['name'],
      units=sensor['units']
      )
    sensor_object.add_property(SensorValueProperty('presentValue', sensor['id']))
    # Add to application
    this_application.add_object(sensor_object)
  bac.core.run()

if __name__ == "__main__":
  main()
