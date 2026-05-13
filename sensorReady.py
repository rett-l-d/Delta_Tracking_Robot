# Type help("robodk.robolink") or help("robodk.robomath") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/robodk.html
# Note: It is not required to keep a copy of this file, your Python script is saved with your RDK project

from robodk import robolink  # RoboDK API
from robodk import robomath  # Robot toolbox
import time

#---------------------------------------
# Settings

# Prefer the Sensor plugin, as it is much more reliable!
# More info here: https://github.com/RoboDK/Plug-In-Interface/tree/master/PluginCollisionSensor
USE_SENSOR_PLUGIN = True
SENSOR_PLUGIN_DLL = 'PluginCollisionSensor.dll'
SENSOR_PLUGIN_NAME = 'Plugin Collision Sensor'

#---------------------------------------
# Enable the sensor plugin
RDK = robolink.Robolink()
RDK.setSelection([])  # Avoid auto-select with ItemUserPick

if USE_SENSOR_PLUGIN:
    status = RDK.Command('PluginLoad', SENSOR_PLUGIN_DLL)
    if status != 'OK':
        RDK.ShowMessage(f'Sensor: Using Python script (failed to load {SENSOR_PLUGIN_NAME})', False)
        USE_SENSOR_PLUGIN = False

#---------------------------------------
# Get the sensor item
sensors = [x for x in RDK.ItemList(robolink.ITEM_TYPE_OBJECT) if 'sensor' in x.Name().lower()]
if not sensors:
    RDK.ShowMessage('No sensor found. Please add a sensor object with "sensor" in its name, i.e. "Laser Sensor"')
    quit()
sensor = RDK.ItemUserPick(itemtype_or_list=sensors) if len(sensors) > 1 else sensors[0]
if not sensor.Valid():
    quit()
SENSOR_IO = sensor.Name()
print(f'Using laser sensor: {SENSOR_IO}')


start_proxs = [x for x in RDK.ItemList(robolink.ITEM_TYPE_OBJECT) if 'start_prox' in x.Name().lower()]
if not start_proxs:
    RDK.ShowMessage('No sensor found. Please add a sensor object with "sensor" in its name, i.e. "Laser Sensor"')
    quit()
start_prox = RDK.ItemUserPick(itemtype_or_list=start_proxs) if len(start_proxs) > 1 else start_proxs[0]
if not start_prox.Valid():
    quit()
START_PROX = start_prox.Name()
print(f'Using laser sensor: {START_PROX}')



#---------------------------------------
# Enable the sensor item
if USE_SENSOR_PLUGIN:
    status = RDK.PluginCommand(SENSOR_PLUGIN_NAME, "Activate", SENSOR_IO)
    if 'OK' == status:
        RDK.ShowMessage(f'Sensor: Using {SENSOR_PLUGIN_NAME}', False)
        quit()

RDK.ShowMessage('Sensor: Using Python script', False)
USE_SENSOR_PLUGIN = False

#---------------------------------------
# Get the conveyor
conveyors = [x for x in RDK.ItemList(robolink.ITEM_TYPE_ROBOT_AXES) if len(x.Joints().tolist()) == 1 and 'conveyor' in x.Name().lower()]
if not conveyors:
    RDK.ShowMessage('No conveyor found. Please add a conveyor with "conveyor" in its name, i.e. "Conveyor Belt"')
    quit()
conveyor = RDK.ItemUserPick(itemtype_or_list=conveyors) if len(conveyors) > 1 else conveyors[0]
if not conveyor.Valid():
    quit()
print(f'Using conveyor: {conveyor.Name()}')

#---------------------------------------
# Get the conveyor's reference frame
frames = [x for x in conveyor.Childs() if x.Type() == robolink.ITEM_TYPE_FRAME]
if not frames:
    frames = [RDK.AddFrame(conveyor.Name() + ' Frame', conveyor)]
conveyor_frame = RDK.ItemUserPick(itemtype_or_list=frames) if len(frames) > 1 else frames[0]
if not conveyor_frame.Valid():
    quit()
print(f'Using conveyor frame: {conveyor_frame.Name()}')

def sensor_ready():

    detection = 0
    childs = conveyor_frame.Childs()
    for box in [x for x in childs if x.Type() == robolink.ITEM_TYPE_OBJECT]:
        if box.Valid(True) and RDK.Collision(sensor, box):
            detection = 1
            break
    
    RDK.setParam(SENSOR_IO, detection)


    start_detection = 0
    childs = conveyor_frame.Childs()
    for box in [x for x in childs if x.Type() == robolink.ITEM_TYPE_OBJECT]:
        if box.Valid(True) and RDK.Collision(start_prox, box):
            start_detection = 1
            break
    
    RDK.setParam(START_PROX, start_detection)
