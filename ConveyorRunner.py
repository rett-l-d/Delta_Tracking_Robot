# Type help("robodk.robolink") or help("robodk.robomath") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/robodk.html
# Note: It is not required to keep a copy of this file, your Python script is saved with your RDK project

from robodk import robolink  # RoboDK API
from robodk import robomath  # Robot toolbox
import time

#---------------------------------------
# Avoid auto-select
RDK = robolink.Robolink()
RDK.setSelection([])

#---------------------------------------
# Get the sensor
sensors = [x for x in RDK.ItemList(robolink.ITEM_TYPE_OBJECT) if 'sensor' in x.Name().lower()]
if not sensors:
    RDK.ShowMessage('No sensor found. Please add a sensor object with "sensor" in its name, i.e. "Laser Sensor"')
    quit()
sensor = RDK.ItemUserPick(itemtype_or_list=sensors) if len(sensors) > 1 else sensors[0]
if not sensor.Valid():
    quit()
SENSOR_IO = sensor.Name()
print(f'Using laser sensor: {SENSOR_IO}')

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
conveyor_min, conveyor_max, _ = conveyor.JointLimits()
conveyor_max = conveyor_max.list()[0]

while True:

    time.sleep(0.01)

    sensor_state = RDK.getParam(SENSOR_IO)
    print("Sensor:", sensor_state)
    if sensor_state == 1:
        conveyor.Stop()
        #pass
    elif not conveyor.Busy():
        target = [min(conveyor_max, conveyor.Joints().list()[0] + 99999)]  # this is to avoid lengthy calculations when the target is to far away (as oppose to conveyor.MoveJ(conveyor_max, False))
        conveyor.MoveJ(target, False)
