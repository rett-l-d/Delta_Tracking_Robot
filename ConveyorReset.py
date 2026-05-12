# Type help("robodk.robolink") or help("robodk.robomath") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/robodk.html
# Note: It is not required to keep a copy of this file, your Python script is saved with your RDK project

from robodk import robolink  # RoboDK API
from robodk import robomath  # Robot toolbox

#---------------------------------------
# Avoid auto-select
RDK = robolink.Robolink()
RDK.setSelection([])

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

#---------------------------------------
# Get the conveyor's reference frame
frames = [x for x in conveyor.Childs() if x.Type() == robolink.ITEM_TYPE_FRAME]
if not frames:
    frames = [RDK.AddFrame(conveyor.Name() + ' Frame', conveyor)]
conveyor_frame = RDK.ItemUserPick(itemtype_or_list=frames) if len(frames) > 1 else frames[0]
if not conveyor_frame.Valid():
    quit()
print(f'Using conveyor frame: {conveyor_frame.Name()}')
conveyor_frame.setVisible(False)
conveyor_frame.setParam("Tree", "Collapse")

#---------------------------------------
# Reset the conveyor
RDK.Render(False)

for child in conveyor_frame.Childs():
    if child.Type() == robolink.ITEM_TYPE_OBJECT and 'box' in child.Name().lower():
        child.Delete()
conveyor.setJoints(conveyor_min)
RDK.Render(True)
