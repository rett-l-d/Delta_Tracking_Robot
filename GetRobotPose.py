from robodk import robolink
from robodk import robomath

RDK = robolink.Robolink()
RDK.setSelection([])

# GET ROBOT
robot = RDK.Item('Fanuc M-1iA/0.5SL', robolink.ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception("No robot found")

current_pose = robot.Pose()
print("Current robot pose:", current_pose)