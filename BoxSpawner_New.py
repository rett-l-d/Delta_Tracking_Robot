# Type help("robodk.robolink") or help("robodk.robomath") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/robodk.html
# Note: It is not required to keep a copy of this file, your Python script is saved with your RDK project

from robodk import robolink  # RoboDK API
from robodk import robomath  # Robot toolbox
import time
import random

RANDOMIZE_SIZE = True
SIZE_RANGE_FACTOR = [1, 1]  # %/100, minimum and maximum scale

RANDOMIZE_ROTATION = True
ROTATION_MAX = 20  # degrees, maximum rotation along reference Z (+/-)

RANDOMIZE_POSITION = True
POSITION_MAX = 100  # mm, maximum lateral offset (+/-)

#---------------------------------------
# Avoid auto-select
RDK = robolink.Robolink()
RDK.setSelection([])




#---------------------------------------
# Get the reference box
boxes = [x for x in RDK.ItemList(robolink.ITEM_TYPE_OBJECT) if 'lid' in x.Name().lower() and 'ref' in x.Name().lower()]
if not boxes:
    RDK.ShowMessage('No reference box found. Please add a reference box with "box" and "ref" in its name, i.e. "Reference Box"')
    quit()
ref_box = RDK.ItemUserPick(itemtype_or_list=boxes) if len(boxes) > 1 else boxes[0]
if not ref_box.Valid():
    quit()
print(f'Using reference box: {ref_box.Name()}')
ref_box.setVisible(False)
ref_box.Parent().setParam('Tree', 'Collapse')

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



def spawn_box():

    # Copy/paste changes the user selection
    RDK.Render(False)
    selection = RDK.Selection()
    ref_box.Copy(copy_children=False)
    new_box = ref_box.Parent().Paste()
    RDK.setSelection(selection)
    new_box.setVisible(False)
    new_box.setName('Box')

    pose = new_box.Pose()
    if RANDOMIZE_SIZE:
        scale = random.uniform(SIZE_RANGE_FACTOR[0], SIZE_RANGE_FACTOR[1])
        new_box.Scale(scale)

    if RANDOMIZE_POSITION:
        lateral_offset = random.uniform(-POSITION_MAX, POSITION_MAX)  # Assuming reference frame has Y crossing the conveyor's width
        pose = robomath.RelTool(pose, 0, lateral_offset, 0)

    if RANDOMIZE_ROTATION:
        rotation = random.uniform(-ROTATION_MAX, ROTATION_MAX)
        pose = robomath.RelTool(pose, 0, 0, 0, 0, 0, rotation)

    new_box.setPose(pose)
    RDK.Render(False)  # Update pose

    new_box.setParentStatic(conveyor_frame)
    new_box.setVisible(True)
    RDK.Render(True)
    return new_box



#while True:
box_queue = conveyor_frame.Childs()
def create_boxes():

    while True:
        time.sleep(2)
    
        # Remove boxes removed from the conveyor, deleted or too far away
        childs = conveyor_frame.Childs()
        for box in box_queue:
            if not box.Valid(True) or box not in childs:
                box_queue.remove(box)

        if not box_queue:
            box_queue.append(spawn_box())
            

        # Spawn new boxes if space is sufficient
        if box_queue[-1].Valid(True) and not RDK.Collision(ref_box, box_queue[-1]):
            box_queue.append(spawn_box())
