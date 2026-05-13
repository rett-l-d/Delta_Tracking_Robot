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
ROTATION_MAX = 45  # degrees, maximum rotation along reference Z (+/-)

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

    # Preserve selection (optional safety)
    selection = RDK.Selection()

    # Copy reference object
    ref_box.Copy(copy_children=False)
    new_box = ref_box.Parent().Paste()

    # Restore selection
    RDK.setSelection(selection)

    new_box.setVisible(False)
    new_box.setName('Box')

    # IMPORTANT: use reference pose (NOT pasted object pose)
    pose = ref_box.Pose()

    # Random size
    if RANDOMIZE_SIZE:
        scale = random.uniform(
            SIZE_RANGE_FACTOR[0],
            SIZE_RANGE_FACTOR[1]
        )
        new_box.Scale(scale)

    # Random lateral offset
    if RANDOMIZE_POSITION:
        lateral_offset = random.uniform(
            -POSITION_MAX,
            POSITION_MAX
        )
        pose = robomath.RelTool(pose, 0, lateral_offset, 0)

    # Random rotation
    if RANDOMIZE_ROTATION:
        rotation = random.uniform(
            -ROTATION_MAX,
            ROTATION_MAX
        )
        pose = robomath.RelTool(pose, 0, 0, 0, 0, 0, rotation)

    # Apply pose
    new_box.setPose(pose)

    # Attach to conveyor (no Render calls here!)
    new_box.setParentStatic(conveyor_frame)

    new_box.setVisible(True)

    return new_box

def delete_boxes():

    DELETE_DISTANCE = 100

    delete_frame = RDK.Item('delete_box_frame', robolink.ITEM_TYPE_FRAME)

    if not delete_frame.Valid():
        print("delete_box_frame not found")
        return

    for box in box_queue[:]:

        if not box.Valid(True):
            if box in box_queue:
                box_queue.remove(box)
            continue

        # Box pose relative to delete frame
        pose_rel = robomath.invH(delete_frame.PoseAbs()) * box.PoseAbs()

        pos = pose_rel.Pos()

        x = pos[0]

        
        # IMPORTANT:
        # you may need x < -DELETE_DISTANCE
        # depending on conveyor direction

        if x > DELETE_DISTANCE:

           
            if box in box_queue:
                box_queue.remove(box)
                box.Delete()


#while True:
box_queue = conveyor_frame.Childs()
def create_boxes():

    while True:
        time.sleep(0.1)
    
        # Remove boxes removed from the conveyor, deleted or too far away
        childs = conveyor_frame.Childs()
        for box in box_queue:
            if not box.Valid(True) or box not in childs:
                if box in box_queue:
                    box_queue.remove(box)

        if not box_queue:
            box_queue.append(spawn_box())
            

        # Spawn new boxes if space is sufficient
        if box_queue[-1].Valid(True) and not RDK.Collision(ref_box, box_queue[-1]):
            box_queue.append(spawn_box())
        
       # delete_boxes()
