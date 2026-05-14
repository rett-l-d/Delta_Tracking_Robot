import threading

from robodk import robolink
from robodk import robomath
import BoxSpawner_New
import ConveyorReset
import sensorReady
import VisionOpenCV
import time

RDK = robolink.Robolink()
RDK.setSelection([])

CAMERA_OFFSET   = 747.0     # mm upstream from robot pick point
PICK_WINDOW_MM  = 100      # ±mm around pick point where robot can reach
CONVEYOR_SPEED = 300        # mm/s, adjust to your conveyor speed
#Joints_home = [65.566661, 65.566661, 65.566661, 0.000000]
Joints_home = [15.40, 15.40, 15.40, 0.000000]

shared = {
    'encoder_value': 0
}

global_speed = {
    'speed': 0
}

global_pick_X = {'pick_x': 0}

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

# Get the conveyor's reference frame
frames = [x for x in conveyor.Childs() if x.Type() == robolink.ITEM_TYPE_FRAME]
if not frames:
    frames = [RDK.AddFrame(conveyor.Name() + ' Frame', conveyor)]
conveyor_frame = RDK.ItemUserPick(itemtype_or_list=frames) if len(frames) > 1 else frames[0]
if not conveyor_frame.Valid():
    quit()
print(f'Using conveyor frame: {conveyor_frame.Name()}')

# -----------------------------
# GET ROBOT
robot = RDK.Item('ABB IRB 360-1/800 4D', robolink.ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception("No robot found")

tool = robot.getLink(robolink.ITEM_TYPE_TOOL)

print("Using robot:", robot.Name())



# -----------------------------
def get_closest_box():
    boxes = [x for x in RDK.ItemList(robolink.ITEM_TYPE_OBJECT)
             if 'box' in x.Name().lower()]
    if not boxes:
        return None

    # choose closest to sensor
    sensor_pose = sensor.PoseAbs()
    min_dist = 1e9
    closest = None

    for box in boxes:
        pose = box.PoseAbs()
        dist = robomath.distance(sensor_pose.Pos(), pose.Pos())
        if dist < min_dist:
            min_dist = dist
            closest = box

    return closest

# -----------------------------


# -----------------------------
def conveyor_encoder(speed, DT,enc):
    if conveyor.Busy():
        enc +=(speed*DT)
        
    return enc

# -----------------------------

# -----------------------------
def delete_boxes():
    boxes = [x for x in RDK.ItemList(robolink.ITEM_TYPE_OBJECT)
             if 'box' in x.Name().lower()]
    if not boxes:
        return None
    
    for box in boxes:
        box.Delete()
    

def pick_and_place(box):

    box_pose = box.PoseAbs()

    base_frame = RDK.Item('Fanuc M-1iA/0.5SL Base', robolink.ITEM_TYPE_FRAME)
    robot.setPoseFrame(base_frame)

    # Approach
  #  approach_pose =  box_pose * robomath.transl(0,0,-PICK_OFFSET_Z)
 #  robot.MoveJ(approach_pose)
   # robot.MoveL(box_pose)
    robot.MoveJ(Joints_home)
    pose = robot.Pose()
    print("Current Z:", pose.Pos()[2])
    print("Target Z:", pose.Pos()[2] + 20)
    pick_pose = robomath.Mat([[ 1.000, 0.000, 0.000, -36.000 ],
                [ 0.000, -1.000, 0.000, 127.000 ],
                [ 0.000, -0.000, -1.000, -453.000 ],
                [ 0.000, 0.000, 0.000, 1.000 ]])
    robot.MoveL(pick_pose)
    time.sleep(0.5)
    # Attach box to tool
    tool.AttachClosest()

    # Lift
    robot.MoveJ(Joints_home)

    drop_pose = robomath.Mat([[ 1.000, 0.000, 0.000, -36.000 ],
                [ 0.000, -1.000, 0.000, -200.000 ],
                [ 0.000, -0.000, -1.000, -413.000 ],
                [ 0.000, 0.000, 0.000, 1.000 ]])
    robot.MoveJ(drop_pose)
    time.sleep(0.5)


    # Delete the object attached to the tool instead of just detaching it
    for child in tool.Childs():
        if child.Type() == robolink.ITEM_TYPE_OBJECT:
            child.Delete()
    #tool.DetachAll()
    robot.MoveJ(Joints_home)

    # Retreat
 #   robot.MoveL(place_pose)

 
# ─────────────────────────────────────────────
# OBJECT QUEUE
# ─────────────────────────────────────────────

object_queue = []
tracked_objects = []
queue_lock   = threading.Lock()
x_predicted = 0
SCALE = 1.95  # mm per encoder unit
OFFSET = -100
def get_current_x(obj, encoder):
    """Real-time X position of object based on encoder delta since detection."""
    encoder_delta = encoder - obj['encoder_at_detection']
    return obj['x_at_detection'] + (encoder_delta * SCALE) + OFFSET


def get_x_prediction():

    #global x_predicted

    while True:
        time.sleep(0.002)
        with queue_lock:   
            if len(object_queue) == 0:
                #no object detected yet
                continue
    
            obj = object_queue[0]

            global_pick_X['pick_x'] = get_current_x(obj, shared['encoder_value']) 
        
            #print(f"Predicted X={global_pick_X['pick_x']:.1f}mm")


def calculate_speed():
        time_last = 0
        last_encoder = 0
        dt = 0.5
        delta_t = 0.5
        shared['encoder_value'] = 0

       
        while True:
            #sensorReady.sensor_ready()
            start_prox_state = int(RDK.getParam(START_PROX))
            if start_prox_state == 0:
        
                while True:

                        time.sleep(dt)
                        shared['encoder_value'] = conveyor_encoder(CONVEYOR_SPEED, dt, shared['encoder_value'])
                        encoder_now = shared['encoder_value']
                        #print(f"Encoder: {encoder_now:.1f}")
                    # if encoder_value == last_encoder_value:
                    #     continue  # avoid division by zero if encoder hasn't updated
                        time_now = time.time()
                        

                        delta_t = time_now - time_last
                        
                        global_speed['speed'] = ((shared['encoder_value'] - last_encoder) / dt)

                        #print("Speed:", global_speed['speed'])
                        #speed=200
                        last_encoder = shared['encoder_value']
                        time_last = time_now

                        if time_last == 0:
                            time_last = time_now
                            continue
                        if last_encoder == 0:
                            last_encoder = encoder_now
                            continue

# ─────────────────────────────────────────────
# PICK LOOP (background thread)
# ─────────────────────────────────────────────

def pick_loop():

    #global encoder_value
    #global last_encoder_value
    x_future = None
    last_encoder = 0
    encoder_now = 0
    speed = global_speed['speed']  # mm/s, adjust to your conveyor speed
    ROBOT_DELAY = 0.12  # seconds from pick command to actual pick (adjust to your robot)
    time_now = 0
    time_last = 0
    delta_t = 0
    pick_timestamps = []
    
    while True:

        time.sleep(0.1)
        speed = global_speed['speed']
        with queue_lock:   
            if len(object_queue) == 0:
                #no object detected yet
                continue
       
            obj = object_queue[0]
            #x_now = global_pick_X['pick_x']
            x_now = get_current_x(obj, shared['encoder_value'])
        
        #x_future = x_now + speed * ROBOT_DELAY 
        
        x_future = x_now
        if x_future == 0:
                continue

        
        

        #print(f"Predicted X_Robot={x_now:.1f}mm")
       
       
        
       # x_future = x_now
        # Object passed pick window — discard
        if x_future> PICK_WINDOW_MM:
            with queue_lock:
                obj = object_queue.pop(0)
            
           # print(f"  Object passed pick window — discarded" f"  | Future X={x_future:.1f}mm")
            continue
        
       

        # Object in pick window — pick it (MoveJ/MoveL are blocking)
        if -PICK_WINDOW_MM <= x_future <= PICK_WINDOW_MM:
            obj = object_queue.pop(0)

            x_pick = x_future
            y_pick = obj['y_at_detection']

            #print(f"  PICKING at X={x_pick:.1f}mm  Y={y_pick:.1f}mm")
            start_move = time.time()
            pick_pose = robomath.Mat([[ 1.000, 0.000, 0.000, x_pick ],
                [ 0.000, -1.000, 0.000, y_pick ],
                [ 0.000, -0.000, -1.000, -1155.000 ],
                [ 0.000, 0.000, 0.000, 1.000 ]])
            robot.MoveL(pick_pose)
            actual_delay = time.time() - start_move
            ROBOT_DELAY = 0.5 * ROBOT_DELAY + 0.1 * actual_delay
            tool.AttachClosest('Box')
            #time.sleep(0.5)
            # Attach box to tool
            now = time.time()

            pick_timestamps.append(now)

            # keep only picks from last 60 seconds
            pick_timestamps[:] = [
                t for t in pick_timestamps
                if now - t <= 5
            ]

            parts_per_min = len(pick_timestamps)*12

            #print(f"PPM: {parts_per_min*12}")
            RDK.ShowMessage(
            f"Parts Per Minute: {parts_per_min:.1f}",
            False
        )

            # Lift
            robot.MoveL(Joints_home)

            drop_pose = robomath.Mat([[ 1.000, 0.000, 0.000, 0.000 ],
                        [ 0.000, -1.000, 0.000, -220.000 ],
                        [ 0.000, -0.000, -1.000, -1155.000 ],
                        [ 0.000, 0.000, 0.000, 1.000 ]])
            robot.MoveL(drop_pose)
          #  time.sleep(0.5)




            #Delete the object attached to the tool instead of just detaching it
            for child in tool.Childs():
                if child.Type() == robolink.ITEM_TYPE_OBJECT:
                    if "box" in child.Name().lower():
                        child.Delete()

            

        #Delete the object attached to the tool instead of just detaching it
        for child in tool.Childs():
            if child.Type() == robolink.ITEM_TYPE_OBJECT:
                if "box" in child.Name().lower():
                    child.Delete()
        with queue_lock:   
            if len(object_queue) == 0:
                #no object detected yet
                robot.MoveL(Joints_home)
                continue
        
        drop_pose = robomath.Mat([[ 1.000, 0.000, 0.000, 0.000 ],
                    [ 0.000, -1.000, 0.000, -220.000 ],
                    [ 0.000, -0.000, -1.000, -1100 ],
                    [ 0.000, 0.000, 0.000, 1.000 ]])
        robot.MoveL(drop_pose)


def pick_non_loop():

    global encoder_value
    global last_encoder_value
    x_future = None
    speed = global_speed['speed']  # mm/s, adjust to your conveyor speed
    ROBOT_DELAY = 0.5  # seconds from pick command to actual pick (adjust to your robot)

  

    # time.sleep(0.005)
    #print(f"Encoder in pick loop: {encoder_value:.1f}")
    #if encoder_value == last_encoder_value:
     #   return  # avoid division by zero if encoder hasn't updated
    #speed = (encoder_value - last_encoder_value) / dt
    speed=200

    with queue_lock:   
        if len(object_queue) == 0:
            #no object detected yet
            return
    
        obj = object_queue[0]

    x_now = get_current_x(obj, shared['encoder_value'])
    x_future = x_now + speed * ROBOT_DELAY
    #x_future = x_now
    print(f"  Future X position: {x_future}")

    # Object passed pick window — discard
    if x_future> PICK_WINDOW_MM:
        object_queue.pop(0)
        print(f"  Object passed pick window — discarded")
        return
    
    

    # Object in pick window — pick it (MoveJ/MoveL are blocking)
    if abs(x_future) <= PICK_WINDOW_MM:
        obj = object_queue.pop(0)

        x_pick = x_future
        y_pick = obj['y_at_detection']

        print(f"  PICKING at X={x_pick:.1f}mm  Y={y_pick:.1f}mm")

        pick_pose = robomath.Mat([[ 1.000, 0.000, 0.000, x_pick ],
            [ 0.000, -1.000, 0.000, y_pick ],
            [ 0.000, -0.000, -1.000, -1155.000 ],
            [ 0.000, 0.000, 0.000, 1.000 ]])
        robot.MoveJ(pick_pose)
        tool.AttachClosest('Box')
        #time.sleep(0.5)
        # Attach box to tool
        

        # Lift
        robot.MoveJ(Joints_home)

        drop_pose = robomath.Mat([[ 1.000, 0.000, 0.000, 0.000 ],
                    [ 0.000, -1.000, 0.000, -220.000 ],
                    [ 0.000, -0.000, -1.000, -1155 ],
                    [ 0.000, 0.000, 0.000, 1.000 ]])
        robot.MoveJ(drop_pose)
        #  time.sleep(0.5)

    
        


        #Delete the object attached to the tool instead of just detaching it
        for child in tool.Childs():
            if child.Type() == robolink.ITEM_TYPE_OBJECT:
                if "box" in child.Name().lower():
                    child.Delete()
        with queue_lock:   
            if len(object_queue) == 0:
                #no object detected yet
                robot.MoveJ(Joints_home)
                return
        
        drop_pose = robomath.Mat([[ 1.000, 0.000, 0.000, 0.000 ],
                    [ 0.000, -1.000, 0.000, -220.000 ],
                    [ 0.000, -0.000, -1.000, -1000 ],
                    [ 0.000, 0.000, 0.000, 1.000 ]])
        robot.MoveJ(drop_pose)
        #  time.sleep(0.5)

box_queue = conveyor_frame.Childs()
def delete_boxes_end():

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



# -----------------------------
#ConveyorReset.clear_boxes()
#current_pose = robot.Pose()
#print("Current robot pose:", current_pose)
#time.sleep(200)
delete_boxes()
#sensorReady.sensor_ready()
conveyor.setJoints(conveyor_min)
robot.MoveJ(Joints_home)

 #set robot speed
robot.setSpeed(500, 3000) 
robot.setAcceleration(5000)



shared['encoder_value'] = 0

threading.Thread(target=get_x_prediction, daemon=True).start()
threading.Thread(target=pick_loop, daemon=True).start()
threading.Thread(target=BoxSpawner_New.create_boxes, daemon=True).start()
threading.Thread(target=BoxSpawner_New.create_boxes, daemon=True).start()

threading.Thread(target=calculate_speed, daemon=True).start()
threading.Thread(
    target=VisionOpenCV.getOpenCVresults,
    args=(shared, queue_lock, object_queue),
    daemon=True
).start()

#BoxSpawner_New.create_boxes()
dt = 0.5

conveyor.setSpeed(CONVEYOR_SPEED)


while True:

    time.sleep(dt)
    delete_boxes_end()
    #sensorReady.sensor_ready()

    # spawn every 1 second
    # if time.time() - last_spawn_time >= 2:
    #  BoxSpawner_New.create_boxes()
    #   last_spawn_time = time.time()
    
    #measured conveyor speed is 354.77 mm/s
    # store previous value FIRST


    #print(f"Encoder value: {encoder_value:.1f}")



    #lt = VisionOpenCV.getOpenCVresults(lt, encoder_value, queue_lock, object_queue)
    

    #pick_non_loop()

    with queue_lock:

        if len(object_queue) > 0:
            pass
            #print(object_queue)
            # obj= object_queue[0]
            # x_now = get_current_x(obj, shared['encoder_value'])
            # if abs(x_now) < 5:
            #     pass
            #     print("Would pick here (no stop)") 
            #     print(f"Predicted X={x_now:.1f}mm")
            
    #         #if x_now > 100:

    #          #   object_queue.pop(0)

    #           #  print("Object removed")

            #  print(
            #      f"X at Detection={obj['x_at_detection']:.1f}  "
            #      f"Y at Detection={obj['y_at_detection']:.1f}  "
            #      #f"Encoder Now={encoder_value:.1f}  "
            #      f"Encoder at Detection={obj['encoder_at_detection']:.1f}  "
            #      f"Predicted X={x_now:.1f}"
            #  )

    
    

# print("Sensor:", sensor_state)
# pick_loop(enc)
    sensor_state = int(RDK.getParam(SENSOR_IO))
    start_prox_state = int(RDK.getParam(START_PROX))
    encoder_start = 0
    encoder_end = 0
    if start_prox_state == 1:
        encoder_start = shared['encoder_value']
       # print("ENcoder STart:", encoder_start)

    if sensor_state == 1:
        encoder_end = shared['encoder_value']
        #print("ENcoder End:", encoder_end)
        pass
        delta_encoder = encoder_end - encoder_start
       
        if len(object_queue) > 0:
            pass
            #print(f"Encoder count: {shared['encoder_value']:.1f}")
        # if delta_encoder > 0:
        #     SCALE = 1954.0 / delta_encoder  # mm per encoder unit, adjust based on your setup
        #     print(f"Calculated SCALE: {SCALE:.2f} mm/unit")
        #conveyor.Stop()
        #print(f"EncoderNowat X0={encoder_value:.1f}  ")
        # print('Detected timestamp:', obj['timestamp'])
        # print('Timestamp now:', time.time())
        # print('Predicted X={x_now:.1f}'.format(x_now=x_now))
        # print('Time since detection:', time.time() - obj['timestamp'])
        #encoder_now = shared['encoder_value']
        #print(f"Encoder: {encoder_now:.1f}")
        
        # print(
        #     f"X at Detection={obj['x_at_detection']:.1f}  "
        #     f"Y at Detection={obj['y_at_detection']:.1f}  "
        #     f"Encoder Now={encoder_now:.1f}  "
        #     f"Encoder at Detection={obj['encoder_at_detection']:.1f}  "
        # )
        #break
        #while conveyor.Busy():
            #time.sleep(0.01)

        # box = get_closest_box()
        # if box:
        #     pass
            #pick_and_place(box)

    # conveyor.MoveJ([conveyor.Joints().list()[0] + 500], False)

    elif not conveyor.Busy():
        target = [min(conveyor_max, conveyor.Joints().list()[0] + 99999)]  # this is to avoid lengthy calculations when the target is to far away (as oppose to conveyor.MoveJ(conveyor_max, False))
        conveyor.MoveJ(target, False)
