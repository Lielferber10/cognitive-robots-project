from controller import Supervisor, Motor, PositionSensor, DistanceSensor

import json
import os
import math


TIME_STEP = 32
SMALL_TIME_STEP = 1
SECOND = 30 * TIME_STEP


# Rotates the robot in place 180 degrees counterclockwise
def u_turn():
    rotation_speed = 1.0
    left_front_wheel.setVelocity(-rotation_speed)
    right_back_wheel.setVelocity(rotation_speed)
    left_back_wheel.setVelocity(-rotation_speed)
    right_front_wheel.setVelocity(rotation_speed)
    
    facing_direction = get_facing_direction()
    current_angle = math.atan2(facing_direction[1], facing_direction[0])

    if current_angle < -0.05: # current_angle is negative (with consideration of possible small error)
        current_angle = math.pi - current_angle

    target_angle = (current_angle + math.pi) % (2*math.pi)

    while robot.step(SMALL_TIME_STEP) != -1:
        # Update the robot's current facing direction
        facing_direction = get_facing_direction()
        current_angle = math.atan2(facing_direction[1], facing_direction[0])
        if current_angle < -0.05:
            current_angle = math.pi - current_angle
            
        # Stop rotation if close enough to the target angle

        if abs(target_angle - current_angle) < 0.01:   
            left_front_wheel.setVelocity(0)
            left_back_wheel.setVelocity(0)
            right_front_wheel.setVelocity(0)
            right_back_wheel.setVelocity(0)
            break
     


    
    
    
       
# Rotates the robot in place until its front is parallel to the target_direction vector
def rotate(target_direction):
    facing_direction = get_facing_direction()
    # Calculate the angles
    current_angle = math.atan2(facing_direction[1], facing_direction[0])
    target_angle = math.atan2(target_direction[1], target_direction[0])
    
    # Calculate the rotation needed
    rotation_needed = target_angle - current_angle
    rotation_needed = (rotation_needed + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]
    if abs(target_angle - current_angle) < 0.01:
        return

    if rotation_needed > 0 and abs(math.pi-rotation_needed) > 0.1:
        # Rotate counterclockwise (90 degrees)
        rotation_speed = 1.0
        left_front_wheel.setVelocity(-rotation_speed)
        right_back_wheel.setVelocity(rotation_speed)
        left_back_wheel.setVelocity(-rotation_speed)
        right_front_wheel.setVelocity(rotation_speed)
        
    elif rotation_needed <= 0 and abs(math.pi+rotation_needed) > 0.1:
        # Rotate clockwise (90 degrees)
        rotation_speed = 1.0
        right_front_wheel.setVelocity(-rotation_speed)
        right_back_wheel.setVelocity(-rotation_speed)
        left_back_wheel.setVelocity(rotation_speed)
        left_front_wheel.setVelocity(rotation_speed)
        
     
    else:
        u_turn()
        return
        
        
    # Rotate until the target angle is achieved
    while robot.step(SMALL_TIME_STEP) != -1:
        # Update the robot's current facing direction
        facing_direction = get_facing_direction()
        current_angle = math.atan2(facing_direction[1], facing_direction[0])
    
        # Stop rotation if close enough to the target angle
        if abs(target_angle - current_angle) < 0.01:
            left_front_wheel.setVelocity(0)
            left_back_wheel.setVelocity(0)
            right_front_wheel.setVelocity(0)
            right_back_wheel.setVelocity(0)
            break

    


# Returns the current position of the robot as (x,y) tuple
def get_current_position():
    robot_node = robot.getFromDef("myRobot")
    position_field = robot_node.getField("translation")
    position = position_field.getSFVec3f()
    return (position[0], position[1])



# Returns the Euclidean distance between current_pos and dest_pos
def calculate_distance(current_pos, dest_pos):
    return ((current_pos[0] - dest_pos[0]) ** 2 + (current_pos[1] - dest_pos[1]) ** 2) ** 0.5



# Moves the robot from start position to destination position
def move_to(start, destination):
    target_direction = (destination[0] - start[0], destination[1] - start[1])
    rotate(target_direction)
    
    # Set a straight-line movement speed
    movement_speed = 5.0  # rad/s
    
    # Start moving forward
    right_front_wheel.setVelocity(movement_speed)
    right_back_wheel.setVelocity(movement_speed)
    left_front_wheel.setVelocity(movement_speed)
    left_back_wheel.setVelocity(movement_speed)
    
    
    # Loop to move towards destination and monitor distance sensor
    last_distance = calculate_distance(get_current_position(), destination)
    while robot.step(SMALL_TIME_STEP) != -1:
        current_position = get_current_position()
        
        if calculate_distance(current_position, destination) > last_distance:  # Stop threshold for destination
            right_front_wheel.setVelocity(0)
            right_back_wheel.setVelocity(0)
            left_front_wheel.setVelocity(0)
            left_back_wheel.setVelocity(0)
            # Fixing small errors
            if calculate_distance(current_position, destination) >= 0.05:
                facing_direction = get_facing_direction()
                current_angle = math.atan2(facing_direction[1], facing_direction[0])
                
                # Strafe right: there is an error towards the left at the destination
                if (abs(current_angle) < 0.05 and destination[1] < current_position[1]) or (abs(math.pi/2 - current_angle) < 0.05 and destination[0] > current_position[0]) or (abs(math.pi - current_angle) < 0.05 and destination[1] > current_position[1]) or (abs(-math.pi/2 - current_angle) < 0.05 and destination[0] < current_position[0]):
                    right_front_wheel.setVelocity(-1)
                    right_back_wheel.setVelocity(1)
                    left_front_wheel.setVelocity(1)
                    left_back_wheel.setVelocity(-1)
                    while robot.step(SMALL_TIME_STEP) != -1:
                        current_position = get_current_position()
                        if calculate_distance(current_position, destination) < 0.02:
                            right_front_wheel.setVelocity(0)
                            right_back_wheel.setVelocity(0)
                            left_front_wheel.setVelocity(0)
                            left_back_wheel.setVelocity(0)
                            break
                
                # Strafe left: there is an error towards the right at the destination
                if (abs(current_angle) < 0.05 and destination[1] > current_position[1]) or (abs(math.pi/2 - current_angle) < 0.05 and destination[0] < current_position[0]) or (abs(math.pi - current_angle) < 0.05 and destination[1] < current_position[1]) or (abs(-math.pi/2 - current_angle) < 0.05 and destination[0] > current_position[0]):
                    right_front_wheel.setVelocity(1)
                    right_back_wheel.setVelocity(-1)
                    left_front_wheel.setVelocity(-1)
                    left_back_wheel.setVelocity(1)
                    while robot.step(SMALL_TIME_STEP) != -1:
                        current_position = get_current_position()
                        if calculate_distance(current_position, destination) < 0.02:
                            right_front_wheel.setVelocity(0)
                            right_back_wheel.setVelocity(0)
                            left_front_wheel.setVelocity(0)
                            left_back_wheel.setVelocity(0)
                            break
                                            
                   
            break
            
        else:
            last_distance = calculate_distance(get_current_position(), destination)
            
        
        # Check distance sensor for obstacles
        left = distance_sensor_left.getValue()
        middle = distance_sensor_middle.getValue()
        right = distance_sensor_right.getValue()

        if (left <= 13 or right <= 13) and (left > 13 or right > 13):    # The robot is in front of a cube but need to strafe
            # Strafe right
            if distance_sensor_right.getValue() <= 13:
                right_front_wheel.setVelocity(-1)
                right_back_wheel.setVelocity(1)
                left_front_wheel.setVelocity(1)
                left_back_wheel.setVelocity(-1)
                while robot.step(SMALL_TIME_STEP) != -1:
                    if distance_sensor_right.getValue() > 50:
                        break
                        
            # Strafe left
            elif distance_sensor_left.getValue() <= 13:
                right_front_wheel.setVelocity(1)
                right_back_wheel.setVelocity(-1)
                left_front_wheel.setVelocity(-1)
                left_back_wheel.setVelocity(1)
                while robot.step(SMALL_TIME_STEP) != -1:
                    if distance_sensor_left.getValue() > 50:
                        break
            break
            
        elif middle <= 12 and left > 13 and right > 13:
            # The robot is standing exactly in front of a cube
            break
        
        elif left <= 8 and middle <= 9 and right <= 8:
            # The robot encounters a large obstacle (the open box)
            break
        
    # Stop the wheels after reaching destination or detecting cube/obstacle
    right_front_wheel.setVelocity(0)
    right_back_wheel.setVelocity(0)
    left_front_wheel.setVelocity(0)
    left_back_wheel.setVelocity(0)




# Pick the a cube which is in front of the robot at position position and put it on the surface of the robot
def pick(position):
    global num_of_collected_cubes
    
    # Step 1: Open the gripper fingers
    fingers[0].setPosition(0.025)
    fingers[1].setPosition(0.025)
    robot.step(SECOND) # Wait second
    
    # Step 2: Lower the arm to grasp the cube
    arms[0].setPosition(0.0)
    arms[1].setPosition(-0.95)
    arms[2].setPosition(-1.25)
    arms[3].setPosition(-0.9)
    arms[4].setPosition(0.0)
    robot.step(2*SECOND)

    # Step 3: Close the gripper fingers to grasp the cube
    fingers[0].setPosition(0.017)
    fingers[1].setPosition(0.017)
    robot.step(SECOND)
   

    # Step 4: Lift the arm with the cube
    if num_of_collected_cubes == 0:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.8)
        robot.step(SECOND)
        arms[2].setPosition(0.73)
        robot.step(2*SECOND)
        arms[3].setPosition(0.74)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)

    elif num_of_collected_cubes == 1:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.40)
        robot.step(SECOND)
        arms[2].setPosition(1.03)
        robot.step(2*SECOND)
        arms[3].setPosition(1.05)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        arms[2].setPosition(1.15)
        robot.step(2*SECOND)
        
    elif num_of_collected_cubes == 2:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.31)
        robot.step(SECOND)
        arms[2].setPosition(0.95)
        robot.step(2*SECOND)
        arms[3].setPosition(1.47)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        arms[1].setPosition(0.44)
        robot.step(SECOND)
        arms[3].setPosition(1.42)
        robot.step(2*SECOND)
        
    elif num_of_collected_cubes == 3:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.6)
        robot.step(SECOND)
        arms[2].setPosition(1.8)
        robot.step(2*SECOND)
        arms[3].setPosition(-0.72)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        
    elif num_of_collected_cubes == 4:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.21)
        robot.step(SECOND)
        arms[2].setPosition(2.44)
        robot.step(2*SECOND)
        arms[3].setPosition(-1)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        arms[1].setPosition(0.25)
        robot.step(SECOND)
        arms[2].setPosition(2.4)
        robot.step(2*SECOND)
    
    elif num_of_collected_cubes == 5:
        arms[0].setPosition(0.0)
        arms[3].setPosition(1.4)
        robot.step(2*SECOND)
        arms[2].setPosition(1.1)
        robot.step(2*SECOND) 
        arms[1].setPosition(0.0)
        robot.step(SECOND)
        arms[4].setPosition(0.0)
        
    
    
    
    # Step 5: Open the grippers to release the cube onto the surface
    fingers[0].setPosition(0.025)  # Open fingers to max allowable position
    fingers[1].setPosition(0.025)
    robot.step(SECOND)

    # Step 6: Return the arm to its initial position
    arms[0].setPosition(0.0)
    arms[1].setPosition(0.0)
    arms[2].setPosition(0.0)
    arms[3].setPosition(0.0)
    arms[4].setPosition(0.0)
    robot.step(2*SECOND)


    num_of_collected_cubes += 1
    
    
    # Move to the position of the collected cube
    current_position = get_current_position()
    move_to(current_position, position)






# Place the a cube which is on the surface of the robot and put it on the open_box positioned at destination
def place(destination):
    global num_of_collected_cubes
    
    # Step 1: Open the gripper fingers
    fingers[0].setPosition(0.025)
    fingers[1].setPosition(0.025)
    robot.step(SECOND) # Wait second
    
    # Step 2: Lower the arm to grasp the cube
    if num_of_collected_cubes == 6:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.4)
        arms[2].setPosition(0.8)
        arms[3].setPosition(1.5)
        arms[4].setPosition(0.0)
        robot.step(SECOND)

    elif num_of_collected_cubes == 5:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.55)
        arms[2].setPosition(0.65)
        arms[3].setPosition(1.33)
        arms[4].setPosition(0.0)
        robot.step(2*SECOND)
        
    elif num_of_collected_cubes == 4:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.75)
        arms[2].setPosition(0.57)
        arms[3].setPosition(0.95)
        arms[4].setPosition(0.0)   
        robot.step(2*SECOND) 
    
    elif num_of_collected_cubes == 3:
        arms[0].setPosition(0.01)
        arms[1].setPosition(0.7)
        arms[2].setPosition(0.9)
        arms[3].setPosition(1.6)
        arms[4].setPosition(0.0)
        robot.step(2*SECOND)
    
    elif num_of_collected_cubes == 2:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.8)
        arms[2].setPosition(0.6)
        arms[3].setPosition(1.35)
        arms[4].setPosition(0.0)
        robot.step(2*SECOND)
        
    elif num_of_collected_cubes == 1:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.9)
        arms[2].setPosition(0.73)
        arms[3].setPosition(0.7)
        arms[4].setPosition(0.0)
        robot.step(2*SECOND)

    
    
    

    # Step 3: Close the gripper fingers to grasp the cube
    fingers[0].setPosition(0.017)
    fingers[1].setPosition(0.017)
    robot.step(SECOND)
   

    # Step 4: Lift the arm with the cube
    if num_of_collected_cubes == 6:
        arms[0].setPosition(0.0)
        arms[1].setPosition(-0.32)
        robot.step(SECOND)
        arms[2].setPosition(-1.15)
        robot.step(2*SECOND)
        arms[3].setPosition(-0.55)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        
    elif num_of_collected_cubes == 5:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.3)
        robot.step(SECOND)
        arms[2].setPosition(-1.6)
        robot.step(2*SECOND)
        arms[3].setPosition(-0.6)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        
    elif num_of_collected_cubes == 4:
        arms[0].setPosition(0.0)
        arms[2].setPosition(-2.6)
        robot.step(2*SECOND)
        arms[1].setPosition(1.1)
        robot.step(SECOND)
        arms[3].setPosition(-0.2)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)    
    
    elif num_of_collected_cubes == 3:
        arms[0].setPosition(0.0)
        arms[1].setPosition(-0.25)
        robot.step(SECOND)
        arms[2].setPosition(-1.2)
        robot.step(2*SECOND)
        arms[3].setPosition(-0.35)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
    
    elif num_of_collected_cubes == 2:
        arms[1].setPosition(0.75)
        robot.step(SECOND) 
        arms[0].setPosition(0.35)
        robot.step(SECOND)
        arms[1].setPosition(0.3)
        robot.step(SECOND)
        arms[0].setPosition(0.0)
        robot.step(SECOND)
        arms[2].setPosition(-1.8)
        robot.step(2*SECOND)
        arms[3].setPosition(-0.25)
        robot.step(2*SECOND)
        arms[1].setPosition(0.25)
        robot.step(SECOND)
        arms[4].setPosition(0.0)
        
    elif num_of_collected_cubes == 1:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.35)
        robot.step(SECOND)
        arms[2].setPosition(-1.13)
        robot.step(2*SECOND)
        arms[3].setPosition(-1.4)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        arms[1].setPosition(0.15)
        robot.step(SECOND)
        
        
    
    
    # Step 5: Open the grippers to release the cube onto the surface
    fingers[0].setPosition(0.025)  # Open fingers to max allowable position
    fingers[1].setPosition(0.025)
    robot.step(SECOND)


    num_of_collected_cubes -= 1
    
    # Finish execution
    if num_of_collected_cubes == 0:
        # Move the arms back to their origin
        arms[1].setPosition(0.5)
        robot.step(SECOND)
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.0)
        arms[2].setPosition(0.0)
        arms[3].setPosition(0.0)
        arms[4].setPosition(0.0)
        robot.step(2*SECOND)
        
        # Disable all working sensors of the robot
        distance_sensor_left.disable()
        distance_sensor_middle.disable()
        distance_sensor_right.disable()
        
        finger_right_position_sensor.disable()
        finger_left_position_sensor.disable()
    


# Returns the current facing direction of the robot as an (x,y) vector
def get_facing_direction():
    robot_node = robot.getFromDef("myRobot")
    orientation = robot_node.getOrientation()
    facing_direction = (orientation[0], orientation[3], orientation[6])
    return facing_direction
    
    
    
    
    
if __name__ == "__main__":
    
    cube_nodes = []
    open_box_node = None
    
    # Load the input file
    controller_dir = os.path.dirname(__file__)
    input_file_path = os.path.join(controller_dir, 'input.json')
    
    with open(input_file_path) as f:
        input = json.load(f)
        
    # Access predefined settings
    floor_size = input.get("floor_size", {})
    cubes_positions = input.get("cubes_positions", {})
    open_box_position = input.get("open_box_position", {})
    
    
    # Load the plan
    with open("plan.json", "r") as file:
        plan = json.load(file)
        
    
    # create the Robot (as supervisor) instance.
    robot = Supervisor()
    
    root_node = robot.getRoot()
    children_field = root_node.getField('children')
    
    
    # Wheels, arms and sensors setup for YouBot
    right_front_wheel = robot.getDevice('wheel1')
    right_back_wheel = robot.getDevice('wheel3')
    left_front_wheel = robot.getDevice('wheel2')
    left_back_wheel = robot.getDevice('wheel4')
    arms = [robot.getDevice('arm1'), robot.getDevice('arm2'), robot.getDevice('arm3'), robot.getDevice('arm4'), robot.getDevice('arm5')]
    fingers =[robot.getDevice('finger::left'), robot.getDevice('finger::right')]
    
    right_front_wheel.setPosition(float('inf'))
    right_back_wheel.setPosition(float('inf'))
    left_front_wheel.setPosition(float('inf'))
    left_back_wheel.setPosition(float('inf'))
    right_front_wheel.setVelocity(0)
    right_back_wheel.setVelocity(0)
    left_front_wheel.setVelocity(0)
    left_back_wheel.setVelocity(0)
        
    
    # Enable the distance sensors
    distance_sensor_left = robot.getDevice("distance sensor l")
    distance_sensor_left.enable(TIME_STEP)
    distance_sensor_middle = robot.getDevice("distance sensor m")
    distance_sensor_middle.enable(TIME_STEP)
    distance_sensor_right = robot.getDevice("distance sensor r")
    distance_sensor_right.enable(TIME_STEP)
        
    finger_right_position_sensor = robot.getDevice("finger::rightsensor")
    finger_right_position_sensor.enable(TIME_STEP)
    finger_left_position_sensor = robot.getDevice("finger::leftsensor")
    finger_left_position_sensor.enable(TIME_STEP)
    
    
    # Creating the room based on input.json
    for cube_name, (initial_pos, destination_pos) in cubes_positions.items():
        cube_node = children_field.importMFNodeFromString(-1, f'cube {{ name "{cube_name}" translation {initial_pos[0]} {initial_pos[1]} {initial_pos[2]} }}')
        cube_nodes.append(cube_node)
    
    open_box_node = children_field.importMFNodeFromString(-1, f'open_box {{ name "open_box" translation {open_box_position[0]} {open_box_position[1]} {open_box_position[2]} }}')
    
    arena_node = robot.getFromDef("RECTANGLE_ARENA")
    floor_size_field = arena_node.getField("floorSize")
    floor_size_field.setSFVec2f(floor_size)
    
    num_of_collected_cubes = 0
    
    # Main loop - Execute the actions in the plan
    for action in plan:
        command = action[0]
        if command == "move":
            start = tuple(action[1])
            destination = tuple(action[2])
            move_to(start, destination)
            
        elif command == "pick":
            cube_id = action[1]
            position = tuple(action[2])
            pick(position)
            
        elif command == "place":
            cube_id = action[1]
            destination = tuple(action[2])
            place(destination)

