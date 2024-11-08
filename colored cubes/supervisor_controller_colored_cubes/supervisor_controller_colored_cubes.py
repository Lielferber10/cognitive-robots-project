from controller import Supervisor, Motor, PositionSensor, DistanceSensor

import json
import os
import math


TIME_STEP = 32
SMALL_TIME_STEP = 1
SECOND = 30 * TIME_STEP

# Define helper functions for moving, picking, and placing

# Perform U-turn (Rotate 180)
def u_turn():
    # Rotate 180 degrees counterclockwise
    rotation_speed = 1.0
    left_front_wheel.setVelocity(-rotation_speed)
    right_back_wheel.setVelocity(rotation_speed)
    left_back_wheel.setVelocity(-rotation_speed)
    right_front_wheel.setVelocity(rotation_speed)
    
    facing_direction = get_facing_direction()
    current_angle = math.atan2(facing_direction[1], facing_direction[0])

    if current_angle < -0.05: # current_angle is negative (with consideration of possible small error)
        current_angle += 2*math.pi

    target_angle = (current_angle + math.pi) % (2*math.pi)


    while robot.step(SMALL_TIME_STEP) != -1:
        # Update the robot's current facing direction
        facing_direction = get_facing_direction()
        current_angle = math.atan2(facing_direction[1], facing_direction[0])
        if current_angle < 0:
            current_angle += 2*math.pi
        
    
        # Stop rotation if close enough to the target angle
        if abs(target_angle - current_angle) < 0.01:   
            left_front_wheel.setVelocity(0)
            left_back_wheel.setVelocity(0)
            right_front_wheel.setVelocity(0)
            right_back_wheel.setVelocity(0)
            break
     


    
    
    
       
# target_direction is pair of integers
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

    


# Get the current position of the robot
def get_current_position():
    robot_node = robot.getFromDef("myRobot")
    position_field = robot_node.getField("translation")
    position = position_field.getSFVec3f()
    return (position[0], position[1])



 # Calculate distance to destination
def calculate_distance(current_pos, dest_pos):
    return ((current_pos[0] - dest_pos[0]) ** 2 + (current_pos[1] - dest_pos[1]) ** 2) ** 0.5



# start and destination are pairs of integers
def move_to(start, destination):
    target_direction = (destination[0] - start[0], destination[1] - start[1])
    rotate(target_direction)
    
    # Set a straight-line movement speed (ensure within max velocity)
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




# position is a pair of integers representing the target location of the cube
def pick(cube_id, position):
    global num_of_collected_cubes
    global cube_nodes
    
    color_field = cube_nodes[cube_id].getField("color")
    cube_color_rgb = color_field.getSFColor()
    
    cube_color = None
    if cube_color_rgb == [1.0, 0.0, 0.0]:
        cube_color = "red"
    elif cube_color_rgb == [0.0, 1.0, 0.0]:
        cube_color = "green"
    elif cube_color_rgb == [0.0, 0.0, 1.0]:
        cube_color = "blue"
        
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
    if cube_color == "red" and num_of_collected_cubes["red"] == 0:
        arms[0].setPosition(0.3)
        arms[1].setPosition(0.7)
        robot.step(SECOND)
        arms[2].setPosition(0.65)
        robot.step(2*SECOND)
        arms[3].setPosition(1.3)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        
    elif cube_color == "red" and num_of_collected_cubes["red"] == 1:
        arms[0].setPosition(0.3)
        arms[1].setPosition(0.37)
        robot.step(SECOND)
        arms[2].setPosition(2.2)
        robot.step(2*SECOND)
        arms[3].setPosition(-0.85)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        
    elif cube_color == "green" and num_of_collected_cubes["green"] == 0:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.47)
        robot.step(SECOND)
        arms[2].setPosition(0.91)
        robot.step(2*SECOND)
        arms[3].setPosition(1.45)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
           
    elif cube_color == "green" and num_of_collected_cubes["green"] == 1:
        arms[0].setPosition(0.0)
        arms[1].setPosition(-0.28)
        robot.step(SECOND)
        arms[2].setPosition(2.45)
        robot.step(2*SECOND)
        arms[3].setPosition(-0.27)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        arms[1].setPosition(-0.5)

    elif cube_color == "blue" and num_of_collected_cubes["blue"] == 0: 
        arms[0].setPosition(-0.3)
        arms[1].setPosition(0.45)
        robot.step(SECOND)
        arms[2].setPosition(1.1)
        robot.step(2*SECOND)
        arms[3].setPosition(0.8)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        
    elif cube_color == "blue" and num_of_collected_cubes["blue"] == 1:
        arms[0].setPosition(-0.3)
        arms[1].setPosition(0.45)
        robot.step(SECOND)
        arms[2].setPosition(2.45)
        robot.step(2*SECOND)
        arms[3].setPosition(-1.35)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
    
    
    num_of_collected_cubes[cube_color] += 1

    
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
    
    
    # Move to the position of the collected cube
    current_position = get_current_position()
    move_to(current_position, position)






# destination is a pair of integers
def place(cube_id, destination):
    global num_of_collected_cubes
    global cube_nodes
    
    color_field = cube_nodes[cube_id].getField("color")
    cube_color_rgb = color_field.getSFColor()
    
    cube_color = None
    if cube_color_rgb == [1.0, 0.0, 0.0]:
        cube_color = "red"
    elif cube_color_rgb == [0.0, 1.0, 0.0]:
        cube_color = "green"
    elif cube_color_rgb == [0.0, 0.0, 1.0]:
        cube_color = "blue"
        
    # Step 1: Open the gripper fingers
    fingers[0].setPosition(0.025)
    fingers[1].setPosition(0.025)
    robot.step(SECOND) # Wait second
    
    # Step 2: Lower the arm to grasp the cube
    if cube_color == "red" and num_of_collected_cubes["red"] == 1:
        arms[0].setPosition(0.31)
        arms[1].setPosition(0.75)
        robot.step(SECOND)
        arms[2].setPosition(0.77)
        robot.step(2*SECOND)
        arms[3].setPosition(1.03)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        
    elif cube_color == "red" and num_of_collected_cubes["red"] == 2:
        arms[0].setPosition(0.32)
        arms[1].setPosition(0.75)
        arms[2].setPosition(0.4)
        arms[3].setPosition(1.3)
        arms[4].setPosition(0.0)   
        robot.step(2*SECOND) 
     
    elif cube_color == "green" and num_of_collected_cubes["green"] == 1:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.55)
        robot.step(SECOND)
        arms[2].setPosition(1.0)
        robot.step(2*SECOND)
        arms[3].setPosition(1.2)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
    
    elif cube_color == "green" and num_of_collected_cubes["green"] == 2:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.5)
        arms[2].setPosition(0.65)
        arms[3].setPosition(1.55)
        arms[4].setPosition(0.0)
        robot.step(2*SECOND)
        
    elif cube_color == "blue" and num_of_collected_cubes["blue"] == 1:
        arms[0].setPosition(-0.3)
        arms[1].setPosition(0.7)
        robot.step(SECOND)
        arms[2].setPosition(0.82)
        robot.step(2*SECOND)
        arms[3].setPosition(1.15)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
               
    elif cube_color == "blue" and num_of_collected_cubes["blue"] == 2:
        arms[0].setPosition(-0.3)
        arms[1].setPosition(0.8)
        robot.step(SECOND)
        arms[2].setPosition(0.2)
        robot.step(2*SECOND)
        arms[3].setPosition(1.55)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        
    

    # Step 3: Close the gripper fingers to grasp the cube
    fingers[0].setPosition(0.017)
    fingers[1].setPosition(0.017)
    robot.step(SECOND)
   

    # Step 4: Lift the arm with the cube
    if num_of_collected_cubes[cube_color] == 2:
        arms[0].setPosition(0.0)
        arms[1].setPosition(-0.32)
        robot.step(SECOND)
        arms[2].setPosition(-1.15)
        robot.step(2*SECOND)
        arms[3].setPosition(-0.55)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        
    elif num_of_collected_cubes[cube_color] == 1:
        arms[0].setPosition(0.0)
        arms[1].setPosition(0.3)
        robot.step(SECOND)
        arms[2].setPosition(-1.6)
        robot.step(2*SECOND)
        arms[3].setPosition(-0.6)
        robot.step(2*SECOND)
        arms[4].setPosition(0.0)
        
    
    num_of_collected_cubes[cube_color] -= 1
    

    # Step 5: Open the grippers to release the cube onto the surface
    fingers[0].setPosition(0.025)  # Open fingers to max allowable position
    fingers[1].setPosition(0.025)
    robot.step(SECOND)

    # Move the arms back to their origin
    arms[1].setPosition(0.5)
    robot.step(SECOND)
    arms[0].setPosition(0.0)
    arms[1].setPosition(0.0)
    arms[2].setPosition(0.0)
    arms[3].setPosition(0.0)
    arms[4].setPosition(0.0)
    robot.step(2*SECOND)
    
    
    
        
        
def move_after_place(last_position, destination): 
    # Set a straight-line movement speed
    movement_speed = -5.0  # rad/s
    
    # Start moving backrward
    right_front_wheel.setVelocity(movement_speed)
    right_back_wheel.setVelocity(movement_speed)
    left_front_wheel.setVelocity(movement_speed)
    left_back_wheel.setVelocity(movement_speed)
    
    
    # Loop to move towards destination and monitor distance sensor
    last_distance = calculate_distance(get_current_position(), last_position)
    while robot.step(SMALL_TIME_STEP) != -1:
        current_position = get_current_position()

        if calculate_distance(current_position, last_position) > last_distance:  # Stop threshold for destination
            right_front_wheel.setVelocity(0)
            right_back_wheel.setVelocity(0)
            left_front_wheel.setVelocity(0)
            left_back_wheel.setVelocity(0)
            break
        else:
            last_distance = calculate_distance(current_position, last_position)
    
    
    if last_position[0] != destination[0] and last_position[1] != destination[1]:
        facing_direction = get_facing_direction()
        current_angle = math.atan2(facing_direction[1], facing_direction[0])
        if current_angle < 0:
            current_angle += 2*math.pi        

        if abs(current_angle) < 0.05 or abs(math.pi - current_angle) < 0.05: 
            # Movment along y-axis
            move_to(last_position, (last_position[0], destination[1]))
            # Movment along x-axis
            move_to((last_position[0], destination[1]), destination)
            
        elif abs(math.pi/2 - current_angle) < 0.05 or abs((3*math.pi)/2 - current_angle) < 0.05:
            # Movment along x-axis
            move_to(last_position, (destination[0], last_position[1]))
            # Movment along y-axis
            move_to((destination[0], last_position[1]), destination)
    
    elif last_position[0] != destination[0] and last_position[1] == destination[1]:
        if last_position[1] < int(floor_size_field[1]//2):
            # Movment along y-axis
            move_to(last_position, (last_position[0], last_position[1]+1))
            # Movment along x-axis
            move_to((last_position[0], last_position[1]+1), (destination[0], last_position[1]+1))   
            # Movment along y-axis
            move_to((destination[0], last_position[1]+1), (destination[0], last_position[1])) 
         
        elif last_position[1] > -int(floor_size_field[1]//2):
            # Movment along y-axis
            move_to(last_position, (last_position[0], last_position[1]-1))
            # Movment along x-axis
            move_to((last_position[0], last_position[1]-1), (destination[0], last_position[1]-1))   
            # Movment along y-axis
            move_to((destination[0], last_position[1]-1), (destination[0], last_position[1]))   
            
    elif last_position[0] == destination[0] and last_position[1] != destination[1]:
        if last_position[0] < int(floor_size_field[0]//2):
            # Movment along x-axis
            move_to(last_position, (last_position[0]+1, last_position[1]))
            # Movment along y-axis
            move_to((last_position[0]+1, last_position[1]), (last_position[0]+1, destination[1]))   
            # Movment along x-axis
            move_to((last_position[0]+1, destination[1]), (last_position[0], destination[1])) 
         
        elif last_position[0] > -int(floor_size_field[0]//2):
            # Movment along x-axis
            move_to(last_position, (last_position[0]-1, last_position[1]))
            # Movment along y-axis
            move_to((last_position[0]-1, last_position[1]), (last_position[0]-1, destination[1]))   
            # Movment along x-axis
            move_to((last_position[0]-1, destination[1]), (last_position[0], destination[1]))        
        
        


# Get the facing direction of the robot
def get_facing_direction():
    robot_node = robot.getFromDef("myRobot")
    orientation = robot_node.getOrientation()
    facing_direction = (orientation[0], orientation[3], orientation[6])
    return facing_direction
    
    
    
    
    
if __name__ == "__main__":
    
    cube_nodes = {}
    open_box_nodes = {}
    
    # Load the input file
    controller_dir = os.path.dirname(__file__)
    input_file_path = os.path.join(controller_dir, 'input1.json')
    
    with open(input_file_path) as f:
        input = json.load(f)
        
    # Access predefined settings
    floor_size = input.get("floor_size", {})
    cubes_positions = input.get("cubes_positions", {})
    open_boxes_positions = input.get("open_boxes_positions", {})
    
    
    # Load the plan
    with open("plan1.json", "r") as file:
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
    for cube_name, (initial_pos, destination_pos, color) in cubes_positions.items():
        cube_color = (0, 0, 0)
        if color == "green":
            cube_color = (0, 1, 0)
        elif color == "red":
            cube_color = (1, 0, 0)
        if color == "blue":
            cube_color = (0, 0, 1)
            
        children_field.importMFNodeFromString(-1, f'cube {{ name "{cube_name}" translation {initial_pos[0]} {initial_pos[1]} {initial_pos[2]} color {cube_color[0]} {cube_color[1]} {cube_color[2]} }}')
    
    for box_color, box_position in open_boxes_positions.items():
        open_box_node = children_field.importMFNodeFromString(-1, f'open_box {{ name "open_box_{box_color}" translation {box_position[0]} {box_position[1]} {box_position[2]} }}')
    
    
    
    # Iterate over all children to find nodes by name
    for i in range(children_field.getCount()):
        child_node = children_field.getMFNode(i)
        if child_node is not None:
            node_type = child_node.getTypeName()
            if node_type in ["CUBE", "OPEN_BOX"]:
                node_name_field = child_node.getField("name")
                node_name = node_name_field.getSFString()
                
                # Check if the node name matches any cube or open box name
                if node_name in cubes_positions.keys():
                    cube_nodes[node_name] = child_node
                    
                elif node_name.startswith("open_box_"):
                    color = node_name.split("_")[-1]
                    if color in open_boxes_positions.keys():
                        open_box_nodes[node_name] = child_node
    
    
    
    arena_node = robot.getFromDef("RECTANGLE_ARENA")
    floor_size_field = arena_node.getField("floorSize")
    floor_size_field.setSFVec2f(floor_size)
    
    num_of_collected_cubes = {"red": 0, "green": 0, "blue": 0}
    
    # Main loop - Execute the actions in the plan
    for i, action in enumerate(plan):
        command = action[0]
        if command == "move":
            if i > 0 and plan[i-1][0] == "place":
                continue
            
            start = tuple(action[1])
            destination = tuple(action[2])
            move_to(start, destination)
            
        elif command == "pick":
            cube_id = action[1]
            position = tuple(action[2])
            pick(cube_id, position)
            
        elif command == "place":
            cube_id = action[1]
            destination = tuple(action[2])
            place(cube_id, destination)
            
            if i < len(plan) - 1 and plan[i+1][0] != "place":
            # Next command exists and it is necessary move
                j=i-1
                while plan[j][0] != "move":
                    j -= 1
                
                   
                previous_command = plan[j]
                next_command = plan[i+1]
                last_position = tuple(previous_command[1])
                destination = tuple(next_command[2])
                move_after_place(last_position, destination)
                
    
    # Finish execution
    if num_of_collected_cubes["red"] == num_of_collected_cubes["green"] == num_of_collected_cubes["blue"] == 0:
        # Disable all working sensors of the robot
        distance_sensor_left.disable()
        distance_sensor_middle.disable()
        distance_sensor_right.disable()
        
        finger_right_position_sensor.disable()
        finger_left_position_sensor.disable()
