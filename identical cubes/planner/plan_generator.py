from unified_planning.shortcuts import *
from unified_planning.io import PDDLWriter
from unified_planning.model.metrics import *
from unified_planning.engines import PlanGenerationResultStatus
from unified_planning.shortcuts import Int

import json
import os
from flask import Flask, request, jsonify


up.shortcuts.get_env().credits_stream = None


# room size should be of the form (odd number, odd number)
# cubes_positions is dict of the form {cube_name: ((x-int, y-int), (x'-int, y'-int)), ...}
# open_box_position is a tuple (x-int, y-int)

def get_plan(room_size, cubes_positions):
    # Define types
    Robot = UserType("Robot")
    Cube = UserType("Cube")
    Location = UserType("Location")

    # Define fluents
    At = Fluent("At", BoolType(), r=Robot, l=Location)
    On = Fluent("On", BoolType(), c=Cube, l=Location)
    Holding = Fluent("Holding", BoolType(), r=Robot, c=Cube)
    Adjacent = Fluent("Adjacent", BoolType(), loc1=Location, loc2=Location)
    
    
    # Define actions
    move = InstantaneousAction("move", r=Robot, l_from=Location, l_to=Location)
    l_from = move.parameter("l_from")
    l_to = move.parameter("l_to")
    r = move.parameter("r")
    move.add_precondition(At(r,l_from))
    move.add_precondition(Not(At(r,l_to)))
    move.add_precondition(Adjacent(l_from,l_to))
    move.add_effect(At(r,l_to),True)
    move.add_effect(At(r,l_from),False)
    
    pick = InstantaneousAction("pick", c=Cube, r=Robot, l=Location)
    c = pick.parameter("c")
    r = pick.parameter("r")
    l = pick.parameter("l")
    pick.add_precondition(At(r,l))
    pick.add_precondition(On(c,l))
    pick.add_precondition(Not(Holding(r,c)))
    pick.add_effect(On(c,l),False)
    pick.add_effect(Holding(r,c),True)
    
    place = InstantaneousAction("place", c=Cube, r=Robot, l=Location)
    c = place.parameter("c")
    r = place.parameter("r")
    l = place.parameter("l")
    place.add_precondition(At(r,l))
    place.add_precondition(Not(On(c,l)))
    place.add_precondition(Holding(r,c))
    place.add_effect(On(c,l),True)
    place.add_effect(Holding(r,c),False)

    # Define problem
    problem_Pick_and_Place = Problem("Pick_and_Place")
    
    # Add objects to the problem
    robot = Object("robot", Robot)
    cubes = []
    pos = []
    positions = {}
    for cube_name in cubes_positions.keys():
        cubes.append(Object(cube_name, Cube))
    
    for x in range(-room_size[0]//2 + 1, room_size[0]//2 + 1):
        for y in range(-room_size[1]//2 + 1, room_size[1]//2 + 1):
            cell = Object(f"cell_{x}_{y}", Location)
            pos.append(cell)
            positions[(x,y)] = cell 

    problem_Pick_and_Place.add_objects([robot])
    problem_Pick_and_Place.add_objects(cubes)
    problem_Pick_and_Place.add_objects(pos)
    
    # Add fluents and actions to the problem
    problem_Pick_and_Place.add_fluent(At, default_initial_value=False)
    problem_Pick_and_Place.add_fluent(On, default_initial_value=False)
    problem_Pick_and_Place.add_fluent(Holding, default_initial_value=False)
    problem_Pick_and_Place.add_fluent(Adjacent, default_initial_value=False)
    problem_Pick_and_Place.add_action(move)
    problem_Pick_and_Place.add_action(pick)
    problem_Pick_and_Place.add_action(place)
    
    # Set initial values and goals in the problem
    cube_index=0
    for cube_name, [cube_initial_pos, cube_destination_pos] in cubes_positions.items():
        cube = cubes[cube_index]
        cube_initial_position = positions[tuple(cube_initial_pos)]
        cube_destination_position = positions[tuple(cube_destination_pos)]
        
        problem_Pick_and_Place.set_initial_value(On(cube, cube_initial_position), True)
        problem_Pick_and_Place.add_goal(On(cube, cube_destination_position))
        
        cube_index += 1
        
        
    for x in range(-room_size[0]//2 + 1, room_size[0]//2 + 1):
        for y in range(-room_size[1]//2 + 1, room_size[1]//2 + 1):   
            current_cell = positions[(x,y)]
            neighbors_cells = []
            if y < room_size[1]//2:         # Up neighbor exists
                neighbors_cells.append(positions[(x,y+1)])
                
            if y > -room_size[1]//2 + 1:    # Dowm neighbor exists
                neighbors_cells.append(positions[(x,y-1)])
            
            if x < room_size[0]//2:         # Right neighbor exists
                neighbors_cells.append(positions[(x+1,y)])
                
            if x > -room_size[0]//2 + 1:    # Left neighbor exists
                neighbors_cells.append(positions[(x-1,y)])
            
            for neighbor in neighbors_cells:
                problem_Pick_and_Place.set_initial_value(Adjacent(current_cell, neighbor), True)
                

    problem_Pick_and_Place.set_initial_value(At(robot, positions[(0, 0)]), True)
    

    # Set the metric that will be used to minimize the total weight
    problem_Pick_and_Place.add_quality_metric(MinimizeSequentialPlanLength())
    
    # Solve the problem
    with OneshotPlanner(problem_kind=problem_Pick_and_Place.kind, optimality_guarantee=PlanGenerationResultStatus.SOLVED_OPTIMALLY) as planner:
        result = planner.solve(problem_Pick_and_Place)
        return result.plan, positions
    
    
    
# Convert the plan into list of tuples of the form (action1 name, param1,...), (action2 name, param1,...),...
def parse_plan(plan, positions):
    parsed_plan = []
    for action in plan.actions:
        if action.action.name == "move":
            l_from = action.actual_parameters[1].object().name
            l_to = action.actual_parameters[2].object().name
            l_from = next(pos for pos, obj in positions.items() if obj.name == l_from)
            l_to = next(pos for pos, obj in positions.items() if obj.name == l_to)
            parsed_plan.append(("move", l_from, l_to))

        elif action.action.name == "pick" or action.action.name == "place":
            cube = action.actual_parameters[0].object().name
            loc = action.actual_parameters[2].object().name
            loc = next(pos for pos, obj in positions.items() if obj.name == loc)
            parsed_plan.append((action.action.name, cube, loc))

        
    return parsed_plan







if __name__ == "__main__":

    cube_nodes = []
    positions = []
    open_box_node = None

    # Load the input file
    current_dir = os.path.dirname(__file__)
    input_file_path = os.path.join(current_dir, 'input.json')

    with open(input_file_path) as f:
        input = json.load(f)
        
    # Access predefined settings
    floor_size = input.get("floor_size", {})
    cubes_positions = input.get("cubes_positions", {})

    # Get a movment plan of the robot based on the current input
    processed_cubes_positions = {
        key: [[round(coord) for coord in pos[:2]] for pos in positions]
        for key, positions in cubes_positions.items()
    }
    plan, positions = get_plan((round(floor_size[0]), round(floor_size[1])), processed_cubes_positions)

    # Parse the plan and write it to plan.json
    parsed_plan = parse_plan(plan, positions)
    with open("plan.json", "w") as file:
        json.dump(parsed_plan, file)


# app = Flask(__name__)

# @app.route('/generate_plan', methods=['POST'])
# def generate_plan():
#     data = request.get_json()
    
#     floor_size = data.get("floor_size", {})
#     cubes_positions = data.get("cubes_positions", {})
    
#     processed_cubes_positions = {
#         key: [[round(coord) for coord in pos[:2]] for pos in positions]
#         for key, positions in cubes_positions.items()
#     }
    
#     plan, positions = get_plan((round(floor_size[0]), round(floor_size[1])), processed_cubes_positions)
#     parsed_plan = parse_plan(plan, positions)
    
#     return jsonify(parsed_plan)

# if __name__ == "__main__":
#     app.run(debug=True)