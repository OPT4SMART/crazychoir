import os

def generate_webots_world_file(robots, source_filename, target_filename):
    with open(source_filename, 'r') as source_file:
        contents = source_file.read()

    with open(target_filename, 'w') as target_file:
        target_file.write(contents)

        for robot in robots:
            template_filename = os.path.join(os.path.dirname(source_filename), f'obj_{robot["type"]}.wbt')
            with open(template_filename, 'r') as template_file:
                template = template_file.read()
                template = template.replace('$NAME', robot["name"])
                template = template.replace('$X', str(robot["position"][0]))
                template = template.replace('$Y', str(robot["position"][1]))
                template = template.replace('$Z', str(robot["position"][2]))
                target_file.write(template)

# Define the robots
robots = [
    {"name": "agent_0", "type": "crazyflie", "position": [0, 0, 0]},
    {"name": "agent_1", "type": "crazyflie", "position": [0, 0, 0]},
    # Add more robots here...
]

# Use the function to generate a Webots world file
generate_webots_world_file(robots, 'empty_world.wbt', 'my_world.wbt')