#! /usr/bin/env python3

import os
import json
import math
import yaml
import shlex
import random
import subprocess

current_path = os.getcwd()
print(current_path)
simulation_path = current_path+'/simulation_files'
print(simulation_path)

try:
    os.mkdir(simulation_path)
except OSError:
    print ("Creation of the directory %s failed" % simulation_path)
else:
    print ("Successfully created the directory %s" % simulation_path)

GOALS = ['ROBOT', 'ENVIRONMENT', 'MODEL/STATE']
METRICS = ['MissionSuccRate', 'AverageTimespan', 'LowBattFailure']

# No primeiro goal(robot uncertainty) teremos 5 robôs, esse numero será constante entre os trials.
# Características que serão mudadas durante cada trial: 
#   - Posição
#   - Bateria
#   - Capabilities


              #  x,   y,     yaw 
robot_pose = [[0.0, 0.0,        0.0],       # r1
              [1.0, 1.0,    math.pi],       # r2
              [2.0, 2.0,        0.0],       # r3
              [3.0, 3.0,        0.0],       # r4
              [4.0, 4.0,  math.pi/2]]       # r5


                    #  r1    r2    r3    r4    r5
robot_batt_levels = [[1.00, 0.90, 0.90, 0.90, 1.00],    # set 1 
                     [1.00, 0.90, 0.90, 0.90, 1.00],    # set 2
                     [1.00, 0.90, 0.90, 0.90, 1.00],    # set 3
                     [1.00, 0.90, 0.90, 0.90, 1.00],    # set 4
                     [1.00, 0.90, 0.90, 0.90, 1.00]]    # set 5


available_capabilities = ['NavToRoom', 'ApproachNurse', 'AuthenticateNurse', 'HandleDrawer', 'WaitForMessage', 'SendMessage']

class Robot(object):
    """docstring for Robot"""
    def __init__(self, n, pose, batt_level, capabilities):
        super(Robot, self).__init__()
        self.id = n
        self.pose = pose
        self.batt_level = batt_level
        self.capabilities = capabilities
        self.repre = {'id': self.id,
                      'pose': self.pose,
                      'batt_level': self.batt_level,
                      'capabilities': self.capabilities
                      }
        self.motion_pkg_name = 'motion_ctrl'
        self.pytrees_pkg_name = 'py_trees'
        self.motiond = None
        self.pytreesd = None
        self.build_motion_docker()
        self.build_pytrees_docker()

    def get_motion_docker(self):
        return ('motion_ctrl'+str(self.id), self.motiond)

    def get_pytrees_docker(self):
        return ('py_trees'+str(self.id), self.pytreesd)

    def get_params(self):
        return self.repre

    def __str__(self):
        return json.dumps(self.repre)

    def build_motion_docker(self):
        # motion_ctrl:
        #     build:
        #       context: ./docker
        #       dockerfile: Dockerfile.motion
        #     container_name: motion_ctrl
        #     runtime: runc
        #     depends_on:
        #       - master
        #     ports:
        #       - "11311:11311"
        #     volumes:
        #       - ./docker/motion_ctrl:/ros_ws/src/motion_ctrl/
        #       - ./docker/turtlebot3_hospital_sim:/ros_ws/src/turtlebot3_hospital_sim/
        #     environment:
        #       - "ROS_HOSTNAME=motion_ctrl"
        #       - "ROS_MASTER_URI=http://motion_ctrl:11311"
        #       - "ROBOT_NAME=turtlebot1"
        #     env_file:
        #       - test.env
        #     # command: /bin/bash -c "source /ros_ws/devel/setup.bash && roslaunch motion_ctrl base_navigation.launch"
        #     command: /bin/bash -c "source /ros_ws/devel/setup.bash && roslaunch motion_ctrl base_navigation.launch & rosrun topic_tools relay /move_base_simple/goal /turtlebot1/move_base_simple/goal"
        #     # command: /bin/bash -c "source /ros_ws/devel/setup.bash && roscore"
        #     tty: true
        #     privileged: true
        #     networks:
        #       morsegatonet:
        #         ipv4_address: 10.2.0.6
        package_name = 'motion_ctrl'
        cointainer_name = self.motion_pkg_name+str(self.id)
        self.motiond = {
            'build': {
                'context' : './docker',
                'dockerfile': 'Dockerfile.motion',
            },
            'container_name': cointainer_name,
            'runtime': 'runc',
            'depends_on': ['master'],
            # 'ports': ["11311:11311"],
            'env_file': ['test.env'],
            'volumes': ['./docker/'+self.motion_pkg_name+':/ros_ws/src/'+self.motion_pkg_name+'/', './docker/turtlebot3_hospital_sim:/ros_ws/src/turtlebot3_hospital_sim/'],
            'environment': ["ROS_HOSTNAME="+cointainer_name, "ROS_MASTER_URI=http://"+cointainer_name+":11311", "ROBOT_NAME=turtlebot"+str(self.id)],
            # 'command': '/bin/bash -c "source /ros_ws/devel/setup.bash && roslaunch motion_ctrl base_navigation.launch & rosrun topic_tools relay /move_base_simple/goal /turtlebot1/move_base_simple/goal"'
            'command': '/bin/bash -c "source /ros_ws/devel/setup.bash && roslaunch motion_ctrl base_navigation.launch"',
            'tty': True,
            'privileged': True,
            # 'networks': {
            #     'morsegatonet': {
            #         'ipv4_address': '10.2.0.6'
            #     }
            # },
            'networks': ['morsegatonet']
        }

    def build_pytrees_docker(self):
        # py_trees1:
        #     build:
        #       context: ./docker
        #       dockerfile: Dockerfile.pytrees
        #     container_name: py_trees1
        #     runtime: runc
        #     depends_on:
        #       - motion_ctrl
        #     env_file:
        #       - .env
        #     devices:
        #       - "/dev/dri"
        #       - "/dev/snd"
        #     environment:
        #       - "ROS_HOSTNAME=py_trees1"
        #       - "ROS_MASTER_URI=http://motion_ctrl:11311"
        #       - "QT_X11_NO_MITSHM=1"
        #       - "DISPLAY=$DISPLAY"
        #       - "XAUTHORITY=$XAUTH"
        #       - "QT_GRAPHICSSYSTEM=native"
        #       - "PULSE_SERVER=unix:${XDG_RUNTIME_DIR}/pulse/native"
        #       - "ROBOT_NAME=turtlebot1"
        #     volumes:
        #       - /tmp/.docker.xauth:/tmp/.docker.xauth:rw
        #       - /tmp/.X11-unix:/tmp/.X11-unix:rw
        #       - /var/run/dbus:/var/run/dbus:ro
        #       - /etc/machine-id:/etc/machine-id:ro
        #       - ${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native
        #       - ~/.config/pulse/cookie:/root/.config/pulse/cookie
        #       - ./docker/py_trees_ros_behaviors:/ros_ws/src/py_trees_ros_behaviors/
        #     # command: /bin/bash -c "source /ros_ws/install/setup.bash && ros2 topic pub /std_out std_msgs/msg/String data:\ \'HelloWorld\'\ "
        #     command: python3 /ros_ws/src/bridge.py
        #     command: /bin/bash -c "source /opt/ros/noetic/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics "
        #     tty: true
        #     networks:
        #       morsegatonet:
        #         ipv4_address: 10.2.0.8
        package_name = 'py_trees'
        cointainer_name = self.pytrees_pkg_name+str(self.id)
        self.pytreesd = {
            'build': {
                'context' : './docker',
                'dockerfile': 'Dockerfile.pytrees',
            },
            'container_name': cointainer_name,
            'runtime': 'runc',
            'depends_on': ['motion_ctrl'+str(self.id)],
            'env_file': ['test.env'],
            'volumes': ['/tmp/.docker.xauth:/tmp/.docker.xauth:rw', '/tmp/.X11-unix:/tmp/.X11-unix:rw', '/var/run/dbus:/var/run/dbus:ro', '/etc/machine-id:/etc/machine-id:ro', '${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native', '~/.config/pulse/cookie:/root/.config/pulse/cookie', './docker/'+package_name+'_ros_behaviors:/ros_ws/src/'+package_name+'_ros_behaviors/'],
            'environment': ["ROS_HOSTNAME="+cointainer_name, "ROS_MASTER_URI=http://"+self.motion_pkg_name+str(self.id)+":11311", "ROBOT_NAME=turtlebot"+str(self.id)],
            # 'command': '/bin/bash -c "source /ros_ws/devel/setup.bash && roslaunch motion_ctrl base_navigation.launch & rosrun topic_tools relay /move_base_simple/goal /turtlebot1/move_base_simple/goal"'
            # 'command': '/bin/bash -c "source /opt/ros/noetic/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics "',
            'tty': True,
            # 'networks': {
            #     'morsegatonet': {
            #         'ipv4_address': '10.2.0.8'
            #     }
            # },
            'networks': ['morsegatonet']
        }

class Trials(object):
    """docstring for Trials"""
    def __init__(self, arg):
        super(Trials, self).__init__()
        self.arg = arg
        
class Experiment(object):
    """docstring for Experiment"""
    def __init__(self, xp_id, nrobots):
        super(Experiment, self).__init__()
        self.sim_process = None
        self.docker_compose = dict()
        display_idx = random.choice([1,2,3])
        morse_cmd = '/bin/bash -c "source /ros_ws/devel/setup.bash && Xvfb -screen 0 100x100x24 :%d & DISPLAY=:%d morse run morse_hospital_sim"'
        self.morse = {
            'build': {
                'context' : './docker',
                'dockerfile': 'Dockerfile.app',
            },
            'runtime': 'nvidia',
            'container_name': 'morse',
            'depends_on': ['master'],
            'devices': ["/dev/dri", "/dev/snd"],
            'env_file': ['.env'],
            'environment': ["ROS_HOSTNAME=morse", "ROS_MASTER_URI=http://master:11311", "QT_X11_NO_MITSHM=1"],
            'volumes': ['/tmp/.X11-unix:/tmp/.X11-unix:rw', '~/.config/pulse/cookie:/root/.config/pulse/cookie', './docker/hmrs_hostpital_simulation/morse_hospital_sim:/ros_ws/morse_hospital_sim'],
            'expose': ["8081", "3000", "3001"],
            # 'command': 'roslaunch rosbridge_server rosbridge_websocket.launch',
            # 'command': 'rosrun tf2_web_republisher tf2_web_republisher',
            'command': (morse_cmd%(display_idx,display_idx)),
            'tty': True,
            'networks': ['morsegatonet']
        }
        self.master = {
            'build': {
                'context' : './docker',
                'dockerfile': 'Dockerfile.app',
            },
            'container_name': 'master',
            'command': '/bin/bash -c "source /ros_ws/devel/setup.bash && roslaunch src/hos/server.launch"',
            'networks': {
                'morsegatonet': {
                    'ipv4_address': '10.2.0.5'
                }
            },
        }
        self.ros1_bridge = {
            'build': {
                'context' : './docker',
                'dockerfile': 'Dockerfile.pytrees',
            },
            'container_name': 'ros1_bridge',
            'runtime': 'runc',
            'depends_on': ['master'],
            'env_file': ['test.env'],
            'volumes': ['/tmp/.docker.xauth:/tmp/.docker.xauth:rw', '/tmp/.X11-unix:/tmp/.X11-unix:rw', '/var/run/dbus:/var/run/dbus:ro'],
            'environment': ["ROS_HOSTNAME=ros1_bridge", "ROS_MASTER_URI=http://master:11311"],
            'command': '/bin/bash -c "source /opt/ros/noetic/setup.bash && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics "',
            'tty': True,
            # 'networks': {
            #     'morsegatonet': {
            #         'ipv4_address': '10.2.0.8'
            #     }
            # },
            'networks': ['morsegatonet']
        }
        self.networks = {
            'morsegatonet': {
                'driver': 'bridge',
                'ipam': {
                    'driver': 'default',
                    'config': [{'subnet': '10.2.0.0/16'}],
                }
            }
        }

        robots = []
        # build robots
        for i in range(0, nrobots):
            robot = Robot(i+1, robot_pose[i], robot_batt_levels[0][i], available_capabilities)
            r_motion_name, r_motion_serv = robot.get_motion_docker()
            r_pytrees_name, r_pytrees_serv = robot.get_pytrees_docker()
            robot_info = {
                'id': i,
                'robot': robot,
                'motion_name': r_motion_name,
                'motion_serv': r_motion_serv,
                'pytrees_name': r_pytrees_name,
                'pytrees_serv': r_pytrees_serv,
            }
            robots.append(robot_info)

        r1 = Robot(1, robot_pose[0], robot_batt_levels[0][0], available_capabilities)
        r1_motion_name, r1_motion_serv = r1.get_motion_docker()
        r1_pytrees_name, r1_pytrees_serv = r1.get_pytrees_docker()
        self.services = {
            'morse': self.morse,
            'master': self.master,
            'ros1_bridge': self.ros1_bridge,
        }
        for i in range(0, nrobots):
            self.services[robots[i]["motion_name"]] = robots[i]["motion_serv"]
            self.services[robots[i]["pytrees_name"]] = robots[i]["pytrees_serv"]
        self.docker_compose = {
            'version': "2.3",
            'services': self.services,
            'networks': self.networks,
        }

    def get_compose_file(self):
        return self.docker_compose

    def start_simulation(self):
        up_docker_str = 'docker-compose -f exp_1_trial_1.yaml up -d'
        print('Run Simulation')
        up_docker_tk = shlex.split(up_docker_str)
        # print(up_docker_tk)

        self.sim_process = subprocess.run(up_docker_tk,
                                     stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE,
                                     universal_newlines=True)
        print(self.sim_process.stdout)
        print(self.sim_process.stderr)

    def close_simulation(self):
        stop_docker_str = 'docker-compose -f exp_1_trial_1.yaml stop'
        stop_docker_tk = shlex.split(stop_docker_str)

        print('Close Simulation')
        self.sim_process = subprocess.run(stop_docker_tk,
                             stdout=subprocess.PIPE, 
                             stderr=subprocess.PIPE,
                             universal_newlines=True)
        print(self.sim_process.stdout)
        print(self.sim_process.stderr)

    def save_compose_file(self):
        self.compose_name = 'exp_1_trial_1.yaml'
        with open(current_path+'/'+self.compose_name, 'w') as file:
            documents = yaml.dump(self.get_compose_file(), file)

r1 = Robot(1, robot_pose[0], robot_batt_levels[0][0], available_capabilities)
r2 = Robot(2, robot_pose[1], robot_batt_levels[0][1], available_capabilities)
r3 = Robot(3, robot_pose[2], robot_batt_levels[0][2], available_capabilities)
r4 = Robot(4, robot_pose[3], robot_batt_levels[0][3], available_capabilities)
r5 = Robot(5, robot_pose[4], robot_batt_levels[0][4], available_capabilities)

xp1 = Experiment(1, 2)

print(str(r1))
with open(current_path+'/exp_1_trial_1.yaml', 'w') as file:
    documents = yaml.dump(xp1.get_compose_file(), file)

xp1.start_simulation()

# xp1.close_simulation()

# up_docker_str = 'docker-compose -f exp_1_trial_1.yaml up -d'
# print('Run Simulation')
# up_docker_tk = shlex.split(up_docker_str)
# print(up_docker_tk)

# process = subprocess.run(up_docker_tk,
#                      stdout=subprocess.PIPE, 
#                      stderr=subprocess.PIPE,
#                      universal_newlines=True)
# print(process.stdout)
# print(process.stderr)

# stop_docker_str = 'docker-compose -f exp_1_trial_1.yaml stop'
# stop_docker_tk = shlex.split(stop_docker_str)

# print('Close Simulation')
# process = subprocess.run(stop_docker_tk,
#                      stdout=subprocess.PIPE, 
#                      stderr=subprocess.PIPE,
#                      universal_newlines=True)
# print(process.stdout)
# print(process.stderr)
