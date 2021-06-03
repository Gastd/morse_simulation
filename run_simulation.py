#! /usr/bin/env python3

import os
import json
import math
import yaml
import copy
import time
import shlex
import random
import datetime
import subprocess

current_path = os.getcwd()
print(current_path)
simulation_path = current_path+'/simulation_files'
env_path = current_path+'/sim.env'
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
#   - skills


#               #  x,   y,     yaw 
# robot_pose = [[0.0, 0.0,        0.0],       # r1
#               [1.0, 1.0,    math.pi],       # r2
#               [2.0, 2.0,        0.0],       # r3
#               [3.0, 3.0,        0.0],       # r4
#               [4.0, 4.0,  math.pi/2]]       # r5


                    #  r1    r2    r3    r4    r5
# robot_batt_levels = [[1.00, 0.90, 0.90, 0.90, 1.00],    # set 1 
#                      [1.00, 0.90, 0.90, 0.90, 1.00],    # set 2
#                      [1.00, 0.90, 0.90, 0.90, 1.00],    # set 3
#                      [1.00, 0.90, 0.90, 0.90, 1.00],    # set 4
#                      [1.00, 0.90, 0.90, 0.90, 1.00]]    # set 5

            # x     y
rooms = [[-39.74, 32.52], # 1
         [-39.55, 26.26], # 2
         [-39.92, 19.31], # 3
         [-40.00, 13.02], # 4
         [-32.90, 31.06], # 5
         [-32.94, 22.06], # 6
         [-28.70, 19.79], # 7
         [-28.70, 12.02], # 8
         [-25.38, 19.78], # 9
         [-24.32, 11.91], # 10
         [-21.81, 19.59], # 11
         [-19.18, 12.15], # 12
         [-17.91, 19.77], # 13
         [-14.31, 19.52], # 14
         [-14.31, 19.52], # 15
         [-10.00, 19.81], # 16
         [-06.37, 24.87], # 17
         [-05.72, 28.96], # 18
         [-00.29, 28.55], # 19
         [-00.57, 24.25]] # 20

def get_pose(loc):
    poses = {
        "IC Corridor": [-37, 15],
        "IC Room 1": [-39.44, 33.98, 0.00],
        "IC Room 2": [-32.88, 33.95, 3.14],
        "IC Room 3": [-40.23, 25.37, 0.00],
        "IC Room 4": [-33.90, 18.93, 3.14],
        "IC Room 5": [-38.00, 21.50, 0.00],
        "IC Room 6": [-38.00, 10.00, 0.00],
        "PC Corridor": [-19, 16],
        "PC Room 1": [-28.50, 18.00,-1.57],
        "PC Room 2": [-27.23, 18.00,-1.57],
        "PC Room 3": [-21.00, 18.00,-1.57],
        "PC Room 4": [-19.00, 18.00,-1.57],
        "PC Room 5": [-13.50, 18.00,-1.57],
        "PC Room 6": [-11.50, 18,-1.57],
        "PC Room 7": [-4, 18,-1.57],
        "PC Room 8": [-27.23, 13.00, 1.57],
        "PC Room 9": [-26.00, 13.00, 1.57],
        "PC Room 10": [-18.00, 13.00, 1.57],
        "Reception": [-1, 20],
        "Pharmacy Corridor": [0, 8],
        "Pharmacy": [-2, 2.6],
    }
    return poses[loc]

available_capabilities = ['NavToRoom', 'ApproachNurse', 'AuthenticateNurse', 'WaitForMessage', 'SendMessage']
handle_capability = ['HandleDrawer']



class Robot(object):
    """docstring for Robot"""
    def __init__(self, n, loc, batt_level, skills, config):
        super(Robot, self).__init__()
        self.id = n
        self.pose = get_pose(loc)
        self.batt_level = batt_level
        self.skills = skills
        # self.plan = plan
        self.config = config
        self.repre = {'id': self.id,
                      'pose': self.pose,
                      'batt_level': self.batt_level,
                      'skills': self.skills
                      }
        self.motion_pkg_name = 'motion_ctrl'
        self.pytrees_pkg_name = 'py_trees'
        self.motiond = None
        self.pytreesd = None
        self.build_motion_docker()
        self.build_pytrees_docker()

    def get_id(self):
        return self.id

    def get_pose(self):
        return self.pose

    def get_batt_level(self):
        return self.batt_level

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
            # 'ports': ["9090:9090"],
            'env_file': [env_path],
            'volumes': ['./docker/'+self.motion_pkg_name+':/ros_ws/src/'+self.motion_pkg_name+'/', './docker/turtlebot3_hospital_sim:/ros_ws/src/turtlebot3_hospital_sim/', './log/:/root/.ros/logger_sim/'],
            'environment': ["ROS_HOSTNAME="+cointainer_name, "ROS_MASTER_URI=http://master:11311", "ROBOT_NAME=turtlebot"+str(self.id)],
            # 'command': '/bin/bash -c "source /ros_ws/devel/setup.bash && roslaunch motion_ctrl base_navigation.launch & rosrun topic_tools relay /move_base_simple/goal /turtlebot1/move_base_simple/goal"'
            'command': '/bin/bash -c "source /ros_ws/devel/setup.bash && roslaunch motion_ctrl base_navigation.launch --wait"',
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
            'env_file': [env_path],
            'volumes': ['/tmp/.docker.xauth:/tmp/.docker.xauth:rw',
                '/tmp/.X11-unix:/tmp/.X11-unix:rw',
                '/var/run/dbus:/var/run/dbus:ro',
                '/etc/machine-id:/etc/machine-id:ro',
                '${XDG_RUNTIME_DIR}/pulse/native:${XDG_RUNTIME_DIR}/pulse/native',
                '~/.config/pulse/cookie:/root/.config/pulse/cookie',
                './docker/py_trees_ros_behaviors:/ros_ws/src/py_trees_ros_behaviors/'
                ],
            'environment': ["ROS_HOSTNAME="+cointainer_name, "ROS_MASTER_URI=http://master:11311", "ROBOT_NAME=turtlebot"+str(self.id), "SKILLS="+str(self.skills), "ROBOT_CONFIG="+json.dumps(self.config)],
            # 'command': '/bin/bash -c "source /ros_ws/devel/setup.bash && roslaunch motion_ctrl base_navigation.launch & rosrun topic_tools relay /move_base_simple/goal /turtlebot1/move_base_simple/goal"'
            'command': '/bin/bash -c "colcon build && source /ros_ws/install/setup.bash && ros2 launch py_trees_ros_behaviors tutorial_seven_docking_cancelling_failing_launch.py"',
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
    def __init__(self, config_file="experiment_trials.json"):
        super(Experiment, self).__init__()
        self.sim_process = None
        self.docker_compose = dict()
        self.xp_id = 0
        self.nrobots = 0
        self.config_file = config_file
        self.simulation_timeout_s = 15*60
        self.load_trials(self.config_file)
        self.endsim = ''

    def load_trials(self, file_name):
        # file_name = "experiment_sample.json"
        curr_path = os.getcwd()+'/'
        file_path = curr_path + file_name
        self.config = None
        with open(file_path) as f:
            self.config = json.load(f)
        print(print(json.dumps(self.config[0]["robots"][0], indent=2, sort_keys=True)))
        print(self.config[0]["nurses"][0])
        print(len(self.config[0]["robots"][1]))
        self.nurses_config = self.config[0]["nurses"]
        self.robots_config = self.config[0]["robots"]

    def run_simulation(self):
        self.endsim = False
        self.nurses_config = self.config[0]["nurses"]
        self.robots_config = self.config[0]["robots"]
        self.create_env_file()
        self.create_dockers()
        self.create_robots()
        self.save_compose_file()
        # print("STARTING SIMULATION...")
        # self.start_simulation()
        # start = time.time()
        # runtime = time.time()
        # self.clear_log_file()
        # # call simulation and watch timeout
        # while (runtime - start) <= self.simulation_timeout_s and self.endsim == False:
        #     time.sleep(1)
        #     runtime = time.time()
        #     self.check_end_simulation()
        # end = time.time()
        # self.close_simulation()
        # print("ENDING SIMULATION...")
        # print(f"Runtime of the simulation is {end - start}")
        # self.save_log_file(1, end - start)

    def run_some_simulations(self, sim_list):
        print("RUNNING %d TRIALS FOR THIS EXPERIMENT"%len(self.config))
        # create files
        for idx in sim_list:
            self.endsim = False
            print("RUNNING TRIALS #%d"%idx)
            self.nurses_config = self.config[idx]["nurses"]
            self.robots_config = self.config[idx]["robots"]
            self.create_env_file()
            self.create_dockers()
            self.create_robots()
            self.save_compose_file()
            print("STARTING SIMULATION #%d..."%idx)
            self.start_simulation()
            start = time.time()
            runtime = time.time()
            self.clear_log_file()
            # call simulation and watch timeout
            while (runtime - start) <= self.simulation_timeout_s and self.endsim == False:
                time.sleep(1)
                runtime = time.time()
                self.check_end_simulation()
                # check simulation end
            end = time.time()
            self.close_simulation()
            print("ENDING SIMULATION #%d..."%idx)
            print(f"Runtime of the simulation #{idx} is {end - start}")
            self.save_log_file(idx, end - start)

    def run_all_simulations(self):
        print("RUNNING %d TRIALS FOR THIS EXPERIMENT"%len(self.config))
        # create files
        for i in range(0, len(self.config)):
            self.endsim = False
            print("RUNNING TRIALS #%d"%i)
            self.nurses_config = self.config[i]["nurses"]
            self.robots_config = self.config[i]["robots"]
            self.create_env_file()
            self.create_dockers()
            self.create_robots()
            self.save_compose_file()
            print("STARTING SIMULATION #%d..."%i)
            self.start_simulation()
            start = time.time()
            runtime = time.time()
            self.clear_log_file()
            # call simulation and watch timeout
            while (runtime - start) <= self.simulation_timeout_s and self.endsim == False:
                time.sleep(1)
                runtime = time.time()
                self.check_end_simulation()
                # check simulation end
            end = time.time()
            self.close_simulation()
            print("ENDING SIMULATION #%d..."%i)
            print(f"Runtime of the simulation #{i} is {end - start}")
            self.save_log_file(i, end - start)

    def clear_log_file(self):
        with open(current_path+'/log/experiment.log', 'w') as file:
            file.write('')

    def save_log_file(self, run, execution_time):
        with open(current_path+'/log/experiment.log', 'r') as file:
            print("Saving log file as: " + current_path+f'/log/experiment_trial1_exp{run}.log')
            lines = file.readlines()
            with open(current_path+f'/log/experiment_exp1_trial{run}.log', 'w') as logfile:
                for line in lines:
                    logfile.write(line)
                if self.endsim == 'reach-target':
                    logfile.write("0.0,[debug],simulation closed,"+self.endsim+','+str(execution_time)+"\n")
                elif self.endsim == 'failure-bt':
                    logfile.write("0.0,[debug],simulation closed,"+self.endsim+','+str(execution_time)+"\n")
                else:
                    logfile.write("0.0,[debug],simulation closed,runner,"+str(execution_time)+"\n")
        self.clear_log_file()
        # self.save_bag_file(run)

    def save_bag_file(self, run):
        current_date = datetime.datetime.today().strftime('%H-%M-%S-%d-%b-%Y')
        os.rename(current_path+'/log/bag.bag', current_path+f'/log/exp1_trial{run}_{current_date}.bag')

    def check_end_simulation(self):
        with open(current_path+'/log/experiment.log', 'r') as file:
            print("Checking simulation...")
            lines = file.readlines()
            print(lines)
            alllines = ''
            for line in lines:
                alllines = alllines+line
                if "ENDSIM" in line:
                    self.endsim = 'reach-target'
                if "FAILURE" in line:
                    self.endsim = 'failure-bt'
            # if alllines.count('LOW BATTERY') >= 5:
            #     self.endsim = True

    def create_env_file(self):
        self.env_name = "sim.env"
        curr_path = os.getcwd()+'/'
        file_path = curr_path + self.env_name
        chosed_robot = ""
        
        for r_config in self.robots_config:
            r_id = r_config["id"]+1
            if r_config["local_plan"] != None:  chosed_robot = "turtlebot"+str(r_id)
        
        with open(file_path, "w") as ef:
            nurse_pos = self.nurses_config[0]["position"]
            nurse_str = str(nurse_pos).replace(',',';')
            ef.write("NURSE_POSE="+nurse_str+'\n')
            ef.write('\n')
            ef.write("CHOSED_ROBOT="+chosed_robot+'\n')
            ef.write('\n')
            ef.write('N_ROBOTS='+str(len(self.robots_config))+'\n')
            ef.write('\n')
            for robot in self.robots_config:
                # name
                id_str = (robot["id"]+1)
                ef.write('ROBOT_NAME_%d=turtlebot%d\n'%(id_str,id_str))
                # pose
                yaw = random.uniform(-math.pi, math.pi)
                # pose_str = str(get_pose(robot["location"])).replace(',',';')
                pose_str = str(robot["position"]).replace(',',';')
                pose_env = ("ROBOT_POSE_%d="%(id_str))+pose_str
                ef.write(pose_env+'\n')
                # batt level
                batt_level_str = robot["battery_level"]*100
                batt_level_env = "BATT_INIT_STATE_%d=%.2f"%(id_str,batt_level_str)
                ef.write(batt_level_env+'\n')
                batt_slope_str = robot["battery_consumption_rate"]*100
                batt_slope_env = "BATT_SLOPE_STATE_%d=%.2f"%(id_str, batt_slope_str)
                ef.write(batt_slope_env+'\n')
                ef.write('\n')

    def create_robots(self):
        robots_servs = []
        # build robots
        for r_config in self.robots_config:
            r_id = r_config["id"]+1
            r_loc = r_config["location"]
            robot = Robot(r_id, r_loc, r_config["battery_level"], r_config["skills"], r_config)
            print(robot)
            r_motion_name, r_motion_serv = robot.get_motion_docker()
            r_pytrees_name, r_pytrees_serv = robot.get_pytrees_docker()
            robot_info = {
                'id': r_id,
                'robot': robot,
                'motion_name': r_motion_name,
                'motion_serv': r_motion_serv,
                'pytrees_name': r_pytrees_name,
                'pytrees_serv': r_pytrees_serv,
            }
            robots_servs.append(robot_info)
            if r_config["local_plan"] != None:  chosed_robot = "turtlebot"+str(r_id)
            print(r_config["local_plan"])
        print(f"ROBOT CHOSED IS: {chosed_robot}")
        for i in range(0, len(self.robots_config)):
            self.services[robots_servs[i]["motion_name"]] = robots_servs[i]["motion_serv"]
            self.services[robots_servs[i]["pytrees_name"]] = robots_servs[i]["pytrees_serv"]
        # print(self.services)

    def create_dockers(self):
        display_idx = random.choice([1,2,3])
        morse_cmd = '/bin/bash -c "source /ros_ws/devel/setup.bash && Xvfb -screen 0 100x100x24 :%d & DISPLAY=:%d morse run morse_hospital_sim"'
        self.morse = {
            'build': {
                'context' : './docker',
                'dockerfile': 'Dockerfile.app',
            },
            'runtime': 'runc',
            'container_name': 'morse',
            'depends_on': ['master'],
            # 'devices': ["/dev/dri", "/dev/snd"],
            'env_file': [env_path],
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
                'dockerfile': 'Dockerfile.motion',
            },
            'container_name': 'master',
            'env_file': [env_path],
            'volumes': ['./log/:/root/.ros/logger_sim/'],
            'command': '/bin/bash -c "source /ros_ws/devel/setup.bash && roslaunch src/motion_ctrl/launch/log.launch"',
            'tty': True,
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
            'env_file': [env_path],
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
        self.services = {
            'morse': self.morse,
            'master': self.master,
            'ros1_bridge': self.ros1_bridge,
        }
        self.docker_compose = {
            'version': "2.3",
            'services': self.services,
            'networks': self.networks,
        }

    def get_compose_file(self):
        return self.docker_compose

    def start_simulation(self):
        up_docker_str = 'docker-compose -f experiment_trials.yaml up -d'
        # up_docker_str = 'docker-compose up -d'
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
        # stop_docker_str = 'docker-compose down'
        stop_docker_str = 'docker-compose -f experiment_trials.yaml down'
        stop_docker_tk = shlex.split(stop_docker_str)

        print('Closing Simulation')
        self.sim_process = subprocess.run(stop_docker_tk,
                             stdout=subprocess.PIPE, 
                             stderr=subprocess.PIPE,
                             universal_newlines=True)
        print(self.sim_process.stdout)
        print(self.sim_process.stderr)

    def save_compose_file(self):
        self.compose_name = 'experiment_trials.yaml'
        with open(current_path+'/'+self.compose_name, 'w') as file:
            documents = yaml.dump(self.get_compose_file(), file)

def choose_poses(n_robots):
    poses = []
    for n in range(0, n_robots):
        pose = []
        pose = random.choice(rooms)
        print(pose)
        pose.append(random.uniform(-math.pi, math.pi)) # choose initial orientation
        if len(pose) > 3:
            raise Exception('len(pose)==', len(pose))
        poses.append(pose)
        print(poses)
        # TODO: check if the choosed pose is already taken
    return poses

# n_robots = 3
# poses = choose_poses(n_robots)

# r1 = Robot(1, poses[0], random.uniform(0, 100), available_capabilities + random.choice([0, 1])*handle_capability)
# r2 = Robot(2, poses[1], random.uniform(0, 100), available_capabilities + random.choice([0, 1])*handle_capability)
# r3 = Robot(3, poses[2], random.uniform(0, 100), available_capabilities + random.choice([0, 1])*handle_capability)
# r4 = Robot(4, robot_pose[3], robot_batt_levels[0][3], available_capabilities)
# r5 = Robot(5, robot_pose[4], robot_batt_levels[0][4], available_capabilities)
# robots = [r1, r2, r3]

xp1 = Experiment()
xp1.run_simulation()
# xp1.run_some_simulations([9, 17, 34, 63, 73, 75])
# xp1.run_all_simulations()

# print(str(r1))
# print(str(r2))
# print(str(r3))

# print(env_path)
# env_file = open(env_path, "w")
# env_file.write('N_ROBOTS='+str(n_robots)+'\n')
# for i in range(0, n_robots):
#     # name
#     id_str = robots[i].get_id()
#     env_file.write('ROBOT_NAME_%d=turtlebot%d\n'%(id_str,id_str))
#     # pose
#     pose_str = str(robots[i].get_pose()).replace(',',';')
#     pose_env = ("ROBOT_POSE_%d="%(id_str))+pose_str
#     env_file.write(pose_env+'\n')
#     # batt level
#     batt_level_str = robots[i].get_batt_level()
#     batt_level_env = "BATT_INIT_STATE_%d=%.2f"%(id_str,batt_level_str)
#     env_file.write(batt_level_env+'\n')
#     batt_slope_env = "BATT_SLOPE_STATE_%d=0.05"%(id_str)
#     env_file.write(batt_slope_env+'\n')
#     env_file.write('\n')

# env_file.close()
# with open(current_path+'/experiment_trials.yaml', 'w') as file:
#     documents = yaml.dump(xp1.get_compose_file(), file)

# xp1.start_simulation()

# xp1.close_simulation()


# up_docker_str = 'docker-compose -f experiment_trials.yaml up -d'
# print('Run Simulation')
# up_docker_tk = shlex.split(up_docker_str)
# print(up_docker_tk)

# process = subprocess.run(up_docker_tk,
#                      stdout=subprocess.PIPE, 
#                      stderr=subprocess.PIPE,
#                      universal_newlines=True)
# print(process.stdout)
# print(process.stderr)

# stop_docker_str = 'docker-compose -f experiment_trials.yaml stop'
# stop_docker_tk = shlex.split(stop_docker_str)

# print('Close Simulation')
# process = subprocess.run(stop_docker_tk,
#                      stdout=subprocess.PIPE, 
#                      stderr=subprocess.PIPE,
#                      universal_newlines=True)
# print(process.stdout)
# print(process.stderr)
