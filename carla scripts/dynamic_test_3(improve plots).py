import argparse
import math
import logging
import time
import glob
import os
import sys
import carla
import numpy as np
import random
import json
from datetime import datetime
import socket
import threading
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import pickle
import signal

# Import necessary modules from existing scripts
from ekf_3D import SimpleEKF, process_lidar_data, load_ekf_model, global_to_local, calculate_true_relative_angle
from cal_rssi_distance_improved import calculate_distance, load_model

try:
    sys.path.append(glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import sumolib

# Import other necessary modules from your original script
from sumo_integration.bridge_helper import BridgeHelper
from sumo_integration.carla_simulation import CarlaSimulation
from sumo_integration.constants import INVALID_ACTOR_ID
from sumo_integration.sumo_simulation import SumoSimulation



class SimulationSynchronization(object):
    def __init__(self, sumo_simulation, carla_simulation, tls_manager='none', sync_vehicle_color=False, sync_vehicle_lights=False):
        self.sumo = sumo_simulation
        self.carla = carla_simulation
        self.tls_manager = tls_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.
        self.carla2sumo_ids = {}  # Contains only actors controlled by carla.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        BridgeHelper.offset = self.sumo.get_net_offset()

        # Configuring carla simulation in sync mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.carla.step_length
        self.carla.world.apply_settings(settings)

        traffic_manager = self.carla.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)


        self.lead_vehicle_id = '0'
        self.following_vehicle_id = '1'

        self.current_phase = 'INACTIVE'
        
        self.protocol_phases = {
            'INACTIVE': 'INACTIVE',     # 未激活
            'APPROACH': 'APPROACH',     # 接近阶段
            'MAINTAIN': 'MAINTAIN',     # 保持阶段
            'RETREAT': 'RETREAT',       # 后退阶段
            'NEW_MAINTAIN': 'NEW_MAINTAIN',  # 新保持阶段
            'COMPLETED': 'COMPLETED'    # 完成
        }

        # 添加阶段配置
        self.phase_durations = {
            'APPROACH': 20,    # 接近阶段20秒
            'MAINTAIN': 20,    # 保持阶段20秒
            'RETREAT': 20,     # 后退阶段30秒
            'NEW_MAINTAIN': 20 # 新保持阶段20秒
        }
        
        self.phase_distances = {
            'APPROACH': 12,    # 接近阶段目标距离
            'MAINTAIN': 10,    # 保持阶段目标距离
            'RETREAT': 14,     # 后退阶段目标距离
            'NEW_MAINTAIN': 10 # 新保持阶段目标距离
        }

        # Parameters for sinusoidal speed control
        self.base_speed = 12.5  # m/s
        self.speed_amplitude = 1.4  # m/s
        self.speed_period = 10  # s
        self.target_distance = 10  # m
        
        self.distance_threshold = 1.0  # m
        self.speed_threshold = 0.5  # m/s

        # Physical challenge protocol
        self.protocol_states = {
            'INACTIVE': 'INACTIVE',
            'ACTIVE': 'ACTIVE',
            'COMPLETED': 'COMPLETED'
        }

        self.krauss_params = {
        'accel': 3,      # 最大加速度 m/s²
        'decel': 4.6,      # 舒适减速度 m/s²
        'emergencyDecel': 9.0,  # 紧急减速度 m/s²
        'tau': 0.5,        # 
        }

        self.challenge_state = 'INACTIVE'
        self.challenge_active = False
        self.challenge_start_time = None
        self.challenge_duration = 80  # seconds in total challenge, add all phase durations
        self.protocol_completed = False

        # 
        self.initialize_metrics()
        self.initialize_sensors_and_communication()

    def initialize_metrics(self):
   
        self.challenge_metrics = {
        'activation_time': None,
        'initial_distance': None,
        'initial_speed': None,
        'min_distance': float('inf'),
        'max_distance': 0,
        'start_timestamp': None,
        'end_timestamp': None,
        'approach_complete_time': None,
        'maintain_start_time': None,
        'maintain_complete_time': None,  
        'retreat_start_time': None,      
        'retreat_complete_time': None,   
        'new_maintain_start_time': None  
        }
    
        self.plot_data = {
        'timestamps': [],
        'true_distances': [],
        'estimated_distances': [],
        'true_angles': [],
        'estimated_angles': [],
        'lead_speeds': [],
        'following_speeds': [],
        'phases': [],
        'phase_changes': [] 
        }

    def signal_handler(self, sig, frame):
        print('Caught interrupt, plotting results...')
        self.plot_results()
        sys.exit(0)

    def initialize_sensors_and_communication(self):
        # EKF初始化
        #self.ekf = load_ekf_model('my_ekf_model.pkl')

        # 创建新的EKF实例
        init_x = np.array([10, 0, 0])  # 初始状态 [rx, ry, theta]
        init_P = np.diag([100, 100, np.pi/2])  # 初始协方差
        self.ekf = SimpleEKF(dt=0.1, init_x=init_x, init_P=init_P)
    
    
        # RSSI模型加载
        self.rssi_model, self.poly_features = load_model('fitting_model.pkl')
    
        # RSSI通信设置
        self.rssi_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rssi_socket.bind(('localhost', 12345))
        self.rssi_socket.settimeout(0.3)
        self.rssi_socket.setblocking(False)
        self.latest_rssi_distance = None
        self.first_rssi_received = False
    
        # 传感器数据存储
        self.sensors = {}
        self.sensor_data = {}
    
  
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def update_challenge_metrics(self, distance):
        """Update metrics during challenge"""
        if self.challenge_active:
            self.challenge_metrics['min_distance'] = min(self.challenge_metrics['min_distance'], distance)
            self.challenge_metrics['max_distance'] = max(self.challenge_metrics['max_distance'], distance)

    def activate_challenge(self, distance, speed):
        if self.protocol_completed:  
            return
   
        current_time = self.carla.world.get_snapshot().timestamp.elapsed_seconds
    
        self.challenge_active = True
        self.challenge_state = 'ACTIVE'
        self.current_phase = self.protocol_phases['APPROACH']
        self.challenge_start_time = current_time
        self.phase_start_time = current_time 
        self.target_distance = self.phase_distances['APPROACH']

        self.plot_data['phase_changes'].append((current_time, self.protocol_phases['APPROACH']))
    
        # 记录初始指标
        self.challenge_metrics.update({
        'activation_time': current_time,
        'initial_distance': distance,
        'initial_speed': speed,
        'min_distance': distance,
        'max_distance': distance,
        'start_timestamp': current_time,
        'approach_complete_time': None,  
        'maintain_start_time': None,
        'maintain_complete_time': None,  
        'retreat_start_time': None,      
        'retreat_complete_time': None,   
        'new_maintain_start_time': None 
        })
    
        logging.info(f"""
        Physical Challenge Protocol Activated:
        - Time: {current_time:.2f}s
        - Initial Distance: {distance:.2f}m
        - Initial Speed: {speed:.2f}m/s
        - Target Duration: {self.challenge_duration}s
        - Starting Phase: APPROACH,
        - Target Distance: {self.target_distance}m
        - Phase Sequence: APPROACH -> MAINTAIN -> RETREAT -> NEW_MAINTAIN
        - Phase Distances: {self.phase_distances}
        - Phase Durations: {self.phase_durations}
        """)

    def complete_challenge(self):
        current_time = self.carla.world.get_snapshot().timestamp.elapsed_seconds
    
        self.challenge_active = False
        self.challenge_state = 'COMPLETED'
        self.current_phase = self.protocol_phases['COMPLETED']
        self.challenge_metrics['end_timestamp'] = current_time
    
        duration = current_time - self.challenge_metrics['activation_time']

        approach_time = self.challenge_metrics.get('approach_complete_time', 'N/A')
        maintain_time = self.challenge_metrics.get('maintain_start_time', 'N/A')
        maintain_complete_time = self.challenge_metrics.get('maintain_complete_time', 'N/A')
        retreat_start_time = self.challenge_metrics.get('retreat_start_time', 'N/A')
        retreat_complete_time = self.challenge_metrics.get('retreat_complete_time', 'N/A')
        new_maintain_start_time = self.challenge_metrics.get('new_maintain_start_time', 'N/A')

        # 计算每个阶段的持续时间
        def calculate_phase_duration(start, end):
            if start != 'N/A' and end != 'N/A':
                return f"{end - start:.2f}"
            return 'N/A'
    
        distance_variation = self.challenge_metrics['max_distance'] - self.challenge_metrics['min_distance']
    
        logging.info(f"""
        Physical Challenge Protocol Completed:
        - Total Duration: {duration:.2f}s
        - Phase Durations:
        * Approach: {calculate_phase_duration(self.challenge_metrics['activation_time'], approach_time)}s
        * Maintain: {calculate_phase_duration(maintain_time, maintain_complete_time)}s
        * Retreat: {calculate_phase_duration(retreat_start_time, retreat_complete_time)}s
        * New Maintain: {calculate_phase_duration(new_maintain_start_time, current_time)}s
    
        Distance Metrics:
        - Initial Distance: {self.challenge_metrics['initial_distance']:.2f}m
        - Minimum Distance: {self.challenge_metrics['min_distance']:.2f}m
        - Maximum Distance: {self.challenge_metrics['max_distance']:.2f}m
        - Distance Variation: {distance_variation:.2f}m
    
        Phase Transitions:
        - Approach -> Maintain: {approach_time if approach_time != 'N/A' else 'Not Reached'}
        - Maintain -> Retreat: {maintain_complete_time if maintain_complete_time != 'N/A' else 'Not Reached'}
        - Retreat -> New Maintain: {retreat_complete_time if retreat_complete_time != 'N/A' else 'Not Reached'}
        """)


    def update_protocol_phase(self, current_time, distance):
        if not self.challenge_active or self.protocol_completed: 
            return

        if not hasattr(self, 'phase_start_time'):
            self.phase_start_time = current_time
            return

        old_phase = self.current_phase        
        phase_duration = self.phase_durations.get(self.current_phase, 0)
        elapsed_time = current_time - self.phase_start_time
        total_elapsed_time = current_time - self.challenge_start_time
    
        if self.current_phase == self.protocol_phases['APPROACH']:
            if elapsed_time >= phase_duration and abs(distance - self.target_distance) < self.distance_threshold:
                self.current_phase = self.protocol_phases['MAINTAIN']
                self.target_distance = self.phase_distances['MAINTAIN']
                self.phase_start_time = current_time
                self.challenge_metrics['approach_complete_time'] = current_time
                self.challenge_metrics['maintain_start_time'] = current_time
                logging.info(f"Phase changed to MAINTAIN at time {current_time:.2f}s")

        elif self.current_phase == self.protocol_phases['MAINTAIN']:
            if elapsed_time >= phase_duration and abs(distance - self.target_distance) < self.distance_threshold:
                self.current_phase = self.protocol_phases['RETREAT']
                self.target_distance = self.phase_distances['RETREAT']
                self.phase_start_time = current_time
                self.challenge_metrics['maintain_complete_time'] = current_time
                self.challenge_metrics['retreat_start_time'] = current_time
                logging.info(f"Phase changed to RETREAT at time {current_time:.2f}s")
            
        elif self.current_phase == self.protocol_phases['RETREAT']:
            if elapsed_time >= phase_duration and abs(distance - self.target_distance) < self.distance_threshold:
                self.current_phase = self.protocol_phases['NEW_MAINTAIN']
                self.target_distance = self.phase_distances['NEW_MAINTAIN']
                self.phase_start_time = current_time
                self.challenge_metrics['retreat_complete_time'] = current_time
                self.challenge_metrics['new_maintain_start_time'] = current_time
                logging.info(f"Phase changed to NEW_MAINTAIN at time {current_time:.2f}s")
            
        elif self.current_phase == self.protocol_phases['NEW_MAINTAIN']:
            if total_elapsed_time >= self.challenge_duration:
                self.current_phase = self.protocol_phases['COMPLETED']
                self.protocol_completed = True
                self.complete_challenge()
                logging.info(f"Protocol completed at time {current_time:.2f}s")
        
        if old_phase != self.current_phase:
            self.plot_data['phase_changes'].append((current_time, self.current_phase))
            logging.info(f"Phase changed from {old_phase} to {self.current_phase} at time {current_time:.2f}s")
            
        logging.info("Current phase: %s, Distance: %.2fm, Target: %.2fm, Elapsed time in phase: %.2fs", 
                 self.current_phase, distance, self.target_distance, elapsed_time)

    def process_rssi_data(self, raw_data):
        rssi_data = json.loads(raw_data.decode())
        rssi = rssi_data['rssi']
        distance = calculate_distance(rssi)
        X_pred = self.poly_features.transform([[distance]])
        improved_distance = self.rssi_model.predict(X_pred)[0]
        return improved_distance, rssi_data['rssi'], rssi_data['messageSendTime'], rssi_data['messageReceiveTime']
    
    def get_speed_difference(self):
        lead_speed = traci.vehicle.getSpeed(self.lead_vehicle_id)
        following_speed = traci.vehicle.getSpeed(self.following_vehicle_id)
        return abs(lead_speed - following_speed)
    

    def calculate_safe_speed(self, gap, speed, lead_speed): # adapted from Krauss follow speed calculation
        tau = self.krauss_params['tau']
        decel = self.krauss_params['decel']
        pred_decel = self.krauss_params['decel']
    
        if gap < 0.01:
            return 0
        
        # bracket_term = 2 * decel * (gap - speed * tau) + lead_speed * lead_speed
        bracket_term = 2 * decel * gap + lead_speed * lead_speed
        
        if bracket_term < 0:
            return 0
        
        # safe_speed = math.sqrt(bracket_term)
        safe_speed = -tau * decel + math.sqrt(bracket_term)

        # safe_speed = safe_speed * 1.1

        return safe_speed
    
    def calculate_protocol_target_speed(self, distance, current_speed, lead_speed):
        """协议状态下的目标速度计算"""
        distance_error = distance - self.target_distance
    
        if self.current_phase == self.protocol_phases['APPROACH']:
            # 接近阶段的速度控制
            if distance_error > 0:
                speed_factor = 1.2
            
            else:
                speed_factor = 0.9
            
            target_speed = lead_speed * speed_factor

            speed_adjustment = distance_error * 0.5  
            target_speed += speed_adjustment

        elif self.current_phase == self.protocol_phases['RETREAT']:
            # 后退阶段的速度控制 - 需要更大的速度差以实现后退
            if distance_error > 0:
                speed_factor = 1.3  # 更大的加速系数
            else:
                speed_factor = 0.8  # 更大的减速系数
        
            target_speed = lead_speed * speed_factor
            speed_adjustment = distance_error * 0.7  # 更大的调整系数
            target_speed += speed_adjustment
           
        else:  # MAINTAIN or NEW MAINTAIN phase
                # 保持阶段的速度控制
            if abs(distance_error) < self.distance_threshold:
                speed_factor = 1.0
            elif distance_error > 0:
                speed_factor = 1.1
            else:
                speed_factor = 0.9

            target_speed = lead_speed * speed_factor

        target_speed = np.clip(target_speed, 0, lead_speed * 1.2)
    
        return target_speed
    
    def calculate_normal_following_speed(self, distance, current_speed, lead_speed, current_time):
        """非协议状态下的跟车速度计算"""
        # 基础跟随距离
        base_following_distance = 10
    
        # 根据速度调整安全距离
        safe_distance = base_following_distance + current_speed * 0.1
    
        # 计算距离误差
        distance_error = distance - safe_distance
    
        # 速度调整系数
        if distance_error > 0:  # 距离过大
            if distance_error > 5:  # 距离明显过大
                speed_factor = 1.15  # 稍微加速
            else:
                speed_factor = 1.08  # 轻微加速
        else:  # 距离过小
            if distance_error < -5:  # 距离明显过小
                speed_factor = 0.92  # 稍微减速
            else:
                speed_factor = 0.95  # 轻微减速
    
        # 计算预期速度变化
        anticipated_lead_speed = self.base_speed + self.speed_amplitude * math.sin(2 * math.pi * (current_time + 0.5) / 10.0)
    
        # 结合当前lead车速度和预期速度
        target_speed = min(lead_speed * speed_factor, anticipated_lead_speed * 1.2)
    
        return target_speed
    

    def control_lead_vehicle(self):
        current_time = self.carla.world.get_snapshot().timestamp.elapsed_seconds
    
        if self.challenge_active:
        # 在协议激活期间保持恒定速度
            current_speed = self.base_speed
        else:
            # 正常状态下的正弦速度控制
            current_speed = self.base_speed + self.speed_amplitude * math.sin(2 * math.pi * current_time / self.speed_period)
    
        traci.vehicle.setSpeed(self.lead_vehicle_id, current_speed)
        return current_speed

    def control_following_vehicle(self):
   
        traci.vehicle.setLaneChangeMode(self.following_vehicle_id, 0b0000000000)
        traci.vehicle.setSpeedMode(self.following_vehicle_id, 0)

        # 获取领航车辆信息
        leader = traci.vehicle.getLeader(self.following_vehicle_id)
        if not leader:
            current_speed = traci.vehicle.getSpeed(self.following_vehicle_id)
            traci.vehicle.setSpeed(self.following_vehicle_id, current_speed)
            print("No leader found")
            return

        # 基本参数获取
        lead_vehicle_id, sumo_distance = leader
        distance = sumo_distance + 5  # 补偿距离
        lead_speed = traci.vehicle.getSpeed(lead_vehicle_id)
        current_speed = traci.vehicle.getSpeed(self.following_vehicle_id)
        current_time = self.carla.world.get_snapshot().timestamp.elapsed_seconds
    
        # 处理RSSI数据
        try:
            data, _ = self.rssi_socket.recvfrom(1024)
            improved_distance, rssi, send_time, receive_time = self.process_rssi_data(data)
            self.first_rssi_received = True
            self.latest_rssi_distance = improved_distance
            print(f"rssi received: {rssi:.2f}dbm") 

        except (socket.error, socket.timeout):
            pass

        # 协议激活检查
        if self.first_rssi_received and distance < 12 and not self.challenge_active and not self.protocol_completed:
            self.activate_challenge(distance, lead_speed)
        
        if self.challenge_active:
            self.update_protocol_phase(current_time, distance)
            target_speed = self.calculate_protocol_target_speed(distance, current_speed, lead_speed)
            print(f"Protocol state - Phase: {self.current_phase}, "  f"Time in phase: {current_time - self.phase_start_time:.2f}s")
        
        else:
            # 非协议状态下的跟车控制
            target_speed = self.calculate_normal_following_speed(distance, current_speed, lead_speed, current_time)

        safe_speed = self.calculate_safe_speed(distance, current_speed, lead_speed)
        
        final_speed = min(safe_speed, target_speed)

        print(f"- Krauss Safe Speed: {safe_speed:.2f} m/s")
        print(f"- Target Speed: {target_speed:.2f} m/s")
        print(f"- Final Speed: {final_speed:.2f} m/s")



        # 应用加速度限制
        accel = (final_speed - current_speed) / 0.1 # (carla.step_length)
        accel = np.clip(accel, -self.krauss_params['decel'], self.krauss_params['accel'])
        final_speed = current_speed + accel * 0.1 # (carla.step_length)
        # 设置速度
        traci.vehicle.setSpeed(self.following_vehicle_id, max(0, final_speed))

        self.update_challenge_metrics(distance)

        # 更新EKF和传感器数据处理
        self.update_sensor_data(distance, current_speed, lead_speed)

        # Update plot data  
        distance_estimate = np.sqrt(self.ekf.x[0]**2 + self.ekf.x[1]**2)  # 使用x,y计算估计距离
        angle_estimate = self.ekf.x[2]  # 使用状态向量中的第三个元素作为角度
        covariance = self.ekf.P
        ego_compass = self.get_vehicle_compass(self.following_vehicle_id)
        lead_compass = self.get_vehicle_compass(lead_vehicle_id)
        true_angle = self.calculate_true_relative_angle(ego_compass, lead_compass)

        self.update_plot_data(distance_estimate, 
                         angle_estimate,
                         distance, 
                         true_angle, 
                         current_speed, 
                         lead_speed, 
                         self.current_phase)
        
        print(f"Following - Distance: {distance:.2f}m, Speed: {current_speed:.2f}m/s") 


    def update_sensor_data(self, distance, current_speed, lead_speed):
        
        self.ekf.predict()
    
        # 处理激光雷达数据
        lidar_points = self.get_lidar_data(self.following_vehicle_id)
        rssi_distance = self.latest_rssi_distance if hasattr(self, 'latest_rssi_distance') else None

        if lidar_points and rssi_distance:
            ego_compass = self.get_vehicle_compass(self.following_vehicle_id)
            lidar_coords = [point['point'] for point in lidar_points]
            lidar_measurement = process_lidar_data(lidar_coords, 0, 0, ego_compass, 
                                             self.carla.world.get_snapshot().timestamp.elapsed_seconds)
        
            if lidar_measurement:
                # 动态调整EKF的测量噪声
                point_spread = lidar_measurement['error_analysis']['point_spread']
                angle_std = lidar_measurement['error_analysis']['angle_std']
            
                # 更新EKF的测量噪声矩阵
                self.ekf.R = np.diag([
                    max(0.1, point_spread),
                    max(0.1, point_spread),
                    0.1,  # RSSI距离测量噪声
                    max(0.1, angle_std)  # 角度测量噪声
                ])
            
                # 构造测量向量 [rx, ry, rssi_distance, angle]
                z = np.array([
                    lidar_measurement['rx'],
                    lidar_measurement['ry'],
                    rssi_distance,
                    lidar_measurement['angle']
                ])
            
                # EKF更新步骤
                if np.all(np.isfinite(z)):
                    self.ekf.update(z)
                else:
                    print("Warning: Invalid measurement detected")
            

    def update_plot_data(self, estimated_distance, estimated_angle, true_distance, true_angle, 
                    following_speed, lead_speed, current_phase):
    
        current_time = self.carla.world.get_snapshot().timestamp.elapsed_seconds
        self.plot_data['timestamps'].append(current_time)
        self.plot_data['estimated_distances'].append(estimated_distance)
        self.plot_data['true_distances'].append(true_distance)
        self.plot_data['estimated_angles'].append(estimated_angle)
        self.plot_data['true_angles'].append(true_angle)
        self.plot_data['lead_speeds'].append(lead_speed)
        self.plot_data['following_speeds'].append(following_speed)

        if not self.challenge_active:
            self.plot_data['phases'].append('INACTIVE')
        else:
            self.plot_data['phases'].append(current_phase)
        

    def calculate_true_relative_angle(self, ego_compass, other_compass):
        relative_angle = self.normalize_angle(other_compass - ego_compass)
        return relative_angle

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def setup_sensors(self, vehicle, vehicle_id):
        # Setup LiDAR
        lidar_bp = self.carla.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('points_per_second', '56000')
        lidar_bp.set_attribute('range', '30')
        lidar_bp.set_attribute('rotation_frequency', '10')
        lidar_bp.set_attribute('sensor_tick', '0.1')

        lidar_location = carla.Location(x=0.0, y=0, z=2.0)
        lidar_rotation = carla.Rotation(pitch=5)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        lidar_semantic = self.carla.world.spawn_actor(
            lidar_bp, lidar_transform, attach_to=vehicle, 
            attachment_type=carla.AttachmentType.Rigid
        )
        lidar_semantic.listen(lambda data: self.on_sensor_data(vehicle_id, 'lidar_semantic', data))

        # Setup IMU
        imu_bp = self.carla.world.get_blueprint_library().find('sensor.other.imu')
        imu_bp.set_attribute('noise_accel_stddev_x', '0.01')
        imu_bp.set_attribute('noise_accel_stddev_y', '0.01')
        imu_bp.set_attribute('noise_accel_stddev_z', '0.01')
        imu_bp.set_attribute('sensor_tick', '0.1')

        imu_location = carla.Location(x=0, y=0, z=0)
        imu_rotation = carla.Rotation(0, 0, 0)
        imu_transform = carla.Transform(imu_location, imu_rotation)
        imu_sensor = self.carla.world.spawn_actor(
            imu_bp, imu_transform, attach_to=vehicle, 
            attachment_type=carla.AttachmentType.Rigid
        )
        imu_sensor.listen(lambda data: self.on_sensor_data(vehicle_id, 'imu_sensor', data))

        # Initialize sensor storage
        if vehicle_id not in self.sensors:
            self.sensors[vehicle_id] = {}
        self.sensors[vehicle_id].update({
            'lidar_semantic': lidar_semantic,
            'imu_sensor': imu_sensor
        })

        if vehicle_id not in self.sensor_data:
            self.sensor_data[vehicle_id] = {}
        self.sensor_data[vehicle_id].update({
            'lidar_semantic': [],
            'imu_sensor': []
        })

        return lidar_semantic, imu_sensor


    def on_sensor_data(self, vehicle_id, sensor_type, data):
        processed_data = self.process_sensor_data(data, sensor_type, vehicle_id)
        if vehicle_id not in self.sensor_data:
            self.sensor_data[vehicle_id] = {}
        if sensor_type not in self.sensor_data[vehicle_id]:
            self.sensor_data[vehicle_id][sensor_type] = []
        self.sensor_data[vehicle_id][sensor_type].append(processed_data)

    def process_sensor_data(self, data, sensor_type, vehicle_id):
        """Process raw sensor data"""
        if sensor_type == 'lidar_semantic':
            timestamp = data.timestamp
            vehicle_lidar_data = []
            for detection in data:
                if detection.object_tag == 14 and str(detection.object_idx) != str(vehicle_id):
                    vehicle_lidar_data.append({
                        'point': [detection.point.x, detection.point.y, detection.point.z],
                        'object_idx': detection.object_idx
                    })
            return {
                'timestamp': timestamp,
                'vehicle_lidar_data': vehicle_lidar_data
            }
        elif sensor_type == 'imu_sensor':
            return {
                'timestamp': data.timestamp,
                'compass': data.compass
            }
        return None
    
    def get_lidar_data(self, sumo_vehicle_id):
        carla_vehicle_id = self.sumo2carla_ids.get(sumo_vehicle_id)
        if carla_vehicle_id is None:
            return []
        
        if carla_vehicle_id in self.sensor_data and 'lidar_semantic' in self.sensor_data[carla_vehicle_id]:
            return self.sensor_data[carla_vehicle_id]['lidar_semantic'][-1]['vehicle_lidar_data']
        return []


    def get_vehicle_compass(self, vehicle_id):
        """Get compass reading from IMU sensor"""
        carla_vehicle_id = self.sumo2carla_ids.get(vehicle_id)
        if carla_vehicle_id is None:
            return 0
        
        if (carla_vehicle_id in self.sensor_data and 
            'imu_sensor' in self.sensor_data[carla_vehicle_id] and 
            self.sensor_data[carla_vehicle_id]['imu_sensor']):
            return self.sensor_data[carla_vehicle_id]['imu_sensor'][-1]['compass']
        return 0
    
    def remove_sensors(self, vehicle_id):
        """Clean up sensors when removing a vehicle"""
        if vehicle_id in self.sensors:
            for sensor in self.sensors[vehicle_id].values():
                sensor.destroy()
            del self.sensors[vehicle_id]
        if vehicle_id in self.sensor_data:
            del self.sensor_data[vehicle_id]
    
    def tick(self):
            # -----------------
            # sumo-->carla sync
            # -----------------
        try:  
            
            self.sumo.tick()

            # Spawning new sumo actors in carla (i.e, not controlled by carla).
            sumo_spawned_actors = self.sumo.spawned_actors - set(self.carla2sumo_ids.values())
            self.control_lead_vehicle()
            self.control_following_vehicle()

            for sumo_actor_id in sumo_spawned_actors:
                self.sumo.subscribe(sumo_actor_id)
                sumo_actor = self.sumo.get_actor(sumo_actor_id)

                carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor, self.sync_vehicle_color)
                if carla_blueprint is not None:
                    carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                                   sumo_actor.extent)

                    carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                    carla_actor = self.carla.get_actor(carla_actor_id)
                    lidar_semantic, imu_sensor= self.setup_sensors(carla_actor, carla_actor_id)

                    if carla_actor_id != INVALID_ACTOR_ID:
                        self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
                else:
                    self.sumo.unsubscribe(sumo_actor_id)
           

            for carla_actor_id in self.carla.destroyed_actors:
                self.remove_sensors(carla_actor_id)

            # Destroying sumo arrived actors in carla.
            for sumo_actor_id in self.sumo.destroyed_actors:
                if sumo_actor_id in self.sumo2carla_ids:
                    self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))

            # Updating sumo actors in carla.
            for sumo_actor_id in self.sumo2carla_ids:
                carla_actor_id = self.sumo2carla_ids[sumo_actor_id]

                sumo_actor = self.sumo.get_actor(sumo_actor_id)
                carla_actor = self.carla.get_actor(carla_actor_id)

                carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                               sumo_actor.extent)
                if self.sync_vehicle_lights:
                    carla_lights = BridgeHelper.get_carla_lights_state(carla_actor.get_light_state(),
                                                                   sumo_actor.signals)
                else:
                    carla_lights = None

                self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)

            # Updates traffic lights in carla based on sumo information.
            if self.tls_manager == 'sumo':
                common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
                for landmark_id in common_landmarks:
                    sumo_tl_state = self.sumo.get_traffic_light_state(landmark_id)
                    carla_tl_state = BridgeHelper.get_carla_traffic_light_state(sumo_tl_state)

                    self.carla.synchronize_traffic_light(landmark_id, carla_tl_state)

            # -----------------
            # carla-->sumo sync
            # -----------------
            self.carla.tick()

            # Spawning new carla actors (not controlled by sumo)
            carla_spawned_actors = self.carla.spawned_actors - set(self.sumo2carla_ids.values())
            sim_time = self.carla.world.get_snapshot().timestamp.elapsed_seconds
            print( f"carla time at: {sim_time}")

            all_vehicles = self.carla.world.get_actors().filter('vehicle.*')


            for carla_actor_id in carla_spawned_actors:
                carla_actor = self.carla.get_actor(carla_actor_id)

                type_id = BridgeHelper.get_sumo_vtype(carla_actor)
                color = carla_actor.attributes.get('color', None) if self.sync_vehicle_color else None
                if type_id is not None:
                    sumo_actor_id = self.sumo.spawn_actor(type_id, color)
                    if sumo_actor_id != INVALID_ACTOR_ID:
                        self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                        self.sumo.subscribe(sumo_actor_id)

            # Destroying required carla actors in sumo.
            for carla_actor_id in self.carla.destroyed_actors:
                if carla_actor_id in self.carla2sumo_ids:
                    self.sumo.destroy_actor(self.carla2sumo_ids.pop(carla_actor_id))

            # Updating carla actors in sumo.
            for carla_actor_id in self.carla2sumo_ids:
                sumo_actor_id = self.carla2sumo_ids[carla_actor_id]

                carla_actor = self.carla.get_actor(carla_actor_id)
                sumo_actor = self.sumo.get_actor(sumo_actor_id)

                sumo_transform = BridgeHelper.get_sumo_transform(carla_actor.get_transform(),
                                                             carla_actor.bounding_box.extent)
                if self.sync_vehicle_lights:
                    carla_lights = self.carla.get_actor_light_state(carla_actor_id)
                    if carla_lights is not None:
                        sumo_lights = BridgeHelper.get_sumo_lights_state(sumo_actor.signals,
                                                                     carla_lights)
                    else:
                        sumo_lights = None
                else:
                    sumo_lights = None

                self.sumo.synchronize_vehicle(sumo_actor_id, sumo_transform, sumo_lights)

            # Updates traffic lights in sumo based on carla information.
            if self.tls_manager == 'carla':
                common_landmarks = self.sumo.traffic_light_ids & self.carla.traffic_light_ids
                for landmark_id in common_landmarks:
                    carla_tl_state = self.carla.get_traffic_light_state(landmark_id)
                    sumo_tl_state = BridgeHelper.get_sumo_traffic_light_state(carla_tl_state)

                    #Updates all the sumo links related to this landmark.
                    self.sumo.synchronize_traffic_light(landmark_id, sumo_tl_state)
        
        except Exception as e:
            logging.error(f"Error in tick: {e}")
            raise

    def close(self):
        """
        Cleans synchronization.
        """
        # Configuring carla simulation in async mode.

        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.sumo2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)
            self.remove_sensors(carla_actor_id)
            
        for sumo_actor_id in self.carla2sumo_ids.values():
            self.sumo.destroy_actor(sumo_actor_id)
       
        # Closing sumo and carla client.
        self.carla.close()
        self.sumo.close()
        self.plot_results()

    def plot_results(self):
        # 创建图形和子图布局
        fig = plt.figure(figsize=(15, 16))
        gs = GridSpec(5, 1, figure=fig, height_ratios=[3, 2, 2, 2, 1])
    
        # 1. 距离跟踪图
        ax1 = fig.add_subplot(gs[0])
        ax1.plot(self.plot_data['timestamps'], self.plot_data['estimated_distances'], 
             'b-', label='Estimated Distance', linewidth=2)
        ax1.plot(self.plot_data['timestamps'], self.plot_data['true_distances'], 
             'r-', label='True Distance', linewidth=2)

        active_indices = [i for i, phase in enumerate(self.plot_data['phases']) if phase != 'INACTIVE']
        if active_indices:
            dist_errors = np.array(self.plot_data['estimated_distances'])[active_indices] - \
                  np.array(self.plot_data['true_distances'])[active_indices]
            dist_rmse = np.sqrt(np.mean(dist_errors**2))
            dist_mae = np.mean(np.abs(dist_errors))
            dist_max_error = np.max(np.abs(dist_errors))
    

            dist_stats = f'RMSE: {dist_rmse:.2f}m\nMAE: {dist_mae:.2f}m\nMax Error: {dist_max_error:.2f}m'
            ax1.text(0.98, 0.02, dist_stats, transform=ax1.transAxes, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'),
                verticalalignment='bottom', horizontalalignment='right', fontsize=10)
    
        # 添加目标距离参考线
        phase_colors = {
        'APPROACH': 'b',
        'MAINTAIN': 'y',
        'RETREAT': 'm',
        'NEW_MAINTAIN': 'r'
        }
    
        for phase, color in phase_colors.items():
            if phase in self.phase_distances:
                target_dist = self.phase_distances[phase]
                ax1.axhline(y=target_dist, color=color, linestyle='--', alpha=0.3, label=f'{phase} Distance ({target_dist}m)')

        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Distance (m)')
        ax1.legend()
        ax1.grid(True)
        ax1.set_title('Vehicle Distance Tracking')

        # 2. 角度误差图
        ax2 = fig.add_subplot(gs[1])
        ax2.plot(self.plot_data['timestamps'], self.plot_data['estimated_angles'], 'b-', label='Estimated Angle', linewidth=2)
        ax2.plot(self.plot_data['timestamps'], self.plot_data['true_angles'], 'r-', label='True Angle', linewidth=2)
        
        angle_errors = np.array(self.plot_data['estimated_angles']) - np.array(self.plot_data['true_angles'])
        ax2.plot(self.plot_data['timestamps'], angle_errors, 'g--', label='Angle Error', linewidth=1.5, alpha=0.7)

        if active_indices:
            angle_errors = np.array(self.plot_data['estimated_angles'])[active_indices] - \
                   np.array(self.plot_data['true_angles'])[active_indices]
            angle_rmse = np.sqrt(np.mean(angle_errors**2))
            angle_mae = np.mean(np.abs(angle_errors))
            angle_max_error = np.max(np.abs(angle_errors))
    

            angle_stats = f'RMSE: {angle_rmse:.3f}rad\nMAE: {angle_mae:.3f}rad\nMax Error: {angle_max_error:.3f}rad'
            ax2.text(0.98, 0.02, angle_stats, transform=ax2.transAxes, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'),
                verticalalignment='bottom', horizontalalignment='right', fontsize=10)

        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angle (rad)')
        ax2.legend()
        ax2.grid(True)
        ax2.set_title('Relative Angle Tracking and Error')

        # 3. 相对距离误差图
        ax3 = fig.add_subplot(gs[2])
        target_distances = []
        for phase in self.plot_data['phases']:
            target_distances.append(self.phase_distances.get(phase, 10))
        distance_errors = np.array(self.plot_data['estimated_distances']) - np.array(target_distances)
        ax3.plot(self.plot_data['timestamps'], distance_errors, 'r-', label='Distance Error', linewidth=2)

        if active_indices:
            target_distances = [self.phase_distances.get(phase, 10) for phase in self.plot_data['phases']]
            target_distances = np.array(target_distances)[active_indices]
            distance_errors = np.array(self.plot_data['estimated_distances'])[active_indices] - target_distances
    
            target_rmse = np.sqrt(np.mean(distance_errors**2))
            target_mae = np.mean(np.abs(distance_errors))
            target_max_error = np.max(np.abs(distance_errors))
    

            target_stats = f'RMSE: {target_rmse:.2f}m\nMAE: {target_mae:.2f}m\nMax Error: {target_max_error:.2f}m'
            ax3.text(0.98, 0.02, target_stats, transform=ax3.transAxes, bbox=dict(facecolor='white', alpha=0.8, edgecolor='none'),
                verticalalignment='bottom', horizontalalignment='right', fontsize=10)

        ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Distance Error (m)')
        ax3.legend()
        ax3.grid(True)
        ax3.set_title('Distance Error from Target')

        # 4. 速度跟踪图
        ax4 = fig.add_subplot(gs[3])
        ax4.plot(self.plot_data['timestamps'], self.plot_data['lead_speeds'], 
             'g-', label='Lead Vehicle Speed', linewidth=2)
        ax4.plot(self.plot_data['timestamps'], self.plot_data['following_speeds'], 
             'm-', label='Following Vehicle Speed', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Speed (m/s)')
        ax4.legend()
        ax4.grid(True)
        ax4.set_title('Vehicle Speed Tracking')

        # 5. 协议阶段时间轴
        ax5 = fig.add_subplot(gs[4])
        phase_colors = {
        'INACTIVE': 'lightgray',
        'APPROACH': 'lightblue',
        'MAINTAIN': 'lightgreen',
        'RETREAT': 'lightcyan',
        'NEW_MAINTAIN': 'plum',
        'COMPLETED': 'wheat'
        }

        if not self.plot_data['phase_changes']:
            self.plot_data['phase_changes'].append((self.plot_data['timestamps'][0], 'INACTIVE'))

        # 添加最后一个时间点
        last_time = max(self.plot_data['timestamps'])
        last_phase = self.plot_data['phases'][-1] if self.plot_data['phases'] else 'COMPLETED'

        if self.plot_data['phase_changes'][-1][0] < last_time:
            self.plot_data['phase_changes'].append((last_time, last_phase))

        if self.plot_data['phase_changes'][0][1] != 'INACTIVE':
            self.plot_data['phase_changes'].insert(0, (min(self.plot_data['timestamps']), 'INACTIVE'))


        for i in range(len(self.plot_data['phase_changes']) - 1):
            current_time, current_phase = self.plot_data['phase_changes'][i]
            next_time, next_phase = self.plot_data['phase_changes'][i + 1]
        
            # 绘制当前阶段块
            ax5.axvspan(current_time, next_time, 
                    ymin=0.1, ymax=0.9,
                    color=phase_colors.get(current_phase, 'white'), 
                    alpha=0.5,
                    label=current_phase)
        
            # 添加阶段标签
            mid_point = (current_time + next_time) / 2
            ax5.text(mid_point, 0.5, current_phase, 
                    ha='center', va='center', 
                    rotation=0,
                    bbox=dict(facecolor='white', alpha=0.7, pad=2))
        
            # 在其他图表中添加阶段转换线
            for ax in [ax1, ax2, ax3, ax4]:
                ax.axvline(x=current_time, color='g', linestyle='--', alpha=0.3)

        # 设置轴标签等
        ax5.set_yticks([])
        ax5.set_xlabel('Time (s)')
        ax5.set_title('Protocol Phases')

        # 添加性能指标文本框
        if hasattr(self, 'challenge_metrics'):
            start_time = self.challenge_metrics.get('start_timestamp')
            end_time = self.challenge_metrics.get('end_timestamp')
            if start_time is not None and end_time is not None:
                maintain_start = self.challenge_metrics.get('maintain_start_time', 0)
                retreat_start = self.challenge_metrics.get('retreat_start_time', 0)
                new_maintain_start = self.challenge_metrics.get('new_maintain_start_time', 0)
            
                metrics_text = f"""
                Protocol Metrics:
                Duration: {end_time - start_time:.1f}s
                Initial Distance: {self.challenge_metrics.get('initial_distance', 0):.1f}m
                Distance Variation: {self.challenge_metrics.get('max_distance', 0) - self.challenge_metrics.get('min_distance', 0):.1f}m
                Approach Phase Duration: {self.challenge_metrics.get('maintain_start_time', 0) - start_time:.1f}s
                Maintain Phase Duration: {retreat_start - maintain_start:.1f}s
                Retreat Phase Duration: {new_maintain_start - retreat_start:.1f}s
                New Maintain Phase Duration: {end_time - new_maintain_start:.1f}s
                """
                plt.figtext(0.75, 0.02, metrics_text, fontsize=10, bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray', boxstyle='round,pad=1'))

        plt.tight_layout(rect=[0, 0.05, 1, 1])  # 为底部文本框留出空间
    
        # 保存图形
        plt.savefig('simulation_results.png', dpi=300, bbox_inches='tight')
        plt.close()

def synchronization_loop(args):

    sumo_simulation = SumoSimulation(args.sumo_cfg_file, args.step_length, args.sumo_host,
                                     args.sumo_port, args.sumo_gui, args.client_order)
    carla_simulation = CarlaSimulation(args.carla_host, args.carla_port, args.step_length)

    synchronization = SimulationSynchronization(sumo_simulation, carla_simulation, args.tls_manager,
                                                args.sync_vehicle_color, args.sync_vehicle_lights)
    try:
        while True:
            start = time.time()
            synchronization.tick()
            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)
    except KeyboardInterrupt:
        logging.info('Cancelled by user.')
    finally:
        logging.info('Cleaning synchronization')
        synchronization.close()

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('sumo_cfg_file', type=str, help='sumo configuration file')
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default=None,
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=None,
                           type=int,
                           help='TCP port to listen to (default: 8813)')
    argparser.add_argument('--sumo-gui', action='store_true', help='run the gui version of sumo')
    argparser.add_argument('--step-length',
                           default=0.1,
                           type=float,
                           help='set fixed delta seconds (default: 0.1s)')
    argparser.add_argument('--client-order',
                           metavar='TRACI_CLIENT_ORDER',
                           default=1,
                           type=int,
                           help='client order number for the co-simulation TraCI connection (default: 1)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-vehicle-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--tls-manager',
                           type=str,
                           choices=['none', 'sumo', 'carla'],
                           help="select traffic light manager (default: none)",
                           default='none')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.sync_vehicle_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)