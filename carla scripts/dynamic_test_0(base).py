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

        # Parameters for sinusoidal speed control
        self.base_speed = 17.5  # m/s
        self.speed_amplitude = 1.4  # m/s
        self.speed_period = 10  # s
        self.target_distance = 20  # m
        self.distance_threshold = 1 # m

        self.krauss_params = {
        'accel': 3,      # 最大加速度 m/s²
        'decel': 4.6,      # 舒适减速度 m/s²
        'emergencyDecel': 9.0,  # 紧急减速度 m/s²
        'tau': 0.5,        # 
        }

        self.initialize_metrics()
        self.initialize_sensors_and_communication()

    def initialize_metrics(self):
     
        self.plot_data = {
        'timestamps': [],
        'true_distances': [],
        'estimated_distances': [],
        'true_angles': [],
        'estimated_angles': [],
        'lead_speeds': [],
        'following_speeds': [],
        }

    def signal_handler(self, sig, frame):
        print('Caught interrupt, plotting results...')
        self.plot_results()
        sys.exit(0)

    def initialize_sensors_and_communication(self):
        # EKF初始化
        #self.ekf = load_ekf_model('my_ekf_model.pkl')

        # 创建新的EKF实例
        init_x = np.array([20, 0, 0])  # 初始状态 [rx, ry, theta]
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
        
        bracket_term = 2 * decel * gap + lead_speed * lead_speed
        
        if bracket_term < 0:
            return 0
        
        safe_speed = -tau * decel + math.sqrt(bracket_term)

        # safe_speed = safe_speed * 1.1

        return safe_speed
    

    def control_lead_vehicle(self):
        current_time = self.carla.world.get_snapshot().timestamp.elapsed_seconds
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

        distance_error = distance - self.target_distance

        if abs(distance_error) < self.distance_threshold:
            speed_factor = 1.0
        elif distance_error > 0:
            speed_factor = 1.1
        else:
            speed_factor = 0.9

        target_speed = lead_speed * speed_factor
        safe_speed = self.calculate_safe_speed(distance, current_speed, lead_speed)
        final_speed = min(safe_speed, target_speed)

        # Apply acceleration limits
        accel = (final_speed - current_speed) / self.carla.step_length
        accel = np.clip(accel, -self.krauss_params['decel'], self.krauss_params['accel'])
        final_speed = current_speed + accel * self.carla.step_length
        
        traci.vehicle.setSpeed(self.following_vehicle_id, max(0, final_speed))

        self.update_sensor_data(distance, current_speed, lead_speed)

        # Update plot data
        distance_estimate = np.sqrt(self.ekf.x[0]**2 + self.ekf.x[1]**2)  # 使用x,y计算估计距离
        angle_estimate = self.ekf.x[2]  # 使用状态向量中的第三个元素作为角度
        ego_compass = self.get_vehicle_compass(self.following_vehicle_id)
        lead_compass = self.get_vehicle_compass(lead_vehicle_id)
        true_angle = self.calculate_true_relative_angle(ego_compass, lead_compass)
        self.update_plot_data(distance_estimate, angle_estimate, distance, true_angle, current_speed, lead_speed)
        
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
                    following_speed, lead_speed):
    
        current_time = self.carla.world.get_snapshot().timestamp.elapsed_seconds
        self.plot_data['timestamps'].append(current_time)
        self.plot_data['estimated_distances'].append(estimated_distance)
        self.plot_data['true_distances'].append(true_distance)
        self.plot_data['estimated_angles'].append(estimated_angle)
        self.plot_data['true_angles'].append(true_angle)
        self.plot_data['lead_speeds'].append(lead_speed)
        self.plot_data['following_speeds'].append(following_speed)

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
        fig = plt.figure(figsize=(12, 12))
        gs = GridSpec(3, 1, figure=fig, height_ratios=[1, 1, 1])

        # Distance plot
        ax1 = fig.add_subplot(gs[0])
        ax1.plot(self.plot_data['timestamps'], self.plot_data['estimated_distances'], 
                'b-', label='Estimated Distance', linewidth=1)
        ax1.plot(self.plot_data['timestamps'], self.plot_data['true_distances'], 
                'r-', label='True Distance', linewidth=1)

        ax1.set_title('Distance Tracking')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Distance (m)')
        ax1.legend()
        ax1.grid(True)

        # Angle plot
        ax2 = fig.add_subplot(gs[1])
        ax2.plot(self.plot_data['timestamps'], self.plot_data['estimated_angles'], 
                'g-', label='Estimated Angle', linewidth=1)
        ax2.plot(self.plot_data['timestamps'], self.plot_data['true_angles'], 
                'm-', label='True Angle', linewidth=1)
        
        ax2.set_title('Angle Tracking')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Angle (rad)')
        ax2.legend()
        ax2.grid(True)

        # Speed comparison plot
        ax3 = fig.add_subplot(gs[2])
        ax3.plot(self.plot_data['timestamps'], self.plot_data['lead_speeds'], 
                'g-', label='Lead Vehicle Speed', linewidth=1)
        ax3.plot(self.plot_data['timestamps'], self.plot_data['following_speeds'], 
                'm-', label='Following Vehicle Speed', linewidth=1)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Speed (m/s)')
        ax3.set_title('Vehicle Speed Comparison')
        ax3.legend()
        ax3.grid(True)

        plt.tight_layout()
        plt.savefig('dynamic_test_result.png', dpi=300, bbox_inches='tight')
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