
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

try:
    sys.path.append(
        glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' %
                  (sys.version_info.major, sys.version_info.minor,
                   'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import sumolib

# ==================================================================================================
# -- sumo integration imports ----------------------------------------------------------------------
# ==================================================================================================

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================

class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """
    def __init__(self,
                 sumo_simulation,
                 carla_simulation,
                 tls_manager='none',
                 sync_vehicle_color=False,
                 sync_vehicle_lights=False):

        self.sumo = sumo_simulation
        self.carla = carla_simulation

        self.tls_manager = tls_manager
        self.sync_vehicle_color = sync_vehicle_color
        self.sync_vehicle_lights = sync_vehicle_lights

        self.lead_vehicle_id = '0'
        self.following_vehicle_id = '1'

        self.speed_change_probability = 0.2  # 每个时间步改变速度的概率
        self.max_speed_change = 2  # m/s，最大速度变化量
        self.min_speed = 10 # m/s
        self.max_speed = 15  # m/s
        self.defaultspeed =  30 # m/s

        self.output_directory = "vehicle_data"  # 输出目录
        if not os.path.exists(self.output_directory):
            os.makedirs(self.output_directory)

        if tls_manager == 'carla':
            self.sumo.switch_off_traffic_lights()
        elif tls_manager == 'sumo':
            self.carla.switch_off_traffic_lights()

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

        self.sensors = {} # track sensor
        self.sensor_data = {} # store sensor data
        self.vehicle_positions = {}  # 存储车辆位置
        self.vehicle_speed = {} #save vehicle speed from sumo
        self.sumo_distance = [] #save distance from sumo

    def control_lead_vehicle(self):
     
        current_speed = traci.vehicle.getSpeed(self.lead_vehicle_id)
        traci.vehicle.setSpeedMode(self.following_vehicle_id, 0)
        traci.vehicle.setMinGap(self.following_vehicle_id, 2)
        
        if self.lead_vehicle_id not in self.vehicle_speed:
            self.vehicle_speed[self.lead_vehicle_id] = []
        
        self.vehicle_speed[self.lead_vehicle_id].append({
                'timestamp':  self.carla.world.get_snapshot().timestamp.elapsed_seconds,
                'speed': current_speed})
        leader_speed = self.vehicle_speed[self.lead_vehicle_id][-1]     
        self.save_speed_data(self.lead_vehicle_id,leader_speed)
                                                                                   
        if random.random() < self.speed_change_probability:
            speed_change = random.uniform(-self.max_speed_change, self.max_speed_change)
            new_speed = max(self.min_speed, min(self.max_speed, current_speed + speed_change))
            traci.vehicle.setSpeed(self.lead_vehicle_id, new_speed)
            print(f"Leader speed changed: {current_speed:.2f} -> {new_speed:.2f} m/s")
        else:
            print(f"Leader maintaining speed: {current_speed:.2f} m/s")
    
    def control_following_vehicle(self):
    
        traci.vehicle.setLaneChangeMode(self.following_vehicle_id, 0b0000000000)  # 禁用变道
        traci.vehicle.setSpeedMode(self.following_vehicle_id, 0)
        traci.vehicle.setMinGap(self.following_vehicle_id, 2)
        
        leader = traci.vehicle.getLeader(self.following_vehicle_id)
        
        if self.following_vehicle_id not in self.vehicle_speed:
            self.vehicle_speed[self.following_vehicle_id] = []
        
        speed = traci.vehicle.getSpeed(self.following_vehicle_id)
        self.vehicle_speed[self.following_vehicle_id].append({
            'timestamp': self.carla.world.get_snapshot().timestamp.elapsed_seconds,
            'speed': speed}) 
        following_speed = self.vehicle_speed[self.following_vehicle_id][-1]
        self.save_speed_data(self.following_vehicle_id,following_speed)
        
        if leader:
            lead_vehicle_id, distance = leader
            current_speed = traci.vehicle.getSpeed(self.following_vehicle_id)
            lead_speed = traci.vehicle.getSpeed(lead_vehicle_id)
            maxdel = traci.vehicle.getDecel(lead_vehicle_id)
            secgap = traci.vehicle.getSecureGap(self.following_vehicle_id,current_speed,lead_speed,maxdel,lead_vehicle_id)
            
            self.sumo_distance.append({
            'timestamp': self.carla.world.get_snapshot().timestamp.elapsed_seconds,
            'distance': distance}
            )
            sumo_distance = self.sumo_distance[-1]
            self.save_sumo_distance_data(sumo_distance)

            target_distance = 5 # 目标跟车距离（米）
            max_accel = 2.6  # 最大加速度 (m/s^2)
            max_decel = 4.5  # 最大减速度 (m/s^2) based on vehicle parameter
        
            # 简单控制器
            speed_diff = lead_speed - current_speed
            distance_error = distance - target_distance
        
            # 调整速度以达到目标距离
            speed_adjustment = distance_error * 0.5 + speed_diff
        
            # 限制加速度和减速度
            speed_adjustment = max(-max_decel, min(max_accel, speed_adjustment))
        
            new_speed = current_speed + speed_adjustment
            new_speed = max(0, new_speed)  # 确保速度非负
        
            traci.vehicle.setSpeed(self.following_vehicle_id, new_speed)
        
            print(f"Following - Distance: {distance:.2f}m, secgap:{secgap:.2f}, Speed: {current_speed:.2f} -> {new_speed:.2f} m/s")
        else:
            print("No leader found, maintaining default speed")
            traci.vehicle.setSpeed(self.following_vehicle_id, self.defaultspeed)

    def setup_sensors(self, vehicle, vehicle_id):

        # setup lidar 
        lidar_bp = self.carla.world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
        lidar_bp.set_attribute('channels', '32')
        lidar_bp.set_attribute('points_per_second', '56000')
        lidar_bp.set_attribute('range', '30')
        lidar_bp.set_attribute('rotation_frequency','10') #10Hz
        lidar_bp.set_attribute('sensor_tick','0.1')

        lidar_location = carla.Location(x=2.0, y=0, z=2.0)
        lidar_rotation = carla.Rotation(pitch=5)
        
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        lidar_semantic = self.carla.world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
        lidar_semantic.listen(lambda data: self.on_sensor_data(vehicle_id, 'lidar_semantic', data))

        # 设置GNSS传感器
        gnss_bp = self.carla.world.get_blueprint_library().find('sensor.other.gnss')
        gnss_bp.set_attribute('sensor_tick', '0.1')  
        gnss_bp.set_attribute('noise_lat_stddev', '1.0')  # 纬度标准差约1米
        gnss_bp.set_attribute('noise_lon_stddev', '1.0')  # 经度标准差约1米
        gnss_bp.set_attribute('noise_seed', '2024')  # 设置随机种子

        gnss_location = carla.Location(x=0, y=0, z=0)
        gnss_rotation = carla.Rotation(0, 0, 0)
        gnss_transform = carla.Transform(gnss_location, gnss_rotation)
        
        gnss_sensor = self.carla.world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
        
        gnss_sensor.listen(lambda data: self.on_sensor_data(vehicle_id, 'gnss_sensor', data))

        # 设置 IMU
        imu_bp = self.carla.world.get_blueprint_library().find('sensor.other.imu')
        # 设置加速度计噪声
        imu_bp.set_attribute('noise_accel_stddev_x', '0.01')
        imu_bp.set_attribute('noise_accel_stddev_y', '0.01')
        imu_bp.set_attribute('noise_accel_stddev_z', '0.01')
    
        # 设置陀螺仪偏差
        #imu_bp.set_attribute('noise_gyro_bias_x', '0.005')
        #imu_bp.set_attribute('noise_gyro_bias_y', '0.005')
        #imu_bp.set_attribute('noise_gyro_bias_z', '0.005')
    
        # 设置陀螺仪噪声
        #imu_bp.set_attribute('noise_gyro_stddev_x', '0.01')
        #imu_bp.set_attribute('noise_gyro_stddev_y', '0.01')
        #imu_bp.set_attribute('noise_gyro_stddev_z', '0.01')
    
        # 设置噪声种子和传感器刷新率
        #imu_bp.set_attribute('noise_seed', '2024')
        imu_bp.set_attribute('sensor_tick', '0.1')  
        imu_location = carla.Location(x=0, y=0, z=0)
        imu_rotation = carla.Rotation(0, 0, 0)
        imu_transform = carla.Transform(imu_location, imu_rotation)
        imu_sensor = self.carla.world.spawn_actor(imu_bp, imu_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
        imu_sensor.listen(lambda data: self.on_sensor_data(vehicle_id, 'imu_sensor', data))

        """  # 设置雷达传感器
        radar_bp = self.carla.world.get_blueprint_library().find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', str(35))
        radar_bp.set_attribute('vertical_fov', str(20))
        radar_bp.set_attribute('range', str(25))
        radar_bp.set_attribute('sensor_tick', '0.1')  # 设置传感器刷新率

        radar_location = carla.Location(x=2.0, z=1.0)
        radar_rotation = carla.Rotation(pitch=5)
        radar_transform = carla.Transform(radar_location, radar_rotation)
    
        radar_sensor = self.carla.world.spawn_actor(radar_bp, radar_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
        radar_sensor.listen(lambda data: self.on_sensor_data(vehicle_id, 'radar_sensor', data)) """

        """  # setup camera
        camera_bp = self.carla.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1080')
        camera_bp.set_attribute('fov', '90')
        camera_bp.set_attribute('sensor_tick', '0.1')
        
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        
        camera = self.carla.world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid) """

        self.sensors[vehicle_id] = {'lidar_semantic': lidar_semantic, 'gnss_sensor':gnss_sensor, 'imu_sensor': imu_sensor}
        self.sensor_data[vehicle_id] = {'lidar_semantic': [], 'gnss_sensor': [], 'imu_sensor': []}

        #camera.listen(lambda data: self.on_sensor_data(vehicle_id, 'camera', data))

        return lidar_semantic, gnss_sensor, imu_sensor
   
    def process_sensor_data(self, data, sensor_type,vehicle_id):
        if sensor_type == 'lidar_semantic':
            timestamp = data.timestamp
            vehicle_lidar_data = []
            for detection in data:
                if detection.object_tag == 14 and str(detection.object_idx) != str(vehicle_id):  # 只处理车辆（tag为14）and not self vehicle
                    vehicle_lidar_data.append({
                    'point': [detection.point.x, detection.point.y, detection.point.z],
                    'object_idx': detection.object_idx
                })
        
            return {
            'timestamp': timestamp,
            'vehicle_lidar_data': vehicle_lidar_data
        }
        
        elif sensor_type == 'gnss_sensor':
            timestamp = data.timestamp
            transform = data.transform
            return {'timestamp': timestamp, 'location x': transform.location.x, 'location y': transform.location.y}
        
        elif sensor_type == 'imu_sensor':
            return {
            'timestamp': data.timestamp,
            'accelerometer': [data.accelerometer.x, data.accelerometer.y],
             'compass': data.compass
            }
            """ 'gyroscope': [data.gyroscope.x, data.gyroscope.y], """

       
            """ 
            elif sensor_type == 'radar_sensor':
            timestamp = self.carla.world.get_snapshot().timestamp.elapsed_seconds
            radar_data = []
            for detection in data:
                radar_data.append({
                'altitude': detection.altitude,
                'azimuth': detection.azimuth,
                'depth': detection.depth,
                'velocity': detection.velocity
                })
            return {
                'timestamp': timestamp,
                'radar_data': radar_data
            }
            """
    def on_sensor_data(self, vehicle_id, sensor_type, data):
        processed_data = self.process_sensor_data(data, sensor_type, vehicle_id)
        if vehicle_id not in self.sensor_data:
            self.sensor_data[vehicle_id] = {}
        if sensor_type not in self.sensor_data[vehicle_id]:
            self.sensor_data[vehicle_id][sensor_type] = []
        self.sensor_data[vehicle_id][sensor_type].append(processed_data)
        self.save_sensor_data(vehicle_id, sensor_type, processed_data)

    def remove_sensors_from_vehicle(self, vehicle_id):
        if vehicle_id in self.sensors:
            for sensor in self.sensors[vehicle_id].values():
                sensor.destroy()
            del self.sensors[vehicle_id]

        if vehicle_id in self.sensor_data:
            del self.sensor_data[vehicle_id]

    def get_carla_vehicle_data(self, vehicle_id):
        # Get CARLA vehicle data
        carla_vehicle = self.carla.world.get_actor(int(vehicle_id))
        if not carla_vehicle:
            return None

        vehicle_location = carla_vehicle.get_location()
        
        """ 
        sumo controls vehicles and carla only teleport them in a new tick, so the values are 0.
        velocity = carla_vehicle.get_velocity()
        angular_velocity = carla_vehicle.get_angular_velocity()
        acc = carla_vehicle.get_acceleration()
        print(f"vehicle: {vehicle_id}, Speed x: {velocity.x:.2f}m/s, speed y: {velocity.y:.2f}m/s, angular velocity x: {angular_velocity.x:.2f}, angular velocity y: {angular_velocity.y:.2f}")
        print(f"vehicle: {vehicle_id}, acc x: {acc.x:.2f}m/s2, acc y: {acc.y:.2f}m/s2") """
        
        vehicle_position = {
                'timestamp': self.carla.world.get_snapshot().timestamp.elapsed_seconds,
                'x': vehicle_location.x,
                'y': vehicle_location.y
            }
        self.save_vehicle_data(vehicle_id, vehicle_position)
        
        return vehicle_position
         
    def save_vehicle_data(self, vehicle_id, data):
        output_file = os.path.join(self.output_directory, f'vehicle_{vehicle_id}.json')
        with open(output_file, 'a') as f:
            json.dump(data, f)
            f.write('\n')

    def save_speed_data(self, vehicle_id, data):
        output_file = os.path.join(self.output_directory, f'speed_{vehicle_id}.json')
        with open(output_file, 'a') as f:
            json.dump(data, f)
            f.write('\n')

    def save_sumo_distance_data(self,data):
        output_file = os.path.join(self.output_directory, f'sumo_distance.json')
        with open(output_file, 'a') as f:
            json.dump(data, f)
            f.write('\n')
    
    def save_sensor_data(self, vehicle_id, sensor_type, data):
        output_file = os.path.join(self.output_directory, f'{sensor_type}_{vehicle_id}.json')
        with open(output_file, 'a') as f:
            json.dump(data, f)
            f.write('\n')


    def tick(self):
        """
        Tick to simulation synchronization
        """
        
        # -----------------
        # sumo-->carla sync
        # -----------------
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
                lidar_semantic, gnss_sensor, imu_sensor= self.setup_sensors(carla_actor, carla_actor_id)

                if carla_actor_id != INVALID_ACTOR_ID:
                    self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
            else:
                self.sumo.unsubscribe(sumo_actor_id)
           

        for carla_actor_id in self.carla.destroyed_actors:
            self.remove_sensors_from_vehicle(carla_actor_id)

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

        # save vehicle position/ground truth
        for vehicle in all_vehicles:
            self.get_carla_vehicle_data(vehicle.id)

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

                # Updates all the sumo links related to this landmark.
                self.sumo.synchronize_traffic_light(landmark_id, sumo_tl_state)

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
            self.remove_sensors_from_vehicle(carla_actor_id)
            
        for sumo_actor_id in self.carla2sumo_ids.values():
            self.sumo.destroy_actor(sumo_actor_id)
       
        # Closing sumo and carla client.
        self.carla.close()
        self.sumo.close()


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
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

