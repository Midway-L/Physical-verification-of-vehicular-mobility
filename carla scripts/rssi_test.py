
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
        self.default_lead_speed = 10 #m/s
        self.desired_following_distance = 10  # meters

        self.output_directory = "vehicle_data"  
        
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
        self.vehicle_positions = {} 
        self.vehicle_speed = {} #save vehicle speed from sumo
        self.sumo_distance = [] #save distance from sumo

    def control_lead_vehicle(self):
        traci.vehicle.setSpeed(self.lead_vehicle_id, self.default_lead_speed)
        current_speed = traci.vehicle.getSpeed(self.lead_vehicle_id)
        
    
        if self.lead_vehicle_id not in self.vehicle_speed:
            self.vehicle_speed[self.lead_vehicle_id] = []
        
        self.vehicle_speed[self.lead_vehicle_id].append({
                'timestamp':  self.carla.world.get_snapshot().timestamp.elapsed_seconds,
                'speed': current_speed})
        leader_speed = self.vehicle_speed[self.lead_vehicle_id][-1]     
        self.save_speed_data(self.lead_vehicle_id,leader_speed)
        print(f"Leader speed: {current_speed:.2f} m/s")
                                                                                   
    
    def control_following_vehicle(self):
    
        traci.vehicle.setLaneChangeMode(self.following_vehicle_id, 0b0000000000)  
        traci.vehicle.setSpeedMode(self.following_vehicle_id, 0)
        traci.vehicle.setMinGap(self.following_vehicle_id, 0)
        
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
            
            # Simple controller for distance keeping
            distance_error = distance - self.desired_following_distance
            speed_adjustment = distance_error * 0.5  # can adjust this factor
            
            target_speed = lead_speed + speed_adjustment
            target_speed = max(0, min(target_speed, self.default_lead_speed))  # Clamp speed
            
            traci.vehicle.setSpeed(self.following_vehicle_id, target_speed)
            
            self.sumo_distance.append({
            'timestamp': self.carla.world.get_snapshot().timestamp.elapsed_seconds,
            'distance': distance}
            )
            sumo_distance = self.sumo_distance[-1]
            self.save_sumo_distance_data(sumo_distance)
            
            print(f"Following - Distance: {distance:.2f}m, Speed: {current_speed:.2f} -> {target_speed:.2f} m/s")

    def setup_sensors(self, vehicle, vehicle_id):

        # 设置GNSS传感器
        gnss_bp = self.carla.world.get_blueprint_library().find('sensor.other.gnss')
        gnss_bp.set_attribute('sensor_tick', '0.1')  
        gnss_bp.set_attribute('noise_lat_stddev', '1.0')  
        gnss_bp.set_attribute('noise_lon_stddev', '1.0')  
        gnss_bp.set_attribute('noise_seed', '2024') 

        gnss_location = carla.Location(x=0, y=0, z=0)
        gnss_rotation = carla.Rotation(0, 0, 0)
        gnss_transform = carla.Transform(gnss_location, gnss_rotation)
        
        gnss_sensor = self.carla.world.spawn_actor(gnss_bp, gnss_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
        
        gnss_sensor.listen(lambda data: self.on_sensor_data(vehicle_id, 'gnss_sensor', data))

    def get_carla_vehicle_data(self, vehicle_id):
        # Get CARLA vehicle data
        carla_vehicle = self.carla.world.get_actor(int(vehicle_id))
        if not carla_vehicle:
            return None

        vehicle_location = carla_vehicle.get_location()
        
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

    def remove_sensors_from_vehicle(self, vehicle_id):
        if vehicle_id in self.sensors:
            for sensor in self.sensors[vehicle_id].values():
                sensor.destroy()
            del self.sensors[vehicle_id]

        if vehicle_id in self.sensor_data:
            del self.sensor_data[vehicle_id]

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
                gnss_sensor = self.setup_sensors(carla_actor, carla_actor_id)
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

