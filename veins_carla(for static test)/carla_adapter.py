#!/usr/bin/env python3

#
# carla_adapter.py -- CARLA daemon for use of Veins with CARLA
# Copyright (C) 2023 Tobias Hardes <tobias.hardes@uni-paderborn.de>
#
# Documentation for these modules is at http://veins.car2x.org/
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#


import logging
import tempfile
import grpc
import os
import sys
from google.protobuf import empty_pb2
from google.protobuf import struct_pb2
from optparse import OptionParser
import sys
import threading
import json
from operator import itemgetter

sys.path.append('./proto')
import carla_pb2_grpc
import carla_pb2

from concurrent import futures
import time
import carla
import random
from datetime import datetime


world = None
traffic_manager = None
client = None
carla_object = None

class CarlaAdapter(carla_pb2_grpc.CarlaAdapterServicer):
    def __init__(self, seed, steplength):
        global world
        global client
        print("Carla_Adapter: " + str(self.GetCurrentTime()) + ": CarlaAdapter: __init__")
        print("Steplength: " + str(steplength))
        client = carla.Client('localhost', 2000) #Default configuration for CARLA
        world = client.get_world()
        
        self.tick_lock = threading.Lock()
        self.tick_event = threading.Event()
        self.last_processed_time = -1
        self.callback_id = world.on_tick(self.on_world_tick)

        print("Carla_Adapter: " + str(self.GetCurrentTime()) + ": CarlaAdapter: __init__ done")
    
    def on_world_tick(self, snapshot):
      global world
      with self.tick_lock:
        self.tick_event.set()
        self.last_processed_time=world.get_snapshot().timestamp.elapsed_seconds
          
    def ExecuteOneTimeStep(self, request, context):
        print(f"Carla_Adapter: {self.GetCurrentTime()}: carla_adapter: ExecuteOneTimeStep")
        timeout = 30.0  # 30s time-out
        start_time = time.time()
        while True:
            if self.tick_event.is_set():
                with self.tick_lock:
                    current_time = self.last_processed_time
                    self.tick_event.clear()
                print(f"Carla_Adapter: {self.GetCurrentTime()}: carla_adapter: ExecuteOneTimeStep done. Carla-time at: {current_time}")
                return struct_pb2.Value()
            
            if time.time() - start_time > timeout:
                context.abort(grpc.StatusCode.DEADLINE_EXCEEDED, "Timeout waiting for Carla tick")

    def GetCurrentTime(self):
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def __del__(self):
        # remove callback
        global world
        if hasattr(self, 'callback_id'):
            world.remove_on_tick(self.callback_id)

    def GetManagedActorsIds(self, request, context):
        global world
        world.get_actors()
        actor_list = world.get_actors()
        vehicles = actor_list.filter('vehicle.*')
        print("Carla_Adapter: GetManagedActorsIds: Found " + str(len(vehicles)) + " vehicles")
        print(vehicles)
        returnValue = carla_pb2.ActorIds()
        for vehicle in vehicles:
            returnValue.actorId.append(vehicle.id)
        print("Carla_Adapter: returning data")
        print(returnValue)
        return returnValue
    
    def GetManagedActorById(self, request, context):
        global world
        world.get_actors()
        actor_list = world.get_actors()
        actor = actor_list.find(request.num)
        aLocation = actor.get_location()
        aSpeed = actor.get_velocity()
        aAcceleration = actor.get_acceleration()
        returnValue = carla_pb2.Vehicle()
        returnValue.id = request.num
        returnValue.location.x = aLocation.x
        returnValue.location.y = aLocation.y
        returnValue.location.z = aLocation.z
        returnValue.speed.x = aSpeed.x
        returnValue.speed.y = aSpeed.y
        returnValue.speed.z = aSpeed.z
        returnValue.acceleration.x = aAcceleration.x
        returnValue.acceleration.y = aAcceleration.y
        returnValue.acceleration.z = aAcceleration.z
        vehicle_data = {
            "timestamp": self.last_processed_time,
                "x": returnValue.location.x,
                "y": returnValue.location.y
            }
        self.save_to_json(request.num, vehicle_data)
        print("Carla_Adapter: vehicle id:" + str(actor.id) + " in position x:" + str(returnValue.location.x) + ", position y:" + str(returnValue.location.y))
        return returnValue
    
    def save_to_json(self, vehicle_id, data):
        filename = f"std_vehicle_{vehicle_id}.json"
    
        # Create the directory if it doesn't exist
        os.makedirs("vehicle_data", exist_ok=True)
    
        filepath = os.path.join("vehicle_data", filename)
    
        # Read existing data if file exists
        existing_data = []
        if os.path.exists(filepath):
            with open(filepath, 'r') as f:
                for line in f:
                    existing_data.append(json.loads(line.strip()))
    
        # Append new data
        existing_data.append(data)
    
        # Sort data by timestamp
        sorted_data = sorted(existing_data, key=itemgetter('timestamp'))
    
        # Write updated data back to file, one JSON object per line
        with open(filepath, 'w') as f:
            for entry in sorted_data:
                json_string = json.dumps(entry, separators=(',', ':'))
                f.write(json_string + '\n')
    
    """ def InsertVehicle(self, request, context):
        global world        
        blueprint_library = [bp for bp in world.get_blueprint_library().filter('vehicle.*')]
        vehicle_bp = blueprint_library[0]
        vehicle_transform = world.get_map().get_spawn_points()[2]
        print("Carla_Adapter: Spawing new vehicle -> " + str(vehicle_bp) + " at " + str(vehicle_transform))
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        print("Carla_Adapter: Setting location to " + str(request.location.x) + ", " + str(request.location.y) + ", " + str(request.location.z))
        loc = vehicle.get_location()
        loc.x = request.location.x
        loc.y = request.location.y
        loc.z = request.location.z+5
        vehicle.set_location(loc)
        number = carla_pb2.Number(num=vehicle.id)        
        print("Carla_Adapter: Spawned new vehicle")
        vehicle.set_autopilot(True)
        return number
    
    def GetRandomSpawnPoint(self, request, context):
        global world
        print("Carla_Adapter: GetRandomSpawnPoint")
        randInt = random.randint(0,len(world.get_map().get_spawn_points())-1)
        randInt = 1 #TODO
        print("Random int " + str(randInt))
        spawnPoint = world.get_map().get_spawn_points()[randInt]
        print(type(spawnPoint))
        print("Carla_Adapter: SpawnPoint: " + str(spawnPoint))
        retvalue = carla_pb2.Transform()
        retvalue.location.x = spawnPoint.location.x
        retvalue.location.y = spawnPoint.location.y
        retvalue.location.z = spawnPoint.location.z
        retvalue.rotation.pitch = spawnPoint.rotation.pitch
        retvalue.rotation.yaw = spawnPoint.rotation.yaw
        retvalue.rotation.roll = spawnPoint.rotation.roll
        print("return")
        return retvalue """
        
    def GetCurrentTime(self):
        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")
        return current_time

def startAdapter(address, port, daemonize, seed, steplength):
    global carla_object
    print("Carla_Adapter:  startAdapter")
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    carla_object = CarlaAdapter(seed, steplength)
    carla_pb2_grpc.add_CarlaAdapterServicer_to_server(carla_object, server)

    grpcPort = server.add_insecure_port('localhost:1337') 
    print("Carla_Adapter: Create server on " + address + ":" + str(grpcPort))
    server.start()

    try:
        while True:
            time.sleep(10) 
            print("Carla_Adapter: Adapter is still alive")
    except:
        sys.exit(1)
        
def main():
    """
    Program entry point when run interactively.
    """    
    parser = OptionParser()
    parser.add_option("-p", "--port", dest="port", type="int", default=1337, action="store", help="listen for connections on PORT [default: %default]", metavar="PORT")
    parser.add_option("-b", "--bind", dest="bind", default="localhost", help="bind to ADDRESS [default: %default]", metavar="ADDRESS")
    parser.add_option("-L", "--logfile", dest="logfile", default="carla-adapter.log", help="log messages to LOGFILE [default: %default]", metavar="LOGFILE")
    parser.add_option("-d", "--daemon", dest="daemonize", default=False, action="store_true", help="detach and run as daemon [default: no]")
    parser.add_option("-q", "--quiet", dest="count_quiet", default=0, action="count", help="decrease verbosity [default: log warnings, errors]")
    parser.add_option("-v", "--verbose", dest="count_verbose", default=0, action="count", help="increase verbosity [default: don't log infos, debug]")
    parser.add_option("-s", "--seed", dest="seed", default=0, type="int", action="store", help="Random seed for the simulation [default: 0]")
    parser.add_option("-l", "--steplength", dest="steplength", default=1, type="float", action="store", help="Length of one simulation step in seconds [default: 1]")
    
    (options, args) = parser.parse_args()
    _LOGLEVELS = (logging.ERROR, logging.WARN, logging.INFO, logging.DEBUG)
    loglevel = _LOGLEVELS[max(0, min(1 + options.count_verbose - options.count_quiet, len(_LOGLEVELS)-1))]

    # configure logging
    logging.basicConfig(filename=options.logfile, level=loglevel)
    if not options.daemonize:
        logging.warning("Superfluous command line arguments: \"%s\"" % " ".join(args))
                
    startAdapter(options.bind, options.port, options.daemonize, options.seed, options.steplength)

# Start main() when run interactively
if __name__ == '__main__':
    main()
   
