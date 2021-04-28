# -*- coding: utf-8 -*-
"""
Created on Fri Oct  9 08:20:22 2020

@author: Jeditou
"""

import glob
import os
import sys
import weakref
import math
import queue

try:
    sys.path.append(glob.glob('./lib/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla


# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================

class RadarSensor(object):
    def __init__(self):
        self.sensor = None
        self.debug = None
        self.sensor_data = None
            
        
    def InitialRadarSensor(self, runningCar, horizontal_fov, vertical_fov, points_per_second=1000, radar_range=7.5, radar_tick=1.0):
        sensorTransform = None
        sensorPosition_X = 2.8
        sensorPosition_Z = 1.0
        sensorRotation_Pitch = 5
        
        world = runningCar.get_world()
        self.debug = world.debug
        
        radarBlurprint = world.get_blueprint_library().find('sensor.other.radar')
        radarBlurprint.set_attribute('horizontal_fov', str(horizontal_fov))
        radarBlurprint.set_attribute('vertical_fov', str(vertical_fov))
        radarBlurprint.set_attribute('points_per_second', str(points_per_second))
        radarBlurprint.set_attribute('range', str(radar_range))
        radarBlurprint.set_attribute('sensor_tick', str(radar_tick))
        
        sensorTransform = carla.Transform(carla.Location(sensorPosition_X, sensorPosition_Z), carla.Rotation(sensorRotation_Pitch))
        self.sensor = world.spawn_actor(radarBlurprint, sensorTransform, attach_to=runningCar)
        
        weak_self = weakref.ref(self)
        
        self.sensor.listen(lambda radar_data: RadarSensor._UpdateSensorDataCallback(weak_self, radar_data))

    def _UpdateSensorDataCallback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        self.sensor_data = radar_data



# ==============================================================================
# -- LidarSensor ---------------------------------------------------------------
# ==============================================================================

class LidarSensor(object):
    def __init__(self):
        self.sensor = None
        self.debug = None
        self.sensor_data = None
            
        
    def InitialLidarSensor(self, runningCar):
        sensorTransform = None
        sensorPosition_X = 0.0
        sensorPosition_Z = 2.4
        sensorRotation_Pitch = 0
              
        world = runningCar.get_world()
        self.debug = world.debug
        
        lidarBlurprint = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidarBlurprint.set_attribute('channels', '64')
        lidarBlurprint.set_attribute('range', '50')
        lidarBlurprint.set_attribute('points_per_second', '100000')
        lidarBlurprint.set_attribute('rotation_frequency', '20')
        lidarBlurprint.set_attribute('dropoff_general_rate', lidarBlurprint.get_attribute('dropoff_general_rate').recommended_values[0])
        lidarBlurprint.set_attribute('dropoff_intensity_limit', lidarBlurprint.get_attribute('dropoff_intensity_limit').recommended_values[0])
        lidarBlurprint.set_attribute('dropoff_zero_intensity', lidarBlurprint.get_attribute('dropoff_zero_intensity').recommended_values[0])
                
        sensorTransform = carla.Transform(carla.Location(sensorPosition_X, sensorPosition_Z), carla.Rotation(sensorRotation_Pitch))
        self.sensor = world.spawn_actor(lidarBlurprint, sensorTransform, attach_to=runningCar)
        
        weak_self = weakref.ref(self)
        
        self.sensor.listen(lambda lidar_data: LidarSensor._UpdateSensorDataCallback(weak_self, lidar_data))

    def _UpdateSensorDataCallback(weak_self, lidar_data):
        self = weak_self()
        if not self:
            return
        self.sensor_data = lidar_data



# ==============================================================================
# -- Camera----- ---------------------------------------------------------------
# ==============================================================================

class CameraRGB(object):
    def __init__(self):
        self.sensor = None
        self.debug = None
        self.image_queue = queue.Queue()
        
        
    def InitialCamera(self, runningCar):
        sensorTransform = None
        sensorPosition_X = 0.0
        sensorPosition_Z = 1.5
        sensorRotation_Pitch = 0
              
        world = runningCar.get_world()
        self.debug = world.debug
        
        CameraBlueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        CameraBlueprint.set_attribute("image_size_x", '1280')
        CameraBlueprint.set_attribute("image_size_y", '720')
        CameraBlueprint.set_attribute("fov", "80")
        CameraBlueprint.set_attribute("lens_circle_falloff", "0")
        CameraBlueprint.set_attribute("sensor_tick", "0.5")  # capture image every 0.3 seconds

                
        sensorTransform = carla.Transform(carla.Location(sensorPosition_X, sensorPosition_Z), carla.Rotation(sensorRotation_Pitch))
        self.sensor = world.spawn_actor(CameraBlueprint, sensorTransform, attach_to=runningCar)
        
        self.sensor.listen(self.image_queue.put)


