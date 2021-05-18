#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides a dummy agent to control the ego vehicle
"""

from __future__ import print_function

import carla

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track

def get_entry_point():
    return 'DummyAgent'

class DummyAgent(AutonomousAgent):

    """
    Dummy autonomous agent to control the ego vehicle
    """

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        pass

    # =================================================================
    # Sensor
    # =================================================================

    # {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'width': 800, 'height': 600, 'fov': 100, 'id': 'CAMERA'}
    # https://carla.readthedocs.io/en/0.9.11/ref_sensors/#rgb-camera

    # {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0, 'id': 'LIDAR'}
    # https://carla.readthedocs.io/en/0.9.11/ref_sensors/#lidar-sensor

    # {'type': 'sensor.other.radar', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0, 'fov': 30, 'id': 'RADAR'}
    # https://carla.readthedocs.io/en/0.9.11/ref_sensors/#radar-sensor

    # {'type': 'sensor.other.gnss', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'id': 'GPS'}
    # https://carla.readthedocs.io/en/0.9.11/ref_sensors/#gnss-sensor

    # {'type': 'sensor.other.imu', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0, 'id': 'IMU'}
    # https://carla.readthedocs.io/en/0.9.11/ref_sensors/#imu-sensor

    # {'type': 'sensor.speedometer', 'reading_frequency': 20, 'id': 'SPEED'}
    # Pseudosensor that provides an approximation of your linear velocity.

    # =================================================================
    # Global waypoints
    # =================================================================
    # Get by call this variable "self._global_plan"
    # [({'lat': 48.99822669411668, 'lon': 8.002271601998707, 'z': 0.0}, RoadOption.LEFT),
    # ({'lat': 48.99822669411668, 'lon': 8.002709765148996, 'z': 0.0}, RoadOption.RIGHT),
    # ...
    # ({'lat': 48.99822679980298, 'lon': 8.002735250105061, 'z': 0.0}, RoadOption.STRAIGHT)]
    # ----------------------------------------------------------------
    # All possible road option
    # RoadOption.CHANGELANELEFT: Move one lane to the left.
    # RoadOption.CHANGELANERIGHT: Move one lane to the right.
    # RoadOption.LANEFOLLOW: Continue in the current lane.
    # RoadOption.LEFT: Turn left at the intersection.
    # RoadOption.RIGHT: Turn right at the intersection.
    # RoadOption.STRAIGHT: Keep straight at the intersection.

    # =================================================================
    # Detail waypoints
    # =================================================================
    # Located in /tracks/track_XX_detail_waypoints.csv
    # Data format
    # | lat | lon | z |
    # -----------------
    # | 48.99822669411668 | 8.002271601998707 | 0.0 |

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.

        """

        # input_data format {key: [index, data]}
        # Example: camera_index = input_data['CAMERA'][0]
        # Example: camera_data = input_data['CAMERA'][1]

        # Get sensor data
        print("=====================>")

        print(f"Timestamp = {timestamp} seconds")
        print(len(self._global_plan))

        # for key, [index, data] in input_data.items():
        #     if hasattr(data, 'shape'):
        #         print(f"[{key} -- {index}] with shape {data.shape}")
        #     else:
        #         print(f"[{key} -- {index} -- {data}] ")

        print("<=====================")

        # DO SOMETHING SMART

        # RETURN CONTROL
        control = carla.VehicleControl()
        control.steer = 0.0         # A scalar value to control the vehicle steering [-1.0, 1.0]. Default is 0.0.
        control.throttle = 0.0      # A scalar value to control the vehicle throttle [0.0, 1.0]. Default is 0.0.
        control.brake = 0.0         # A scalar value to control the vehicle brake [0.0, 1.0]. Default is 0.0.
        control.hand_brake = False  # Determines whether hand brake will be used. Default is False.
        control.reverse = False     # Determines whether the vehicle will move backwards. Default is False.

        return control