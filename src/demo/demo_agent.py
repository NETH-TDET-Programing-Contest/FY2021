

"""
Demo of waypoint following and emergency break.
"""


# ============================== Import library =============================
try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

import carla

from leaderboard.autoagents.autonomous_agent import AutonomousAgent, Track

from enum import Enum, auto
import math


def get_entry_point():
    return 'DemoAgent'

# ============================= Display ====================================

class Display():
    """
    Class to display the video stream from front camera.
    """

    def __init__(self):
        self._width = 800
        self._height = 600
        self._surface = None

        pygame.init()
        pygame.font.init()
        self._clock = pygame.time.Clock()
        self._display = pygame.display.set_mode((self._width, self._height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("Demo Agent")

    def render(self, input_data):
        # Did the user click the window close button?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.quit()
        
         # process sensor data
        image_center = input_data['CAMERA'][1][:, :, -2::-1]

        # display image
        self._surface = pygame.surfarray.make_surface(image_center.swapaxes(0, 1))
        if self._surface is not None:
            self._display.blit(self._surface, (0, 0))
        pygame.display.flip()

    def quit(self):
        pygame.quit()


# ============================= Waypoint Following =========================
class WaypointFollowing():
    def __init__(self):
        self._waypoint_index = 0

    def __phrase_waypoint(self, waypoint):
        lat = waypoint[0]['lat']
        lon = waypoint[0]['lon']
        z = waypoint[0]['z']
        road_option = waypoint[1]

        return {"lat": lat, "lon": lon, "z": z, "road_option": road_option}


    def __get_heading(self, current_waypoint, next_waypoint):
        current_wp = self.__phrase_waypoint(current_waypoint)
        next_wp = self.__phrase_waypoint(next_waypoint)

        diff_lat = next_wp['lat'] - current_wp['lat']
        diff_lon = next_wp['lon'] - current_wp['lon']

        heading = math.degrees(math.atan2(diff_lon, diff_lat))

        if (heading < 0.0):
            heading += 360.0

        return heading

    def __heading_diff(self, angle1, angle2):
        angle_diff = angle1 - angle2
        if(angle_diff > 180.0):
            angle_diff -= 360.0
        elif(angle_diff < -180.0):
            angle_diff += 360.0
        
        return angle_diff

    # https://www.kite.com/python/answers/how-to-find-the-distance-between-two-lat-long-coordinates-in-python
    def __get_distance(self, lat1, lon1, lat2, lon2):
        # approximate radius of earth in km
        EARTH_RADIUS = 6373.0

        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        diff_lat = lat2 - lat1
        diff_lon = lon2 - lon1

        a = math.sin(diff_lat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(diff_lon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = EARTH_RADIUS * c * 1000 # km to m

        return distance

    
    # Pick 2 pair of waypoints base on current location
    def __pick_waypoint(self, input_data, _global_plan):
        current_waypoint  = _global_plan[self._waypoint_index]
        next_waypoint     = _global_plan[self._waypoint_index+4]

        next_wp = self.__phrase_waypoint(next_waypoint)
        [gps_frame, [gps_lat, gps_lon, gps_z]] = input_data['GPS']

        distance = self.__get_distance(next_wp['lat'], next_wp['lon'], gps_lat, gps_lon)
        if (distance < 0.5):
            self._waypoint_index += 1

        return current_waypoint, next_waypoint
        
            
    def __steering(self, input_data, current_waypoint, next_waypoint):

        wp_heading = self.__get_heading(current_waypoint, next_waypoint)

        [frame, [ax, ay, az, gx, gy, gz, compass]] = input_data['IMU']
        car_heading = math.degrees(compass)

        diff_heading = self.__heading_diff(car_heading, wp_heading)

        print(f"heading= ({wp_heading}, {car_heading}, {diff_heading})")

        if (-1.0 < diff_heading < 1.0):
            steering = 0.0
        elif (diff_heading < -1.0):
            steering = 0.5
        elif (diff_heading > 1.0):
            steering = -0.5
        else:
            steering = 0.0

        return steering


    def __throttling(self, input_data):
        target_speed = 2 #m/s = 18 km/hr
        car_speed = input_data['SPEED'][1]['speed']

        diff_speed = car_speed - target_speed

        print(f"speed= ({target_speed}, {car_speed}, {diff_speed})")

        if (-0.25 < diff_speed < 0.25):
            throttle = 0.25
            brake = 0.0
        elif (diff_speed < -0.25):
            throttle = 0.35
            brake = 0.0
        elif (diff_speed > 0.25):
            throttle = 0.0
            brake = 0.25
        else:
            throttle = 0.0
            brake = 1.0

        return throttle, brake


    def execute(self, input_data, _global_plan):
        control = carla.VehicleControl()
        if (_global_plan != None):
            # Select relate waypoint
            current_waypoint, next_waypoint = self.__pick_waypoint(input_data, _global_plan)

            # Calculate steering
            control.steer = self.__steering(input_data, current_waypoint, next_waypoint)
            control.throttle, control.brake = self.__throttling(input_data)

        return control

# ============================== DemoAgent =================================

class DemoAgent(AutonomousAgent):

    """
    Demo autonomous agent to control the ego vehicle
    """
    
    class State(Enum):
        waypoint_following = auto()
        emergency_break = auto()

    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        
        self.display = Display()
        self.state = self.State.waypoint_following
        self.waypoint_following = WaypointFollowing()


    # =================================================================
    # Sensor
    # =================================================================

    # {'type': 'sensor.camera.rgb', 'x': 0.45, 'y': 0.0, 'z': 2.15, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'width': 800, 'height': 600, 'fov': 100, 'id': 'CAMERA'}
    # https://carla.readthedocs.io/en/0.9.11/ref_sensors/#rgb-camera

    # {'type': 'sensor.lidar.ray_cast', 'x': 0.45, 'y': 0.0, 'z': 2.15, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'LIDAR'}
    # https://carla.readthedocs.io/en/0.9.11/ref_sensors/#lidar-sensor

    # {'type': 'sensor.other.radar', 'x': 0.45, 'y': 0.0, 'z': 2.15, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'fov': 30, 'id': 'RADAR'}
    # https://carla.readthedocs.io/en/0.9.11/ref_sensors/#radar-sensor

    # {'type': 'sensor.other.gnss', 'x': 0.45, 'y': 0.0, 'z': 2.15, 'id': 'GPS'}
    # https://carla.readthedocs.io/en/0.9.11/ref_sensors/#gnss-sensor

    # {'type': 'sensor.other.imu', 'x': 0.45, 'y': 0.0, 'z': 2.15, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'id': 'IMU'}
    # https://carla.readthedocs.io/en/0.9.11/ref_sensors/#imu-sensor

    # {'type': 'sensor.speedometer', 'reading_frequency': 20, 'id': 'SPEED'}
    # Pseudosensor that provides an approximation of your linear velocity.

    # =================================================================
    # Global waypoints
    # =================================================================
    # Global waypoints are in "self._global_plan" variable
    # [({'lat': 48.99822669411668, 'lon': 8.002271601998707, 'z': 0.0}, RoadOption.LEFT),
    # ({'lat': 48.99822669411668, 'lon': 8.002709765148996, 'z': 0.8}, RoadOption.RIGHT),
    # ...
    # ({'lat': 48.99822679980298, 'lon': 8.002735250105061, 'z': 1.2}, RoadOption.STRAIGHT)]
    # ----------------------------------------------------------------
    # All possible road option
    # RoadOption.CHANGELANELEFT: Move one lane to the left.
    # RoadOption.CHANGELANERIGHT: Move one lane to the right.
    # RoadOption.LANEFOLLOW: Continue in the current lane.
    # RoadOption.LEFT: Turn left at the intersection.
    # RoadOption.RIGHT: Turn right at the intersection.
    # RoadOption.STRAIGHT: Keep straight at the intersection.      

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """

        # input_data format {key: [index, data]}
        # Example: camera_index = input_data['CAMERA'][0]
        # Example: camera_data = input_data['CAMERA'][1]
        # Example: lidar_index = input_data['LIDAR'][0]
        # Example: lidar_data = input_data['LIDAR'][1]

        self.display.render(input_data)

        # DO SOMETHING SMART

        # Simple statemachine with 2 states
        # Switch state
        if (self.state == self.State.waypoint_following):
            self.state = self.State.waypoint_following
        elif(self.state == self.State.emergency_break):
            self.state = self.State.emergency_break
        else:
            print("State not change")

        # Execute state
        if (self.state == self.State.waypoint_following):
            control = self.waypoint_following.execute(input_data, self._global_plan)
        elif(self.state == self.State.emergency_break):
            pass
        else:
            pass

        # RETURN CONTROL
        # control = carla.VehicleControl()
        # control.steer = 0.0         # A scalar value to control the vehicle steering [-1.0, 1.0]. Default is 0.0.
        # control.throttle = 0.0      # A scalar value to control the vehicle throttle [0.0, 1.0]. Default is 0.0.
        # control.brake = 0.0         # A scalar value to control the vehicle brake [0.0, 1.0]. Default is 0.0.
        # control.hand_brake = False  # Determines whether hand brake will be used. Default is False.
        # control.reverse = False     # Determines whether the vehicle will move backwards. Default is False.

        return control

    def destroy(self):
        self.display.quit()
