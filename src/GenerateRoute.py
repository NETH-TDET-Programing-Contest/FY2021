import glob
import os
import sys
import argparse
import logging
import pandas as pd

try:
    sys.path.append(glob.glob('./lib/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# https://sebastian-pagel.net/ for Opendrive viewer
# [road_id, lane_id, s] s=distance from starting point of each road (-1 = drive along all road until end)
route_town3 = [
    [7, -2, 20], # Starting point
    [7, -2, -1],
    [494, -1, -1], # Turn Right
    [43, 1, -1], [1805, 1, -1], [42, 1, -1],
    [442, 1, -1], # Turn left
    [18, -1, -1], [1230, -1, -1], [19, -1, -1],
    [783, -1, -1], # Turn Right
    [2, 3, -1], [667, 3, -1], [1, 3, -1], [588, 3, -1], [0, 3, -1], [499, 1, -1],
    [65, -2, 215], [65, -1, -1], # Tunnel and change lane after exit
    [1156, -1, -1], # Turn Right
    [5, -1, -1], [6, -1, -1], [1174, -1, -1], [7, -1, -1], [482, -1, -1], [8, -1, -1], [1197, -3, -1],[34, 3, -1],
    [1089, 3, -1], [36, -4, -1], [37, -4, -1], [1624, -4, -1], [38, -4, -1], [1470, -4 ,-1], [13, -4, -1], [1661, -3, -1], [16, 3, -1], [1737, 3, -1], # Roundabout
    [30, -1, -1],
    [213, -1, -1], # Turn left
    [76, -1, -1], [77, -1, -1], [78, -1, -1], [79, -1, -1], [80, -1, -1],
    [1070, -1, -1], # Turn left
    [48, -1, -1], [49, -1, -1], [50, -1, -1], [52, -1, -1],
    [1431, -1, -1], [60, -2, -1], [45, -2, -1], [1435, -1, -1], # Roundabout
    [52, 1, -1], [50, 1, -1], [49, 1, -1], [48, 1, -1], [1049, 1, -1], [47, 1, -1],
    [1382, 1, -1], # Turn lefts
    [66, 2, -1], [1166, 2, -1],
    [65, 2, 230], [65, 3, -1], # Lane change before Tunnel
    [526, 1, -1], [0, -2, -1], [589, -2, -1],
    [1, -2, 20], [1, -1, -1], # Lane change before turn left
    [703, -1, -1], [29, 2, -1],
    [1243, 1, -1], # Turn left
    [18, 1, -1], [383, 1, -1], [17, 1, -1],
    [1713, 1, -1], # Turn right
    [74, -1, -1], [1467, -1, -1], [75, -1, -1],
    [1188, -1, -1], # Turn right
    [7, -2, 20] # Turn right
    ]

class generateRoute:
    def __init__(self, client):
        self.world = client.get_world()
        self.map = self.world.get_map()

        self.route = route_town3
        self.wp_distance = 1.0 # Waypoint search distance
        self.route_waypoints = self.__gen_waypoints()

    def get_waypoints(self):
        return self.route_waypoints

        # Function for draw waypoints in world
    def draw_waypoint(self, life_time=10):
        for wp in self.route_waypoints:
            self.world.debug.draw_point(location=wp.transform.location, size=0.1, color=carla.Color(255, 0, 0), life_time=life_time)

    def export_csv(self, path):
        route_waypoints = self.route_waypoints

        df = pd.DataFrame(([wp.transform.location.x, wp.transform.location.y, wp.transform.location.z] for wp in route_waypoints),
                            columns=['x', 'y', 'z'])

        # df = pd.DataFrame(([wp.transform.location.x, wp.transform.location.y, wp.transform.location.z,
        #                     wp.transform.rotation.pitch, wp.transform.rotation.yaw, wp.transform.rotation.roll] for wp in route_waypoints),
        #                     columns=['x', 'y', 'z', 'pitch', 'yaw', 'roll'])

        df.to_csv(path, index=False)

    # Generate route waypoints
    def __gen_waypoints(self):
        route_waypoints = []

        # Get first designed route as a start point
        road_id, lane_id, s = self.route[0]
        route_waypoints.append(self.map.get_waypoint_xodr(road_id, lane_id, s))
        next_wp = self.__search_next_available_waypoints(route_waypoints[-1], self.wp_distance)

        # Loop through designed route from 1 to n
        for road_id, lane_id, s in self.route[1:]:
            # print(f'designed:\troad_id= {road_id},\tlane_id= {lane_id},\ts= {s}')
            
            # # Print available waypoint
            # for wp in next_wp:
            #     print(f'available:\troad_id= {wp.road_id},\tlane_id= {wp.lane_id}')

            # Loop through all available waypoints
            for wp in next_wp:
                # Select waypoint match designed route
                if (wp.road_id == road_id) and (wp.lane_id == lane_id):
                    # print(f'selected:\troad_id= {wp.road_id},\tlane_id= {wp.lane_id}')

                    route_waypoints.append(wp) # Collect this waypoint

                    if(s == -1):
                        # Collect all waypoints of this road
                        route_waypoints.extend(wp.next_until_lane_end(self.wp_distance))
                    else:
                        # Collect waypoints until meet S distance point
                        route_waypoints.extend(self.__next_until_s(wp, s, self.wp_distance))
                    
                    # Search for next available waypoints from lastest waypoint in route_waypoints
                    next_wp = self.__search_next_available_waypoints(route_waypoints[-1], self.wp_distance)
                    break
                    
        return route_waypoints

    # Get waypoints along road until meet S distance point
    def __next_until_s(self, wp, s, wp_distance):
        waypoints = []

        wp = wp.next(wp_distance)[0] # Use 1st waypoint from waypoint list

        while abs(wp.s - s) >= wp_distance:
            waypoints.append(wp)
            wp = wp.next(wp_distance)[0]

        return waypoints

    def __search_next_available_waypoints(self, wp, wp_distance):
        # Search for next available waypoint from CARLA API
        next_wp = wp.next(wp_distance)

        # Search for lane changed
        left_lane = wp.get_left_lane()
        if (left_lane is not None and left_lane.lane_type == carla.LaneType.Driving):
            next_wp.append(left_lane)

        right_lane = wp.get_right_lane()
        if (right_lane is not None and right_lane.lane_type == carla.LaneType.Driving):
            next_wp.append(right_lane)

        return next_wp

def main(args):
    world = None
    
    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)
        world = client.get_world()

        gen_route = generateRoute(client, route_town3)
        gen_route.draw_waypoint()
        gen_route.export_csv('C:/Users/supav/Desktop/neth_project/FY2021/src/town03_waypoints.csv')


    finally:
        # if (world and world.recording_enabled):
        #     client.stop_recorder()

        # if world is not None:
        #     world.destroy()
        pass

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description='Generate route')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    try:
        main(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')