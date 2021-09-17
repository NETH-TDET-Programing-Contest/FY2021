#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Vehicle Maneuvering In Opposite Direction:

Vehicle is passing another vehicle in a rural area, in daylight, under clear
weather conditions, at a non-junction and encroaches into another
vehicle traveling in the opposite direction.
"""

from six.moves.queue import Queue   # pylint: disable=relative-import

import math
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      ActorSource,
                                                                      ActorSink,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class ManeuverOppositeDirectionVehicleBlock(BasicScenario):

    """
    "Vehicle Maneuvering In Opposite Direction" (Traffic Scenario 06)

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 obstacle_type='barrier', timeout=600):
        """
        Setup all relevant parameters and create scenario
        obstacle_type -> flag to select type of leading obstacle. Values: vehicle, barrier
        """
        self._world = world
        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 0
        self._ego_vehicle_drive_distance = self._first_vehicle_location + 60 * 2
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._blackboard_queue_name = 'ManeuverOppositeDirectionVehicleBlock/actor_flow_queue'
        self._queue = py_trees.blackboard.Blackboard().set(self._blackboard_queue_name, Queue())
        self._obstacle_type = obstacle_type
        self._first_actor_transform = None
        self._second_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(ManeuverOppositeDirectionVehicleBlock, self).__init__(
            "ManeuverOppositeDirectionVehicleBlock",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        
        first_actor_model = 'vehicle.toyota.prius'

        first_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)

        position_yaw = math.radians(first_actor_waypoint.transform.rotation.yaw + 90)
        offset_location = carla.Location(
            0.25 * first_actor_waypoint.lane_width * math.cos(position_yaw),
            0.25 * first_actor_waypoint.lane_width * math.sin(position_yaw))
        first_actor_transform = carla.Transform(
            first_actor_waypoint.transform.location + offset_location,
            first_actor_waypoint.transform.rotation)

        first_actor = CarlaDataProvider.request_new_actor(first_actor_model, first_actor_transform)
        first_actor.set_simulate_physics(True)
        self.other_actors.append(first_actor)
        self._first_actor_transform = first_actor_transform

        if self._obstacle_type == 'barrier':
            second_actor_model = 'static.prop.streetbarrier'

            second_actor_waypoint = first_actor_waypoint.previous(5.0)[0]

            position_yaw = math.radians(second_actor_waypoint.transform.rotation.yaw + 90)
            offset_location = carla.Location(
                0.00 * second_actor_waypoint.lane_width * math.cos(position_yaw),
                0.00 * second_actor_waypoint.lane_width * math.sin(position_yaw))
            rotation_tmp = second_actor_waypoint.transform.rotation
            rotation_tmp.yaw -= 45
            second_actor_transform = carla.Transform(
                second_actor_waypoint.transform.location + offset_location,
                rotation_tmp)

            second_prop_actor = CarlaDataProvider.request_new_actor(second_actor_model, second_actor_transform)
            second_prop_actor.set_simulate_physics(True)
            self.other_actors.append(second_prop_actor)
            self._second_actor_transform = second_actor_transform

    def _create_behavior(self):
        """
        The behavior tree returned by this method is as follows:
        The ego vehicle is trying to pass a leading vehicle in the same lane
        by moving onto the oncoming lane while another vehicle is moving in the
        opposite direction in the oncoming lane.
        """

        scenario_sequence = py_trees.composites.Sequence()
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self._first_actor_transform))
        if self._obstacle_type == 'barrier':
            scenario_sequence.add_child(ActorTransformSetter(self.other_actors[1], self._second_actor_transform))
        scenario_sequence.add_child(DriveDistance(self.ego_vehicles[0], self._ego_vehicle_drive_distance))
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))
        if self._obstacle_type == 'barrier':
            scenario_sequence.add_child(ActorDestroy(self.other_actors[1]))

        return scenario_sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
