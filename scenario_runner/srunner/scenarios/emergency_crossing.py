#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Non-signalized junctions: crossing negotiation:

The hero vehicle is passing through a junction without traffic lights
And encounters another vehicle passing across the junction.
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      SyncArrival,
                                                                      KeepVelocity,
                                                                      StopVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerRegion, InTriggerDistanceToLocation, DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario


class EmergencyCrossing(BasicScenario):

    """
    Implementation class for
    'Non-signalized junctions: crossing negotiation' scenario,
    (Traffic Scenario 10).

    This is a single ego vehicle scenario
    """

    # ego vehicle parameters
    _ego_vehicle_max_velocity = 20
    _ego_vehicle_driven_distance = 105

    # other vehicle
    _other_actor_max_brake = 1.0
    _other_actor_target_velocity = 15

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario
        """

        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        self._trigger_point = config.trigger_points[0].location
        # print(f"xxxxxxxxxx={config.parameters[0]}")
        # self._intersect_point = config.parameters[0].intersection_location
        # print(f"self._intersect_point ========= {self._intersect_point}")

        super(EmergencyCrossing, self).__init__("EmergencyCrossing",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        other_actor_model = 'vehicle.toyota.prius'
        self._other_actor_transform = config.other_actors[0].transform
        first_vehicle_transform = carla.Transform(
            carla.Location(config.other_actors[0].transform.location.x,
                           config.other_actors[0].transform.location.y,
                           config.other_actors[0].transform.location.z - 500),
            config.other_actors[0].transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(other_actor_model, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        After invoking this scenario, it will wait for the user
        controlled vehicle to enter the start region,
        then make a traffic participant to accelerate
        until it is going fast enough to reach an intersection point.
        at the same time as the user controlled vehicle at the junction.
        Once the user controlled vehicle comes close to the junction,
        the traffic participant accelerates and passes through the junction.
        After 60 seconds, a timeout stops the scenario.
        """

        # Creating leaf nodes
        start_other_trigger = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._trigger_point,
            2.0)

        self._intersect_point = carla.Location(-187.83721923828125, -3.7657418251037598 , 0.691417932510376) #TODO: Change to configable parameter from JSON
        sync_arrival = SyncArrival(
            self.other_actors[0], self.ego_vehicles[0],
            self._intersect_point)

        pass_through_trigger = InTriggerDistanceToLocation(
            self.ego_vehicles[0],
            self._intersect_point,
            20.0)

        keep_velocity_other = KeepVelocity(
            self.other_actors[0],
            self._other_actor_target_velocity)

        stop_other_trigger = DriveDistance(self.other_actors[0], 20.0)

        end_condition = DriveDistance(self.ego_vehicles[0], 20.0)

        # Creating non-leaf nodes
        root = py_trees.composites.Sequence()
        scenario_sequence = py_trees.composites.Sequence()
        sync_arrival_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        keep_velocity_other_parallel = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # Building tree
        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform))
        scenario_sequence.add_child(start_other_trigger)
        scenario_sequence.add_child(sync_arrival_parallel)
        scenario_sequence.add_child(keep_velocity_other_parallel)
        scenario_sequence.add_child(end_condition)
        scenario_sequence.add_child(ActorDestroy(self.other_actors[0]))

        sync_arrival_parallel.add_child(sync_arrival)
        sync_arrival_parallel.add_child(pass_through_trigger)
        keep_velocity_other_parallel.add_child(keep_velocity_other)
        keep_velocity_other_parallel.add_child(stop_other_trigger)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collison_criteria = CollisionTest(self.ego_vehicles[0])
        criteria.append(collison_criteria)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
