import carla
from leaderboard.autoagents.autonomous_agent import AutonomousAgent

# For get class name
def get_entry_point():
    return 'Agent_22' # Change this to match your class name below

class Agent_22(AutonomousAgent):
    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """
        pass

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """
        # Edit your code here

        # frame = Frame of sensor data
        # image = 800x600 BGRA 32-bit/pixel image
        [frame, image] = input_data['CAMERA']
        print(f"frame: {frame}, image: {image}")

        # Return control value to CARLA server
        control = carla.VehicleControl()
        control.steer = 0.0         # A scalar value to control the vehicle steering [-1.0, 1.0]. Default is 0.0.
        control.throttle = 0.0      # A scalar value to control the vehicle throttle [0.0, 1.0]. Default is 0.0.
        control.brake = 0.0         # A scalar value to control the vehicle brake [0.0, 1.0]. Default is 0.0.
        control.hand_brake = False  # Determines whether hand brake will be used. Default is False.
        control.reverse = False     # Determines whether the vehicle will move backwards. Default is False.
        return control
