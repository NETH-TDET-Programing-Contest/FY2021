import carla
from leaderboard.autoagents.autonomous_agent import AutonomousAgent

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

class Display():
    """
    Class to display the video stream from front camera.
    """

    def __init__(self):
        self.__width = 800
        self.__height = 600
        self.__surface = None
        
        pygame.init() # Initialize pygame
        self.__display = pygame.display.set_mode((self.__width, self.__height), pygame.HWSURFACE | pygame.DOUBLEBUF) # Create the window and set the size
        pygame.display.set_caption("Agent 23") # Set the window caption

    def render(self, input_data):
        """
        Render the image from the front camera
        """
        # Did the user click the window close button?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit() # Close the window
        
        image = input_data['CAMERA'][1][:, :, -2::-1] # Get the image from the sensor data and convert it to RGB

        self.__surface = pygame.surfarray.make_surface(image.swapaxes(0, 1)) # Convert the image to a pygame surface and display it
        if self.__surface is not None:
            self.__display.blit(self.__surface, (0, 0)) # Display the image on the screen
        pygame.display.flip() # Update the display

# For get class name
def get_entry_point():
    return 'Agent_23' # Change this to match your class name below

class Agent_23(AutonomousAgent):
    def setup(self, path_to_conf_file):
        """
        Setup the agent parameters
        """

        # Initial display class
        self.display = Display()
        pass

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        """
        # Edit your code here

        self.display.render(input_data)

        # Return control value to CARLA server
        control = carla.VehicleControl()
        control.steer = 0.0         # A scalar value to control the vehicle steering [-1.0, 1.0]. Default is 0.0.
        control.throttle = 0.0      # A scalar value to control the vehicle throttle [0.0, 1.0]. Default is 0.0.
        control.brake = 0.0         # A scalar value to control the vehicle brake [0.0, 1.0]. Default is 0.0.
        control.hand_brake = False  # Determines whether hand brake will be used. Default is False.
        control.reverse = False     # Determines whether the vehicle will move backwards. Default is False.
        return control
