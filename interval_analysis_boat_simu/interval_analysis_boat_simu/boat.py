from interval_analysis_boat_simu.draw import *
from interval_analysis_boat_simu.sea_object import *


class Boat(SeaObject):

    def __init__(self, tag, x, y, v, theta, R, sight_angle):
        super().__init__(tag, x, y, v, theta, R, sight_angle)  # call the superclass's constructor
        self.privilege = 0

    
    # Draw circle around boat
    def draw(self, ax, Ɛ):
        draw_boat_and_vector(ax, self.get_state_vector())           # Display of the boat
        # -----> draw the arc here
        draw_cone_arc(self.x, self.y, self.R, self.sight_angle, self.theta, ax)

    # Get the color displayed on the rules
    def get_color(self):
        return "green"

    



                    
    