from interval_analysis_boat_simu.calcul_tools import *
from interval_analysis_boat_simu.draw import *
from interval_analysis_boat_simu.potential_fields import *

def Jφ0(p):
    """ Jacobian Matrix of φ0 """
    p1, p2 = p.flatten()
    return array([[-3 * p1 ** 2 - p2 ** 2 + 1, -2 * p1 * p2 - 1],
                  [-2 * p1 * p2 + 1, -3 * p2 ** 2 - p2 ** 2 + 1]])

def dφ(x, c, D):
    p1, p2, v, θ = x.flatten()
    z = inv(D) @ array([[p1 - c[0,0]], [p2 - c[1,0]]])
    dv = D @ Jφ0(z) @ inv(D) @ array([[cos(θ)], [sin(θ)]])
    return dv.flatten()


def control(x, φ, c, D, k, r):
    dφ1, dφ2 = dφ(x, c, D)
    x, y, v, θ = x.flatten()
    φ1, φ2 = φ(x, y, c, D, k, r)
    u1 = 0
    u2 = -sawtooth(θ - arctan2(φ2, φ1)) - (φ2 * dφ1 - φ1 * dφ2) / ((φ1 ** 2) + (φ2 ** 2))
    return array([[u1], [u2]])


class SeaObject:

    # x, y are positions, v is speed, theta is direction, R is the sight radius of the camera
    def __init__(self, tag, x, y, v, theta, R, sight_angle):
        self.tag = tag
        self.x = x
        self.y = y
        self.v = v
        self.theta = theta
        self.R = R
        self.sight_angle = sight_angle

        destination_distance = 20 # the distance from initial position to destination
        self.phat = array([[self.x + destination_distance * cos(self.theta)], [self.y + destination_distance * sin(self.theta)]])

        self.privilege = 0
        self.r = 2 # collision avoidance radius for the object
        self.in_area = False

    # Update the position of an object based on up controller
    def update(self, u, dt):
        x, y, v, theta = self.x, self.y, self.v, self.theta
        self.x += dt * v * cos(theta) 
        self.y += dt * v * sin(theta) 
        self.v = v + dt * u[0][0] 
        self.theta = (self.theta + dt * u[1][0])%(2*pi)

    # Return the object's x, y, speed and direction in a state vector
    def get_state_vector(self):
        return np.vstack((self.x, self.y, self.v, self.theta))
    
    # Move the object straight when there is no risk of collision
    # Called by move()
    def move_straight(self):
        # Control commande to reach the final destination if there is no risk of collision
        tethabar = arctan2(self.phat[1,0]-self.y,self.phat[0,0]-self.x)
        kw = -2.0
        w = kw*sawtooth(self.theta-tethabar)
        up = array([[0], [w]])

        """
        vhat = array([[1], [1]])
        wp = vhat - 2 * (array([[self.x], [self.y]]) - self.phat)
        thetabar_p = arctan2(wp[1, 0], wp[0, 0])

        up = array([[0], [10*arctan(tan(0.5*(thetabar_p - self.theta)))]])
        """
        return up

    def vector_and_heading(self, xA, yA, xB, yB):
        # Create the vector from A to B
        vector = array([xB - xA, yB - yA])
        # Calculate the heading (angle with respect to the x-axis)
        heading = arctan2(vector[1], vector[0])  # atan2(delta_y, delta_x)
        # Return the vector and the heading (in radians)
        return heading


    # Moves the object every iteration
    def move(self, record_data, sea_objects, ax, Ɛ, s, k, dt):

        # Check risks of collision with every other object
        for other_object in sea_objects:
            # Extracting the heading of the vector between the drone and the considerated buoy
            buoy_heading = self.vector_and_heading(self.x, self.y, other_object.x, other_object.y)
            # Angle between two agents (in our project, we only care about the angle between the drone and the buoy/obstacle
            α = abs(sawtooth(self.theta - buoy_heading))

            if self != other_object:
                # When distance is in the detection area, and the angle in the sight angle of the cameras
                if (dist(array([[other_object.x], [other_object.y]]), array([[self.x], [self.y]])) <= self.R) and (α <= self.sight_angle/2):
                    other_object.in_area = True
                    #print('tag, in_area=', self.tag, self.in_area)
                    #print('yeaaa')
                    up = self.move_straight()
                else :
                    other_object.in_area = False
                    up = self.move_straight()
                    #print('stop')

        # # If there is no need to avoid collision
        # if not in_collision:
        #     up = self.move_straight()
            
        # Update position
        self.update(up, dt)

        return [self.tag, self.get_state_vector()]

