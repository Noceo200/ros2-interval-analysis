from interval_analysis_boat_simu.calcul_tools import *
from interval_analysis_boat_simu.draw import *
from interval_analysis_boat_simu.boat import Boat
from interval_analysis_boat_simu.potential_fields import *
import csv


class Simulation:
    def __init__(self, sea_objects, dt, k):
        self.sea_objects = sea_objects
        self.dt = dt
        self.k = k


    def run(self, record_data, num_steps, ax, Ɛ, s):

            for _ in range(num_steps):

                clear(ax)

                for sea_objects in self.sea_objects:
                    sea_objects.move(record_data, self.sea_objects, ax, Ɛ, s, self.k, self.dt)
                    sea_objects.draw(ax, Ɛ)

                ax.set_xlim(-s, s)
                ax.set_ylim(-s, s)

    def run_with_data(self, record_data, num_steps, ax, Ɛ, s):
        # Create and open the .csv file
        with open('simulation_log.csv', 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)

            for _ in range(num_steps):

                for sea_objects in self.sea_objects:
                    log_data = sea_objects.move(record_data, self.sea_objects, ax, Ɛ, s, self.k, self.dt)

                    # Add the log data to the CSV file
                    csv_writer.writerow(log_data)