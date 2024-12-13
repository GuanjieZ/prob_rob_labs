import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random

class Satellite:
    def __init__(self, num):
        self.x = random.randint(num // 2 * 10, num // 2 * 10 + 10)
        self.y = random.randint(num // 2 * 10, num // 2 * 10 + 10)
        if num % 2 == 0:
            self.dir = 'x'
            self.z = random.randint(190, 200)
        else:
            self.dir = 'y'
            self.z = random.randint(210, 220)
        self.vel = random.randint(3, 5)
    
    def move(self, grid_size, cell_size):
        if self.dir == 'x':
            self.x += self.vel
            if self.x > grid_size * cell_size:  # Restart at x = 0
                self.x = 0
        elif self.dir == 'y':
            self.y += self.vel
            if self.y > grid_size * cell_size:  # Restart at y = 0
                self.y = 0

def initialize_satellites(max_satellites):
    """ Initialize up to max_satellites satellites."""
    return [Satellite(num) for num in range(max_satellites)]

def update_positions(satellites, grid_size, cell_size):
    """ Update positions of satellites based on their direction and velocity."""
    for satellite in satellites:
        satellite.move(grid_size, cell_size)

def plot_3d_grid(ax, satellites, grid_size, cell_size, step):
    """ Plot the 3D grid and satellite positions on the same graph."""
    ax.clear()
    ax.set_title(f"3D Grid World Simulation - Step {step}")
    ax.set_xlim(0, grid_size * cell_size)
    ax.set_ylim(0, grid_size * cell_size)
    ax.set_zlim(100, 200)
    
    # Draw grid lines
    for i in range(grid_size + 1):
        ax.plot([i * cell_size, i * cell_size], [0, grid_size * cell_size], zs=100, color='k', linestyle='--', linewidth=0.5)
        ax.plot([0, grid_size * cell_size], [i * cell_size, i * cell_size], zs=100, color='k', linestyle='--', linewidth=0.5)
    
    # Plot satellites
    for satellite in satellites:
        color = 'blue' if satellite.dir == 'x' else 'red'
        ax.scatter(satellite.x, satellite.y, satellite.z, color=color)

def simulate(grid_size=5, cell_size=10, max_satellites=10, steps=50):
    """ Run the 3D simulation. """
    satellites = initialize_satellites(max_satellites)
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    for step in range(steps):
        update_positions(satellites, grid_size, cell_size)
        plot_3d_grid(ax, satellites, grid_size, cell_size, step)
        plt.pause(1.0)  # Update at 1 Hz
    plt.show()

if __name__ == "__main__":
    simulate()