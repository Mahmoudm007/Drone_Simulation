import tkinter as tk
import random
import pandas as pd
import numpy as np
import heapq

def rescale_to_grayscale(value):
    # Ensure the value is within the expected range
    if value < 0:
        value = 0
    elif value > 106:
        value = 106
    
    # Rescale the value to the range 0-255
    scaled_value = int((value / 106) * 255)
    
    # Convert the scaled value to a hexadecimal string
    hex_value = '#{:02x}{:02x}{:02x}'.format(scaled_value, scaled_value, scaled_value)
    
    return hex_value


class AStarApp:
    def __init__(self, master, grid, height_cost_weight=1):
        self.master = master
        master.wm_title("A* Algorithm")
        self.grid_size = len(grid)
        self.cell_size = 8  # Size of each cell in the grid
        self.grid = grid
        self.start = None
        self.goal = None
        self.height_cost_weight = height_cost_weight
        self.canvas = tk.Canvas(master, width=self.grid_size * self.cell_size, height=self.grid_size * self.cell_size)
        self.canvas.pack()
        self.mode = 0  # 0 for setting start, 1 for setting goal, 2 for running A*
        self.path = []  # Add this line to store the path
        self.init_grid()

        self.master.bind('r', self.reset_grid)
        self.master.bind('c', self.clear_and_randomize_grid)

        center_gui(master)

    def init_grid(self):
        self.canvas.delete("all")
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                x1, y1 = j * self.cell_size, i * self.cell_size
                x2, y2 = x1 + self.cell_size, y1 + self.cell_size
                color = 'white' if self.grid[i][j] == 0 else 'grey'  # Grey for obstacles, white for free space
                if self.grid[i][j] == 0:
                    color = 'white'
                else:
                    color = rescale_to_grayscale(self.grid[i][j])
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline='grey')
        self.canvas.bind("<Button-1>", self.set_points)

    def set_points(self, event):
        i, j = event.y // self.cell_size, event.x // self.cell_size
        if self.mode == 0:
            self.start = (i, j, self.grid[i][j])
            self.canvas.create_rectangle(j * self.cell_size, i * self.cell_size, (j + 1) * self.cell_size,
                                         (i + 1) * self.cell_size, fill='green')
            self.mode = 1
        elif self.mode == 1:
            self.goal = (i, j, self.grid[i][j])
            self.canvas.create_rectangle(j * self.cell_size, i * self.cell_size, (j + 1) * self.cell_size,
                                         (i + 1) * self.cell_size, fill='red')
            self.mode = 2
            self.master.after(10, self.a_star_algorithm, self.start, self.goal)

    def heuristic(self, node1, node2):
        i1, j1, h1 = node1
        i2, j2, h2 = node2
        # Calculate the weighted height difference as part of the heuristic
        height_cost = abs(h1 - h2) * self.height_cost_weight
        return (abs(i1 - i2) + abs(j1 - j2)) + height_cost

    def find_neighbors(self, current):
        neighbors = []
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        for d in directions:
            neighbor_x, neighbor_y = current[0] + d[0], current[1] + d[1]
            if 0 <= neighbor_x < self.grid_size and 0 <= neighbor_y < self.grid_size:
                neighbor_z = self.grid[neighbor_x][neighbor_y]
                neighbors.append((neighbor_x, neighbor_y, neighbor_z))
        return neighbors

    def reconstruct_path(self, came_from, current):
        path = []
        while current != self.start:
            path.append((current[0], current[1]))
            x, y = current[1] * self.cell_size, current[0] * self.cell_size
            self.canvas.create_rectangle(x, y, x + self.cell_size, y + self.cell_size, fill='orange')
            current = came_from[current]
        path.append((self.start[0], self.start[1]))  # Add the start point to the path
        self.path = path[::-1]  # Store the path in reverse order (from start to goal)

    def get_path(self):
        return self.path

    def a_star_algorithm(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start, goal), start))
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        came_from = {}

        while open_set:
            self.master.update_idletasks()
            current = heapq.heappop(open_set)[1]

            if current == goal:
                self.reconstruct_path(came_from, current)
                return

            for neighbor in self.find_neighbors(current):
                # Calculate the actual movement cost considering height difference
                height_cost = abs(current[2] - neighbor[2]) * self.height_cost_weight
                tentative_g_score = g_score.get(current, float('inf')) + 1 + height_cost
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        x, y = neighbor[1] * self.cell_size, neighbor[0] * self.cell_size
                        color = 'red' if neighbor[2] > 0 else 'blue'
                        self.canvas.create_rectangle(x, y, x + self.cell_size, y + self.cell_size, fill=color)
            self.master.update()

        print("Path not found!")

    def generate_random_grid(self, size, obstacle_probability=0.3):
        grid = np.zeros((size, size))
        for i in range(size):
            for j in range(size):
                if random.random() < obstacle_probability:
                    grid[i, j] = random.randint(1, 10)  # Random height for obstacles
        return grid

    def reset_grid(self, event):
        self.mode = 0
        self.start = None
        self.goal = None
        self.init_grid()

    def clear_and_randomize_grid(self, event):
        self.grid = self.generate_random_grid(self.grid_size)
        self.reset_grid(event)

def center_gui(root):
    windowWidth = root.winfo_reqwidth()
    windowHeight = root.winfo_reqheight()
    positionRight = int(root.winfo_screenwidth() / 2 - windowWidth / 2)
    positionDown = int(root.winfo_screenheight() / 2 - windowHeight / 2)
    root.geometry("+{}+{}".format(positionRight, positionDown))

if __name__ == '__main__':
    # Replace the generate_random_grid function with reading the CSV file and creating the grid
    colliders_file_path = 'colliders.csv'

    # Extract relevant columns from the colliders DataFrame
    colliders_df = pd.read_csv(colliders_file_path, skiprows=1)

    # Convert relevant columns to float
    colliders_df = colliders_df.astype(float)

    # Define the size of the grid
    min_posX = colliders_df['posX'].min()
    max_posX = colliders_df['posX'].max()
    min_posY = colliders_df['posY'].min()
    max_posY = colliders_df['posY'].max()

    grid_size = (int(max_posX - min_posX) + 1, int(max_posY - min_posY) + 1)
    grid = np.zeros(grid_size)

    # Fill the grid with obstacles
    for index, obstacle in colliders_df.iterrows():
        x = int(obstacle['posX'] - min_posX)
        y = int(obstacle['posY'] - min_posY)
        z = int(obstacle['posZ'])  # Use original Z value for height
        half_size_x = int(obstacle['halfSizeX'])
        half_size_y = int(obstacle['halfSizeY'])

        grid[max(0, x - half_size_x):min(grid_size[0], x + half_size_x + 1),
             max(0, y - half_size_y):min(grid_size[1], y + half_size_y + 1)] = z

    # Scale the grid to 300x300 using nearest-neighbor interpolation
    scaled_grid_size = 100
    scale_x = scaled_grid_size / grid.shape[0]
    scale_y = scaled_grid_size / grid.shape[1]

    scaled_grid = np.zeros((scaled_grid_size, scaled_grid_size))

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            scaled_x = int(i * scale_x)
            scaled_y = int(j * scale_y)
            scaled_grid[scaled_x, scaled_y] = grid[i, j]

    root = tk.Tk()
    app = AStarApp(root, scaled_grid, height_cost_weight=0.3)  # You can adjust the weight here
    root.mainloop()
