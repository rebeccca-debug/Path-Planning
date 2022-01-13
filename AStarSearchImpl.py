import numpy as np
import matplotlip.pyplot as plt


start = np.array([842, 160])
goal = np.array([95, 518])
grid = np.load('new_york.npy')

# Copies of the grid to be used for visualizing results.
path = np.zeros([len(grid), len(grid)], dtype=int)
#Make path results stand out more in figure.
path -= 1000
best_path = np.zeros([len(grid), len(grid)], dtype=int)

class AStarSearch:
    def __init__(self, start, goal, grid, path, h_grid):
        self.pos = start
        self.pos_str = str(start)
        self.pos_depth = 0
        self.goal_str = str(goal)
        self.explored = {}
        self.not_explored = {}
        self.not_explored[str(start)] = 0
        self.grid = grid
        self.path = path
        self.h_grid = h_grid

    def get_possible_moves(self):
        potential_moves = self.generate_potential_moves(self.pos)
        for move in potential_moves:
            # Check if potential move is valid.
            if not self.valid_move(move):
                continue
            # Check if move has already been explored.
            if (str(move) not in self.explored) and (
                str(move) not in self.not_explored):
                self.not_explored[str(move)] = self.pos_depth + 1 + self.heuristic(move)
                #Visualize the Heuristic Grid
                self.h_grid[move[0], move[1]] = self.pos_depth + 1 + self.heuristic(move)
        # Since all next possible moves have been determined,
        # consider current location explored.
        self.explored[self.pos_str] = 0
        return True

    def goal_found(self):
        if self.goal_str in self.not_explored:
            # Add goal to path.
            self.pos = self.string_to_array(self.goal_str)
            self.pos_depth = self.not_explored.pop(self.goal_str)
            self.path[self.pos[0], self.pos[1]] = self.pos_depth
            return True
        return False

    def explore_next_move(self):
        #Determine next move to explore
        sorted_not_explored = sorted(
            self.not_explored,
            key=self.not_explored.get,
            reverse=False)

        #Determine the pos and depth of the next move.
        self.pos_str = sorted_not_explored[0]
        self.pos = self.string_to_array(self.pos_str)
        self.pos_depth = self.not_explored.pop(self.pos_str) - self.heuristic(self.pos)

        #Write depth of next move onto path.
        self.path[self.pos[0], self.pos[1]] = round(self.pos_depth,1)

        return True

    def heuristic(self, move):
        distance = move - goal
        distance_squared = distance * distance
        return round(np.sqrt(sum(distance_squared)), 1)

    def generate_potential_moves(self,pos):
        u = np.array([-1, 0])
        d = np.array([-1, 0])
        l = np.array([0, -1])
        r = np.array([0, 1])

        potential_moves = [pos + u, pos + d, pos + l, pos + r]

        return potential_moves

    def valid_move(self, move):
        #Check if out of boundary.
        if (move[0] < 0) or (move[0] > 9):
            return False
        if (move[1] < 0) or (move[1] > 9):
            return False
        #Check if wall or obstacle exists.
        if self.grid[move[0], move[1]] == 1:
            return False
        return True

    def string_to_array(self, string):
        array = [int(string[1]), int(string[3])]
        return np.array(array)

astar = AStarSearch(start, goal, grid, path)

explored_count = 0
while True:
    # Determine next possible moves
    astar.get_possible_moves()
    if astar.goal_found():
        break
    astar.explore_next_move()

print('')
print('Heuristic Grid')
print('--------------')
print(grid)
print('')
print('')
print('Explored Path')