import heapq # Helps with A* search

class World:
    def __init__(self, min_x, max_x, min_y, max_y):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.rows = max_x - min_x + 1
        self.cols = max_y - min_y + 1
        self.grid = [['~~' for _ in range(self.cols)] for _ in range(self.rows)]
        self.player = None
        self.treasures = [(-1, 3), (-1, 6), (-2, 7), (2, 5)]
        self.traps = [(-2, 6, "T1"), (0, 1, "T2"), (3, 5, "T2"), (0, 5, "T3"), (-2, 4, "T3"), (-1, 2, "T4")]
        self.rewards = [(-2, 2, "R1"), (2, 3, "R1"), (-2, 5, "R2"), (2, 7, "R2")]
        self.obstacles = [(3, 3), (1, 3), (1, 4), (0, 4), (-3, 5), (0, 6), (2, 6), (0, 7), (1, 7)]
        self.in_bounds = [(5, i) for i in range(5, 6)] + [(4, i) for i in range(4, 7)] + [(3, i) for i in range(3, 8)] + \
                         [(2, i) for i in range(2, 9)] + [(1, i) for i in range(1, 10)] + [(0, i) for i in range(1, 10)] + \
                         [(-1, i) for i in range(0, 9)] + [(-2, i) for i in range(1, 8)] + [(-3, i) for i in range(2, 7)] + \
                         [(-4, i) for i in range(3, 6)] + [(-5, i) for i in range(4, 5)] + [(-5, i) for i in range(4, 5)]
        self.steps = 0

    def display_world(self):
        for row in self.grid:
            print(" ".join(row))

    def set_player(self, player):
        self.player = player
        self.update_grid(player.position, 'PS')

    def arrange_grid(self):
        # Add boundaries of the playable area
        for pos in self.in_bounds:
            self.update_grid(pos, 'EE')

        # Add treasures
        for treasure in self.treasures:
            self.update_grid(treasure, 'XX')

        # Add traps
        for trap in self.traps:
            self.update_grid((trap[0], trap[1]), trap[2])

        # Add obstacles
        for obstacle in self.obstacles:
            self.update_grid(obstacle, 'OO')

        # Add rewards
        for reward in self.rewards:
            self.update_grid((reward[0], reward[1]), reward[2])

    def update_grid(self, position, symbol):
        grid_x = position[0] - self.min_x
        grid_y = position[1] - self.min_y
        if 0 <= grid_x < len(self.grid) and 0 <= grid_y < len(self.grid[0]):
            self.grid[grid_x][grid_y] = symbol

    def apply_traps(self):
        for trap in self.traps:
            if self.player.position == (trap[0], trap[1]):
                trap_type = trap[2]
                if trap_type == "T1":
                    self.player.energy_per_step = 2
                elif trap_type == "T2":
                    self.player.speed *= 0.5
                elif trap_type == "T3":
                    displacement = (self.player.last_direction[0] * 2, self.player.last_direction[1] * 2)
                    new_position = (self.player.position[0] + displacement[0], self.player.position[1] + displacement[1])
                    if self.min_x <= new_position[0] <= self.max_x and self.min_y <= new_position[1] <= self.max_y:
                        self.player.position = new_position
                elif trap_type == "T4":
                    self.remove_uncollected_treasures()

    def apply_rewards(self):
        for reward in self.rewards:
            if self.player.position == (reward[0], reward[1]):
                reward_type = reward[2]
                if reward_type == "R1":
                    self.player.energy_per_step = 0.5
                elif reward_type == "R2":
                    self.player.speed *= 2
                self.rewards.remove(reward)
                break

    def move_player(self, direction):
        self.update_grid(self.player.position, 'EE')  # Clear previous position
        self.player.move(direction)
        self.steps += 1
        self.player.energy -= self.player.energy_per_step
        self.apply_traps()
        self.apply_rewards()
        self.update_grid(self.player.position, 'PS')  # Update new position

    def remove_uncollected_treasures(self):
        self.treasures = []
        for row in range(self.rows):
            for col in range(self.cols):
                if self.grid[row][col] == 'XX':
                    self.grid[row][col] = 'EE'
    
    # Heuristic function to estimate the cost from current node 'a' to target node 'b'
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def neighbors(self, node):
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
        result = []
        for direction in directions:
            neighbor = (node[0] + direction[0], node[1] + direction[1])
            if self.min_x <= neighbor[0] <= self.max_x and self.min_y <= neighbor[1] <= self.max_y:
                if self.grid[neighbor[0] - self.min_x][neighbor[1] - self.min_y] not in ['~~', 'OO', 'T1', 'T2', 'T3', 'T4']:
                    result.append(neighbor)
        return result

    def a_star_search(self, start, goal):
        # Priority queue to store the nodes to be explored
        open_list = []
        # Push the start node into the open list with f_score of 0
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            _, current = heapq.heappop(open_list)

            # If the goal is reached, reconstruct and return the path
            if current == goal: 
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            # Explore each neighbor of the current node
            for neighbor in self.neighbors(current):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        # Return an empty path if the goal is not reachable
        return []

    def collect_all_treasures(self):
        for treasure in self.treasures:
            initial_steps = self.steps
            initial_energy = self.player.energy
            path = self.a_star_search(self.player.position, treasure)
            for step in path:
                direction = (step[0] - self.player.position[0], step[1] - self.player.position[1])
                self.move_player(direction)
            print(f"\nTreasure at {treasure} collected.")
            print(f"Path taken to this treasure: {path}")
            print(f"Steps taken to this treasure: {self.steps - initial_steps}")
            print(f"Energy consumed for this path: {initial_energy - self.player.energy}")
            print(f"Current Total Player Energy: {self.player.energy}")

class Reward:
    def __init__(self, position, reward_type):
        self.position = position
        self.reward_type = reward_type

class Player:
    def __init__(self, position):
        self.position = position
        self.energy = 100
        self.energy_per_step = 1
        self.speed = 1
        self.last_direction = (0, 0)

    def move(self, direction):
        self.last_direction = direction
        new_position = (self.position[0] + direction[0] * self.speed,
                        self.position[1] + direction[1] * self.speed)
        if world.min_x <= new_position[0] <= world.max_x and world.min_y <= new_position[1] <= world.max_y:
            self.position = (int(new_position[0]), int(new_position[1]))

class Trap:
    def __init__(self, position, trap_type):
        self.position = position
        self.trap_type = trap_type

# Create a virtual world with defined bounds
world = World(-5, 5, 0, 9)

# Create and set the player
player = Player((0, 0))
world.set_player(player)

# Arrange the grid as per the given layout
world.arrange_grid()

# Display the legend
print("[Legend of the virtual world is displayed below]")
print("\n~~: Out of Bounds")
print("EE: Empty Space")
print("PS: Player Start")
print("XX: Treasure")
print("T1 & T2 & T3 & T4: Traps")
print("OO: Obstacle")
print("R1 & R2: Rewards")

# Display the initial virtual world
print("\n]------[Initial World]------[")
print()
world.display_world()

# Collect all treasures using A* search
world.collect_all_treasures()

# Display the final virtual world
print("\n]-------[Final World]-------[")
print()
world.display_world()

# Display final energy value and steps taken
print(f"\nFinal Player Energy: {player.energy}")
print(f"Total Steps Taken: {world.steps}")
