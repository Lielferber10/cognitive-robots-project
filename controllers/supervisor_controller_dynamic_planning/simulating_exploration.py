import random
import math
import matplotlib.pyplot as plt

# Define the grid size and target location
GRID_SIZE = 20  # 20x20 grid
target_position = (random.randint(0, GRID_SIZE - 1), random.randint(0, GRID_SIZE - 1))

# Bresenham's line algorithm to get points between start and end
def bresenham_line(start, end):
    points = []
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if (x0, y0) == (x1, y1):
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points

# Simulate exploration with or without heuristics
def explore_new_destination(current_position, uninspected_positions, use_heuristics=False, lines_check_num=30):
    if not use_heuristics:
        destination = random.choice(uninspected_positions)
        return destination
    else:
        best_value = -math.inf
        best_destination = None
        for _ in range(lines_check_num):
            destination = random.choice(uninspected_positions)
            points_on_line = bresenham_line(current_position, destination)
            value = sum(1 for point in points_on_line if point in uninspected_positions)
            if value > best_value:
                best_value = value
                best_destination = destination
        return best_destination

# Run the simulation and collect data
def run_simulation(use_heuristics, num_trials=10):
    results = []
    for _ in range(num_trials):
        current_position = (0, 0)  # Starting position of the robot
        uninspected_positions = {(x, y) for x in range(GRID_SIZE) for y in range(GRID_SIZE)}
        moves = 0
        re_plans = 0

        while current_position != target_position:
            uninspected_positions.discard(current_position)
            destination = explore_new_destination(current_position, list(uninspected_positions), use_heuristics)
            path = bresenham_line(current_position, destination)

            # Move to the destination one step at a time
            for step in path:
                moves += 1
                current_position = step
                if current_position == target_position:
                    break

            re_plans += 1

        results.append((moves, re_plans))

    return results

# Collect data and plot comparisons
def plot_results(random_results, heuristic_results):
    random_moves, random_replans = zip(*random_results)
    heuristic_moves, heuristic_replans = zip(*heuristic_results)

    plt.figure(figsize=(12, 5))
    plt.subplot(1, 2, 1)
    plt.plot(random_moves, label='Random Moves', marker='o')
    plt.plot(heuristic_moves, label='Heuristic Moves', marker='x')
    plt.xlabel('Trial')
    plt.ylabel('Moves')
    plt.title('Number of Moves Comparison')
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.plot(random_replans, label='Random Re-Plans', marker='o')
    plt.plot(heuristic_replans, label='Heuristic Re-Plans', marker='x')
    plt.xlabel('Trial')
    plt.ylabel('Re-Plans')
    plt.title('Number of Re-Plans Comparison')
    plt.legend()

    plt.tight_layout()
    plt.show()

# Run simulations and plot the graph
random_results = run_simulation(use_heuristics=False, num_trials=80)
heuristic_results = run_simulation(use_heuristics=True, num_trials=80)
plot_results(random_results, heuristic_results)
avg_random_moves = sum(moves for moves, _ in random_results) / len(random_results)
avg_heuristic_moves = sum(moves for moves, _ in heuristic_results) / len(heuristic_results)
print(f'Average number of moves for random exploration: {avg_random_moves}')
print(f'Average number of moves for heuristic exploration: {avg_heuristic_moves}')
avg_random_replans = sum(replans for _, replans in random_results) / len(random_results)
avg_heuristic_replans = sum(replans for _, replans in heuristic_results) / len(heuristic_results)
print(f'Average number of re-plans for random exploration: {avg_random_replans}')
print(f'Average number of re-plans for heuristic exploration: {avg_heuristic_replans}')

