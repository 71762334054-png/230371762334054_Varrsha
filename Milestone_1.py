import json
import math

def travel_time(curr_pos, die, velocity):
    dx = die[0] - curr_pos[0]
    dy = die[1] - curr_pos[1]
    distance = math.sqrt(dx**2 + dy**2)
    return distance / velocity if velocity > 0 else float('inf')

def path_time(start_point, path, velocity):
    total = 0
    curr = start_point
    for die in path:
        total += travel_time(curr, die, velocity)
        curr = die
    return total

def nearest_neighbor(start_point, die_centers, velocity):
    unvisited = die_centers[:]
    path = []
    curr = start_point
    while unvisited:
        next_die = min(unvisited, key=lambda d: travel_time(curr, d, velocity))
        path.append(next_die)
        unvisited.remove(next_die)
        curr = next_die
    return path

def two_opt(start_point, path, velocity):
    best = path[:]
    improved = True
    while improved:
        improved = False
        for i in range(len(best) - 2):
            for j in range(i + 2, len(best)):
                new_path = best[:]
                new_path[i:j] = reversed(best[i:j])
                if path_time(start_point, new_path, velocity) < path_time(start_point, best, velocity):
                    best = new_path
                    improved = True
        path = best
    return best


with open('Input_Milestone1_Testcase4.json', 'r') as file:
    data = json.load(file)

start_point = tuple(data["InitialPosition"])
velocity = data["StageVelocity"]


die_centers = []
for die in data["Dies"]:
    corners = die["Corners"]
    cx = sum(p[0] for p in corners) / len(corners)
    cy = sum(p[1] for p in corners) / len(corners)
    die_centers.append((cx, cy))


nn_path = nearest_neighbor(start_point, die_centers, velocity)

opt_path = two_opt(start_point, nn_path, velocity)


opt_path_lists = [[x, y] for (x, y) in opt_path]

full_path = [[start_point[0], start_point[1]]] + opt_path_lists

total_time = path_time(start_point, opt_path, velocity)
output = {"TotalTime": total_time, "Path": full_path}

print(output)

with open("TestCase_1_4.json", "w") as f:
    json.dump(output, f, indent=4)
