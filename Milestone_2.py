import json
import math


def get_die_orientation(corners):
    p0 = corners[0]
    p1 = corners[1]
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    return math.degrees(math.atan2(dy, dx))

def angular_diff(a1, a2):
    a1 = a1 % 360
    a2 = a2 % 360
    diff = abs(a1 - a2) % 90
    return min(diff, 90 - diff)


def travel_time(curr_pos, target_pos, velocity):
    if velocity <= 0: return float('inf')
    dist = math.hypot(target_pos[0] - curr_pos[0], target_pos[1] - curr_pos[1])
    return dist / velocity

def rotation_time(curr_angle, target_angle, velocity):
    if velocity <= 0: return 0
    return angular_diff(curr_angle, target_angle) / velocity

def calculate_step_cost(curr_pos, curr_angle, target_pos, target_angle, stage_v, cam_v):
    t_move = travel_time(curr_pos, target_pos, stage_v)
    t_rot = rotation_time(curr_angle, target_angle, cam_v)
    return max(t_move, t_rot)


def nearest_neighbour(start_pos, start_angle, dies_data, stage_v, cam_v):

    unvisited = dies_data[:]
    path = []
    
    curr_pos = start_pos
    curr_angle = start_angle

    while unvisited:
        best_die = None
        min_cost = float('inf')

        for die in unvisited:
            cost = calculate_step_cost(
                curr_pos, curr_angle, 
                die['center'], die['angle'], 
                stage_v, cam_v
            )
            
            if cost < min_cost:
                min_cost = cost
                best_die = die
        
        path.append(best_die)
        unvisited.remove(best_die)
        
        curr_pos = best_die['center']
        curr_angle = best_die['angle']

    return path

def calculate_path_total_time(path, start_pos, start_angle, stage_v, cam_v):
    total = 0.0
    curr_pos = start_pos
    curr_angle = start_angle
    
    for die in path:
        cost = calculate_step_cost(
            curr_pos, curr_angle, 
            die['center'], die['angle'], 
            stage_v, cam_v
        )
        total += cost
        curr_pos = die['center']
        curr_angle = die['angle']
        
    return total

def two_opt_time_based(path, start_pos, start_angle, stage_v, cam_v):
    best_path = path[:]
    best_time = calculate_path_total_time(best_path, start_pos, start_angle, stage_v, cam_v)
    
    improved = True
    while improved:
        improved = False
        for i in range(len(best_path) - 1):
            for j in range(i + 1, len(best_path)):
                
                new_path = best_path[:i] + best_path[i:j+1][::-1] + best_path[j+1:]
                
                new_time = calculate_path_total_time(new_path, start_pos, start_angle, stage_v, cam_v)
                
                if new_time < best_time - 1e-9:
                    best_path = new_path
                    best_time = new_time
                    improved = True
                    break 
            if improved: break
            
    return best_path



def compute_total_time_verified(full_path, die_lookup, stage_v, cam_v, initial_angle):

    total_time = 0.0
    curr_pos = full_path[0]
    curr_angle = initial_angle

    for i in range(1, len(full_path)):
        target_pos = tuple(full_path[i])

        lookup_key = (round(target_pos[0], 4), round(target_pos[1], 4))
        
        if lookup_key in die_lookup:
            target_angle = die_lookup[lookup_key]
        else:
            target_angle = curr_angle 

        # Calculate Cost
        step_cost = calculate_step_cost(
            curr_pos, curr_angle, 
            target_pos, target_angle, 
            stage_v, cam_v
        )
        
        total_time += step_cost
        
        curr_pos = target_pos
        curr_angle = target_angle
        
    return total_time


input_filename = "Input_Milestone2_Testcase4.json"
    
with open(input_filename, "r") as f:
    data = json.load(f)

start_pos = tuple(data["InitialPosition"])
initial_angle = data.get("InitialAngle", 0)
stage_v = data["StageVelocity"]
cam_v = data["CameraVelocity"]


dies_data = []
die_lookup = {} 

for die in data["Dies"]:
    corners = die["Corners"]

    cx = sum(p[0] for p in corners) / 4
    cy = sum(p[1] for p in corners) / 4

    angle = get_die_orientation(corners)
    
    dies_data.append({'center': (cx, cy), 'angle': angle})

    key = (round(cx, 4), round(cy, 4))
    die_lookup[key] = angle

print("Running Greedy Solver...")
greedy_path = nearest_neighbour(start_pos, initial_angle, dies_data, stage_v, cam_v)

print("Running 2-Opt Optimization (Time-Based)...")
optimized_path = two_opt_time_based(greedy_path, start_pos, initial_angle, stage_v, cam_v)

full_path_coords = [[start_pos[0], start_pos[1]]] + \
                    [[d['center'][0], d['center'][1]] for d in optimized_path]

final_time = compute_total_time_verified(
    full_path_coords, 
    die_lookup, 
    stage_v, 
    cam_v, 
    initial_angle
)

output = {
    "TotalTime": final_time,
    "Path": full_path_coords
}

print(f"Total Time: {final_time:.4f}")

with open("TestCase_2_4.json", "w") as f:
    json.dump(output, f, indent=4)
