import json
import math


def get_die_orientation(corners):
    p0 = corners[0]
    p1 = corners[1]
    return math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0])) % 360

def angular_diff(a1, a2):
    diff = abs(a1 - a2) % 90
    return min(diff, 90 - diff)


def motion_time(distance, vmax, acc):
    if distance == 0:
        return 0.0

    if acc == 0:
        return distance / vmax

    t_acc = vmax / acc
    d_acc = 0.5 * acc * t_acc * t_acc

    if distance >= 2 * d_acc:
        return 2 * t_acc + (distance - 2 * d_acc) / vmax
    else:
        return 2 * math.sqrt(distance / acc)



def travel_time(curr_pos, target_pos, velocity):
    dist = math.hypot(
        target_pos[0] - curr_pos[0],
        target_pos[1] - curr_pos[1]
    )
    return motion_time(dist, velocity, STAGE_ACC)

def rotation_time(curr_angle, target_angle, velocity):
    ang = angular_diff(curr_angle, target_angle)
    return motion_time(ang, velocity, CAMERA_ACC)

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
        best_die = min(
            unvisited,
            key=lambda d: calculate_step_cost(
                curr_pos, curr_angle,
                d['center'], d['angle'],
                stage_v, cam_v
            )
        )
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
        total += calculate_step_cost(
            curr_pos, curr_angle,
            die['center'], die['angle'],
            stage_v, cam_v
        )
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

                if new_time < best_time:
                    best_path = new_path
                    best_time = new_time
                    improved = True
                    break
            if improved:
                break

    return best_path

def compute_total_time_verified(full_path, die_lookup, stage_v, cam_v, initial_angle):
    total = 0.0
    curr_pos = full_path[0]
    curr_angle = initial_angle

    for i in range(1, len(full_path)):
        pos = tuple(full_path[i])
        target_angle = die_lookup[(round(pos[0],4), round(pos[1],4))]

        total += calculate_step_cost(
            curr_pos, curr_angle,
            pos, target_angle,
            stage_v, cam_v
        )
        curr_pos = pos
        curr_angle = target_angle

    return total


with open("Input_Milestone3_Testcase4.json") as f:
    data = json.load(f)

STAGE_ACC = data.get("StageAcceleration", 0)
CAMERA_ACC = data.get("CameraAcceleration", 0)

start_pos = tuple(data["InitialPosition"])
initial_angle = data.get("InitialAngle", 0)
stage_v = data["StageVelocity"]
cam_v = data["CameraVelocity"]

dies_data = []
die_lookup = {}
wafer_radius = data["WaferDiameter"] / 2

for die in data["Dies"]:
    corners = die["Corners"]
    cx = sum(p[0] for p in corners) / 4
    cy = sum(p[1] for p in corners) / 4
    angle = get_die_orientation(corners)
    if math.hypot(cx, cy) > wafer_radius: 
        continue

    dies_data.append({'center': (cx, cy), 'angle': angle})
    die_lookup[(round(cx,4), round(cy,4))] = angle

greedy = nearest_neighbour(start_pos, initial_angle, dies_data, stage_v, cam_v)
optimized = two_opt_time_based(greedy, start_pos, initial_angle, stage_v, cam_v)

path_coords = [[start_pos[0], start_pos[1]]] + \
              [[d['center'][0], d['center'][1]] for d in optimized]

total_time = compute_total_time_verified(
    path_coords,
    die_lookup,
    stage_v,
    cam_v,
    initial_angle
)

output = {
    "TotalTime": total_time,
    "Path": path_coords
}

with open("TestCase_3_4.json", "w") as f:
    json.dump(output, f, indent=4)

print("Total Time:", total_time)
