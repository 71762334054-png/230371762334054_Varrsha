import json
import math
import random
import heapq
import time

def get_die_orientation(corners):
    p0 = corners[0]
    p1 = corners[1]
    return math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0])) % 360

def angular_diff(a1, a2):
    diff = abs(a1 - a2) % 90
    return min(diff, 90 - diff)

def motion_time(distance, vmax, acc):
    if distance < 1e-9: return 0.0
    if acc == 0: return distance / vmax
    t_acc = vmax / acc
    d_acc = 0.5 * acc * t_acc * t_acc
    if distance >= 2 * d_acc:
        return 2 * t_acc + (distance - 2 * d_acc) / vmax
    else:
        return 2 * math.sqrt(distance / acc)

def is_point_in_rect(pt, rect):

    x, y = pt
    rx1, ry1, rx2, ry2 = rect
    return (rx1 <= x <= rx2) and (ry1 <= y <= ry2)

def is_blocked(p1, p2, obstacles):

    EPS = 1e-7 
    
    lx_min, lx_max = min(p1[0], p2[0]), max(p1[0], p2[0])
    ly_min, ly_max = min(p1[1], p2[1]), max(p1[1], p2[1])

    for obs in obstacles:
        ox1, oy1, ox2, oy2 = obs
        
        if is_point_in_rect(p1, obs) or is_point_in_rect(p2, obs):
            return True

        sx1, sy1 = ox1 + EPS, oy1 + EPS
        sx2, sy2 = ox2 - EPS, oy2 - EPS
        
        if lx_max < sx1 or lx_min > sx2 or ly_max < sy1 or ly_min > sy2:
            continue

        def ccw(A, B, C):
            return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])
        def intersect(A, B, C, D):
            return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

        c1, c2 = (sx1, sy1), (sx2, sy1)
        c3, c4 = (sx2, sy2), (sx1, sy2)
        
        
        if intersect(p1, p2, c1, c2): return True 
        if intersect(p1, p2, c2, c3): return True 
        if intersect(p1, p2, c3, c4): return True 
        if intersect(p1, p2, c4, c1): return True 
        if intersect(p1, p2, c1, c3): return True 
        if intersect(p1, p2, c2, c4): return True 

    return False


def build_visibility_graph(obstacles, wafer_r): #contains the coordinates of the obstacles
    nodes = []
    
    margin = 0.001   
    for x1, y1, x2, y2 in obstacles:
        candidates = [
            (x1 - margin, y1 - margin), #bottom-left
            (x2 + margin, y1 - margin), #bottom-right
            (x2 + margin, y2 + margin), #top-right
            (x1 - margin, y2 + margin)  #top-left
        ]
        
        for pt in candidates:
            if math.hypot(pt[0], pt[1]) > wafer_r:
                continue
                
            inside_any = False
            for obs in obstacles:
                if is_point_in_rect(pt, obs):
                    inside_any = True
                    break
            
            if not inside_any:
                nodes.append(pt)

    graph = {i: [] for i in range(len(nodes))}
    
    print(f"Building Visibility Graph with {len(nodes)} safe intermediate nodes...")
      
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            if not is_blocked(nodes[i], nodes[j], obstacles):
                dist = math.hypot(nodes[i][0]-nodes[j][0], nodes[i][1]-nodes[j][1])
                graph[i].append((j, dist))
                graph[j].append((i, dist))
                
    return nodes, graph


def find_path_heuristic(start, end, graph, nodes, obstacles):
    if not is_blocked(start, end, obstacles):
        return [start, end]

    start_links = []
    end_links = []
    

    for i, node_pos in enumerate(nodes):
        if not is_blocked(start, node_pos, obstacles):
            d = math.hypot(start[0]-node_pos[0], start[1]-node_pos[1])
            start_links.append((i, d))

        if not is_blocked(end, node_pos, obstacles):
            d = math.hypot(end[0]-node_pos[0], end[1]-node_pos[1])
            end_links.append((i, d))

    if not start_links or not end_links:
        return [start, end] 

    pq = []

    for idx, dist in start_links:
        heapq.heappush(pq, (dist, idx, [idx]))
        
    visited = {} 
    best_path_indices = None
    min_dist = float('inf')

    end_reachable = {idx: d for idx, d in end_links}

    while pq:
        d, curr, path = heapq.heappop(pq)
        

        if d > min_dist: continue


        if curr in end_reachable:
            total_d = d + end_reachable[curr]
            if total_d < min_dist:
                min_dist = total_d
                best_path_indices = path
        
        if curr in visited and visited[curr] <= d: continue
        visited[curr] = d
        
        for neighbor, weight in graph[curr]:
            new_d = d + weight
            if new_d < min_dist:
                heapq.heappush(pq, (new_d, neighbor, path + [neighbor]))
                
    if best_path_indices:
        return [start] + [nodes[i] for i in best_path_indices] + [end]
    else:
        return [start, end]



def calculate_full_cost(p1, a1, p2, a2, graph, nodes, obstacles, sv, sa, cv, ca):
    #nodes -> intermediate points
    path_points = find_path_heuristic(p1, p2, graph, nodes, obstacles)
    
    t_stage = 0.0
    for i in range(len(path_points) - 1):
        dist = math.hypot(path_points[i+1][0] - path_points[i][0], 
                          path_points[i+1][1] - path_points[i][1])
        t_stage += motion_time(dist, sv, sa)
        
    t_rot = motion_time(angular_diff(a1, a2), cv, ca)
    
    return max(t_stage, t_rot), path_points

def build_matrix(dies, start_pos, start_angle, graph, nodes, obstacles, sv, sa, cv, ca):
    n = len(dies)
    mat = [[0.0]*n for _ in range(n)]
    start_vec = [0.0]*n
    path_cache = {}
    start_path_cache = {}
    
    print("Calculating travel costs...")
    for i in range(n):
        c, p = calculate_full_cost(start_pos, start_angle, dies[i]['center'], dies[i]['angle'],
                                   graph, nodes, obstacles, sv, sa, cv, ca)
        start_vec[i] = c
        start_path_cache[i] = p

    for i in range(n):
        for j in range(n):
            if i != j:
                c, p = calculate_full_cost(dies[i]['center'], dies[i]['angle'], 
                                           dies[j]['center'], dies[j]['angle'],
                                           graph, nodes, obstacles, sv, sa, cv, ca)
                mat[i][j] = c
                path_cache[(i,j)] = p
    return mat, start_vec, start_path_cache, path_cache



def get_cost(path, start_vec, mat):
    t = start_vec[path[0]]
    for i in range(len(path)-1): t += mat[path[i]][path[i+1]]
    return t

def two_opt(path, start_vec, mat):
    best_path = path[:]
    best_cost = get_cost(best_path, start_vec, mat)
    improved = True
    while improved:
        improved = False
        for i in range(len(best_path)-1):
            for j in range(i+1, len(best_path)):
                new_path = best_path[:i] + best_path[i:j+1][::-1] + best_path[j+1:]
                c = get_cost(new_path, start_vec, mat)
                if c < best_cost - 1e-9:
                    best_cost = c
                    best_path = new_path
                    improved = True
                    break
            if improved: break
    return best_path, best_cost

def perturbation_kick(path):
    if len(path) < 4: return path[:]
    cuts = sorted(random.sample(range(1, len(path)), 3))
    i, j, k = cuts
    return path[:i] + path[k:] + path[j:k] + path[i:j]

def run_ils(dies, start_vec, mat, iterations=200):
    n = len(dies)
    unvisited = set(range(n))
    curr = -1
    initial_path = []
    
    while unvisited:
        if curr == -1: nxt = min(unvisited, key=lambda x: start_vec[x])
        else: nxt = min(unvisited, key=lambda x: mat[curr][x])
        initial_path.append(nxt)
        unvisited.remove(nxt)
        curr = nxt
        
    current_path, current_cost = two_opt(initial_path, start_vec, mat)
    best_path = current_path[:]
    best_cost = current_cost
    
    print(f"Initial Optimized Cost: {best_cost:.4f}")
    
    for _ in range(iterations):
        mutated = perturbation_kick(current_path)
        new_path, new_cost = two_opt(mutated, start_vec, mat)
        if new_cost < best_cost:
            best_cost = new_cost
            best_path = new_path
            current_path = new_path
        elif new_cost < current_cost:
            current_path = new_path
            current_cost = new_cost
            
    return best_path

start_time = time.time()

input_file = "Input_Milestone4_Testcase1.json"
output_file = "TestCase_4_1.json"

try:
    with open(input_file) as f:
        data = json.load(f)
except FileNotFoundError:
    print(f"File {input_file} not found")
    exit()

SV = data["StageVelocity"]
SA = data["StageAcceleration"]
CV = data["CameraVelocity"]
CA = data["CameraAcceleration"]
start_pos = tuple(data["InitialPosition"])
init_angle = data.get("InitialAngle", 0)
wafer_r = data["WaferDiameter"]/2


obstacles = []
for zone in data.get("ForbiddenZones", []):
    if isinstance(zone, dict):
        p1 = zone["BottomLeft"]
        p2 = zone["TopRight"]
    else:
        p1 = [float(c) for c in zone[0]]
        p2 = [float(c) for c in zone[1]]
    

    obstacles.append((min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])))


graph_nodes, visibility_graph = build_visibility_graph(obstacles, wafer_r)


dies_data = []
for die in data["Dies"]:
    c = die["Corners"]
    cx = sum(p[0] for p in c)/4
    cy = sum(p[1] for p in c)/4
    if math.hypot(cx, cy) > wafer_r: continue
    dies_data.append({'center': (cx, cy), 'angle': get_die_orientation(c)})

mat, s_vec, s_path_cache, path_cache = build_matrix(
    dies_data, start_pos, init_angle, visibility_graph, graph_nodes, obstacles, SV, SA, CV, CA
)


print("Running ILS Optimization...")
best_indices = run_ils(dies_data, s_vec, mat, iterations=150)


final_path = [list(start_pos)]
final_time = get_cost(best_indices, s_vec, mat)

seg0 = s_path_cache[best_indices[0]]
final_path.extend([list(p) for p in seg0[1:]])

for k in range(len(best_indices)-1):
    u, v = best_indices[k], best_indices[k+1]
    seg = path_cache[(u,v)]
    final_path.extend([list(p) for p in seg[1:]])

print(f"✅ Final Time: {final_time:.4f}")
with open(output_file, "w") as f:
    json.dump({"TotalTime": final_time, "Path": final_path}, f, indent=4)
    
end_time = time.time()

print(end_time - start_time)