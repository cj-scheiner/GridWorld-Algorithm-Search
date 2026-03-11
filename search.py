import matplotlib.path as mpath
import matplotlib.pyplot as plt
import numpy as np
import time
import matplotlib.animation as animation

from utils import *
from grid import *


def gen_polygons(worldfilepath):
    polygons = []
    with open(worldfilepath, "r") as f:
        lines = f.readlines()
        lines = [line[:-1] for line in lines]
        for line in lines:
            polygon = []
            pts = line.split(';')
            for pt in pts:
                xy = pt.split(',')
                polygon.append(Point(int(xy[0]), int(xy[1])))
            polygons.append(polygon)
    return polygons


def get_next_move(point, epolygons):
    valid_neighbors = []
    curr_x = point.x
    curr_y = point.y
    # Up, right, down, left
    steps = [(curr_x, curr_y + 1), (curr_x + 1, curr_y), (curr_x, curr_y - 1), (curr_x - 1, curr_y)]

    for next_x, next_y in steps:
        if 0 <= next_x < 50 and 0 <= next_y < 50:
            target_pt = Point(next_x, next_y)
            blocked = False
            for shape in epolygons:
                area = mpath.Path([(p.x, p.y) for p in shape])
                if area.contains_point((next_x, next_y), radius=0.1) or area.contains_point((next_x, next_y),
                                                                                            radius=-0.1):
                    blocked = True
                    break
            if not blocked:
                valid_neighbors.append(target_pt)
    return valid_neighbors


def get_cost(loc, tpolygons):
    for green_zone in tpolygons:
        turf_path = mpath.Path([(p.x, p.y) for p in green_zone])
        if turf_path.contains_point((loc.x, loc.y), radius=0.1) or turf_path.contains_point((loc.x, loc.y),
                                                                                            radius=-0.1):
            return 1.5
    return 1.0


def calc_path_cost(final_path, tpolygons):
    if not final_path: return 0
    total = 0
    for idx in range(1, len(final_path)):
        total += get_cost(final_path[idx], tpolygons)
    return total


def heuristic(start, goal):  # sqrt( (x-x)^2 + (y-y)^2 )
    return np.sqrt(np.square(start.x - goal.x) + np.square(start.y - goal.y))


def trace_path(history, start, end):
    actual_path = []
    step = end
    while step != start and step is not None:
        actual_path.append(step)
        step = history.get(step.to_tuple())
    actual_path.append(start)
    actual_path.reverse()
    return actual_path


# DFS
def depth_first_search(start, goal, epolygons):
    to_visit = Stack()
    to_visit.push(start)
    been_there = set()
    leads_from = {start.to_tuple(): None}
    count = 0

    while not to_visit.isEmpty():
        current_node = to_visit.pop()
        count += 1
        # Search for goal
        if current_node == goal:
            return trace_path(leads_from, start, goal), count
        been_there.add(current_node.to_tuple())
        # Neighbor check
        for move in get_next_move(current_node, epolygons):
            m_id = move.to_tuple()
            # Add to frontier
            if m_id not in been_there and m_id not in [node.to_tuple() for node in to_visit.list]:
                leads_from[m_id] = current_node
                to_visit.push(move)
    return [], count


# BFS
def breadth_first_search(start, goal, epolygons):
    waiting_line = Queue()
    waiting_line.push(start)
    visited_coords = set()
    parent_dict = {start.to_tuple(): None}
    nodes_checked = 0

    while not waiting_line.isEmpty():
        curr = waiting_line.pop()
        nodes_checked += 1
        # Search for goal
        if curr == goal:
            return trace_path(parent_dict, start, goal), nodes_checked
        visited_coords.add(curr.to_tuple())
        # Check neighbors
        for neighbor in get_next_move(curr, epolygons):
            coords = neighbor.to_tuple()
            # Add frontier
            if coords not in visited_coords and coords not in [n.to_tuple() for n in waiting_line.list]:
                parent_dict[coords] = curr
                waiting_line.push(neighbor)
    return [], nodes_checked


# GBFS
def greedy_best_first_search(start, goal, epolygons):
    open_nodes = PriorityQueue()
    open_nodes.push(start, heuristic(start, goal))
    explored = set()
    came_from = {start.to_tuple(): None}
    expanded = 0

    while not open_nodes.isEmpty():
        curr = open_nodes.pop()
        expanded += 1
        # Search for goal
        if curr == goal:
            return trace_path(came_from, start, goal), expanded
        explored.add(curr.to_tuple())
        # Neighbor check
        for neighbor in get_next_move(curr, epolygons):
            n_key = neighbor.to_tuple()
            # Checking expl + curr -> add to frontier
            if n_key not in explored and n_key not in [item[2].to_tuple() for item in open_nodes.heap]:
                came_from[n_key] = curr
                open_nodes.push(neighbor, heuristic(neighbor, goal))
    return [], expanded


# A*
def astar_search(start, goal, epolygons, tpolygons):
    pq = PriorityQueue()
    pq.push(start, heuristic(start, goal))
    back_map = {start.to_tuple(): None}
    g_score = {start.to_tuple(): 0}
    seen = set()
    expanded_count = 0

    while not pq.isEmpty():
        curr = pq.pop()
        expanded_count += 1
        # Search for goal
        if curr == goal:
            return trace_path(back_map, start, goal), expanded_count
        seen.add(curr.to_tuple())
        # Check neighbors
        for neighbor in get_next_move(curr, epolygons):
            # New g(n)
            tentative_g = g_score[curr.to_tuple()] + get_cost(neighbor, tpolygons)
            loc = neighbor.to_tuple()
            # Add to frontier
            if loc not in g_score or tentative_g < g_score[loc]:
                g_score[loc] = tentative_g
                # f(n) = g(n) + h(n)
                f_total = tentative_g + heuristic(neighbor, goal)
                back_map[loc] = curr
                pq.update(neighbor, f_total)
    return [], expanded_count


if __name__ == "__main__":
    # World select
    while True:
        print("World Selection:")
        print("1: Use Default World")
        print("2: Use Custom World")
        world_choice = input("Choose World 1 or 2: ")

        if world_choice == '1':
            efile = 'TestingGrid/world1_enclosures.txt'
            tfile = 'TestingGrid/world1_turfs.txt'
            break
        elif world_choice == '2':
            efile = 'TestingGrid/world2_enclosures.txt'
            tfile = 'TestingGrid/world2_turfs.txt'
            break
        else:
            print("\nInvalid selection! Please enter 1 or 2.\n")

    enclosures = gen_polygons(efile)
    turfs = gen_polygons(tfile)

    # Point choice
    print("\nPoint Selection:")
    print("1: Use Default Points (8,10 -> 43,45)")
    print("2: Enter Custom Points")
    pt_choice = input("Select points (1 or 2): ")

    if pt_choice == '2':
        while True:
            try:
                print("\nEnter coordinates between 0 and 49:")
                sx = int(input("Start X: "))
                sy = int(input("Start Y: "))
                dx = int(input("End X: "))
                dy = int(input("End Y: "))
                source = Point(sx, sy)
                dest = Point(dx, dy)

                # Enclosure check
                is_inside = False
                for poly in enclosures:
                    poly_tuples = [(p.x, p.y) for p in poly]
                    path_obj = mpath.Path(poly_tuples)
                    if path_obj.contains_point((source.x, source.y)) or path_obj.contains_point((dest.x, dest.y)):
                        is_inside = True
                        break
                if is_inside:
                    print("\nSource or Destination is inside an enclosure! Provide new coordinates.")
                    continue
                break
            except ValueError:
                print("\nInvalid input. #'s only.")
    else:
        # Professor points
        source = Point(8, 10)
        dest = Point(43, 45)

    fig, ax = draw_board()
    draw_grids(ax)
    draw_source(ax, source.x, source.y)
    draw_dest(ax, dest.x, dest.y)

    # Draw enclosure polygons
    for poly in enclosures:
        for p in poly: draw_point(ax, p.x, p.y)
        for i in range(len(poly)):
            draw_line(ax, [poly[i].x, poly[(i + 1) % len(poly)].x], [poly[i].y, poly[(i + 1) % len(poly)].y])

    # Draw turf polygons
    for poly in turfs:
        for p in poly: draw_green_point(ax, p.x, p.y)
        for i in range(len(poly)):
            draw_green_line(ax, [poly[i].x, poly[(i + 1) % len(poly)].x], [poly[i].y, poly[(i + 1) % len(poly)].y])

    # Search alg choice
    while True:
        print("\nSearch Algorithm Selection:")
        print("1: DFS | 2: BFS | 3: Greedy | 4: A*")
        choice = input("Choose your search (1-4): ")

        if choice == '1':
            name, search_func, needs_turf = "DFS", depth_first_search, False
            break
        elif choice == '2':
            name, search_func, needs_turf = "BFS", breadth_first_search, False
            break
        elif choice == '3':
            name, search_func, needs_turf = "GBFS", greedy_best_first_search, False
            break
        elif choice == '4':
            name, search_func, needs_turf = "A*", astar_search, True
            break
        else:
            print("\nInvalid selection.")

    print(f"\nRunning {name}...")

    # Added duration as well
    t0 = time.time()
    # Execute search
    if needs_turf:
        res_path, expanded_count = search_func(source, dest, enclosures, turfs)
    else:
        res_path, expanded_count = search_func(source, dest, enclosures)
    duration = time.time() - t0

    # Determine type of cost
    if res_path:
        if name in ["BFS", "DFS"]:
            cost = len(res_path) - 1
        else:
            cost = calc_path_cost(res_path, turfs)

        # To add info to grid window
        plt.suptitle(f"{name} | Cost: {cost} | Nodes: {expanded_count}", fontsize=14)

        print(f"\n--- {name} RESULTS ---")
        print(f"Path Cost: {cost}")
        print(f"Nodes Expanded: {expanded_count}")
        print(f"Search Duration: {duration:.4f} seconds")

        for i in range(len(res_path) - 1):
            draw_result_line(ax, [res_path[i].x, res_path[i + 1].x], [res_path[i].y, res_path[i + 1].y])
            plt.pause(0.01)
    else:
        # Path not found
        ax.set_title(f"{name}: FAILURE")
        print(f"FAILURE: {name} could not find a path.")

    plt.show()