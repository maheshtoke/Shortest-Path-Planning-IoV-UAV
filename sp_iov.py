#!/usr/bin/env python3
"""
Shortest-Path Planning in a Large-Scale IoV/UAV Mobility Network
================================================================
Comparative algorithmic analysis of Dijkstra's Algorithm vs. A* Algorithm
on the 9th DIMACS USA Road Network (New York Benchmark: USA-road-d.NY).

Features:
- DIMACS .gr/.co (compressed .gz or plain) loader & preprocessor
- Adjacency list representation (no external graph libraries used)
- BFS-based compact connected induced subgraphs for multi-scale evaluation
- Custom Dijkstra's algorithm with binary min-heap (heapq) & lazy deletion
- Custom A* algorithm with scaled Haversine distance heuristic (admissible & consistent)
- Explicit shortest path reconstruction
- Disconnected source-destination demonstration
- Dynamic edge costs simulation (congestion / delay rerouting bonus task)
- Automated CSV logging, formatted reports, and publication-quality plots

Author: DAA Lab Project
"""

import argparse
import gzip
import heapq
import math
import os
import random
import sys
import time
from collections import deque

# Earth radius in meters
EARTH_RADIUS_METERS = 6371000.0


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Computes the great-circle distance between two points on the Earth's surface
    using the Haversine formula (returns distance in meters).
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * (math.sin(dlam / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
    return EARTH_RADIUS_METERS * c


def haversine_fast(lat1_rad: float, lon1_rad: float, cos_lat1: float,
                   lat2_rad: float, lon2_rad: float, cos_lat2: float) -> float:
    """
    Optimized Haversine distance using pre-computed radians and cosines.
    """
    dphi = lat2_rad - lat1_rad
    dlam = lon2_rad - lon1_rad
    a = math.sin(dphi / 2.0) ** 2 + cos_lat1 * cos_lat2 * (math.sin(dlam / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
    return EARTH_RADIUS_METERS * c


class MobilityGraph:
    """
    Represents a mobility / vehicular network using an adjacency list.
    Vertices represent intersections/waypoints (0-indexed).
    Edges represent directed road segments with positive movement weights.
    """
    def __init__(self, num_vertices: int, coords: list, adj: list | None = None):
        self.num_vertices = num_vertices
        # coords: list of tuples (lat, lon, lat_rad, lon_rad, cos_lat)
        self.coords = coords
        self.adj = adj if adj is not None else [[] for _ in range(num_vertices)]
        self._scale: float | None = None

    def add_edge(self, u: int, v: int, w: float):
        self.adj[u].append((v, w))

    def get_stats(self):
        total_edges = sum(len(neighbors) for neighbors in self.adj)
        avg_deg = total_edges / self.num_vertices if self.num_vertices > 0 else 0.0
        max_deg = max((len(neighbors) for neighbors in self.adj), default=0)
        return {
            "vertices": self.num_vertices,
            "edges": total_edges,
            "avg_degree": avg_deg,
            "max_degree": max_deg
        }

    def compute_heuristic_scale(self) -> float:
        """
        Computes the maximum allowable scale factor such that for ALL edges (u, v):
            scale * haversine(u, v) <= weight(u, v)
        This guarantees admissibility (h(v) <= d*(v, D)) and consistency (monotonicity):
            h(u) <= weight(u, v) + h(v).
        """
        if self._scale is not None:
            return self._scale

        min_ratio = float('inf')
        checked_edges = 0
        for u in range(self.num_vertices):
            lat_u_rad, lon_u_rad, cos_u = self.coords[u][2], self.coords[u][3], self.coords[u][4]
            for v, w in self.adj[u]:
                lat_v_rad, lon_v_rad, cos_v = self.coords[v][2], self.coords[v][3], self.coords[v][4]
                geo_dist = haversine_fast(lat_u_rad, lon_u_rad, cos_u, lat_v_rad, lon_v_rad, cos_v)
                if geo_dist > 1e-6:
                    ratio = w / geo_dist
                    min_ratio = min(min_ratio, ratio)
                    checked_edges += 1

        if min_ratio == float('inf') or checked_edges == 0:
            self._scale = 1.0
        else:
            self._scale = min_ratio
        return self._scale


def load_dimacs_dataset(gr_path: str, co_path: str) -> MobilityGraph:
    """
    Parses DIMACS 9th Challenge format road network files (.gr and .co).
    Supports both gzip compressed (.gz) and uncompressed text files.
    Preprocesses 1-based IDs to 0-based, drops self-loops and non-positive weights,
    and converts micro-degrees to standard degrees and precomputed radians.
    """
    print(f"[*] Loading coordinates from: {co_path}")
    open_fn_co = gzip.open if co_path.endswith('.gz') else open
    coords_dict = {}
    with open_fn_co(co_path, 'rt', encoding='utf-8', errors='replace') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.split()
                # format: v <id> <lon_micro> <lat_micro>
                node_id = int(parts[1]) - 1  # 0-indexed
                lon_deg = float(parts[2]) / 1e6
                lat_deg = float(parts[3]) / 1e6
                lat_rad = math.radians(lat_deg)
                lon_rad = math.radians(lon_deg)
                cos_lat = math.cos(lat_rad)
                coords_dict[node_id] = (lat_deg, lon_deg, lat_rad, lon_rad, cos_lat)

    num_vertices = len(coords_dict)
    print(f"    Loaded coordinates for {num_vertices:,} vertices.")

    # Flatten coordinates into contiguous list
    coords_list = [coords_dict[i] for i in range(num_vertices)]

    print(f"[*] Loading road segments (arcs) from: {gr_path}")
    open_fn_gr = gzip.open if gr_path.endswith('.gz') else open
    adj = [[] for _ in range(num_vertices)]
    total_arcs = 0
    discarded = 0

    with open_fn_gr(gr_path, 'rt', encoding='utf-8', errors='replace') as f:
        for line in f:
            if line.startswith('a '):
                parts = line.split()
                # format: a <u> <v> <w>
                u = int(parts[1]) - 1
                v = int(parts[2]) - 1
                w = float(parts[3])
                # Preprocessing: discard self-loops and non-positive weights
                if u == v or w <= 0:
                    discarded += 1
                    continue
                if 0 <= u < num_vertices and 0 <= v < num_vertices:
                    adj[u].append((v, w))
                    total_arcs += 1

    print(f"    Loaded {total_arcs:,} valid directed arcs (discarded {discarded} invalid/self-loop arcs).")
    graph = MobilityGraph(num_vertices, coords_list, adj)
    return graph


def generate_synthetic_graph(grid_n: int = 100, seed: int = 42) -> MobilityGraph:
    """
    Generates a synthetic grid-based mobility network with realistic coordinates
    and positive weights. Useful for quick verification or smoke testing.
    """
    print(f"[*] Generating synthetic {grid_n}x{grid_n} mobility network (seed={seed})...")
    rng = random.Random(seed)
    num_vertices = grid_n * grid_n
    coords = []
    base_lat = 40.7128  # New York base
    base_lon = -74.0060

    # ~100m grid spacing in degrees
    lat_step = 0.001
    lon_step = 0.0013

    for r in range(grid_n):
        for c in range(grid_n):
            lat_deg = base_lat + r * lat_step
            lon_deg = base_lon + c * lon_step
            lat_rad = math.radians(lat_deg)
            lon_rad = math.radians(lon_deg)
            coords.append((lat_deg, lon_deg, lat_rad, lon_rad, math.cos(lat_rad)))

    adj = [[] for _ in range(num_vertices)]
    for r in range(grid_n):
        for c in range(grid_n):
            u = r * grid_n + c
            # Directed connections (grid + diagonals)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < grid_n and 0 <= nc < grid_n:
                    v = nr * grid_n + nc
                    geo = haversine(coords[u][0], coords[u][1], coords[v][0], coords[v][1])
                    w = geo * rng.uniform(1.05, 1.35)  # road winding factor
                    adj[u].append((v, round(w, 2)))

    graph = MobilityGraph(num_vertices, coords, adj)
    print(f"    Synthetic graph created: {num_vertices:,} vertices, {sum(len(a) for a in adj):,} edges.")
    return graph


def extract_bfs_subgraph(full_graph: MobilityGraph, target_size: int, start_node: int) -> MobilityGraph:
    """
    Extracts a connected, geographically compact induced subgraph of approximately
    target_size vertices using Breadth-First Search (BFS) starting at start_node.
    Vertices are re-indexed to 0..k-1.
    """
    if target_size >= full_graph.num_vertices:
        return full_graph

    visited = set()
    queue = deque([start_node])
    visited.add(start_node)
    ordered_nodes = [start_node]

    while queue and len(ordered_nodes) < target_size:
        curr = queue.popleft()
        for neighbor, _ in full_graph.adj[curr]:
            if neighbor not in visited:
                visited.add(neighbor)
                ordered_nodes.append(neighbor)
                queue.append(neighbor)
                if len(ordered_nodes) >= target_size:
                    break

    sub_size = len(ordered_nodes)
    node_map = {orig_id: new_id for new_id, orig_id in enumerate(ordered_nodes)}

    sub_coords = [full_graph.coords[orig_id] for orig_id in ordered_nodes]
    sub_adj = [[] for _ in range(sub_size)]

    for new_id, orig_id in enumerate(ordered_nodes):
        for v, w in full_graph.adj[orig_id]:
            if v in node_map:
                sub_adj[new_id].append((node_map[v], w))

    return MobilityGraph(sub_size, sub_coords, sub_adj)


def reconstruct_path(parent: list, destination: int, source: int) -> list:
    """
    Reconstructs the shortest path vertex sequence from source to destination
    following parent pointers backwards.
    """
    path = []
    curr = destination
    while curr != -1:
        path.append(curr)
        if curr == source:
            break
        curr = parent[curr]
    if not path or path[-1] != source:
        return []
    path.reverse()
    return path


def dijkstra(graph: MobilityGraph, source: int, destination: int):
    """
    Dijkstra's shortest-path algorithm using a min-priority queue (binary min-heap).
    Includes early exit when destination is settled and handles lazy deletion.
    Returns:
        dict: {
            'cost': float or None,
            'path': list of vertex ids,
            'explored': int (vertices popped & settled),
            'examined': int (edges evaluated),
            'time_sec': float,
            'reachable': bool
        }
    """
    n = graph.num_vertices
    dist = [float('inf')] * n
    parent = [-1] * n
    visited = [False] * n

    dist[source] = 0.0
    pq = [(0.0, source)]

    explored_count = 0
    examined_count = 0

    t_start = time.perf_counter()

    found = False
    while pq:
        d_u, u = heapq.heappop(pq)

        if visited[u]:
            continue
        visited[u] = True
        explored_count += 1

        if u == destination:
            found = True
            break

        for v, w in graph.adj[u]:
            examined_count += 1
            if visited[v]:
                continue
            cand_dist = d_u + w
            if cand_dist < dist[v]:
                dist[v] = cand_dist
                parent[v] = u
                heapq.heappush(pq, (cand_dist, v))

    t_end = time.perf_counter()
    elapsed = t_end - t_start

    if not found or dist[destination] == float('inf'):
        return {
            'cost': None,
            'path': [],
            'explored': explored_count,
            'examined': examined_count,
            'time_sec': elapsed,
            'reachable': False
        }

    path = reconstruct_path(parent, destination, source)
    return {
        'cost': dist[destination],
        'path': path,
        'explored': explored_count,
        'examined': examined_count,
        'time_sec': elapsed,
        'reachable': True
    }


def astar(graph: MobilityGraph, source: int, destination: int, scale: float):
    """
    A* shortest-path algorithm evaluated with f(v) = g(v) + h(v).
    Uses the scaled Haversine distance heuristic, which is admissible and consistent.
    Returns the same metrics dictionary as dijkstra().
    """
    n = graph.num_vertices
    g_score = [float('inf')] * n
    parent = [-1] * n
    visited = [False] * n

    dest_lat_rad = graph.coords[destination][2]
    dest_lon_rad = graph.coords[destination][3]
    dest_cos_lat = graph.coords[destination][4]

    def heuristic(node_idx: int) -> float:
        c = graph.coords[node_idx]
        geo = haversine_fast(c[2], c[3], c[4], dest_lat_rad, dest_lon_rad, dest_cos_lat)
        return scale * geo

    g_score[source] = 0.0
    h_source = heuristic(source)
    pq = [(h_source, 0.0, source)]

    explored_count = 0
    examined_count = 0

    t_start = time.perf_counter()

    found = False
    while pq:
        _f_u, g_u, u = heapq.heappop(pq)

        if visited[u]:
            continue
        visited[u] = True
        explored_count += 1

        if u == destination:
            found = True
            break

        for v, w in graph.adj[u]:
            examined_count += 1
            if visited[v]:
                continue
            cand_g = g_u + w
            if cand_g < g_score[v]:
                g_score[v] = cand_g
                parent[v] = u
                cand_f = cand_g + heuristic(v)
                heapq.heappush(pq, (cand_f, cand_g, v))

    t_end = time.perf_counter()
    elapsed = t_end - t_start

    if not found or g_score[destination] == float('inf'):
        return {
            'cost': None,
            'path': [],
            'explored': explored_count,
            'examined': examined_count,
            'time_sec': elapsed,
            'reachable': False
        }

    path = reconstruct_path(parent, destination, source)
    return {
        'cost': g_score[destination],
        'path': path,
        'explored': explored_count,
        'examined': examined_count,
        'time_sec': elapsed,
        'reachable': True
    }


def find_reachable_pairs(graph: MobilityGraph, num_pairs: int, rng: random.Random) -> list:
    """
    Samples valid (source, destination) pairs where destination is guaranteed
    reachable from source, with source != destination.
    """
    pairs = []
    n = graph.num_vertices
    attempts = 0
    max_attempts = num_pairs * 50

    while len(pairs) < num_pairs and attempts < max_attempts:
        attempts += 1
        s = rng.randint(0, n - 1)
        # Explore forward reachable component from s via BFS
        q = deque([s])
        reachable = []
        visited = {s}
        max_depth = 5000
        while q and len(reachable) < max_depth:
            curr = q.popleft()
            for v, _ in graph.adj[curr]:
                if v not in visited:
                    visited.add(v)
                    reachable.append(v)
                    q.append(v)

        if reachable:
            # Pick a destination that is non-trivial (at least somewhat separated)
            d = rng.choice(reachable)
            if s != d and (s, d) not in pairs:
                pairs.append((s, d))

    # Fallback if graph is very sparse
    if len(pairs) < num_pairs:
        for s in range(n):
            if graph.adj[s]:
                d = graph.adj[s][0][0]
                if s != d and (s, d) not in pairs:
                    pairs.append((s, d))
            if len(pairs) >= num_pairs:
                break

    return pairs


def run_bonus_dynamic_experiment(subgraph: MobilityGraph, s: int, d: int,
                                 scale: float, alpha: float, beta: float,
                                 rng: random.Random, out_dir: str):
    """
    Bonus Task: Dynamic Edge Costs Simulation
    Model: w'(u, v) = alpha * d(u, v) + beta * c(u, v)
    1. Computes baseline shortest path under static physical distances.
    2. Injects high congestion (c = 5 * d) on the middle 40% of the baseline path
       and moderate congestion (c = 1..3 * d) on ~3% of background road segments.
    3. Re-executes Dijkstra and A* to find dynamic optimal rerouted path.
    4. Analyzes path change, shared vertices, detour cost savings, and heuristic consistency.
    """
    print("\n" + "=" * 75)
    print("BONUS TASK: DYNAMIC EDGE COSTS & CONGESTION-AWARE REROUTING")
    print("=" * 75)

    base_res = dijkstra(subgraph, s, d)
    if not base_res['reachable']:
        print("[-] Base query unreachable, skipping bonus experiment.")
        return

    base_path = base_res['path']
    base_cost = base_res['cost']
    base_hops = len(base_path) - 1
    print(f"[*] Base route: Source={s} -> Dest={d} | Distance={base_cost:.1f} | Hops={base_hops}")

    # Build edge lookup for the graph
    # edge: (u, v) -> base_w
    edge_map = {}
    for u in range(subgraph.num_vertices):
        for v, w in subgraph.adj[u]:
            edge_map[(u, v)] = w

    # Select middle 40% of edges in baseline route for severe bottleneck/incident
    route_edges = [(base_path[i], base_path[i+1]) for i in range(len(base_path)-1)]
    m_start = int(len(route_edges) * 0.3)
    m_end = int(len(route_edges) * 0.7)
    jammed_route_edges = set(route_edges[m_start:m_end])
    if not jammed_route_edges and route_edges:
        jammed_route_edges.add(route_edges[len(route_edges)//2])

    print(f"[*] Simulating traffic incident on {len(jammed_route_edges)} segment(s) along original path.")

    # Construct dynamic adjacency list
    # w'(u,v) = alpha * d(u,v) + beta * c(u,v)
    dynamic_adj = [[] for _ in range(subgraph.num_vertices)]
    congested_edge_count = 0

    for u in range(subgraph.num_vertices):
        for v, d_uv in subgraph.adj[u]:
            if (u, v) in jammed_route_edges:
                # Severe congestion incident
                c_uv = 5.0 * d_uv
                congested_edge_count += 1
            elif rng.random() < 0.03:
                # 3% ambient urban traffic delays
                c_uv = rng.uniform(1.0, 3.0) * d_uv
                congested_edge_count += 1
            else:
                c_uv = 0.0

            w_prime = alpha * d_uv + beta * c_uv
            dynamic_adj[u].append((v, w_prime))

    # Evaluate cost of the OLD route under NEW congested conditions
    old_route_congested_cost = 0.0
    # Map for fast lookup of dynamic weights
    dyn_weight_map = {}
    for u in range(subgraph.num_vertices):
        for v, w_prime in dynamic_adj[u]:
            dyn_weight_map[(u, v)] = w_prime

    for u, v in route_edges:
        old_route_congested_cost += dyn_weight_map.get((u, v), 0.0)

    # Wrap in dynamic graph
    dynamic_graph = MobilityGraph(subgraph.num_vertices, subgraph.coords, dynamic_adj)

    # Scale for dynamic A*: since w' >= alpha * d, effective scale can be alpha * scale
    dynamic_scale = min(scale * alpha, dynamic_graph.compute_heuristic_scale())

    # Re-run Dijkstra and A*
    dyn_dijkstra = dijkstra(dynamic_graph, s, d)
    dyn_astar = astar(dynamic_graph, s, d, dynamic_scale)

    new_path = dyn_dijkstra['path']
    new_cost = dyn_dijkstra['cost']
    new_hops = len(new_path) - 1

    shared_vertices = set(base_path).intersection(set(new_path))
    shared_pct = (len(shared_vertices) / len(base_path)) * 100.0 if base_path else 0.0
    cost_savings = old_route_congested_cost - new_cost
    savings_pct = (cost_savings / old_route_congested_cost) * 100.0 if old_route_congested_cost > 0 else 0.0

    print(f"[*] Old Route Under Congestion Cost: {old_route_congested_cost:.2f}")
    print(f"[*] New Dynamic Shortest Route Cost: {new_cost:.2f} (Hops: {new_hops})")
    print(f"[*] Cost Savings by Dynamic Rerouting: {cost_savings:.2f} ({savings_pct:.2f}% improvement)")
    print(f"[*] Overlap with original route: {len(shared_vertices)}/{len(base_path)} nodes ({shared_pct:.1f}%)")
    print(f"[*] Dijkstra vs A* Cost Match: {abs(dyn_dijkstra['cost'] - dyn_astar['cost']) < 1e-4}")
    print(f"[*] A* Explored: {dyn_astar['explored']} nodes vs Dijkstra Explored: {dyn_dijkstra['explored']} nodes")

    # Save bonus results
    bonus_file = os.path.join(out_dir, "bonus_dynamic.txt")
    with open(bonus_file, "w", encoding="utf-8") as f:
        f.write("=" * 70 + "\n")
        f.write("BONUS TASK: DYNAMIC EDGE COSTS EXPERIMENTAL RESULTS\n")
        f.write("=" * 70 + "\n\n")
        f.write("Edge Cost Model: w'(u, v) = alpha * d(u, v) + beta * c(u, v)\n")
        f.write(f"Parameters: alpha={alpha}, beta={beta}\n")
        f.write("Incident: Heavy delay c(u, v) = 5*d on middle 40% of baseline route\n")
        f.write("Ambient congestion: ~3% random road segments delayed by 1..3*d\n\n")
        f.write(f"Query: Source={s} -> Destination={d}\n")
        f.write(f"Baseline Path Distance: {base_cost:.2f} (Hops: {base_hops})\n")
        f.write(f"Cost of Baseline Route Under Congestion: {old_route_congested_cost:.2f}\n")
        f.write(f"New Dynamic Shortest Route Cost: {new_cost:.2f} (Hops: {new_hops})\n")
        f.write(f"Dynamic Rerouting Cost Savings: {cost_savings:.2f} ({savings_pct:.2f}% saving)\n")
        f.write(f"Shared Vertices with Baseline: {len(shared_vertices)} / {len(base_path)} ({shared_pct:.1f}%)\n")
        f.write(f"Dijkstra Cost: {dyn_dijkstra['cost']:.4f} | Explored: {dyn_dijkstra['explored']} | Time: {dyn_dijkstra['time_sec']*1000:.3f} ms\n")
        f.write(f"A* Cost:       {dyn_astar['cost']:.4f} | Explored: {dyn_astar['explored']} | Time: {dyn_astar['time_sec']*1000:.3f} ms\n")
        f.write(f"Cost Equality (Dijkstra == A*): {abs(dyn_dijkstra['cost'] - dyn_astar['cost']) < 1e-4}\n\n")
        f.write("Baseline Vertex Sequence:\n" + " -> ".join(map(str, base_path[:40])) + ("..." if len(base_path) > 40 else "") + "\n\n")
        f.write("Dynamic Rerouted Vertex Sequence:\n" + " -> ".join(map(str, new_path[:40])) + ("..." if len(new_path) > 40 else "") + "\n")

    print(f"[+] Bonus dynamic report saved to: {bonus_file}")


def run_disconnected_demo(graph: MobilityGraph, scale: float):
    """
    Demonstrates graceful handling of disconnected source-destination pairs
    by adding an isolated synthetic vertex and querying reachability.
    """
    print("\n" + "=" * 75)
    print("STAGE 8: DISCONNECTED SOURCE-DESTINATION DEMONSTRATION")
    print("=" * 75)
    # Create a small graph with an isolated vertex
    n = 10
    coords = [graph.coords[i] for i in range(n)]
    adj = [[] for _ in range(n)]
    # Connect 0..8 in a line, leave 9 completely isolated
    for i in range(8):
        adj[i].append((i + 1, 100.0))
    disc_graph = MobilityGraph(n, coords, adj)

    s, d = 0, 9
    print(f"[*] Querying path between connected Source={s} and ISOLATED Destination={d}...")
    res_d = dijkstra(disc_graph, s, d)
    res_a = astar(disc_graph, s, d, scale)

    print(f"    Dijkstra result: Reachable={res_d['reachable']}, Cost={res_d['cost']}, Path={res_d['path']}, Explored={res_d['explored']}")
    print(f"    A* result:       Reachable={res_a['reachable']}, Cost={res_a['cost']}, Path={res_a['path']}, Explored={res_a['explored']}")
    assert res_d['reachable'] is False and res_d['cost'] is None and res_d['path'] == []
    assert res_a['reachable'] is False and res_a['cost'] is None and res_a['path'] == []
    print("[+] Both algorithms cleanly returned reachable=False without error or infinite loop.")


def generate_plots(summary_data: list, out_dir: str):
    """
    Generates comparative visualization plots for execution time,
    vertices explored, and edges examined across graph sizes.
    """
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("[-] Matplotlib not installed; skipping plot generation.")
        return

    sizes = sorted({row['Graph_Size'] for row in summary_data})
    dijkstra_time = [next(r['Avg_Time_ms'] for r in summary_data if r['Graph_Size'] == s and r['Algorithm'] == 'Dijkstra') for s in sizes]
    astar_time = [next(r['Avg_Time_ms'] for r in summary_data if r['Graph_Size'] == s and r['Algorithm'] == 'A*') for s in sizes]

    dijkstra_explored = [next(r['Avg_Explored'] for r in summary_data if r['Graph_Size'] == s and r['Algorithm'] == 'Dijkstra') for s in sizes]
    astar_explored = [next(r['Avg_Explored'] for r in summary_data if r['Graph_Size'] == s and r['Algorithm'] == 'A*') for s in sizes]

    dijkstra_examined = [next(r['Avg_Examined'] for r in summary_data if r['Graph_Size'] == s and r['Algorithm'] == 'Dijkstra') for s in sizes]
    astar_examined = [next(r['Avg_Examined'] for r in summary_data if r['Graph_Size'] == s and r['Algorithm'] == 'A*') for s in sizes]

    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')

    # 1. Execution Time Plot
    fig, ax = plt.subplots(figsize=(8, 5.2), dpi=300)
    ax.plot(sizes, dijkstra_time, marker='o', markersize=7, linewidth=2.3, color='#e63946', label="Dijkstra's Algorithm (Min-Heap)")
    ax.plot(sizes, astar_time, marker='s', markersize=7, linewidth=2.3, color='#1d3557', label="A* Algorithm (Scaled Haversine)")
    for s, dt, at in zip(sizes, dijkstra_time, astar_time):
        ax.annotate(f"{dt:.2f}ms", (s, dt), textcoords="offset points", xytext=(0, 7), ha='center', fontsize=8, color='#e63946', weight='bold')
        ax.annotate(f"{at:.2f}ms", (s, at), textcoords="offset points", xytext=(0, -14), ha='center', fontsize=8, color='#1d3557', weight='bold')
    ax.set_title("Shortest-Path Execution Time vs. Graph Size", fontsize=13, fontweight='bold', pad=12)
    ax.set_xlabel("Graph Size (Number of Vertices)", fontsize=11, fontweight='bold')
    ax.set_ylabel("Mean Execution Time (ms)", fontsize=11, fontweight='bold')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=10, loc='upper left')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot_time.png"))
    plt.close(fig)

    # 2. Vertices Explored Plot
    fig, ax = plt.subplots(figsize=(8, 5.2), dpi=300)
    ax.plot(sizes, dijkstra_explored, marker='o', markersize=7, linewidth=2.3, color='#e63946', label="Dijkstra's Algorithm")
    ax.plot(sizes, astar_explored, marker='^', markersize=7, linewidth=2.3, color='#2a9d8f', label="A* Algorithm")
    for s, de, ae in zip(sizes, dijkstra_explored, astar_explored):
        pct = (de - ae) / de * 100.0 if de > 0 else 0.0
        ax.annotate(f"-{pct:.1f}%", (s, ae), textcoords="offset points", xytext=(0, 7), ha='center', fontsize=8, color='#1b4332', weight='bold')
    ax.set_title("Search State Space: Vertices Explored vs. Graph Size", fontsize=13, fontweight='bold', pad=12)
    ax.set_xlabel("Graph Size (Number of Vertices)", fontsize=11, fontweight='bold')
    ax.set_ylabel("Mean Vertices Explored (Settled Nodes)", fontsize=11, fontweight='bold')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=10, loc='upper left')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot_vertices.png"))
    plt.close(fig)

    # 3. Edges Examined Plot
    fig, ax = plt.subplots(figsize=(8, 5.2), dpi=300)
    ax.plot(sizes, dijkstra_examined, marker='o', markersize=7, linewidth=2.3, color='#e63946', label="Dijkstra's Algorithm")
    ax.plot(sizes, astar_examined, marker='D', markersize=6, linewidth=2.3, color='#457b9d', label="A* Algorithm")
    ax.set_title("Edge Relaxations: Edges Examined vs. Graph Size", fontsize=13, fontweight='bold', pad=12)
    ax.set_xlabel("Graph Size (Number of Vertices)", fontsize=11, fontweight='bold')
    ax.set_ylabel("Mean Edges Examined during Relaxations", fontsize=11, fontweight='bold')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(fontsize=10, loc='upper left')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot_edges.png"))
    plt.close(fig)

    # 4. Combined 3-Panel Dashboard
    fig, axes = plt.subplots(1, 3, figsize=(16, 5), dpi=300)
    axes[0].plot(sizes, dijkstra_time, marker='o', color='#e63946', lw=2.2, label='Dijkstra')
    axes[0].plot(sizes, astar_time, marker='s', color='#1d3557', lw=2.2, label='A*')
    axes[0].set_title('(a) Execution Time (ms)', fontweight='bold')
    axes[0].set_xlabel('Graph Size (Vertices)')
    axes[0].set_ylabel('Mean Time (ms)')
    axes[0].grid(True, linestyle='--', alpha=0.5)
    axes[0].legend()

    axes[1].plot(sizes, dijkstra_explored, marker='o', color='#e63946', lw=2.2, label='Dijkstra')
    axes[1].plot(sizes, astar_explored, marker='^', color='#2a9d8f', lw=2.2, label='A*')
    axes[1].set_title('(b) Vertices Explored', fontweight='bold')
    axes[1].set_xlabel('Graph Size (Vertices)')
    axes[1].set_ylabel('Mean Explored Vertices')
    axes[1].grid(True, linestyle='--', alpha=0.5)
    axes[1].legend()

    axes[2].plot(sizes, dijkstra_examined, marker='o', color='#e63946', lw=2.2, label='Dijkstra')
    axes[2].plot(sizes, astar_examined, marker='D', color='#457b9d', lw=2.2, label='A*')
    axes[2].set_title('(c) Edges Examined', fontweight='bold')
    axes[2].set_xlabel('Graph Size (Vertices)')
    axes[2].set_ylabel('Mean Examined Edges')
    axes[2].grid(True, linestyle='--', alpha=0.5)
    axes[2].legend()

    fig.suptitle('Comparative Algorithmic Performance: Dijkstra vs. A* on IoV Mobility Networks', fontsize=14, fontweight='bold', y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "plot_dashboard.png"), bbox_inches='tight')
    plt.close(fig)

    print(f"[+] Performance evaluation plots and dashboard successfully saved to: {out_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Comparative Analysis of Dijkstra and A* on Large-Scale IoV/Mobility Networks"
    )
    parser.add_argument("--gr", type=str, default="", help="Path to DIMACS .gr / .gr.gz road network file")
    parser.add_argument("--co", type=str, default="", help="Path to DIMACS .co / .co.gz coordinates file")
    parser.add_argument("--synthetic", type=int, default=0, help="Grid dimension N for synthetic test graph (e.g. 200)")
    parser.add_argument("--sizes", type=str, default="1000,5000,10000,25000,50000,100000", help="Comma-separated graph sizes")
    parser.add_argument("--pairs", type=int, default=10, help="Number of S-D query pairs per graph size (>=5 required)")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for reproducibility")
    parser.add_argument("--repeats", type=int, default=3, help="Execution timing repeats per query (best kept)")
    parser.add_argument("--alpha", type=float, default=1.0, help="Weight for physical distance in dynamic edge cost")
    parser.add_argument("--beta", type=float, default=2.0, help="Weight for delay/congestion in dynamic edge cost")
    parser.add_argument("--out", type=str, default="results", help="Directory for output CSVs, plots, and logs")

    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    rng = random.Random(args.seed)

    print("=" * 75)
    print("DAA LAB PROJECT: SHORTEST-PATH PLANNING IN LARGE-SCALE IoV NETWORKS")
    print("=" * 75)
    print(f"[*] Configuration: seed={args.seed}, pairs={args.pairs}, repeats={args.repeats}, out={args.out}")

    # STAGE 1: LOAD DATASET
    if args.synthetic > 0:
        full_graph = generate_synthetic_graph(args.synthetic, args.seed)
    else:
        if not args.gr or not args.co:
            print("[-] Error: Please specify --gr and --co paths or use --synthetic <N>.")
            sys.exit(1)
        if not os.path.exists(args.gr) or not os.path.exists(args.co):
            print(f"[-] Error: Dataset file not found: {args.gr} or {args.co}")
            sys.exit(1)
        full_graph = load_dimacs_dataset(args.gr, args.co)

    # STAGE 2: REPORT FULL GRAPH STATISTICS
    full_stats = full_graph.get_stats()
    print("\n" + "-" * 75)
    print("FULL GRAPH TOPOLOGICAL STATISTICS (Part B)")
    print("-" * 75)
    print(f"  Number of Vertices (|V|) : {full_stats['vertices']:,}")
    print(f"  Number of Edges    (|E|) : {full_stats['edges']:,}")
    print(f"  Average Degree     (E/V) : {full_stats['avg_degree']:.4f}")
    print(f"  Maximum Degree           : {full_stats['max_degree']}")
    print("-" * 75)

    # Compute global heuristic scaling factor
    print("[*] Computing admissible & consistent Haversine scale factor...")
    global_scale = full_graph.compute_heuristic_scale()
    print(f"    Heuristic scale factor: {global_scale:.6f}")

    # STAGE 3: MULTI-SCALE EXPERIMENTAL EVALUATION (Part E)
    raw_sizes = [int(s.strip()) for s in args.sizes.split(",") if s.strip()]
    raw_sizes.sort()

    graph_stats_rows = []
    per_query_rows = []
    summary_rows = []
    sample_query_saved = False

    # Choose a high-degree, centrally-located start node for BFS to ensure large components
    candidate_start = full_graph.adj.index(max(full_graph.adj, key=len))

    for target_v in raw_sizes:
        print(f"\n[*] Constructing induced subgraph for target size: {target_v:,} vertices...")
        subgraph = extract_bfs_subgraph(full_graph, target_v, candidate_start)
        stats = subgraph.get_stats()
        sub_scale = subgraph.compute_heuristic_scale()
        print(f"    Active Subgraph: |V|={stats['vertices']:,}, |E|={stats['edges']:,}, Avg Deg={stats['avg_degree']:.3f}, Max Deg={stats['max_degree']}")

        graph_stats_rows.append({
            "Graph_Size": target_v,
            "Vertices": stats['vertices'],
            "Edges": stats['edges'],
            "Avg_Degree": round(stats['avg_degree'], 4),
            "Max_Degree": stats['max_degree']
        })

        # STAGE 4: SELECT REACHABLE S-D PAIRS
        pairs = find_reachable_pairs(subgraph, args.pairs, rng)
        print(f"    Generated {len(pairs)} reachable Source-Destination query pairs.")

        size_dijkstra_cost = []
        size_astar_cost = []
        size_dijkstra_explored = []
        size_astar_explored = []
        size_dijkstra_examined = []
        size_astar_examined = []
        size_dijkstra_time = []
        size_astar_time = []

        for q_idx, (s, d) in enumerate(pairs, 1):
            # Evaluate Dijkstra with timing repeats
            dijkstra_best_time = float('inf')
            res_dijkstra = None
            for _ in range(args.repeats):
                r = dijkstra(subgraph, s, d)
                if r['time_sec'] < dijkstra_best_time:
                    dijkstra_best_time = r['time_sec']
                    res_dijkstra = r

            # Evaluate A* with timing repeats
            astar_best_time = float('inf')
            res_astar = None
            for _ in range(args.repeats):
                r = astar(subgraph, s, d, sub_scale)
                if r['time_sec'] < astar_best_time:
                    astar_best_time = r['time_sec']
                    res_astar = r

            if res_dijkstra is None or res_astar is None:
                continue

            cost_d = res_dijkstra['cost']
            cost_a = res_astar['cost']
            same_cost = (abs(cost_d - cost_a) < 1e-4) if (cost_d is not None and cost_a is not None) else False

            t_d_ms = dijkstra_best_time * 1000.0
            t_a_ms = astar_best_time * 1000.0
            speedup = (t_d_ms / t_a_ms) if t_a_ms > 0 else 1.0
            explored_reduct = ((res_dijkstra['explored'] - res_astar['explored']) / res_dijkstra['explored'] * 100.0) if res_dijkstra['explored'] > 0 else 0.0

            per_query_rows.append({
                "Graph_Size": target_v,
                "Query_Index": q_idx,
                "Source": s,
                "Destination": d,
                "Dijkstra_Cost": round(cost_d, 2) if cost_d is not None else -1,
                "AStar_Cost": round(cost_a, 2) if cost_a is not None else -1,
                "Same_Cost": same_cost,
                "Dijkstra_Explored": res_dijkstra['explored'],
                "AStar_Explored": res_astar['explored'],
                "Dijkstra_Examined": res_dijkstra['examined'],
                "AStar_Examined": res_astar['examined'],
                "Dijkstra_Time_ms": round(t_d_ms, 3),
                "AStar_Time_ms": round(t_a_ms, 3),
                "Speedup_Factor": round(speedup, 2),
                "Explored_Reduction_Pct": round(explored_reduct, 2)
            })

            size_dijkstra_cost.append(cost_d)
            size_astar_cost.append(cost_a)
            size_dijkstra_explored.append(res_dijkstra['explored'])
            size_astar_explored.append(res_astar['explored'])
            size_dijkstra_examined.append(res_dijkstra['examined'])
            size_astar_examined.append(res_astar['examined'])
            size_dijkstra_time.append(t_d_ms)
            size_astar_time.append(t_a_ms)

            # Save detailed sample path from a representative intermediate size (e.g. 10000 or 5000)
            if not sample_query_saved and target_v >= 5000 and len(res_dijkstra['path']) > 15:
                sample_file = os.path.join(args.out, "sample_query_path.txt")
                with open(sample_file, "w", encoding="utf-8") as sf:
                    sf.write("=" * 70 + "\n")
                    sf.write("REPRESENTATIVE SHORTEST-PATH QUERY & VERTEX SEQUENCE\n")
                    sf.write("=" * 70 + "\n\n")
                    sf.write(f"Graph Size: {target_v:,} vertices\n")
                    sf.write(f"Source Node (S): {s}\n")
                    sf.write(f"Destination Node (D): {d}\n")
                    sf.write(f"Shortest Path Cost: {cost_d:.2f}\n")
                    sf.write(f"Hop Count (Number of road segments): {len(res_dijkstra['path']) - 1}\n\n")
                    sf.write("ALGORITHM PERFORMANCE COMPARISON:\n")
                    sf.write(f"  - Dijkstra: Explored={res_dijkstra['explored']}, Examined={res_dijkstra['examined']}, Time={t_d_ms:.3f} ms\n")
                    sf.write(f"  - A*:       Explored={res_astar['explored']}, Examined={res_astar['examined']}, Time={t_a_ms:.3f} ms\n")
                    sf.write(f"  - Exploration Reduction: {explored_reduct:.2f}%\n")
                    sf.write(f"  - Execution Speedup:     {speedup:.2f}x\n\n")
                    sf.write("COMPLETE SEQUENCE OF VERTICES IN SHORTEST PATH:\n")
                    sf.write(" -> ".join(map(str, res_dijkstra['path'])) + "\n\n")
                    sf.write("DETAILED WAYPOINT COORDINATES & CUMULATIVE ROUTE PROGRESSION:\n")
                    sf.write(f"{'Step':<6}{'Vertex':<10}{'Latitude':<12}{'Longitude':<12}\n")
                    sf.write("-" * 45 + "\n")
                    for step_i, node_id in enumerate(res_dijkstra['path']):
                        c = subgraph.coords[node_id]
                        sf.write(f"{step_i:<6}{node_id:<10}{c[0]:<12.6f}{c[1]:<12.6f}\n")
                sample_query_saved = True
                print(f"    [+] Saved representative sample path to {sample_file}")

        # Summary averages for this graph size
        avg_d_cost = sum(size_dijkstra_cost) / len(size_dijkstra_cost)
        avg_a_cost = sum(size_astar_cost) / len(size_astar_cost)
        avg_d_exp = sum(size_dijkstra_explored) / len(size_dijkstra_explored)
        avg_a_exp = sum(size_astar_explored) / len(size_astar_explored)
        avg_d_exm = sum(size_dijkstra_examined) / len(size_dijkstra_examined)
        avg_a_exm = sum(size_astar_examined) / len(size_astar_examined)
        avg_d_time = sum(size_dijkstra_time) / len(size_dijkstra_time)
        avg_a_time = sum(size_astar_time) / len(size_astar_time)

        summary_rows.append({
            "Graph_Size": target_v,
            "Algorithm": "Dijkstra",
            "Avg_Cost": round(avg_d_cost, 2),
            "Avg_Explored": round(avg_d_exp, 1),
            "Avg_Examined": round(avg_d_exm, 1),
            "Avg_Time_ms": round(avg_d_time, 3)
        })
        summary_rows.append({
            "Graph_Size": target_v,
            "Algorithm": "A*",
            "Avg_Cost": round(avg_a_cost, 2),
            "Avg_Explored": round(avg_a_exp, 1),
            "Avg_Examined": round(avg_a_exm, 1),
            "Avg_Time_ms": round(avg_a_time, 3)
        })

        print(f"    Averages ({args.pairs} queries):")
        print(f"      Dijkstra -> Explored: {avg_d_exp:,.1f} | Examined: {avg_d_exm:,.1f} | Time: {avg_d_time:.3f} ms")
        print(f"      A*       -> Explored: {avg_a_exp:,.1f} | Examined: {avg_a_exm:,.1f} | Time: {avg_a_time:.3f} ms")
        print(f"      Ratio A*/Dijkstra Explored: {(avg_a_exp / avg_d_exp * 100.0):.1f}% | Cost Match: {all(r['Same_Cost'] for r in per_query_rows if r['Graph_Size'] == target_v)}")

    # STAGE 7: WRITE OUTPUT CSV FILES
    stats_csv = os.path.join(args.out, "graph_stats.csv")
    with open(stats_csv, "w", encoding="utf-8") as f:
        f.write("Graph_Size,Vertices,Edges,Avg_Degree,Max_Degree\n")
        for r in graph_stats_rows:
            f.write(f"{r['Graph_Size']},{r['Vertices']},{r['Edges']},{r['Avg_Degree']},{r['Max_Degree']}\n")

    per_query_csv = os.path.join(args.out, "per_query_results.csv")
    with open(per_query_csv, "w", encoding="utf-8") as f:
        f.write("Graph_Size,Query_Index,Source,Destination,Dijkstra_Cost,AStar_Cost,Same_Cost,Dijkstra_Explored,AStar_Explored,Dijkstra_Examined,AStar_Examined,Dijkstra_Time_ms,AStar_Time_ms,Speedup_Factor,Explored_Reduction_Pct\n")
        for r in per_query_rows:
            f.write(f"{r['Graph_Size']},{r['Query_Index']},{r['Source']},{r['Destination']},{r['Dijkstra_Cost']},{r['AStar_Cost']},{r['Same_Cost']},{r['Dijkstra_Explored']},{r['AStar_Explored']},{r['Dijkstra_Examined']},{r['AStar_Examined']},{r['Dijkstra_Time_ms']},{r['AStar_Time_ms']},{r['Speedup_Factor']},{r['Explored_Reduction_Pct']}\n")

    summary_csv = os.path.join(args.out, "summary_results.csv")
    with open(summary_csv, "w", encoding="utf-8") as f:
        f.write("Graph_Size,Algorithm,Avg_Cost,Avg_Explored,Avg_Examined,Avg_Time_ms\n")
        for r in summary_rows:
            f.write(f"{r['Graph_Size']},{r['Algorithm']},{r['Avg_Cost']},{r['Avg_Explored']},{r['Avg_Examined']},{r['Avg_Time_ms']}\n")

    print(f"\n[+] Saved experimental CSV files: {stats_csv}, {per_query_csv}, {summary_csv}")

    # Print Formatted Summary Table to Console
    print("\n" + "=" * 80)
    print("EXPERIMENTAL EVALUATION SUMMARY TABLE (Part E)")
    print("=" * 80)
    print(f"{'Graph Size':<12}{'Algorithm':<12}{'Avg Cost':<14}{'Vertices Explored':<20}{'Edges Examined':<18}{'Time (ms)':<12}")
    print("-" * 80)
    for r in summary_rows:
        print(f"{r['Graph_Size']:<12}{r['Algorithm']:<12}{r['Avg_Cost']:<14.2f}{r['Avg_Explored']:<20.1f}{r['Avg_Examined']:<18.1f}{r['Avg_Time_ms']:<12.3f}")
    print("=" * 80)

    # STAGE 8: DISCONNECTED PAIR DEMONSTRATION
    run_disconnected_demo(full_graph, global_scale)

    # STAGE 9: BONUS TASK DYNAMIC EDGE COSTS
    # Run dynamic experiment on representative graph (e.g. 25000 or 10000)
    bonus_size = min(25000, max(raw_sizes))
    bonus_subgraph = extract_bfs_subgraph(full_graph, bonus_size, candidate_start)
    bonus_pairs = find_reachable_pairs(bonus_subgraph, 1, rng)
    if bonus_pairs:
        bs, bd = bonus_pairs[0]
        run_bonus_dynamic_experiment(
            bonus_subgraph, bs, bd,
            bonus_subgraph.compute_heuristic_scale(),
            args.alpha, args.beta, rng, args.out
        )

    # GENERATE PUBLICATION-QUALITY PLOTS
    generate_plots(summary_rows, args.out)

    print("\n" + "=" * 75)
    print("[SUCCESS] All experimental stages completed successfully!")
    print(f"Results, logs, and plots are available in folder: {args.out}/")
    print("=" * 75)


if __name__ == "__main__":
    main()
