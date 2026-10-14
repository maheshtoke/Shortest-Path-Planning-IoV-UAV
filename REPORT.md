# Technical Report: Shortest-Path Planning in a Large-Scale IoV/UAV Mobility Network

**Course:** Design and Analysis of Algorithms (DAA)  
**Laboratory Project:** Algorithmic Performance Evaluation of Shortest-Path Routing on Real-World Mobility Networks  
**Benchmark Network:** 9th DIMACS Implementation Challenge (USA Road Network: New York Benchmark - `USA-road-d.NY`)  
**Algorithms Evaluated:** Dijkstra's Algorithm (Min-Priority Heap) vs. A* Search Algorithm (Scaled Haversine Heuristic)  
**Date:** October 2026  

---

## Executive Summary
This project investigates the theoretical foundations, algorithmic mechanics, and practical computational performance of shortest-path planning algorithms applied to large-scale Internet of Vehicles (IoV) and Unmanned Aerial Vehicle (UAV) mobility networks. Representing realistic transportation topologies as directed weighted graphs $G = (V, E)$, we implement and benchmark **Dijkstra's Algorithm** and the **A* Search Algorithm** from first principles across multiple induced sub-network sizes ranging from $1,000$ to $100,000+$ vertices.

Our empirical findings demonstrate that:
1. **Algorithmic Optimality:** A* and Dijkstra produce mathematically identical shortest-path costs across 100% of evaluated queries, validating the empirical correctness and admissibility of the developed spatial heuristic.
2. **Search Space Pruning:** A* explores **$75.6\%$ to $88.5\%$ fewer vertices** and evaluates **$74.5\%$ to $88.1\%$ fewer edges** than Dijkstra, restricting search exploration to a focused directional corridor between source and destination.
3. **Execution Latency:** A* achieves an average speedup of **$2.1\times$ to $5.1\times$** over Dijkstra, maintaining sub-3 millisecond route generation even on $100,000$-vertex networks.
4. **Dynamic Resiliency (Bonus Task):** Under simulated traffic congestion incidents ($w'(u,v) = \alpha \cdot d + \beta \cdot c$), real-time dynamic rerouting achieves a **$79.85\%$ cost reduction** compared to traversing the congested baseline route.

---

## Part A: Dataset Selection & Justification

### 1. Dataset Overview & Source
To evaluate realistic vehicular mobility without relying on synthetic or artificial toy topologies, we selected the official benchmark dataset from the **9th DIMACS Implementation Challenge on Shortest Paths**:
- **Dataset Name:** USA Road Networks: New York City Region (`USA-road-d.NY`)
- **Official Repository URL:** [http://www.diag.uniroma1.it/challenge9/download.shtml](http://www.diag.uniroma1.it/challenge9/download.shtml)
- **Primary Reference:**  
  > C. Demetrescu, A. V. Goldberg, and D. S. Johnson (Eds.), *The Shortest Path Problem: Ninth DIMACS Implementation Challenge*, DIMACS Series in Discrete Mathematics and Theoretical Computer Science, Vol. 74, American Mathematical Society (AMS), 2009.

### 2. Dataset Characteristics & Schema
The dataset originates from the US Census Bureau TIGER/Line geographic database, mapping the actual road infrastructure of New York State and metropolitan areas:

| Attribute | Specification in Benchmark |
| :--- | :--- |
| **Total Intersections / Vertices ($|V|$)** | $264,346$ |
| **Total Road Segments / Directed Arcs ($|E|$)** | $733,846$ |
| **Spatial Coordinates Provided** | Geodetic Longitude and Latitude (micro-degrees, $10^{-6}$ deg) |
| **Edge Weight Metric ($w$)** | Physical road travel distance (measured in decimeters) |
| **Graph Type** | Directed, non-planar, spatially embedded road network |

### 3. Graph Modeling Formulations
- **Vertex Representation ($V$):** Each vertex $v \in V$ represents a physical roadway intersection, interchange, highway ramp, or dead-end terminus (a waypoint / mobility node where vehicular trajectory decisions occur).
- **Edge Representation ($E$):** Each directed edge $(u, v) \in E$ represents a traversable roadway segment allowing lawful vehicular or UAV travel from intersection $u$ to intersection $v$. Two-way streets are modeled as pairs of opposing directed edges $(u, v)$ and $(v, u)$, while one-way streets appear as single directed arcs.
- **Edge Weights ($w(u,v)$):** Each weight represents the physical length of the roadway segment. All edge weights are strictly positive ($w(u,v) > 0$).

### 4. Justification for IoV/UAV Mobility Route Planning
1. **True Physical Topology:** Vehicular traffic conforms to physical roadway boundaries. Unlike abstract random graphs (e.g., Erdős–Rényi or Barabási–Albert), road networks exhibit high clustering, low average degree, and strict spatial embedding.
2. **Accurate Geospatial Coordinates:** Latitude and longitude are provided for every node, enabling calculation of real-world spherical distances for A* heuristics.
3. **Sufficient Scale for Computational Analysis:** With over a quarter-million intersections, the dataset is large enough to expose asymptotic scaling behaviors, cache effects, and priority queue overheads across orders of magnitude ($10^3$ to $10^5$ nodes).
4. **Reproducibility & Scientific Standard:** DIMACS road benchmarks serve as the universal gold standard in algorithmic literature for shortest-path research.

---

## Part B: Graph Construction & Preprocessing

### 1. Data Cleaning & Extraction Methodology
The raw data files (`USA-road-d.NY.gr.gz` and `USA-road-d.NY.co.gz`) were ingested using a streaming decompressor without disk decompression:
1. **Index Standardization:** Vertex identifiers were translated from 1-based indexing ($1 \dots |V|$) to 0-based indexing ($0 \dots |V|-1$).
2. **Degeneracy Elimination:** Self-loops (arcs where $u = v$) and non-positive weights ($w \le 0$) were filtered out to maintain strict metric validity.
3. **Coordinate Transformation:** Longitudinal and latitudinal coordinates in micro-degrees were scaled by $10^{-6}$ to standard decimal degrees. Radians and trigonometric constants ($\cos(\phi)$) were precomputed to accelerate Haversine distance calculations.
4. **Adjacency List Construction:** An array of outgoing adjacency lists `adj[u] = [(v_1, w_1), (v_2, w_2), ...]` was constructed, storing only the necessary neighbor and weight tuples to maximize cache locality and minimize memory footprint ($\sim 45\text{ MB}$ total).

### 2. Topological Characteristics
The topological metrics for the complete graph and the induced subgraphs are summarized below:

| Network Scale | Vertices ($|V|$) | Directed Edges ($|E|$) | Average Degree ($|E|/|V|$) | Maximum Degree ($\Delta$) | Sparsity Factor ($|E| / |V|^2$) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Full NY Network** | **264,346** | **733,846** | **2.7761** | **8** | $1.05 \times 10^{-5}$ |
| Induced Subgraph 1k | 1,000 | 3,468 | 3.4680 | 8 | $3.47 \times 10^{-3}$ |
| Induced Subgraph 5k | 5,000 | 16,800 | 3.3600 | 8 | $6.72 \times 10^{-4}$ |
| Induced Subgraph 10k | 10,000 | 33,992 | 3.3992 | 8 | $3.40 \times 10^{-4}$ |
| Induced Subgraph 25k | 25,000 | 84,532 | 3.3813 | 8 | $1.35 \times 10^{-4}$ |
| Induced Subgraph 50k | 50,000 | 161,546 | 3.2309 | 8 | $6.46 \times 10^{-5}$ |
| Induced Subgraph 100k | 100,000 | 304,170 | 3.0417 | 8 | $3.04 \times 10^{-5}$ |

**Key Observation:** Across all scales, the average degree remains tightly bounded between $2.77$ and $3.47$, and the maximum degree never exceeds $8$. This confirms that real-world road networks are **ultra-sparse spatial graphs** whose edge count grows strictly as $O(V)$ rather than $O(V^2)$.

---

## Part C: Dijkstra's Algorithm Implementation & Complexity

### 1. Algorithm Architecture
Dijkstra's algorithm is implemented from scratch using a min-priority queue based on a binary min-heap (`heapq`).
- **Data Structures:**
  - `dist`: Array of size $|V|$, initialized to $+\infty$, with `dist[S] = 0.0`.
  - `parent`: Array of size $|V|$, initialized to $-1$, storing the predecessor tree.
  - `visited`: Boolean array of size $|V|$ tracking settled vertices.
  - `pq`: Min-priority queue storing entries $(d_u, u)$.
- **Key Algorithmic Refinements:**
  - **Lazy Deletion:** Rather than incurring expensive decrease-key operations, updated shorter distances are pushed onto the heap. When an entry $(d_u, u)$ is popped, if `visited[u]` is already true, it is discarded immediately.
  - **Goal-Directed Early Termination:** As soon as destination vertex $D$ is popped from the priority queue, search halts. In non-negative graphs, the popped distance is guaranteed to be the final shortest-path cost.
  - **Explicit Path Reconstruction:** Shortest paths are reconstructed by backtracking predecessor pointers from $D$ to $S$ in $O(L)$ time, where $L$ is the path hop count.

### 2. Theoretical Complexity Analysis
- **Time Complexity:**
  - Each vertex is settled at most once: $|V|$ heap pop operations.
  - Each directed edge $(u, v)$ is examined at most once: at most $|E|$ heap push operations.
  - The maximum size of the binary heap with lazy deletion is bounded by $|E|$.
  - Each push and pop operation takes $O(\log |E|) = O(\log |V|^2) = O(2 \log |V|) = O(\log |V|)$ time.
  - Total worst-case time complexity:
    $$\mathcal{O}((|V| + |E|) \log |V|)$$
  - Since road networks are sparse ($|E| \approx 3 |V|$), the practical running time is:
    $$\mathcal{O}(|V| \log |V|)$$
- **Space Complexity:**
  - Adjacency list: $O(|V| + |E|)$ space.
  - Distance array, parent array, and visited flags: $3 \times O(|V|)$ space.
  - Priority queue: at most $O(|E|)$ entries.
  - Total space complexity: $\mathcal{O}(|V| + |E|)$.

---

## Part D: A* Algorithm Implementation & Heuristic Evaluation

### 1. Algorithm Architecture
The A* search algorithm evaluates priority using the evaluation function:
$$f(v) = g(v) + h(v)$$
where:
- $g(v)$ is the exact shortest distance from source $S$ to vertex $v$ discovered so far.
- $h(v)$ is the heuristic estimate of the remaining cost from $v$ to destination $D$.

### 2. Heuristic Design: Scaled Haversine Distance
Because the Earth is an oblate spheroid, planar Euclidean distances introduce spatial distortion over large regional scales. We employ the **Haversine formula** to compute great-circle distance $d_{\text{geo}}(v, D)$:
$$a = \sin^2\left(\frac{\Delta \phi}{2}\right) + \cos(\phi_1)\cos(\phi_2)\sin^2\left(\frac{\Delta \lambda}{2}\right)$$
$$c = 2 \cdot \text{atan2}(\sqrt{a}, \sqrt{1-a}), \quad d_{\text{geo}} = R \cdot c \quad (R = 6,371,000\text{ m})$$

#### Derivation of the Heuristic Scaling Factor ($\text{scale}$)
In the DIMACS road dataset, edge weights are integers representing physical travel distances in decimeters. If raw Haversine distances in meters were used directly, the heuristic units would mismatch edge costs. Furthermore, road segments can curve or follow elevation changes.

To guarantee mathematical correctness, we compute the global scaling constant:
$$\text{scale} = \min_{(u, v) \in E, \, d_{\text{geo}}(u, v) > 0} \frac{w(u, v)}{d_{\text{geo}}(u, v)}$$
Empirically computed on the New York dataset:
$$\text{scale} = 7.152478$$
The heuristic function is defined as:
$$h(v) = \text{scale} \cdot d_{\text{geo}}(v, D)$$

### 3. Mathematical Proof of Admissibility & Consistency

#### Proof of Consistency (Monotonicity)
A heuristic is consistent if for every directed edge $(u, v) \in E$:
$$h(u) \le w(u, v) + h(v)$$
By definition of great-circle distance on a spherical metric space, the triangle inequality holds:
$$d_{\text{geo}}(u, D) \le d_{\text{geo}}(u, v) + d_{\text{geo}}(v, D)$$
Subtracting $d_{\text{geo}}(v, D)$ from both sides:
$$d_{\text{geo}}(u, D) - d_{\text{geo}}(v, D) \le d_{\text{geo}}(u, v)$$
Multiplying by $\text{scale} > 0$:
$$\text{scale} \cdot d_{\text{geo}}(u, D) - \text{scale} \cdot d_{\text{geo}}(v, D) \le \text{scale} \cdot d_{\text{geo}}(u, v)$$
$$h(u) - h(v) \le \text{scale} \cdot d_{\text{geo}}(u, v)$$
By the construction of $\text{scale}$, $\text{scale} \le \frac{w(u, v)}{d_{\text{geo}}(u, v)}$ for all edges $(u, v) \in E$. Therefore:
$$\text{scale} \cdot d_{\text{geo}}(u, v) \le w(u, v)$$
Combining the inequalities:
$$h(u) - h(v) \le w(u, v) \iff h(u) \le w(u, v) + h(v) \quad \blacksquare$$

#### Proof of Admissibility
A heuristic is admissible if $h(v) \le d^*(v, D)$ for all $v \in V$, where $d^*(v, D)$ is the true shortest-path distance.
Since $h$ is consistent and $h(D) = \text{scale} \cdot d_{\text{geo}}(D, D) = 0$, by induction along any path $v = p_0, p_1, \dots, p_k = D$:
$$h(v) \le \sum_{i=0}^{k-1} w(p_i, p_{i+1}) + h(D) = \sum_{i=0}^{k-1} w(p_i, p_{i+1})$$
Taking the infimum over all feasible paths yields:
$$h(v) \le d^*(v, D) \quad \blacksquare$$

**Significance:** Because $h(v)$ is proven consistent, A* never requires node re-openings. Once a vertex is popped and settled from the min-heap, its $g$-value is guaranteed to be optimal.

---

## Part E: Experimental Evaluation & Results

Experiments were performed across six induced sub-network sizes ($1,000$ to $100,000$ vertices). For each size, 10 distinct reachable source-destination queries were evaluated with 3 timing repeats per query (fastest timing recorded to eliminate OS scheduling jitter).

### Summary Results Table

| Graph Size ($|V|$) | Algorithm | Shortest-Path Cost | Vertices Explored | Edges Examined | Execution Time (ms) | Node Reduction | Speedup Factor |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1,000** | Dijkstra | 13,592.90 | 389.5 | 1,398.2 | 0.211 ms | — | — |
| **1,000** | **A\*** | **13,592.90** | **44.8** | **166.0** | **0.079 ms** | **-88.50%** | **2.67x** |
| **5,000** | Dijkstra | 54,194.20 | 2,918.8 | 9,895.2 | 4.492 ms | — | — |
| **5,000** | **A\*** | **54,194.20** | **633.6** | **2,203.3** | **1.795 ms** | **-78.29%** | **2.50x** |
| **10,000** | Dijkstra | 46,282.10 | 3,010.8 | 10,313.2 | 4.376 ms | — | — |
| **10,000** | **A\*** | **46,282.10** | **441.5** | **1,519.3** | **0.854 ms** | **-85.34%** | **5.12x** |
| **25,000** | Dijkstra | 38,256.70 | 2,019.2 | 6,878.0 | 2.415 ms | — | — |
| **25,000** | **A\*** | **38,256.70** | **490.4** | **1,676.0** | **1.136 ms** | **-75.71%** | **2.13x** |
| **50,000** | Dijkstra | 46,910.20 | 2,203.2 | 7,540.2 | 2.978 ms | — | — |
| **50,000** | **A\*** | **46,910.20** | **482.4** | **1,635.3** | **1.097 ms** | **-78.10%** | **2.71x** |
| **100,000** | Dijkstra | 65,590.00 | 3,398.6 | 10,521.7 | 4.542 ms | — | — |
| **100,000** | **A\*** | **65,590.00** | **828.6** | **2,522.8** | **1.771 ms** | **-75.62%** | **2.56x** |

### Disconnected Node Demonstration
To verify robust error handling, an isolated synthetic node was queried:
- **Query:** Connected Source ($S=0$) $\to$ Isolated Target ($D=9$).
- **Dijkstra Result:** `Reachable = False`, `Cost = None`, `Path = []`, `Explored = 9`.
- **A* Result:** `Reachable = False`, `Cost = None`, `Path = []`, `Explored = 9`.
Both algorithms terminated gracefully upon priority queue exhaustion without crashes or infinite loops.

---

## Representative Sample Query Breakdown

From the $5,000$-vertex benchmark, a detailed representative multi-hop path was extracted:
- **Query:** Source $S = 1791 \to$ Destination $D = 767$
- **Optimal Shortest-Path Cost:** $39,045.00$
- **Total Road Hops:** $45$ road segments
- **Performance Contrast:**
  - Dijkstra: Settled **$2,071$ vertices**, examined **$7,199$ edges** in **$2.008\text{ ms}$**.
  - A*: Settled **$268$ vertices**, examined **$948$ edges** in **$0.555\text{ ms}$**.
  - **Exploration Reduction:** **$87.06\%$** | **Execution Speedup:** **$3.62\times$**.

### Trajectory Waypoint Progression
The route smoothly traverses Brooklyn urban corridors from $(40.705389^\circ\text{N}, -73.926001^\circ\text{W})$ to $(40.712412^\circ\text{N}, -73.967449^\circ\text{W})$:
```
1791 -> 1655 -> 1535 -> 1438 -> 1342 -> 1260 -> 1187 -> 1121 -> 1060 -> 1004 -> 
961  -> 910  -> 854  -> 804  -> 852  -> 862  -> 811  -> 810  -> 860  -> 809  -> 
757  -> 753  -> 752  -> 751  -> 697  -> 641  -> 696  -> 640  -> 578  -> 518  -> 
516  -> 457  -> 414  -> 413  -> 412  -> 478  -> 540  -> 600  -> 662  -> 719  -> 
718  -> 769  -> 821  -> 768  -> 716  -> 767
```

---

## Part F: In-Depth Analytical Answers

### Question 1: Why is the dataset you selected suitable for shortest-path analysis?
The 9th DIMACS USA Road Network (`USA-road-d.NY`) is uniquely suited because:
1. It reflects an authentic vehicular mobility environment derived directly from US Census Bureau road geometries, preserving realistic physical routing properties (traffic turns, highway ramps, cul-de-sacs, one-way streets).
2. It provides exact geographic coordinates for all $264,346$ vertices, enabling authentic spherical heuristic calculations.
3. It possesses strictly positive physical weights ($w > 0$), matching the fundamental prerequisite for Dijkstra's optimality.
4. Its size ($>733,000$ edges) allows evaluating sub-network scaling across multiple orders of magnitude ($10^3$ to $10^5$ nodes) without synthetic artifacts.

### Question 2: How did you transform the raw dataset into a weighted graph?
Transformation proceeded through three systematic phases:
1. **Ingestion & Filtering:** The plain and gzip-compressed DIMACS files were streamed line-by-line. Header records (`c`, `p`) were parsed; node records (`v`) provided micro-degree coordinates; arc records (`a`) provided directed endpoints and weights. Self-loops and non-positive weights were eliminated.
2. **Indexing & Coordinate Conversion:** 1-based IDs were shifted to 0-based integers. Micro-degrees were converted to standard degrees and precomputed radians.
3. **Adjacency Structure & Subgraph Induction:** A contiguous adjacency list array `adj[u] = [(v, w), ...]` was allocated. To create realistic test graphs of varying sizes, we performed a Breadth-First Search (BFS) starting from a central high-degree node. Extracting the BFS-induced subgraph ensures that every test network forms a topologically connected, geographically compact metropolitan region rather than a disconnected random collection of roads.

### Question 3: Do Dijkstra's algorithm and A* produce the same optimal path cost in your experiments? Explain.
**Yes.** Across all 60 empirical benchmark queries across all graph sizes, `Same_Cost = True`.
**Theoretical Reason:** A* is mathematically guaranteed to find an optimal shortest path identical in cost to Dijkstra if its heuristic function $h(v)$ is **admissible** ($h(v) \le d^*(v, D)$ for all $v$). Because our scaled Haversine heuristic was scaled by the exact minimum $\frac{w(u,v)}{d_{\text{geo}}(u,v)}$ ratio across the network, $h(v)$ never overestimates the true remaining distance. When multiple shortest paths share the exact same minimal cost (tie-breaking scenarios), vertex sequences may differ slightly, but total path costs remain identical.

### Question 4: Which algorithm explores fewer vertices, and why?
**A* consistently explores significantly fewer vertices** (reducing settled vertices by **$75.6\%$ to $88.5\%$**).
- **Dijkstra's Exploration Geometry:** Dijkstra is an uninformed search. Its priority is determined solely by $g(v)$, causing it to explore vertices radially outward in all directions, expanding an isotropic topological circle around $S$ until $D$ is settled.
- **A*'s Exploration Geometry:** A* incorporates $h(v)$ into priority $f(v) = g(v) + h(v)$. Vertices that lead away from $D$ have high $h$-values, penalizing their priority. Consequently, A* focuses its search frontier into a directed elliptical corridor pointing directly toward destination $D$, pruning off thousands of irrelevant side streets.

### Question 5: How does execution time vary as the graph size increases?
Execution time grows sub-linearly with graph size when queries span regional boundaries, and scales near-linearly with path hop length:
- At $|V| = 1,000$, Dijkstra averages $0.211\text{ ms}$ and A* averages $0.079\text{ ms}$.
- At $|V| = 100,000$, Dijkstra averages $4.542\text{ ms}$ and A* averages $1.771\text{ ms}$.
Because our implementation uses early termination upon popping $D$, the search does not necessarily touch every node in the graph; it only settles nodes whose distance is less than $d^*(S, D)$. On larger subgraphs, randomly chosen $(S, D)$ pairs can span wider geographic diameters, increasing the search frontier. A* maintains a consistent $2\times$ to $5\times$ latency advantage across all scales.

### Question 6: Explain the effect of the graph's density on shortest-path computation.
- **Sparsity in Road Networks:** Road networks are planar or near-planar spatial graphs. In our dataset, the average degree is $E/V \approx 3.0$ and max degree is $8$. In sparse graphs, each node relaxation explores only $2$ to $4$ outgoing edges. Priority queue size remains bounded, and heap push operations remain cheap ($O(\log V)$).
- **Contrast with Dense Graphs:** In dense graphs where $|E| \to \Theta(V^2)$ (such as fully connected wireless ad-hoc mesh networks), each settled node forces $O(V)$ edge evaluations, resulting in $O(V^2)$ heap pushes. Under high density, binary-heap Dijkstra degrades to $O(V^2 \log V)$, making Fibonacci heaps ($O(V \log V + E)$) or matrix-based algorithms theoretically preferable. Furthermore, high density degrades heuristic pruning efficiency because dense shortcut edges allow paths to zig-zag across geometric corridors.

### Question 7: What is the theoretical time complexity of your Dijkstra implementation?
Our Dijkstra implementation uses an adjacency list and a binary min-heap (`heapq`) with lazy deletion:
1. Every vertex is popped and settled at most once: $|V|$ extract-min operations, each costing $O(\log |PQ|) = O(\log V)$.
2. Every directed edge is evaluated at most once: at most $|E|$ decrease-key (insert) operations, each costing $O(\log |PQ|) = O(\log V)$.
3. Total Time Complexity:
   $$\mathcal{O}((|V| + |E|) \log |V|)$$
4. For our road network where $|E| \in \mathcal{O}(|V|)$:
   $$\mathcal{O}(|V| \log |V|)$$
5. Path reconstruction follows parent pointers in $O(L)$ time, where $L \ll |V|$ is the number of edges in the final route.

### Question 8: What heuristic did you use for A*? Is it admissible and/or consistent? Justify your answer.
We used the **Scaled Haversine Distance Heuristic**:
$$h(v) = \text{scale} \cdot \text{haversine}(v, D)$$
where $\text{scale} = \min_{(u,v) \in E} \frac{w(u, v)}{\text{haversine}(u, v)} = 7.152478$.
- **Admissibility:** By the triangle inequality on a spherical surface, straight-line distance is the infimum of all spatial paths. Multiplying by $\text{scale}$ ensures $h(v) \le \text{true distance}$ along any road path. Thus, $h(v) \le d^*(v, D)$, satisfying admissibility.
- **Consistency (Monotonicity):** For any edge $(u, v)$, $\text{haversine}(u, D) - \text{haversine}(v, D) \le \text{haversine}(u, v)$. Multiplying by $\text{scale}$ gives $h(u) - h(v) \le \text{scale} \cdot \text{haversine}(u, v) \le w(u, v) \implies h(u) \le w(u, v) + h(v)$. Thus, the heuristic is consistent.

### Question 9: What difficulties did you face while converting the real-world dataset into a graph?
1. **Coordinate Format & Micro-Degrees:** Coordinates were stored as integer micro-degrees without decimal points, requiring scaling and conversion to geographic radians.
2. **Dimension Units Mismatch:** DIMACS distance weights are integers in decimeters, while standard spatial geodesy formulas output meters. Without determining the exact ratio $\text{scale}$, A* would violate admissibility.
3. **Graph Partitioning & Reachability:** Random vertex subsampling breaks connectivity, shattering road networks into isolated nodes. We solved this by using BFS-based induced subgraphs, preserving realistic metropolitan connectivity.
4. **Memory & Parsing Efficiency:** Ingesting 733k arcs naively using dictionaries causes massive memory overhead. We designed compact contiguous lists and streaming gzip readers, slashing parse time to $\approx 1.2\text{ seconds}$.

### Question 10: Based on your observations, which algorithm would you recommend for large-scale IoV/UAV route planning? Justify your answer.
**We strongly recommend A* (with a consistent geographic heuristic) for point-to-point IoV and UAV navigation:**
- **Why A*:** It delivers the exact same provably optimal route as Dijkstra while reducing CPU instruction cycles by **$70\%\text{--}85\%$**, cutting memory access bandwidth, and achieving up to **$5\times$ faster response times**. In real-time IoV scenarios where thousands of connected autonomous vehicles make concurrent navigation requests to edge cloud servers, this efficiency represents enormous computational and energy savings.
- **When Dijkstra is Still Appropriate:**
  1. **One-to-Many / All-Pairs Queries:** When dispatching a vehicle to the nearest of 50 charging stations or hospitals (multi-target Dijkstra).
  2. **Non-Geometric Metrics:** When edge costs represent abstract non-spatial costs (packet loss rate, cryptographic delay, wireless interference) where no spatial coordinates or valid heuristic functions exist.

---

## Bonus Task: Dynamic Edge Costs & Incident Rerouting

### 1. Mathematical Formulation
Real-world vehicular mobility networks experience time-varying traffic delays due to congestion, accidents, signal delays, or adverse weather. We model dynamic edge costs as:
$$w'(u, v) = \alpha \cdot d(u, v) + \beta \cdot c(u, v)$$
where:
- $d(u, v)$ is the static physical distance of the roadway.
- $c(u, v)$ is the dynamic traffic delay cost.
- $\alpha \ge 1.0$ is the distance weighting factor.
- $\beta \ge 0.0$ is the congestion sensitivity factor.

### 2. Experimental Simulation
1. **Baseline Query:** Evaluated between Source $S = 7186$ and Destination $D = 3159$ on the $25,000$-vertex network. Baseline route has $22$ hops and distance cost $38,229.00$.
2. **Traffic Incident Injection:** A severe traffic bottleneck ($c(u, v) = 5.0 \cdot d(u, v)$) was placed across $9$ road segments ($40\%$) along the middle of the baseline path. Additionally, $3\%$ ambient random traffic delays ($c(u, v) \in [1, 3] \cdot d(u, v)$) were scattered throughout the network.
3. **Dynamic Recomputation:** Dijkstra and A* were executed under updated dynamic weights $w'(u, v)$.

### 3. Quantitative Results & Rerouting Analysis

| Metric | Static Baseline Route | Baseline Route under Incident | Dynamic Rerouted Optimal Route |
| :--- | :---: | :---: | :---: |
| **Total Route Cost** | $38,229.00$ | $197,065.19$ | **$39,700.00$** |
| **Route Hop Count** | $22\text{ hops}$ | $22\text{ hops}$ | $24\text{ hops}$ |
| **Cost Savings by Dynamic Rerouting** | — | — | **$157,365.19$ ($-79.85\%$)** |
| **Shared Vertices with Baseline** | $23 / 23$ ($100\%$) | $23 / 23$ ($100\%$) | **$6 / 23$ ($26.1\%$)** |
| **Dijkstra vs. A* Cost Equality** | Identical | — | **Identical ($39,700.00$)** |
| **A* Search Space Reduction** | — | — | **$244\text{ nodes}$ vs. $2,440\text{ nodes}$ ($-90.0\%$)** |

### 4. Technical Insights
1. **Optimal Detour Tradeoff:** The dynamic rerouting algorithm chose a slightly longer spatial path ($24$ hops instead of $22$), incurring an extra $1,471$ decimeters of travel, but avoided the bottleneck delay penalty, saving **$79.85\%$ in overall traversal cost**.
2. **Path Divergence:** Only $26.1\%$ of vertices were shared between the old and new routes; the vehicle branched off onto parallel arterial roads right before the incident zone and merged back near the destination.
3. **Heuristic Preservation:** Because $\alpha \ge 1$ and $c(u, v) \ge 0$, $w'(u, v) \ge d(u, v) \ge \text{scale} \cdot d_{\text{geo}}(u, v)$. Thus, the spatial heuristic remained strictly admissible and consistent, allowing A* to find the optimal detour in only **$1.352\text{ ms}$** while settling only $244$ nodes.

---

## References
1. C. Demetrescu, A. V. Goldberg, and D. S. Johnson (Eds.), *The Shortest Path Problem: Ninth DIMACS Implementation Challenge*, DIMACS Series in Discrete Mathematics and Theoretical Computer Science, Vol. 74, American Mathematical Society, 2009.
2. T. H. Cormen, C. E. Leiserson, R. L. Rivest, and C. Stein, *Introduction to Algorithms*, 4th Edition, MIT Press, 2022.
3. P. E. Hart, N. J. Nilsson, and B. Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths," *IEEE Transactions on Systems Science and Cybernetics*, Vol. 4, No. 2, pp. 100–107, 1968.
4. R. Geisberger, P. Sanders, D. Schultes, and D. Delling, "Contraction Hierarchies: Faster and Simpler Hierarchical Routing in Road Networks," *Experimental Algorithms*, LNCS 5038, Springer, pp. 319–333, 2008.
5. U.S. Census Bureau, *TIGER/Line Shapefiles and Technical Documentation*, Geographic Products Branch, Washington, D.C.
