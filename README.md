# Shortest-Path Planning in a Large-Scale IoV/UAV Mobility Network

## Project Overview
This project presents an algorithmic study and empirical performance evaluation of shortest-path routing algorithms on large-scale vehicular mobility (Internet of Vehicles - IoV) networks. Using the **9th DIMACS Implementation Challenge USA Road Network (New York Benchmark: USA-road-d.NY)**, we evaluate and compare **Dijkstra's Algorithm** (with a min-priority queue binary heap) and the **A* Search Algorithm** (with an admissible and consistent scaled Haversine distance heuristic) across multiple network scales ($1,000$ to $100,000+$ vertices). 

Additionally, a **Dynamic Edge Cost** simulation models real-time traffic congestion ($w'(u,v) = \alpha \cdot d(u,v) + \beta \cdot c(u,v)$) to evaluate dynamic obstacle avoidance and vehicle rerouting.

---

## Repository Structure
```
daa lab project/
├── sp_iov.py                 # Core engine (Graph loader, Dijkstra, A*, Benchmarking, Dynamic Rerouting)
├── generate_report_docx.py   # Automated script to generate Word DOCX submission report
├── REPORT.md                 # Complete comprehensive technical lab report
├── REPORT.docx               # Formatted submission report document (with tables & plots)
├── README.md                 # Project documentation and execution instructions
├── README.txt                # Plaintext submission README
├── data/                     # Dataset storage folder
│   ├── USA-road-d.NY.gr.gz   # DIMACS New York road graph arcs & physical distance weights
│   └── USA-road-d.NY.co.gz   # DIMACS New York vertex coordinates (longitude, latitude)
└── results/                  # Experimental outputs
    ├── graph_stats.csv       # Topological stats across tested graph sizes
    ├── per_query_results.csv # Query-level metrics for all runs (both algorithms)
    ├── summary_results.csv   # Aggregated performance comparison table
    ├── sample_query_path.txt # Full vertex sequence, coordinates, & hops of a sample query
    ├── bonus_dynamic.txt     # Dynamic edge cost rerouting experimental results
    ├── plot_time.png         # Mean execution time vs. graph size
    ├── plot_vertices.png     # Vertices explored vs. graph size
    ├── plot_edges.png        # Edges examined vs. graph size
    └── plot_dashboard.png    # 3-panel comparative performance dashboard
```

---

## Dataset Reference & Details
- **Dataset Name:** 9th DIMACS Implementation Challenge: USA Road Networks (New York - Distance Metric)
- **Source URL:** http://www.diag.uniroma1.it/challenge9/download.shtml
- **Citation:** 
  > C. Demetrescu, A. V. Goldberg, and D. S. Johnson (Eds.), *The Shortest Path Problem: Ninth DIMACS Implementation Challenge*, DIMACS Series in Discrete Mathematics and Theoretical Computer Science, Vol. 74, American Mathematical Society (AMS), 2009.
- **Data Attributes:**
  - **Vertices ($|V|$):** $264,346$ intersections / mobility waypoints.
  - **Edges ($|E|$):** $733,846$ directed road segments.
  - **Coordinates:** Exact geographical longitude and latitude (in micro-degrees).
  - **Edge Weights:** Positive physical distances (in decimeters).

---

## Requirements & Environment
- **Python Version:** Python 3.8+ (tested on Python 3.11.9)
- **Standard Library:** `sys`, `os`, `gzip`, `math`, `time`, `random`, `heapq`, `argparse`, `collections`
- **Plotting & Reporting Libraries:**
  ```bash
  pip install matplotlib python-docx
  ```
*(Note: As required by the assignment guidelines, no external graph-processing libraries such as NetworkX or graph-tool are used. All graph representations, min-priority queues, and shortest-path algorithms are implemented independently from scratch).*

---

## Step-by-Step Execution Guide

### 1. Download Dataset (Automated or Manual)
The dataset files are placed in `data/`:
- Direct links:
  - `http://www.diag.uniroma1.it/challenge9/data/USA-road-d/USA-road-d.NY.gr.gz`
  - `http://www.diag.uniroma1.it/challenge9/data/USA-road-d/USA-road-d.NY.co.gz`
*(Do not decompress the `.gz` files; `sp_iov.py` streams them directly in memory).*

### 2. Fast Smoke Test (Synthetic Verification)
To verify the implementation without downloading large datasets, run the synthetic grid test:
```bash
python sp_iov.py --synthetic 200 --sizes 1000,5000 --pairs 5 --out results
```

### 3. Full Benchmark Run on Real-World DIMACS Dataset
To execute the complete experimental evaluation across sizes $1,000$, $5,000$, $10,000$, $25,000$, $50,000$, and $100,000$ vertices with 10 random reachable queries each and 3 timing repeats:
```bash
python sp_iov.py --gr data/USA-road-d.NY.gr.gz --co data/USA-road-d.NY.co.gz --sizes 1000,5000,10000,25000,50000,100000 --pairs 10 --seed 42 --repeats 3 --out results
```

### 4. Dynamic Edge Cost Parameters (Bonus Experiment)
To test varying dynamic congestion penalties ($\alpha \cdot d + \beta \cdot c$):
```bash
python sp_iov.py --gr data/USA-road-d.NY.gr.gz --co data/USA-road-d.NY.co.gz --alpha 1.0 --beta 3.0 --seed 42 --out results
```

### 5. Generate Word Document Lab Report
```bash
python generate_report_docx.py
```
This compiles `REPORT.docx` containing all required analysis, tables, figures, complexity derivations, and Part F responses.

---

## Key Experimental Results Summary

| Graph Size ($|V|$) | Algorithm | Avg Shortest-Path Cost | Vertices Explored | Edges Examined | Execution Time (ms) | Speedup / Exploration Reduction |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **1,000** | Dijkstra | 13,592.90 | 389.5 | 1,398.2 | 0.211 ms | Baseline |
| **1,000** | **A\*** | **13,592.90** | **44.8** | **166.0** | **0.079 ms** | **2.67x speedup (-88.5% nodes)** |
| **5,000** | Dijkstra | 54,194.20 | 2,918.8 | 9,895.2 | 4.492 ms | Baseline |
| **5,000** | **A\*** | **54,194.20** | **633.6** | **2,203.3** | **1.795 ms** | **2.50x speedup (-78.3% nodes)** |
| **10,000** | Dijkstra | 46,282.10 | 3,010.8 | 10,313.2 | 4.376 ms | Baseline |
| **10,000** | **A\*** | **46,282.10** | **441.5** | **1,519.3** | **0.854 ms** | **5.12x speedup (-85.3% nodes)** |
| **25,000** | Dijkstra | 38,256.70 | 2,019.2 | 6,878.0 | 2.415 ms | Baseline |
| **25,000** | **A\*** | **38,256.70** | **490.4** | **1,676.0** | **1.136 ms** | **2.13x speedup (-75.7% nodes)** |
| **50,000** | Dijkstra | 46,910.20 | 2,203.2 | 7,540.2 | 2.978 ms | Baseline |
| **50,000** | **A\*** | **46,910.20** | **482.4** | **1,635.3** | **1.097 ms** | **2.71x speedup (-78.1% nodes)** |
| **100,000** | Dijkstra | 65,590.00 | 3,398.6 | 10,521.7 | 4.542 ms | Baseline |
| **100,000** | **A\*** | **65,590.00** | **828.6** | **2,522.8** | **1.771 ms** | **2.56x speedup (-75.6% nodes)** |

- **Exact Cost Equality:** 100% of queries produced identical path costs (`Same_Cost = True`).
- **Search Space Reduction:** A* explores **$75.6\%$ to $88.5\%$ fewer vertices** than Dijkstra by pruning unproductive search frontiers.
- **Bonus Rerouting:** Dynamic traffic congestion injection ($c = 5d$ on $40\%$ of baseline route) triggered dynamic rerouting yielding a **$79.85\%$ cost saving** compared to persisting through the congested bottleneck.

---

## Reproducibility Notice
All experiments use a fixed pseudo-random seed (`--seed 42`). Running the command above reproduces the exact numbers reported in the summary tables and figures.
