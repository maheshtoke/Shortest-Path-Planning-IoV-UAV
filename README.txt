=====================================================================
Title: Dijkstra vs A* on the DIMACS USA (NY) Road Network
Project: Shortest-Path Planning in a Large-Scale IoV/UAV Mobility Network
=====================================================================

Requirements:
  Python 3.8+ (tested on Python 3.11.9)
  Optional: matplotlib (for PNG plots), python-docx (for report docx generation)

Dataset:
  USA-road-d.NY.gr.gz and USA-road-d.NY.co.gz from
  http://www.diag.uniroma1.it/challenge9/download.shtml
  (placed in data/; do not unzip).

Execution:
  python sp_iov.py --gr data/USA-road-d.NY.gr.gz --co data/USA-road-d.NY.co.gz --sizes 1000,5000,10000,25000,50000,100000 --pairs 10 --seed 42 --repeats 3 --out results

Smoke Test (Synthetic):
  python sp_iov.py --synthetic 200 --sizes 1000,5000 --pairs 5 --out results

Reproducibility:
  Fixed seed 42; all random choices (subgraph seeding, query generation, congestion distribution) originate from this seed.

Outputs:
  See results/ folder:
    - graph_stats.csv: Topological stats across tested graph sizes
    - per_query_results.csv: Query-level metrics for all runs (both algorithms)
    - summary_results.csv: Aggregated performance comparison table
    - sample_query_path.txt: Full vertex sequence, coordinates, & hops of a sample query
    - bonus_dynamic.txt: Dynamic edge cost rerouting experimental results
    - plot_time.png: Mean execution time vs. graph size
    - plot_vertices.png: Vertices explored vs. graph size
    - plot_edges.png: Edges examined vs. graph size
    - plot_dashboard.png: 3-panel comparative performance dashboard

Files:
  sp_iov.py: Complete standalone program (loader, graph stats, Dijkstra, A*, experiments, plots, disconnected demo, bonus).
  REPORT.md / REPORT.docx: Comprehensive technical laboratory report with full answers to Part F, tables, and proofs.
  README.md / README.txt: Documentation and execution guide.

No external graph-library shortest-path functions (such as NetworkX) are used.
All graph modeling, min-priority queues, and shortest-path algorithms are implemented independently from scratch.
=====================================================================
