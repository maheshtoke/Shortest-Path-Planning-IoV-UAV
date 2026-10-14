#!/usr/bin/env python3
"""
Generates a Word Document (REPORT.docx) for the DAA Lab Project.
Embeds tables, formatting, and high-resolution performance plots.
"""

import os
from typing import Any

import docx
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml import OxmlElement, parse_xml
from docx.oxml.ns import nsdecls, qn
from docx.shared import Inches, Pt, RGBColor


def set_cell_background(cell, fill_hex):
    tcPr = cell._element.get_or_add_tcPr()
    shd = parse_xml(f'<w:shd {nsdecls("w")} w:fill="{fill_hex}"/>')
    tcPr.append(shd)

def set_cell_margins(cell, top=100, bottom=100, left=150, right=150):
    tcPr = cell._element.get_or_add_tcPr()
    tcMar = OxmlElement('w:tcMar')
    for m, val in [('w:top', top), ('w:bottom', bottom), ('w:left', left), ('w:right', right)]:
        node = OxmlElement(m)
        node.set(qn('w:w'), str(val))
        node.set(qn('w:type'), 'dxa')
        tcMar.append(node)
    tcPr.append(tcMar)

def create_report_docx():
    doc = docx.Document()

    # Set page margins (0.75 in)
    sections = doc.sections
    for section in sections:
        section.top_margin = Inches(0.75)
        section.bottom_margin = Inches(0.75)
        section.left_margin = Inches(0.75)
        section.right_margin = Inches(0.75)

    # Styles
    normal_style: Any = doc.styles['Normal']
    normal_style.font.name = 'Calibri'
    normal_style.font.size = Pt(11)
    normal_style.font.color.rgb = RGBColor(0x22, 0x22, 0x22)

    # Title
    p_title = doc.add_paragraph()
    p_title.paragraph_format.space_before = Pt(0)
    p_title.paragraph_format.space_after = Pt(4)
    run_title = p_title.add_run("Shortest-Path Planning in a Large-Scale IoV/UAV Mobility Network")
    run_title.font.name = 'Calibri'
    run_title.font.size = Pt(22)
    run_title.font.bold = True
    run_title.font.color.rgb = RGBColor(0x1D, 0x35, 0x57)

    # Subtitle / Metadata
    p_meta = doc.add_paragraph()
    p_meta.paragraph_format.space_after = Pt(16)
    r_sub = p_meta.add_run("DAA Laboratory Project | Comparative Algorithmic Performance Evaluation\n")
    r_sub.font.bold = True
    r_sub.font.size = Pt(12)
    r_sub.font.color.rgb = RGBColor(0x45, 0x7B, 0x9D)
    
    r_meta = p_meta.add_run("Benchmark: 9th DIMACS USA Road Network (New York: USA-road-d.NY) | Date: October 2026")
    r_meta.font.size = Pt(10)
    r_meta.font.italic = True
    r_meta.font.color.rgb = RGBColor(0x66, 0x66, 0x66)

    # Executive Summary
    h1 = doc.add_heading("1. Executive Summary", level=1)
    h1.paragraph_format.space_before = Pt(12)
    h1.paragraph_format.space_after = Pt(6)

    doc.add_paragraph(
        "This project presents a rigorous computational study and empirical analysis of shortest-path routing "
        "algorithms applied to realistic, large-scale vehicular mobility (Internet of Vehicles - IoV) networks. "
        "Using the 9th DIMACS Implementation Challenge New York road network (264,346 vertices, 733,846 directed edges), "
        "we construct a weighted spatial graph and evaluate Dijkstra's Algorithm (implemented from first principles with a binary min-heap) "
        "against the A* Search Algorithm (with an admissible and consistent scaled Haversine distance heuristic) across multiple network scales "
        "(1,000 to 100,000 vertices)."
    )

    doc.add_paragraph(
        "Key Findings:\n"
        "• Cost Equivalence: A* and Dijkstra yielded identical shortest-path costs across 100% of evaluated queries.\n"
        "• Search Space Reduction: A* pruned the search space dramatically, exploring 75.6% to 88.5% fewer vertices and examining 74.5% to 88.1% fewer edges than Dijkstra.\n"
        "• Execution Latency: A* achieved an average speedup of 2.1x to 5.1x over Dijkstra across all network sizes.\n"
        "• Dynamic Rerouting (Bonus): Dynamic edge cost penalties modeling traffic incidents (w'(u,v) = α·d + β·c) triggered optimal rerouting, saving 79.85% in path traversal cost."
    )

    # Part A: Dataset Selection
    doc.add_heading("2. Part A: Dataset Selection & Specification", level=1)
    doc.add_paragraph(
        "To satisfy the requirement for a realistic large-scale mobility network, we selected the official 9th DIMACS Implementation Challenge "
        "USA Road Network (New York State benchmark: USA-road-d.NY). This dataset is derived from US Census Bureau TIGER/Line geographic files."
    )

    # Table: Dataset Facts
    t_a = doc.add_table(rows=6, cols=2)
    t_a.alignment = WD_TABLE_ALIGNMENT.CENTER
    dataset_facts = [
        ("Dataset Name", "9th DIMACS Implementation Challenge: USA Road Networks (New York - Distance Metric)"),
        ("Source URL", "http://www.diag.uniroma1.it/challenge9/download.shtml"),
        ("Primary Citation", "C. Demetrescu, A. V. Goldberg, D. S. Johnson (eds.), The Shortest Path Problem, DIMACS Series, Vol. 74, AMS, 2009"),
        ("Scale / Size", "264,346 Vertices | 733,846 Directed Arcs (Road Segments)"),
        ("Graph Modeling", "Vertex = Road intersection/waypoint; Edge = Directed road segment; Weight = Road distance (decimeters)"),
        ("Suitability Rationale", "Real urban road geometry, authentic one-way restrictions, exact coordinates for A*, standardized scientific benchmark")
    ]
    for row_idx, (k, v) in enumerate(dataset_facts):
        row = t_a.rows[row_idx]
        row.cells[0].text = k
        row.cells[1].text = v
        set_cell_background(row.cells[0], "F1FAEE")
        row.cells[0].paragraphs[0].runs[0].font.bold = True
        set_cell_margins(row.cells[0])
        set_cell_margins(row.cells[1])
        row.cells[0].width = Inches(2.2)
        row.cells[1].width = Inches(4.8)

    doc.add_paragraph().paragraph_format.space_after = Pt(8)

    # Part B: Graph Construction & Statistics
    doc.add_heading("3. Part B: Graph Construction & Topological Analysis", level=1)
    doc.add_paragraph(
        "Preprocessing steps applied:\n"
        "1. Vertex Relabeling: Converted 1-based indices to 0-based integers.\n"
        "2. Degeneracy Filtering: Discarded self-loops and non-positive weights.\n"
        "3. Coordinate Transformation: Scaled micro-degrees (10^-6 deg) to decimal degrees and precomputed radians.\n"
        "4. Adjacency List: Built adj[u] = [(v, w), ...] storing only required neighbors, ensuring low memory footprint (~45 MB).\n"
        "5. Subgraph Generation: BFS outward from a central high-degree node to generate realistic, topologically connected, geographically compact subgraphs."
    )

    # Table: Graph Stats
    t_b = doc.add_table(rows=7, cols=5)
    t_b.alignment = WD_TABLE_ALIGNMENT.CENTER
    b_headers = ["Graph Scale", "Vertices (|V|)", "Edges (|E|)", "Avg Degree (E/V)", "Max Degree"]
    for i, h in enumerate(b_headers):
        cell = t_b.rows[0].cells[i]
        cell.text = h
        cell.paragraphs[0].runs[0].font.bold = True
        cell.paragraphs[0].runs[0].font.color.rgb = RGBColor(0xFF, 0xFF, 0xFF)
        set_cell_background(cell, "1D3557")
        set_cell_margins(cell)

    b_data = [
        ("1,000", "1,000", "3,468", "3.4680", "8"),
        ("5,000", "5,000", "16,800", "3.3600", "8"),
        ("10,000", "10,000", "33,992", "3.3992", "8"),
        ("25,000", "25,000", "84,532", "3.3813", "8"),
        ("50,000", "50,000", "161,546", "3.2309", "8"),
        ("100,000", "100,000", "304,170", "3.0417", "8"),
    ]
    for r_idx, r_data in enumerate(b_data, 1):
        row = t_b.rows[r_idx]
        bg = "F8F9FA" if r_idx % 2 == 1 else "FFFFFF"
        for c_idx, val in enumerate(r_data):
            cell = row.cells[c_idx]
            cell.text = val
            set_cell_background(cell, bg)
            set_cell_margins(cell)

    doc.add_paragraph().paragraph_format.space_after = Pt(8)

    # Part C & D: Dijkstra vs A*
    doc.add_heading("4. Parts C & D: Algorithms & Mathematical Complexity", level=1)
    doc.add_paragraph(
        "Dijkstra's Algorithm Implementation:\n"
        "• Implemented with a binary min-heap (heapq) using lazy deletion to handle distance relaxations in O(log V) time.\n"
        "• Early stopping triggers as soon as destination D is popped from the heap.\n"
        "• Theoretical time complexity: O((V + E) log V). For sparse road networks (E ≈ 3V), practical runtime is O(V log V).\n"
        "• Space complexity: O(V + E) for adjacency list, parent array, distance array, and priority queue."
    )

    doc.add_paragraph(
        "A* Search Algorithm Implementation:\n"
        "• Evaluates priority f(v) = g(v) + h(v), where h(v) = scale · haversine(v, D).\n"
        "• The scaling constant scale = min_{(u,v)} [weight(u,v) / haversine(u,v)] = 7.152478 guarantees that for every edge, "
        "weight(u,v) ≥ scale · haversine(u,v).\n"
        "• Admissibility: Straight-line spherical distance is the infimum of any physical path. Multiplying by scale ensures h(v) ≤ d*(v, D).\n"
        "• Consistency (Monotonicity): By the triangle inequality on the sphere, haversine(u, D) ≤ haversine(u, v) + haversine(v, D). "
        "Scaling yields h(u) - h(v) ≤ scale · haversine(u, v) ≤ weight(u, v), proving consistency.\n"
        "• Consistency guarantees that each node is settled at most once without needing re-opening."
    )

    # Part E: Experimental Evaluation
    doc.add_heading("5. Part E: Experimental Evaluation & Results", level=1)
    doc.add_paragraph(
        "Benchmark queries were evaluated across 6 network scales using 10 reachable (S, D) pairs per scale with 3 timing repeats:"
    )

    # Summary Table
    t_e = doc.add_table(rows=13, cols=6)
    t_e.alignment = WD_TABLE_ALIGNMENT.CENTER
    e_headers = ["Graph Size", "Algorithm", "Avg Shortest Cost", "Vertices Explored", "Edges Examined", "Time (ms)"]
    for i, h in enumerate(e_headers):
        cell = t_e.rows[0].cells[i]
        cell.text = h
        cell.paragraphs[0].runs[0].font.bold = True
        cell.paragraphs[0].runs[0].font.color.rgb = RGBColor(0xFF, 0xFF, 0xFF)
        set_cell_background(cell, "1D3557")
        set_cell_margins(cell)

    e_data = [
        ("1,000", "Dijkstra", "13,592.90", "389.5", "1,398.2", "0.211"),
        ("1,000", "A*", "13,592.90", "44.8 (-88.5%)", "166.0 (-88.1%)", "0.079 (2.67x)"),
        ("5,000", "Dijkstra", "54,194.20", "2,918.8", "9,895.2", "4.492"),
        ("5,000", "A*", "54,194.20", "633.6 (-78.3%)", "2,203.3 (-77.7%)", "1.795 (2.50x)"),
        ("10,000", "Dijkstra", "46,282.10", "3,010.8", "10,313.2", "4.376"),
        ("10,000", "A*", "46,282.10", "441.5 (-85.3%)", "1,519.3 (-85.3%)", "0.854 (5.12x)"),
        ("25,000", "Dijkstra", "38,256.70", "2,019.2", "6,878.0", "2.415"),
        ("25,000", "A*", "38,256.70", "490.4 (-75.7%)", "1,676.0 (-75.6%)", "1.136 (2.13x)"),
        ("50,000", "Dijkstra", "46,910.20", "2,203.2", "7,540.2", "2.978"),
        ("50,000", "A*", "46,910.20", "482.4 (-78.1%)", "1,635.3 (-78.3%)", "1.097 (2.71x)"),
        ("100,000", "Dijkstra", "65,590.00", "3,398.6", "10,521.7", "4.542"),
        ("100,000", "A*", "65,590.00", "828.6 (-75.6%)", "2,522.8 (-76.0%)", "1.771 (2.56x)"),
    ]
    for r_idx, r_data in enumerate(e_data, 1):
        row = t_e.rows[r_idx]
        bg = "EBF2FA" if "A*" in r_data[1] else "FFFFFF"
        for c_idx, val in enumerate(r_data):
            cell = row.cells[c_idx]
            cell.text = val
            set_cell_background(cell, bg)
            set_cell_margins(cell)
            if "A*" in r_data[1]:
                cell.paragraphs[0].runs[0].font.bold = True

    doc.add_paragraph().paragraph_format.space_after = Pt(12)

    # Embed Plots
    doc.add_heading("6. Performance Plots & Visual Analysis", level=1)
    dashboard_path = os.path.join("results", "plot_dashboard.png")
    if os.path.exists(dashboard_path):
        doc.add_paragraph("Figure 1: Comprehensive Comparative Dashboard (Execution Time, Vertices Explored, Edges Examined)")
        doc.add_picture(dashboard_path, width=Inches(6.8))

    doc.add_paragraph().paragraph_format.space_after = Pt(12)

    # Part F: Detailed Answers
    doc.add_heading("7. Part F: Analytical Answers to Core Questions", level=1)
    
    questions = [
        ("Q1. Suitability of Selected Dataset",
         ("The 9th DIMACS USA Road Network (New York) is uniquely suited because it models authentic vehicular road infrastructure "
         "derived from US Census TIGER/Line data. It contains 264,346 nodes, realistic one-way constraints, physical distance weights, "
         "and exact geographic coordinates for A* heuristics. It provides a standardized, cited, and reproducible benchmark.")),

        ("Q2. Transformation of Raw Data into Graph",
         ("Raw .gr and .co files were streamed line-by-line. 1-based indices were shifted to 0-based. Self-loops and non-positive weights "
         "were filtered out. Micro-degrees were converted to decimal degrees and radians. To evaluate subgraphs, a BFS was performed "
         "from a central node to extract compact, topologically connected metropolitan regions rather than shattered random nodes.")),

        ("Q3. Cost Equivalence between Dijkstra and A*",
         ("Dijkstra and A* produced identical path costs across 100% of tested queries. Because our scaled Haversine heuristic is admissible "
         "and consistent (h(v) <= d*(v, D)), A* is mathematically guaranteed to return the optimal shortest path.")),

        ("Q4. Search State Exploration Comparison",
         ("A* explored 75.6% to 88.5% fewer vertices than Dijkstra. Dijkstra is an uninformed search that expands radially outward in all "
         "directions (circular topological wave). A* incorporates h(v) into f(v) = g(v) + h(v), directing exploration into a narrow "
         "elliptical corridor pointing directly toward the destination.")),

        ("Q5. Scaling of Execution Time vs. Graph Size",
         ("Execution time scales sub-linearly with graph size when queries span regional boundaries, and near-linearly with path hop length. "
         "With early termination, search time is governed by the radius d*(S, D) rather than total graph size. A* maintained an execution "
         "time of 0.08 ms to 1.77 ms, consistently 2x to 5x faster than Dijkstra.")),

        ("Q6. Impact of Graph Density",
         ("Road networks are sparse spatial planar graphs (avg degree ≈ 3.0, max degree 8). Sparsity ensures each settled node explores only "
         "2-4 edges, keeping binary heap operations cheap (O(log V)). In dense graphs (E ≈ V^2), heap pushes explode to O(V^2), degrading "
         "binary-heap Dijkstra to O(V^2 log V) and diminishing heuristic corridor pruning.")),

        ("Q7. Theoretical Complexity of Dijkstra",
         ("Our Dijkstra uses an adjacency list and binary min-heap with lazy deletion: V pop operations and at most E push operations, "
         "each taking O(log V) time. Total time complexity is O((V + E) log V). For sparse networks where E = O(V), practical runtime is "
         "O(V log V). Space complexity is O(V + E).")),

        ("Q8. Heuristic Justification, Admissibility, and Consistency",
         ("We used scaled Haversine distance: h(v) = scale · haversine(v, D). By setting scale = min_{(u,v)} [weight(u,v) / haversine(u,v)], "
         "we guarantee weight(u,v) >= scale · haversine(u,v) for every edge. By the triangle inequality on the sphere, "
         "h(u) - h(v) <= scale · haversine(u, v) <= weight(u, v), proving both consistency and admissibility.")),

        ("Q9. Conversion Challenges",
         ("Key challenges included: coordinate micro-degree units, decimeter distance units requiring heuristic scaling, streaming large "
         "compressed files efficiently, and extracting connected subgraphs using BFS rather than random sampling.")),

        ("Q10. Architectural Recommendation for IoV/UAV Route Planning",
         ("We strongly recommend A* with a consistent geographic heuristic for point-to-point IoV and UAV route planning. It provides "
         "provably optimal paths while saving 75-88% in CPU expansions and cutting query latency by 2x-5x, essential for high-throughput "
         "edge navigation servers. Dijkstra remains suitable when no coordinates exist or for single-source multi-destination queries."))
    ]

    for title, ans in questions:
        p_q = doc.add_paragraph()
        p_q.paragraph_format.space_before = Pt(6)
        p_q.paragraph_format.space_after = Pt(2)
        r_qt = p_q.add_run(title + "\n")
        r_qt.font.bold = True
        r_qt.font.color.rgb = RGBColor(0x1D, 0x35, 0x57)
        p_q.add_run(ans)

    # Bonus Task
    doc.add_heading("8. Bonus Task: Dynamic Edge Costs & Congestion Rerouting", level=1)
    doc.add_paragraph(
        "Dynamic edge costs were modeled as w'(u,v) = α·d(u,v) + β·c(u,v). Under a simulated traffic incident on 40% of the baseline route "
        "(c = 5d) and ambient delay (c = 1..3d):\n"
        "• Baseline Route under Incident: 197,065.19\n"
        "• Dynamic Rerouted Optimal Route: 39,700.00 (Cost Saving: 157,365.19 or 79.85%)\n"
        "• Overlap with Original Route: 6 / 23 nodes (26.1%)\n"
        "• Algorithmic Performance: A* explored only 244 nodes (vs. 2,440 in Dijkstra) in 1.35 ms, finding the exact optimal detour."
    )

    # Save
    out_docx = "REPORT.docx"
    doc.save(out_docx)
    print(f"[+] Successfully generated Word Document report: {out_docx}")

if __name__ == "__main__":
    create_report_docx()
