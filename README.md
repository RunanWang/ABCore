# ABCore

This repo implements **Constraint-Generation (CG)** and **Exchange-Search (ES)**.

Given an undirected graph, a connected target subgraph $C$, and a threshold $k$, the program first computes the minimum anchor set $A=\{v\in C:d_C(v)<k\}$, then runs CG to compute a lower bound (LB) and ES to compute an upper bound (UB) on the minimum external blocker count.

## Build and run

Requirements: a C++17 compiler, CMake 3.16 or later, and Git. CG uses the open-source integer programming solver [HiGHS](https://github.com/ERGO-Code/HiGHS). The build automatically downloads HiGHS v1.14.0 if no installed CMake package is found, so the first build requires access to GitHub. ES does not invoke an integer programming solver.

Run the following commands from the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
./build/abcore_case
```

To use an installed HiGHS CMake package, specify `-Dhighs_DIR=/path/to/lib/cmake/highs`. For an offline build, install HiGHS beforehand or provide its downloaded source with `-DFETCHCONTENT_SOURCE_DIR_HIGHS=/path/to/HiGHS-v1.14.0`.

## Minimal example

The default instance uses the subgraph on the third line of `Dataset/citeseer/citeseer.subgraph.txt`, with zero-based `subgraph-id=2` and `k=6`.

You can explicitly specify the dataset directory, subgraph ID, and threshold:

```bash
./build/abcore_case Dataset/citeseer 2 6
```

## Time limits

CG and ES each have an independent default wall-clock budget of **300 seconds**, measured from the start of the respective algorithm and including its preprocessing. Each limit accepts a finite positive number, including fractional seconds, and can be configured separately:

```bash
./build/abcore_case --cg-time-limit 300 --es-time-limit 300
./build/abcore_case Dataset/citeseer 2 6 --cg-time-limit 60 --es-time-limit 120
```

CG passes the remaining total budget to each HiGHS solve. ES checks the budget at safe search checkpoints. Preprocessing, constraint generation, and result assembly stop at safe checkpoints, so elapsed time may slightly exceed the budget.


## Data format and download sources

The bundled citeseer dataset is the cleaned version from the development environment. Each line of the graph file contains two nonnegative vertex IDs; each line of the subgraph file lists the vertices of one connected target subgraph. Graph loading treats edges as undirected and removes self-loops and duplicate edges. The graph and subgraphs share the same vertex ID space.

| Dataset | Download source | Notes |
| --- | --- | --- |
| email-eu | [SNAP email-Eu-core](https://snap.stanford.edu/data/email-Eu-core.html) | Email network and department labels |
| cora, citeseer | [gnn-benchmark](https://github.com/shchur/gnn-benchmark/tree/master/data/npz) | Citation network source; the bundled citeseer subgraphs have been cleaned |
| facebook | [SNAP Facebook](https://snap.stanford.edu/data/ego-Facebook.html) | Ego networks and circles |
| photo | [gnn-benchmark](https://github.com/shchur/gnn-benchmark/tree/master/data/npz) | Amazon Photo |
| googleplus | [SNAP Google+](https://snap.stanford.edu/data/ego-Gplus.html) | Ego networks and circles; the cleaned development version has 7,768 non-isolated vertices |
| blogcatalog | [Syracuse Social Computing Repository](https://datasets.syr.edu/) | Select BlogCatalog v3, with approximately 10K vertices |
| dblp3 | [Co-authorship networks (Xueyu Mao)](https://xueyumao.github.io/coauthorship) | Select DBLP3: download its adjacency matrix and ground-truth membership matrix; the cleaned development version has approximately 13K vertices and three target subgraphs |
| flickr | [Syracuse Flickr](https://flickr.datasets.syr.edu/) | Contact network and group memberships |
| youtube | [Syracuse Social Computing Repository](https://datasets.syr.edu/) | Select YouTube |

