# Ruth Planner (WIP)
[Ru]st Pa[th] Planner aims to be a comprehensive collection of search-based path planning algorithms written in Rust.
This crate aims to use minimal external dependencies, except for visualization purposes.

## Implemented algorithms

### Breadth-First Search
<img src="test_maps/planners/bfs/test_plan1.png" width="250">

### Depth-First Search
<img src="test_maps/planners/dfs/test_plan3.png" width="250">

### Djikstra
<img src="test_maps/planners/dijkstra/test_plan4.png" width="250">

### A*
<img src="test_maps/planners/a_star/test_plan4.png" width="250">

## Roadmap
1. Implement other algorithms (ARA*, D* Lite, Theta*)

2. Optimize planners

3. Generate animations for the path search.

## Testing
```rust 
cargo test --show-output
```
