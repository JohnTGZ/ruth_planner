# Ruth Planner (WIP)
[Ru]st Pa[th] Planner aims to be a collection of search-based path planning algorithms written in Rust.
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

## Testing
```rust 
cargo test --show-output
```
