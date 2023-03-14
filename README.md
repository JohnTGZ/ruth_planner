# Ruth Planner (WIP)
[Ru]st Pa[th] Planner aims to be a collection of search-based path planning algorithms written in Rust.
This crate aims to use minimal external dependencies, except for visualization purposes.

# Search-Based Algorithms

### Breadth-First Search
<img src="test_maps/planners/bfs/test_plan1.png" width="250">

### Depth-First Search
<img src="test_maps/planners/dfs/test_plan3.png" width="250">

### Djikstra
<img src="test_maps/planners/dijkstra/test_plan_ros2.png" width="250">

### A*
<img src="test_maps/planners/a_star/test_plan_ros2.png" width="250">


# Testing
```rust 
cargo test --show-output
```

# License

The source code in this package is released under the Apache-2.0 License. For further details, see the [LICENSE](LICENSE) file.

# Acknowledgement

The test maps in [test_maps/nav2_maps](test_maps/nav2_maps) were taken from the [ROS-Planning Nav2 Repository](https://github.com/ros-planning/navigation2)