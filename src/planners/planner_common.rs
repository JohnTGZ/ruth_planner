use crate::maps::gridmap::Gridmap;
use std::collections::{HashMap, HashSet};

/// Contains the path from start to goal and the list of visited cells
#[derive(Debug)]
pub struct MotionPlan {
    pub path: Vec<(u32, u32)>,
    pub closed_list: HashSet<(u32, u32)>,
}

/// Get 4 way connected neighbors (Up, Down, Left and Right)
pub fn get_neighbors_4_con(pos: (u32, u32), gridmap: &Gridmap) -> Vec<(u32, u32)> {
    let mut neighbors: Vec<(u32, u32)> = Vec::new();

    let pos_0_non_zero = pos.0 != 0;
    let pos_1_non_zero = pos.1 != 0;
    let pos_0_non_max = pos.0 < gridmap.get_width() - 1;
    let pos_1_non_max = pos.1 < gridmap.get_height() - 1;

    if pos_0_non_zero {
        if gridmap.xy_is_traversable((pos.0 - 1, pos.1)) {
            neighbors.push((pos.0.checked_sub(1).unwrap(), pos.1)); // Left (x-1, y))
        }
    }

    if pos_0_non_max {
        if gridmap.xy_is_traversable((pos.0 + 1, pos.1)) {
            neighbors.push((pos.0 + 1, pos.1)); // Right (x+1, y)
        }
    }

    if pos_1_non_zero {
        if gridmap.xy_is_traversable((pos.0, pos.1 - 1)) {
            neighbors.push((pos.0, pos.1.checked_sub(1).unwrap())); // Top (x, y-1)
        }
    }

    if pos_1_non_max {
        if gridmap.xy_is_traversable((pos.0, pos.1 + 1)) {
            neighbors.push((pos.0, pos.1 + 1)); // Bottom (x, y+1), (idx)
        }
    }

    return neighbors;
}

/// Get 8 way connected neighbors (4 Way + diagonals)
pub fn get_neighbors_8_con(pos: (u32, u32), gridmap: &Gridmap) -> Vec<(u32, u32)> {
    let mut neighbors: Vec<(u32, u32)> = Vec::new();

    let pos_0_non_zero = pos.0 != 0;
    let pos_1_non_zero = pos.1 != 0;
    let pos_0_non_max = pos.0 < gridmap.get_width() - 1;
    let pos_1_non_max = pos.1 < gridmap.get_height() - 1;

    if pos_0_non_zero {
        let left = (pos.0 - 1, pos.1);

        if gridmap.xy_is_traversable(left) {
            neighbors.push(left); // Left (x-1, y))
        }

        if pos_1_non_zero {
            let top_left = (pos.0 - 1, pos.1 - 1);
            if gridmap.xy_is_traversable(top_left) {
                neighbors.push(top_left); // Top-Left (x-1, y-1))
            }
        }
    }

    if pos_0_non_max {
        let right = (pos.0 + 1, pos.1);

        if gridmap.xy_is_traversable(right) {
            neighbors.push(right); // Right (x+1, y)
        }

        if pos_1_non_max {
            let bottom_right = (pos.0 + 1, pos.1 + 1);
            if gridmap.xy_is_traversable(bottom_right) {
                neighbors.push(bottom_right); // Bottom-Right (x+1, y+1))
            }
        }
    }

    if pos_1_non_zero {
        let top = (pos.0, pos.1 - 1);
        if gridmap.xy_is_traversable(top) {
            neighbors.push(top); // Top (x, y-1)
        }

        if pos_0_non_max {
            let top_right = (pos.0 + 1, pos.1 - 1);
            if gridmap.xy_is_traversable(top_right) {
                neighbors.push(top_right); // Top-Right (x+1, y-1))
            }
        }
    }

    if pos_1_non_max {
        let bottom = (pos.0, pos.1 + 1);
        if gridmap.xy_is_traversable(bottom) {
            neighbors.push(bottom); // Bottom (x, y+1), (idx)
        }

        if pos_0_non_zero {
            let bottom_left = (pos.0 - 1, pos.1 + 1);
            if gridmap.xy_is_traversable(bottom_left) {
                neighbors.push(bottom_left); // Bottom-Left (x-1, y+1))
            }
        }
    }

    return neighbors;
}

pub fn trace_path(
    goal_cell: &(u32, u32),
    parents: &HashMap<(u32, u32), (u32, u32)>,
) -> Vec<(u32, u32)> {
    let mut path: Vec<(u32, u32)> = Vec::new();
    let mut cur_cell = goal_cell;
    path.push(*cur_cell);

    while parents.get(&cur_cell).unwrap() != cur_cell {
        cur_cell = parents.get(&cur_cell).unwrap();
        path.push(*cur_cell);
    }

    path
}

// Common methods to calculate costs

/// Get the Euclidean (L2) Distance
pub fn get_l2_cost(pos_1: (u32, u32), pos_2: (u32, u32)) -> u32 {
    let dx = (pos_1.0 as i32 - pos_2.0 as i32).abs();
    let dy = (pos_1.1 as i32 - pos_2.1 as i32).abs();

    ((dx as f32).hypot(dy as f32) * 100.0) as u32
}

// /// Get the Manhattan Distance
// pub fn get_manhattan_cost(pos_1: (i32, i32), pos_2: (i32, i32)) -> f32
// {
//   let dx = ((pos_1.0 - pos_2.0)).abs();
//   let dy = ((pos_1.1 - pos_2.1)).abs();

//   (dx + dy) as f32
// }

// /// Get the Chebyshev distance
// pub fn get_chebyshev_cost(pos_1: (i32, i32), pos_2: (i32, i32)) -> f32
// {
//   let dx = ((pos_1.0 - pos_2.0)).abs();
//   let dy = ((pos_1.1 - pos_2.1)).abs();

//   max(dx, dy) as f32
// }
