use std::collections::{HashMap, HashSet};

use super::planner_common::*;
use crate::maps::gridmap::Gridmap;

// Depth-First Search

/// Retrieve a motion plan given start and goal location
pub fn generate_plan(
    start_cell: (u32, u32),
    goal_cell: (u32, u32),
    gridmap: &Gridmap,
) -> MotionPlan {
    let mut path: Vec<(u32, u32)> = Vec::new();

    let mut open_list: Vec<(u32, u32)> = Vec::new();
    let mut closed_list: HashSet<(u32, u32)> = HashSet::new();

    let mut parents: HashMap<(u32, u32), (u32, u32)> = HashMap::new();

    open_list.push(start_cell);
    parents.insert(start_cell, start_cell);

    while !open_list.is_empty() {
        let cur_cell = open_list.pop().unwrap();

        if cur_cell == goal_cell {
            path = trace_path(&goal_cell, &parents);
            break;
        }

        closed_list.insert(cur_cell);

        // Explore neighbors
        for nb_cell in get_neighbors_8_con(cur_cell, &gridmap) {
            if !closed_list.contains(&nb_cell) {
                parents.insert(nb_cell, cur_cell);
                open_list.push(nb_cell);
            }
        }
    }

    return MotionPlan {
        path: path,
        closed_list: closed_list,
    };
}
