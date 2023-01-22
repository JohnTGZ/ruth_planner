use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap, HashSet};

use super::planner_common::*;
use crate::maps::gridmap::Gridmap;

// Djikstra

/// Retrieve a motion plan given start and goal location
pub fn generate_plan(
    start_cell: (u32, u32),
    goal_cell: (u32, u32),
    gridmap: &Gridmap,
) -> MotionPlan {
    let mut path: Vec<(u32, u32)> = Vec::new();

    let mut open_list: BinaryHeap<Reverse<Cell2D>> = BinaryHeap::new();
    let mut closed_list: HashSet<(u32, u32)> = HashSet::new();
    let mut parents: HashMap<(u32, u32), (u32, u32)> = HashMap::new();

    let mut g_cost: Vec<u32> = vec![std::u32::MAX; gridmap.get_cells().len()];

    g_cost[gridmap.xy_to_idx(start_cell)] = 0;
    open_list.push(Reverse(Cell2D::new(
        start_cell,
        g_cost[gridmap.xy_to_idx(start_cell)],
    )));
    parents.insert(start_cell, start_cell);

    while !open_list.is_empty() {
        let cur_cell = open_list.pop().unwrap().0;

        if cur_cell.pos == goal_cell {
            path = trace_path(&goal_cell, &parents);
            break;
        }
        closed_list.insert(cur_cell.pos);

        // Explore neighbors
        for nb_cell_pos in get_neighbors_8_con(cur_cell.pos, &gridmap) {

            if closed_list.contains(&nb_cell_pos) {
                continue;
            }

            let alt_g_cost =
                g_cost[gridmap.xy_to_idx(cur_cell.pos)] + get_l2_cost(cur_cell.pos, nb_cell_pos);

            if alt_g_cost < g_cost[gridmap.xy_to_idx(nb_cell_pos)] {
                g_cost[gridmap.xy_to_idx(nb_cell_pos)] = alt_g_cost;
                parents.insert(nb_cell_pos, cur_cell.pos);
                open_list.push(Reverse(Cell2D::new(
                    nb_cell_pos,
                    g_cost[gridmap.xy_to_idx(nb_cell_pos)],
                )));
            }
        }
    }

    return MotionPlan { path, closed_list };
}

#[derive(Eq)]
struct Cell2D {
    pub pos: (u32, u32),
    pub g_cost: u32,
}

impl Ord for Cell2D {
    fn cmp(&self, other: &Self) -> Ordering {
        self.g_cost.cmp(&other.g_cost)
    }
}

impl PartialOrd for Cell2D {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for Cell2D {
    fn eq(&self, other: &Self) -> bool {
        self.g_cost == other.g_cost
    }
}

impl Cell2D {
    fn new(pos: (u32, u32), g_cost: u32) -> Cell2D {
        Cell2D { pos, g_cost }
    }
}
