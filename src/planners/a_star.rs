use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap, HashSet};

use super::planner_base::*;
use super::planner_common::*;
use crate::maps::gridmap::Gridmap;

// A Star
pub struct AStarPlanner {
    start: (u32, u32),
    goal: (u32, u32),
    gridmap: Gridmap,
}

impl Planner for AStarPlanner {
    fn new(gridmap: &Gridmap) -> AStarPlanner {
        AStarPlanner {
            start: (0, 0),
            goal: (0, 0),
            gridmap: gridmap.clone(),
        }
    }

    fn generate_plan(&mut self) -> MotionPlan {
        let mut path: Vec<(u32, u32)> = Vec::new();

        let mut open_list: BinaryHeap<Reverse<Cell2D>> = BinaryHeap::new();
        let mut closed_list: HashSet<(u32, u32)> = HashSet::new();
        let mut parents: HashMap<(u32, u32), (u32, u32)> = HashMap::new();

        // Movement cost from start to cell
        let mut g_cost: Vec<u32> = vec![std::u32::MAX; self.gridmap.get_cells().len()];

        g_cost[self.gridmap.xy_to_idx(self.start)] = 0;

        open_list.push(Reverse(Cell2D::new(
            self.start,
            get_l2_cost(self.start, self.goal),
        )));
        parents.insert(self.start, self.start);

        while !open_list.is_empty() {
            let cur_cell = open_list.pop().unwrap().0;

            if cur_cell.pos == self.goal {
                path = trace_path(&self.goal, &parents);
                break;
            }

            closed_list.insert(cur_cell.pos);

            // Explore neighbors
            for nb_cell_pos in get_neighbors_8_con(cur_cell.pos, &self.gridmap) {
                if closed_list.contains(&nb_cell_pos) {
                    continue;
                }

                let alt_g_cost = g_cost[self.gridmap.xy_to_idx(cur_cell.pos)]
                    + get_l2_cost(cur_cell.pos, nb_cell_pos);

                if alt_g_cost < g_cost[self.gridmap.xy_to_idx(nb_cell_pos)] {
                    g_cost[self.gridmap.xy_to_idx(nb_cell_pos)] = alt_g_cost;

                    parents.insert(nb_cell_pos, cur_cell.pos);
                    open_list.push(Reverse(Cell2D::new(
                        nb_cell_pos,
                        alt_g_cost + get_l2_cost(nb_cell_pos, self.goal),
                    )));
                }
            }
        }

        return MotionPlan { path, closed_list };
    }

    fn update_gridmap(&mut self, gridmap: &Gridmap) -> bool {
        self.gridmap = gridmap.clone();
        return true;
    }

    fn update_start(&mut self, start: (u32, u32)) -> bool {
        self.start = start;
        return true;
    }

    fn update_goal(&mut self, goal: (u32, u32)) -> bool {
        self.goal = goal;
        return true;
    }
}

#[derive(Eq)]
struct Cell2D {
    pub pos: (u32, u32),
    pub f_cost: u32,
}

impl Ord for Cell2D {
    fn cmp(&self, other: &Self) -> Ordering {
        self.f_cost.cmp(&other.f_cost)
    }
}

impl PartialOrd for Cell2D {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for Cell2D {
    fn eq(&self, other: &Self) -> bool {
        self.f_cost == other.f_cost
    }
}

impl Cell2D {
    fn new(pos: (u32, u32), f_cost: u32) -> Cell2D {
        Cell2D { pos, f_cost }
    }
}
