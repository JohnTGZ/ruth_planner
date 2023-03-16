use std::collections::{HashMap, HashSet, VecDeque};

use super::planner_base::*;
use super::planner_common::*;
use crate::maps::gridmap::Gridmap;

// Breadth-First Search
pub struct BFSPlanner {
    start: (u32, u32),
    goal: (u32, u32),
    gridmap: Gridmap,
}

impl Planner for BFSPlanner {
    fn new(gridmap: &Gridmap) -> BFSPlanner {
        BFSPlanner {
            start: (0, 0),
            goal: (0, 0),
            gridmap: gridmap.clone(),
        }
    }

    fn generate_plan(&mut self) -> MotionPlan {
        let mut path: Vec<(u32, u32)> = Vec::new();

        let mut open_list: VecDeque<(u32, u32)> = VecDeque::new();
        let mut closed_list: HashSet<(u32, u32)> = HashSet::new();

        let mut parents: HashMap<(u32, u32), (u32, u32)> = HashMap::new();

        open_list.push_back(self.start);
        parents.insert(self.start, self.start);

        while !open_list.is_empty() {
            let cur_cell = open_list.pop_front().unwrap();

            if cur_cell == self.goal {
                path = trace_path(&self.goal, &parents);
                break;
            }

            closed_list.insert(cur_cell);

            // Explore neighbors
            for nb_cell in get_neighbors_8_con(cur_cell, &self.gridmap) {
                if !closed_list.contains(&nb_cell) {
                    parents.insert(nb_cell, cur_cell);
                    open_list.push_back(nb_cell);
                }
            }
        }

        return MotionPlan::new(path, closed_list);
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
