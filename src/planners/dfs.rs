use std::collections::{HashMap, HashSet};

use super::planner_common::*;
use super::planner_base::*;
use crate::maps::gridmap::Gridmap;

// Depth-First Search
pub struct DFSPlanner{
    start: (u32, u32),
    goal: (u32, u32),
    gridmap: Gridmap,
}

impl Planner for DFSPlanner {

    fn new(gridmap: &Gridmap) -> DFSPlanner {
        DFSPlanner {
            start: (0, 0),
            goal: (0, 0),
            gridmap: gridmap.clone(),
        }
    }

    fn generate_plan(&self) -> MotionPlan {
        let mut path: Vec<(u32, u32)> = Vec::new();
    
        let mut open_list: Vec<(u32, u32)> = Vec::new();
        let mut closed_list: HashSet<(u32, u32)> = HashSet::new();
    
        let mut parents: HashMap<(u32, u32), (u32, u32)> = HashMap::new();
    
        open_list.push(self.start);
        parents.insert(self.start, self.start);
    
        while !open_list.is_empty() {
            let cur_cell = open_list.pop().unwrap();
    
            if cur_cell == self.goal {
                path = trace_path(&self.goal, &parents);
                break;
            }
    
            closed_list.insert(cur_cell);
    
            // Explore neighbors
            for nb_cell in get_neighbors_8_con(cur_cell, &self.gridmap) {
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

