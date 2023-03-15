use super::planner_common::*;
use crate::maps::gridmap::Gridmap;

pub trait Planner {
    fn new(gridmap: &Gridmap) -> Self;

    /// Retrieve a motion plan given start and goal location
    fn generate_plan(&self) -> MotionPlan;

    // Default method definitions
    fn update_start_and_goal(&mut self, start: (u32, u32), goal: (u32, u32)) -> bool {
        self.update_start(start) && self.update_goal(goal)
    }

    fn update_gridmap(&mut self, gridmap: &Gridmap) -> bool;

    fn update_start(&mut self, start: (u32, u32)) -> bool;

    fn update_goal(&mut self, goal: (u32, u32)) -> bool;
}
