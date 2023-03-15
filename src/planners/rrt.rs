use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap, HashSet};

use super::planner_base::*;
use super::planner_common::*;
use crate::maps::gridmap::Gridmap;

use rand::distributions::Uniform;
use rand::rngs::ThreadRng;
use rand::{thread_rng, Rng};

// RRT
pub struct RRTPlanner {
    start: (u32, u32),
    goal: (u32, u32),
    gridmap: Gridmap,

    start_f32: (f32, f32),
    goal_f32: (f32, f32),

    rng: ThreadRng,
    x_samp_distr: Uniform<f32>,
    y_samp_distr: Uniform<f32>,

    max_iter: usize,
    goal_sample_rate: u32,
    expand_dist: f32,
    path_resolution: f32,
}

impl RRTPlanner {
    pub fn set_parameters(
        &mut self,
        max_iter: usize,
        goal_sample_rate: u32,
        expand_dist: f32,
        path_resolution: f32,
    ) {
        self.goal_sample_rate = goal_sample_rate;
        self.expand_dist = expand_dist;
        self.max_iter = max_iter;
        self.path_resolution = path_resolution;
    }

    /// Sample from uniform distribution within free space
    pub fn sample(&mut self) -> Cell2D {
        let mut v_samp_pos: (f32, f32);

        if self.rng.gen_range(0..100) > self.goal_sample_rate {
            // Sample random point in map at random
            v_samp_pos = (
                self.rng.sample(self.x_samp_distr),
                self.rng.sample(self.y_samp_distr),
            );
            // Keep sampling until a traversable cell is obtained
            while !self.gridmap.xy_is_traversable_f32(v_samp_pos) {
                v_samp_pos = (
                    self.rng.sample(self.x_samp_distr),
                    self.rng.sample(self.y_samp_distr),
                );
            }
        } else {
            // Sample goal point
            v_samp_pos = self.goal_f32;
        }
        return Cell2D::new(v_samp_pos);
    }

    /// Returns the vertex in V that is "closest" to x in terms of a given distance fucntion
    pub fn nearest(&self, tree: &Vec<Cell2D>, v_samp: &Cell2D) -> Cell2D {
        // // Get distances of every vertex in the tree from sample vertex v_samp
        // let dists_from_v_samp: Vec<u32> =
        //     tree.into_iter().map(|&v| get_l2_cost(v, v_samp)).collect();
        // // TODO: Optimize here by using built-in methods to retrieve the min index
        // let min_idx =

        // TODO: Check if minimum index is extracted
        let min_idx = tree
            .iter()
            .enumerate()
            .map(|(idx, cell)| (idx, get_l2_cost_f32(cell.pos, v_samp.pos)))
            .max_by(|(_, a), (_, b)| b.partial_cmp(a).unwrap())
            .map(|(index, _)| index)
            .expect("Unable to get index of vertex within minimum distance to v_samp");

        return tree[min_idx].clone();
    }

    // /// Returns the vertices in V that are contained in a ball of radius r centered at x
    // pub fn near(&self, x, r) {}

    /// Given 2 points, x and y. Returns a point z such that z is "closer" to y than x is.
    /// Point z returned by the function Steer will be such that z minimizes || z - y || while at the same time
    /// maintaining || z - x || <= eta, for a prescribed eta > 0.
    pub fn steer(&self, v_x: &Cell2D, v_y: &Cell2D) -> Cell2D {
        let mut v_new = Cell2D::new(v_x.pos);

        let dist_to_y = get_l2_cost_f32(v_new.pos, v_y.pos);
        let theta = get_angle(v_x.pos, v_y.pos);

        v_new.path.push(v_new.pos);

        let tmp_expand_dist = match self.expand_dist {
            e_d if e_d <= dist_to_y => e_d,
            _ => dist_to_y,
        };

        let expand_itr = (tmp_expand_dist / self.path_resolution) as usize;

        for _ in 0..expand_itr {
            // Update position and append nodes to path
            v_new.pos.0 = self.path_resolution * theta.cos();
            v_new.pos.1 = self.path_resolution * theta.sin();
            v_new.path.push(v_new.pos);
        }

        let dist_xy = get_l2_cost_f32(v_new.pos, v_y.pos);

        // Add last position to path
        if dist_to_y <= self.path_resolution {
            v_new.path.push(v_y.pos);
            v_new.pos = v_y.pos;
        }

        return v_new;
    }

    /// Given 2 points x,x' in X, this function returns True if the line segment between x and x' lies in X_free, and false otherwise.
    pub fn check_collision() {}
}

impl Planner for RRTPlanner {
    fn new(gridmap: &Gridmap) -> RRTPlanner {
        // Uniform distribution sampling
        let x_samp_distr: Uniform<f32> =
            rand::distributions::Uniform::new(0.0, gridmap.get_width() as f32);
        let y_samp_distr: Uniform<f32> =
            rand::distributions::Uniform::new(0.0, gridmap.get_height() as f32);

        RRTPlanner {
            start: (0, 0),
            goal: (0, 0),
            gridmap: gridmap.clone(),

            start_f32: (0.0, 0.0),
            goal_f32: (0.0, 0.0),

            rng: thread_rng(),
            x_samp_distr: x_samp_distr,
            y_samp_distr: y_samp_distr,

            max_iter: 500,
            goal_sample_rate: 5, // Will sample goal 5% of the time
            expand_dist: 500.0,
            path_resolution: 1.0,
        }
    }

    fn generate_plan(&mut self) -> MotionPlan {
        let mut closed_list: HashSet<(u32, u32)> = HashSet::new();
        let mut path: Vec<(u32, u32)> = Vec::new();

        let mut tree: Vec<Cell2D> = Vec::new();
        tree.push(Cell2D::new(self.start_f32));

        for _ in 0..self.max_iter {
            let v_samp = self.sample();

            let v_nearest = self.nearest(&tree, &v_samp);

            let v_new = self.steer(&v_nearest, &v_samp);

            // TODO check if new node is traversable
            if get_l2_cost_f32(
                tree.last().unwrap().pos,
                self.goal_f32,
            ) <= self.expand_dist
            {
                let v_final = self.steer(&tree.last().unwrap(), &Cell2D::new(self.goal_f32));

                if self.check_collision(v_final){
                    self.generate_final_path();
                    break;
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
        self.start_f32 = (self.start.0 as f32, self.start.1 as f32);
        return true;
    }

    fn update_goal(&mut self, goal: (u32, u32)) -> bool {
        self.goal = goal;
        self.goal_f32 = (self.goal.0 as f32, self.goal.1 as f32);
        return true;
    }
}

#[derive(Clone)]
pub struct Cell2D {
    pub pos: (f32, f32),
    pub path: Vec<(f32, f32)>,
    pub parent: (f32, f32),
}

impl Cell2D {
    fn new(pos: (f32, f32)) -> Cell2D {
        Cell2D {
            pos: pos,
            path: Vec::new(),
            // TODO: Change parent to a valid None value
            parent: pos,
        }
    }
}
