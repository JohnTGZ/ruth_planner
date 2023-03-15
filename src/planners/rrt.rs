// use std::cmp::{Ordering, Reverse};
// use std::collections::{BinaryHeap, HashMap, HashSet};

// use super::planner_common::*;
// use crate::maps::gridmap::Gridmap;

// use rand::{Rng, thread_rng};
// use rand::rngs::ThreadRng;
// use rand::distributions::Uniform;

// // RRT

// struct RRT

// /// Sample from uniform distribution
// pub fn sample() {}

// /// Sample from uniform distribution within free space
// pub fn sample_free(rng: &ThreadRng, x_distr: &Uniform<i32>, goal_sample_rate: u32) -> (i32, i32) {
//     let x_rand: u32;
//     if rng.gen_range(0..100) > goal_sample_rate {
//         x_rand = rng.sample(x_distr)
//     }
//     else { // Sample goal point
//         x_rand = 
//     }
//     return x_rand
// }

// /// Returns the vertex in V that is "closest" to x in terms of a given distance fucntion
// pub fn nearest(gridmap, x) {}

// /// Returns the vertices in V that are contained in a ball of radius r centered at x
// pub fn near(gridmap, x, r) {}

// /// Given 2 points, x and y. Returns a point z such that z is "closer" to y than x is. 
// /// Point z returned by the function Steer will be such that z minimizes || z - y || while at the same time
// /// maintaining || z - x || <= eta, for a prescribed eta > 0.
// pub fn steer(x, y, eta: f32) {}

// /// Given 2 points x,x' in X, this function returns True if the line segment between x and x' lies in X_free, and false otherwise.
// pub fn obstacle_free() {}

// /// Retrieve a motion plan given start and goal location
// pub fn generate_plan(
//     start_cell: (u32, u32),
//     goal_cell: (u32, u32),
//     gridmap: &Gridmap,
// ) -> MotionPlan {
//     // Parameters
//     let max_iter = 1000;
//     let goal_sample_rate: u32 = 5; // Will sample goal 5% of the time
    
//     // Uniform distribution sampling
//     let mut rng = thread_rng(); 
//     let dist = rand::distributions::Uniform::new_inclusive(1, 100);

//     let mut path: Vec<(u32, u32)> = Vec::new();
//     let mut tree: Vec<(u32, u32)> = Vec::new();
//     tree.push(start_cell);

//     for i in 0..max_iter {
//         let x_rand = sample_free(&rng, &dist, goal_sample_rate);

//         let x_nearest = nearest(&tree, x_rand);

//         let x_new = steer(x_nearest, x_rand, expand_dist);

//         if obstacle_free(x_nearest, x_new) {}
//     }

//     return MotionPlan { path, closed_list };
// }
