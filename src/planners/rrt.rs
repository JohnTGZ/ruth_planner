// use std::cmp::{Ordering, Reverse};
use std::collections::{HashSet, HashMap};

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
    pub fn sample(&mut self) -> Node2D {
        let mut v_samp_pos: (f32, f32);

        if self.rng.gen_range(0..100) > self.goal_sample_rate {
            // Sample random point in map at random
            v_samp_pos = (
                self.rng.sample(self.x_samp_distr),
                self.rng.sample(self.y_samp_distr),
            );
        } else {
            // Sample goal point
            v_samp_pos = self.goal_f32;
        }
        return Node2D::new(v_samp_pos);
    }

    /// Get the index of the nearest node in the tree to v_samp
    pub fn get_nearest_node_idx(&self, tree: &Tree, v_samp: &Node2D) -> usize {
        tree.get_nearest_node_idx(&v_samp)
    }

    /// Given 2 points, x and y. Returns a point z such that z is "closer" to y than x is.
    /// Point z returned by the function Steer will be such that z minimizes || z - y || while at the same time
    /// maintaining || z - x || <= eta, for a prescribed eta > 0.
    pub fn steer(&self, v_x: &Node2D, v_y: &Node2D) -> Node2D {
        let mut v_new = Node2D::new(v_x.pos);

        let dist_to_y = get_l2_cost_f32(v_new.pos, v_y.pos);
        let theta_xy = get_angle(v_x.pos, v_y.pos);

        v_new.path.push(v_new.pos);

        let tmp_expand_dist = match self.expand_dist {
            e_d if (e_d <= dist_to_y) => e_d,
            _ => dist_to_y,
        };

        let expand_itr = (tmp_expand_dist / self.path_resolution) as usize;

        println!("  dist_to_y: {dist_to_y}");
        println!("  tmp_expand_dist: {tmp_expand_dist}");
        println!("  expand_itr: {expand_itr}");

        for _ in 0..expand_itr {
            // Update position and append nodes to path
            v_new.pos.0 += self.path_resolution * theta_xy.cos();
            v_new.pos.1 += self.path_resolution * theta_xy.sin();
            v_new.path.push(v_new.pos);
        }

        let dist_to_y = get_l2_cost_f32(v_new.pos, v_y.pos);

        // Add last position to path
        if dist_to_y <= self.path_resolution {
            v_new.path.push(v_y.pos);
            v_new.pos = v_y.pos;
        }

        return v_new;
    }

    /// Given 2 points x,x' in X, this function returns True if the line segment between x and x' lies in X_free, and false otherwise.
    pub fn check_collision() {}

    pub fn generate_path(&self, tree: &Tree) -> Vec<(f32, f32)>{
        let mut path: Vec<(f32, f32)> = Vec::new();
        path.push(self.goal_f32);
        
        let mut cur_idx = tree.nodes.len() - 1;
        while let Some(parent_idx) = tree.get_parents(&cur_idx) {
            path.push(tree.get_node(cur_idx).pos);
            cur_idx = parent_idx;
        }
        path.push(tree.get_node(cur_idx).pos);

        return path
    }

    pub fn generate_closed_list(&self, tree: &Tree) -> Vec<(f32, f32)> {
        let mut closed_list: Vec<(f32, f32)> = Vec::new();
        for node in &tree.nodes {
            closed_list.push(node.pos);
        }
        closed_list
    }


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
            expand_dist: 2.0,
            path_resolution: 1.0,
        }
    }


    fn generate_plan(&mut self) -> MotionPlan {
        let mut path: Vec<(f32, f32)> = Vec::new();

        let mut tree = Tree::new();
        tree.add_node(&Node2D::new(self.start_f32), None);

        for i in 0..self.max_iter {
            println!("Itr {i}");

            let v_rand = self.sample();

            let v_nearest_idx = self.get_nearest_node_idx(&tree, &v_rand);

            let v_new = self.steer(tree.get_node(v_nearest_idx), &v_rand);

            // TODO check if new node is traversable
            if self.gridmap.xy_is_traversable_f32(v_new.pos) {
                tree.add_node(&v_new, Some(v_nearest_idx));
                println!("  Added v_new");
            }

            println!("  v_nearest: {:?} -> v_rand {:?} => v_new: {:?}", 
                &tree.nodes[v_nearest_idx].pos, v_rand.pos, v_new.pos);

            // If distance from vertex to goal <= expand_dist
            if get_l2_cost_f32(tree.nodes.last().unwrap().pos, self.goal_f32) <= self.expand_dist {
                // let v_final = self.steer(&tree.last().unwrap(), &Node2D::new(self.goal_f32));
                
                println!(" PATH FOUND! ");
                path = self.generate_path(&tree);
                break;

                // if self.check_collision(v_final){
                //     self.generate_final_path();
                //     break;
                // }
            }
        }

        // println!(" GENERATING PATH ANYWAY! ");
        // path = self.generate_path(&tree);

        let closed_list = self.generate_closed_list(&tree);

        return MotionPlan::new_f32(path, closed_list);
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

pub struct Tree {
    pub nodes: Vec<Node2D>,
    pub parents: HashMap<usize, Option<usize>>,
}

impl Tree {
    fn new() -> Tree {
        Tree {
            nodes: Vec::new(),
            parents: HashMap::new(),
        }
    }

    fn get_parents(&self, idx: &usize) -> Option<usize> {
        self.parents[idx]
    }

    fn add_node(&mut self, cell: &Node2D, parent: Option<usize>) {
        let new_cell_idx = self.nodes.len();

        self.nodes.push(cell.clone());
        self.parents.insert(new_cell_idx, parent);
    }

    fn get_node(&self, idx: usize) -> &Node2D {
        &self.nodes[idx]
    }

    fn get_nearest_node_idx(&self, cell_from: &Node2D) -> usize {
        
        // // Get distances of every vertex in the tree from sample vertex v_samp
        // let dists_from_v_samp: Vec<u32> =
        //     tree.into_iter().map(|&v| get_l2_cost(v, v_samp)).collect();
        // // TODO: Optimize here by using built-in methods to retrieve the min index
        // let min_idx =

        return self.nodes
            .iter()
            .enumerate()
            .map(|(idx, cell)| (idx, get_l2_cost_f32(cell.pos, cell_from.pos)))
            .max_by(|(_, a), (_, b)| b.partial_cmp(a).unwrap())
            .map(|(index, _)| index)
            .expect("Unable to get index of vertex within minimum distance of given cell");
    }
}

#[derive(Clone, Debug)]
pub struct Node2D {
    pub pos: (f32, f32),
    pub path: Vec<(f32, f32)>,
}

impl Node2D {
    fn new(pos: (f32, f32)) -> Node2D {
        Node2D {
            pos: pos,
            path: Vec::new(),
        }
    }
}
