mod common;

use ruth_planner::maps::builder;
use ruth_planner::planners::planner_base::Planner;
use ruth_planner::rrt::RRTPlanner;

// #[test]
// fn test_rrt_plan0() {
//     let arr_map: Vec<Vec<u8>> = common::create_snake_arr_map();
//     let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();

//     let (start_cell, goal_cell) = ((0, 9), (9, 0));
//     let (max_iter, 
//         goal_sample_rate, 
//         expand_dist, 
//         path_resolution) = 
//         (   10, 
//             5, 
//             2.0, 
//             1.0);
//     let mut planner = RRTPlanner::new(&gridmap);
//     planner.update_start_and_goal(start_cell, goal_cell);
//     planner.set_parameters(max_iter, goal_sample_rate, expand_dist, path_resolution);

//     let motion_plan = planner.generate_plan();

//     builder::plot_gridmap(
//         &gridmap,
//         &start_cell,
//         &goal_cell,
//         &motion_plan,
//         "test_maps/planners/rrt/test_plan0.png",
//         &30,
//     );

//     // assert_ne!(motion_plan.path.len(), 0);
// }

#[test]
fn test_rrt_plan4() {
    let arr_map: Vec<Vec<u8>> = common::create_maze_2_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 9), (5, 5));

    let (max_iter, 
        goal_sample_rate, 
        expand_dist, 
        path_resolution) = 
        (   500, 
            5, 
            1.0, 
            0.5);
    let mut planner = RRTPlanner::new(&gridmap);
    planner.update_start_and_goal(start_cell, goal_cell);
    planner.set_parameters(max_iter, goal_sample_rate, expand_dist, path_resolution);

    let motion_plan = planner.generate_plan();

    builder::plot_gridmap(
        &gridmap,
        &start_cell,
        &goal_cell,
        &motion_plan,
        "test_maps/planners/rrt/test_plan4.png",
        &20,
    );

}