mod common;

use ruth_planner::maps::builder;
use ruth_planner::planners::planner_base::Planner;
use ruth_planner::rrt::RRTPlanner;

#[test]
fn test_rrt_plan4() {
    let arr_map: Vec<Vec<u8>> = common::create_maze_2_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 9), (7, 2));

    let (max_iter, 
        goal_sample_rate, 
        expand_dist, 
        path_resolution) = 
        (   500, 
            1, 
            0.5, 
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

    assert_ne!(motion_plan.path_f32.len(), 0);
}


#[test]
fn test_rrt_plan_ros1() {
    let gridmap =
        builder::build_gridmap_from_pgm("test_maps/nav2_maps/turtlebot3_world.pgm").unwrap();

    let (start_cell, goal_cell) = ((165, 220), (230, 155));

    let (max_iter, 
        goal_sample_rate, 
        expand_dist, 
        path_resolution) = 
        (   1000, 
            5, 
            3.0, 
            1.0);
    let mut planner = RRTPlanner::new(&gridmap);
    planner.update_start_and_goal(start_cell, goal_cell);
    planner.set_parameters(max_iter, goal_sample_rate, expand_dist, path_resolution);


    let motion_plan = planner.generate_plan();

    builder::plot_gridmap(
        &gridmap,
        &start_cell,
        &goal_cell,
        &motion_plan,
        "test_maps/planners/rrt/test_plan_ros1.png",
        &2,
    );

    // assert_ne!(motion_plan.path.len(), 0);
}


#[test]
fn test_rrt_plan_ros2() {
    let gridmap =
        builder::build_gridmap_from_pgm("test_maps/nav2_maps/ostc_map.pgm").unwrap();

    let (start_cell, goal_cell) = ((5, 5), (60, 35));

    let (max_iter, 
        goal_sample_rate, 
        expand_dist, 
        path_resolution) = 
        (   10000, 
            5, 
            7.5, 
            1.0);
    let mut planner = RRTPlanner::new(&gridmap);
    planner.update_start_and_goal(start_cell, goal_cell);
    planner.set_parameters(max_iter, goal_sample_rate, expand_dist, path_resolution);

    let motion_plan = planner.generate_plan();
    
    builder::plot_gridmap(
        &gridmap,
        &start_cell,
        &goal_cell,
        &motion_plan,
        "test_maps/planners/rrt/test_plan_ros2.png",
        &2,
    );

    // assert_ne!(motion_plan.path.len(), 0);
}


#[test]
fn test_rrt_plan_ros3() {
    let gridmap =
        builder::build_gridmap_from_pgm("test_maps/nav2_maps/ostc_map.pgm").unwrap();

    let unique_vals = gridmap.get_unique_values();

    println!("For ostc_map:");
    for val in unique_vals {
        println!("  {}", val);
    }

    // assert_ne!(motion_plan.path.len(), 0);
}