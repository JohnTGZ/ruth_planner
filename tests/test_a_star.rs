mod common;

use ruth_planner::maps::builder;
use ruth_planner::a_star::AStarPlanner;
use ruth_planner::planners::planner_base::Planner;

#[test]
fn test_a_star_plan0() {
    let arr_map: Vec<Vec<u8>> = common::create_snake_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 4), (6, 4));

    let mut planner = AStarPlanner::new(&gridmap);
    planner.update_start_and_goal(start_cell, goal_cell);

    let motion_plan = planner.generate_plan();

    assert_ne!(motion_plan.path.len(), 0);
}

#[test]
fn test_a_star_plan1() {
    let arr_map: Vec<Vec<u8>> = common::create_maze_0_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 9), (9, 0));

    let mut planner = AStarPlanner::new(&gridmap);
    planner.update_start_and_goal(start_cell, goal_cell);

    let motion_plan = planner.generate_plan();

    assert_ne!(motion_plan.path.len(), 0);
}

#[test]
fn test_a_star_plan2() {
    let arr_map: Vec<Vec<u8>> = common::create_maze_1_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 9), (9, 0));

    let mut planner = AStarPlanner::new(&gridmap);
    planner.update_start_and_goal(start_cell, goal_cell);

    let motion_plan = planner.generate_plan();

    assert_eq!(motion_plan.path.len(), 0);
}

#[test]
fn test_a_star_plan3() {
    let arr_map: Vec<Vec<u8>> = common::create_maze_2_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 9), (9, 0));

    let mut planner = AStarPlanner::new(&gridmap);
    planner.update_start_and_goal(start_cell, goal_cell);

    let motion_plan = planner.generate_plan();
    
    assert_ne!(motion_plan.path.len(), 0);
}

#[test]
fn test_a_star_plan4() {
    let arr_map: Vec<Vec<u8>> = common::create_maze_2_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 9), (5, 5));

    let mut planner = AStarPlanner::new(&gridmap);
    planner.update_start_and_goal(start_cell, goal_cell);

    let motion_plan = planner.generate_plan();

    builder::plot_gridmap(
        &gridmap,
        &start_cell,
        &goal_cell,
        &motion_plan,
        "test_maps/planners/a_star/test_plan4.png",
        &30,
    );

    assert_ne!(motion_plan.path.len(), 0);
}

#[test]
fn test_a_star_plan_ros1() {
    let gridmap =
        builder::build_gridmap_from_pgm("test_maps/nav2_maps/turtlebot3_world.pgm").unwrap();

    let (start_cell, goal_cell) = ((165, 220), (230, 155));

    let mut planner = AStarPlanner::new(&gridmap);
    planner.update_start_and_goal(start_cell, goal_cell);

    let motion_plan = planner.generate_plan();

    builder::plot_gridmap(
        &gridmap,
        &start_cell,
        &goal_cell,
        &motion_plan,
        "test_maps/planners/a_star/test_plan_ros1.png",
        &2,
    );

    assert_ne!(motion_plan.path.len(), 0);
}

#[test]
fn test_a_star_plan_ros2() {
    let gridmap =
        builder::build_gridmap_from_pgm("test_maps/nav2_maps/ostc_map.pgm").unwrap();

    let (start_cell, goal_cell) = ((5, 5), (60, 35));

    let mut planner = AStarPlanner::new(&gridmap);
    planner.update_start_and_goal(start_cell, goal_cell);

    let motion_plan = planner.generate_plan();
    
    builder::plot_gridmap(
        &gridmap,
        &start_cell,
        &goal_cell,
        &motion_plan,
        "test_maps/planners/a_star/test_plan_ros2.png",
        &5,
    );

    assert_ne!(motion_plan.path.len(), 0);
}
