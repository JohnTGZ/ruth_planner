mod common;

use ruth_planner::*;

use maps::builder;
use planners;

#[test]
fn test_bfs_plan0() {
    let arr_map: Vec<Vec<u8>> = common::create_snake_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 4), (6, 4));

    let motion_plan = planners::bfs::generate_plan(start_cell, goal_cell, &gridmap);

    builder::plot_gridmap(
        &gridmap,
        &start_cell,
        &goal_cell,
        &motion_plan,
        "test_maps/planners/bfs/test_plan0.png",
        &45,
    );

    assert_ne!(motion_plan.path.len(), 0);
}

#[test]
fn test_bfs_plan1() {
    let arr_map: Vec<Vec<u8>> = common::create_maze_0_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 9), (9, 0));

    let motion_plan = planners::bfs::generate_plan(start_cell, goal_cell, &gridmap);

    // builder::plot_gridmap(
    //     &gridmap,
    //     &start_cell,
    //     &goal_cell,
    //     &motion_plan,
    //     "test_maps/planners/bfs/test_plan1.png",
    //     &30,
    // );

    assert_ne!(motion_plan.path.len(), 0);
}

#[test]
fn test_bfs_plan2() {
    let arr_map: Vec<Vec<u8>> = common::create_maze_1_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 9), (9, 0));

    let motion_plan = planners::bfs::generate_plan(start_cell, goal_cell, &gridmap);

    assert_eq!(motion_plan.path.len(), 0);
}

#[test]
fn test_bfs_plan3() {
    let arr_map: Vec<Vec<u8>> = common::create_maze_2_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 9), (9, 0));

    let motion_plan = planners::bfs::generate_plan(start_cell, goal_cell, &gridmap);

    // builder::plot_gridmap(
    //     &gridmap,
    //     &start_cell,
    //     &goal_cell,
    //     &motion_plan,
    //     "test_maps/planners/bfs/test_plan3.png",
    //     &30,
    // );

    assert_ne!(motion_plan.path.len(), 0);
}

#[test]
fn test_bfs_plan4() {
    let arr_map: Vec<Vec<u8>> = common::create_maze_2_arr_map();
    let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
    let (start_cell, goal_cell) = ((0, 9), (5, 5));

    let motion_plan = planners::bfs::generate_plan(start_cell, goal_cell, &gridmap);

    builder::plot_gridmap(
        &gridmap,
        &start_cell,
        &goal_cell,
        &motion_plan,
        "test_maps/planners/bfs/test_plan4.png",
        &45,
    );

    assert_ne!(motion_plan.path.len(), 0);
}
