mod common;

use ruth_planner::*;

use maps::builder;

#[test]
fn test_build_gridmap_from_pgm() {
  let gridmap = builder::build_gridmap_from_pgm(
    "test_maps/test_maps/blank_map_with_obstacle.pgm").unwrap();

  //TODO: Test assert pixels in gridmap 
}

#[test]
fn test_build_gridmap_from_2d_arr(){

  let snake_arr_map: Vec<Vec<u8>> = common::create_snake_arr_map();

  let gridmap = builder::build_gridmap_from_2d_arr(&snake_arr_map).unwrap();

  //TODO: Test assert pixels in gridmap 

  // builder::save_gridmap(&gridmap, "test_maps/snake_map.pgm");
}
