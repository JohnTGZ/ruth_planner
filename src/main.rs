use ruth_planner::maps::builder;
use ruth_planner::planners;

const O: u8 = 0;// freespace
const X: u8 = 255;// occupied

pub fn create_maze_0_arr_map() -> Vec<Vec<u8>> {
  return vec![
    vec![O, O, O, X, O, X, O, O, X, O],
    vec![O, X, O, O, O, O, O, O, X, O],
    vec![O, X, X, X, X, X, X, O, X, O],
    vec![O, O, O, O, X, O, X, O, X, O],
    vec![O, O, O, O, O, O, X, O, O, O],
    vec![X, X, X, O, O, O, X, X, X, X],
    vec![O, O, O, O, X, O, X, O, O, O],
    vec![O, O, X, O, X, O, X, O, O, O],
    vec![X, X, X, O, X, O, X, O, O, O],
    vec![O, O, O, O, X, O, O, O, O, O],
  ];
}


fn main() {
  let arr_map: Vec<Vec<u8>> = create_maze_0_arr_map();
  let gridmap = builder::build_gridmap_from_2d_arr(&arr_map).unwrap();
  let (start_cell, goal_cell) = ((0, 9), (9, 0));

  let motion_plan = 
  planners::dijkstra::generate_plan(start_cell, goal_cell, &gridmap);

  builder::plot_gridmap(&gridmap, &start_cell, &goal_cell, &motion_plan, "test_maps/planners/dijkstra/test_plan1.png");
}
