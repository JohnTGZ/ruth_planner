const O: u8 = 0;// freespace
const X: u8 = 255;// occupied

pub fn setup() {
  // some setup code, like creating required files/directories, starting
  // servers, etc.
}

pub fn create_snake_arr_map() -> Vec<Vec<u8>> {
  return vec![
    vec![O, O, O, X, O, O, O],
    vec![O, X, O, X, O, X, O],
    vec![O, X, O, X, O, X, O],
    vec![O, X, O, X, O, X, O],
    vec![O, X, O, O, O, X, O],
  ];
}

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

pub fn create_maze_1_arr_map() -> Vec<Vec<u8>> {
  return vec![
    vec![O, O, O, X, O, X, O, O, X, O],
    vec![O, X, O, O, O, O, O, O, X, O],
    vec![O, X, X, X, X, X, X, O, X, O],
    vec![O, O, O, O, X, O, X, O, X, X],
    vec![O, O, O, O, O, O, X, O, O, O],
    vec![X, X, X, O, O, O, X, X, X, X],
    vec![O, O, O, O, X, O, X, O, O, O],
    vec![O, O, X, O, X, O, X, O, O, O],
    vec![X, X, X, O, X, O, X, O, O, O],
    vec![O, O, O, O, X, O, O, O, O, O],
  ];
}

pub fn create_maze_2_arr_map() -> Vec<Vec<u8>> {
  return vec![
    vec![O, O, O, O, O, O, O, O, O, O],
    vec![O, O, O, O, O, O, O, O, O, O],
    vec![O, O, O, O, O, O, O, O, O, O],
    vec![O, O, O, X, X, X, X, O, O, O],
    vec![O, O, O, O, O, O, X, O, O, O],
    vec![O, O, O, O, O, O, X, O, O, O],
    vec![O, O, O, O, O, O, X, O, O, O],
    vec![O, O, O, O, O, O, O, O, O, O],
    vec![O, O, O, O, O, O, O, O, O, O],
    vec![O, O, O, O, O, O, O, O, O, O],
  ];
}