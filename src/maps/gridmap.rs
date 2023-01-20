
// TODO: Add doc comments here

const NO_INFORMATION: u8 = 255; // Unknown part of map
const LETHAL_OBSTACLE: u8 = 254; // Threshold for lethal obstacle
const INSCRIBED_INFLATED_OBSTACLE: u8 = 253; // Indicates taht robot is certainly in collision with some obstacle if the robot center is in this cell
const FREE_SPACE: u8 = 0;

/// Gridmap is a map representation where the
/// origin is at the bottom left of the map by default
/// and the y-axis is positive upwards, and the x-axis
/// is positive rightwards.
#[derive(Clone)]
pub struct Gridmap{
  width: u32,
  height: u32,
  cells: Vec<u8>, 
}

impl Gridmap{
  // Public functions

  /// Constructor
  pub fn build(width: u32, height: u32) -> Gridmap {
    Gridmap{
      width: width,
      height: height,
      cells: vec![255; (width * height) as usize],
    }
  }

  /// Get the width of the costmap
  pub fn get_width(&self) -> u32 {
    return self.width;
  }

  /// Get the height of the costmap
  pub fn get_height(&self) -> u32 {
    return self.height;
  }

  /// Get the size of the costmap
  pub fn get_size(&self) -> usize {
    return self.cells.len();
  }

  /// Flip y coordinate and return tuple of u32
  pub fn coord_flip_y_u32(&self, xy: (u32, u32)) -> (u32, u32){
    (xy.0, self.get_height() - 1 - xy.1)
  }

  /// Flip y coordinate and return tuple of f64
  pub fn coord_flip_y_f64(&self, xy: (u32, u32)) -> (f64, f64){
    (xy.0 as f64, (self.get_height() - 1 - xy.1) as f64)
  }

  /// Set value at 1D index
  pub fn set_val_idx(&mut self, val: u8, idx: usize) -> bool {
    if !self.idx_in_map(idx){
      return false;
    }
    self.cells[idx] = val;
    return true;
  }

  /// Set value at 2D position
  pub fn set_val_xy(&mut self, val: u8, pos: (u32, u32)) -> bool {
    self.set_val_idx(val, self.xy_to_idx(pos))
  }

  /// Get value at 1D index
  pub fn get_val_idx(&self, idx: usize) -> u8 {
    self.cells[idx]
  }

  /// Get value at 2D position
  pub fn get_val_xy(&self, pos: (u32, u32)) -> u8 {
    self.get_val_idx(self.xy_to_idx(pos))
  }

  /// Check if cell at 1D index is within the map
  pub fn idx_in_map(&self, idx: usize) -> bool {
    idx < self.get_size()
  }

  /// Check if 2D position is in the map
  pub fn xy_in_map(&self, pos: (u32, u32)) -> bool {
    self.idx_in_map(self.xy_to_idx(pos))
  }

  /// Check if cell at 1D index is in freespace
  pub fn idx_in_freespace(&self, idx: usize) -> bool {
    self.get_val_idx(idx) < INSCRIBED_INFLATED_OBSTACLE
  }

  /// Check if cell at 2D position is in freespace
  pub fn xy_in_freespace(&self, pos: (u32, u32)) -> bool {
    self.idx_in_freespace(self.xy_to_idx(pos))
  }

  // /// Render the gripmap
  // pub fn render(&self) -> () {
  //   for y in (0..self.height).rev() {
  //     for x in 0..self.width {
  //       if self.get_val_xy((x,y)) < INSCRIBED_INFLATED_OBSTACLE{
  //         print!("o");
  //       }
  //       else {
  //         print!("x");
  //       }
  //     }
  //     print!("\n");
  //   }
  // }

  // Private functions

  /// Convert from 2D position to 
  pub fn xy_to_idx(&self, pos: (u32, u32)) -> usize{
    (pos.1 * (self.width) + pos.0) as usize
  }

  // fn idx_to_xy(&self, idx: u32) -> (u32, u32){
  //   (idx % self.width, idx / self.width)
  // }
  
}

// TODO Write tests for gridmap xy to idx conversion and freespace checking

// #[cfg(test)]
// mod tests {
//   use super::*;

//   #[test]
//   fn build_gridmap(){
//     let gridmap = Gridmap::build(5, 5);

//   } 
// }