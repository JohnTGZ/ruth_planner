// TODO: Add doc comments here

/// Cell is unknown part of map
pub const NO_INFORMATION: u8 = 255;
/// Cell contains lethal obstacle
pub const LETHAL_OBSTACLE: u8 = 254;
/// Cell that has collision with some obstacle if the robot center is in it
pub const INSCRIBED_INFLATED_OBSTACLE: u8 = 253;
/// Cell is in freespace
pub const FREE_SPACE: u8 = 0;

/// Gridmap is a map representation where the
/// origin is at the top left of the map by default.
/// The y-axis is positive downwards, and the x-axis
/// is positive rightwards.
#[derive(Clone)]
pub struct Gridmap {
    /// Number of cells along x-axis
    width: u32,
    /// Number of cells along y-axis
    height: u32,
    /// Resolution of the map in meter/cell
    resolution: f32,
    /// 1D vector of gridmap cells
    cells: Vec<u8>,
}

impl Gridmap {
    // Public functions

    /// Constructor
    // TODO: Change name of function from 'build' to 'new'
    pub fn new(width: u32, height: u32, resolution: f32) -> Gridmap {
        Gridmap {
            width: width,
            height: height,
            resolution: resolution,
            cells: vec![FREE_SPACE; (width * height) as usize],
        }
    }

    /// Get the width of the gridmap
    pub fn get_width(&self) -> u32 {
        return self.width;
    }

    /// Get the height of the gridmap
    pub fn get_height(&self) -> u32 {
        return self.height;
    }

    /// Get the resolution of the gridmap
    pub fn get_resolution(&self) -> f32 {
        return self.resolution;
    }

    /// Get total number of cells in gridmap
    pub fn get_cells(&self) -> &Vec<u8> {
        return &self.cells;
    }

    pub fn flip_y_u32(&self, y: u32) -> u32 {
      self.get_height() - 1 - y
    }

    /// Get centroid of cell in meters in gridmap coordinate system,
    /// with the origin in the bottom left corner.
    pub fn get_cell_centroid(&self, xy: (u32, u32)) -> (f32, f32) {
      (
        (xy.0 as f32) * self.resolution + (self.resolution/2.0),
        (xy.1 as f32) * self.resolution + (self.resolution/2.0),
      )
    }

    // TODO: Use generic types

    /// Set cell value at 1D index
    pub fn set_val_idx(&mut self, val: u8, idx: usize) {
        if !self.idx_in_map(idx) {
            panic!(
                "Tried to set cell index {} beyond gridmap size of {}",
                idx,
                self.get_cells().len()
            );
        }

        self.cells[idx] = val;
    }

    /// Get cell value at 1D index
    pub fn get_val_idx(&self, idx: usize) -> u8 {
        if !self.idx_in_map(idx) {
            panic!(
                "Tried to get cell index {} beyond gridmap size of {}",
                idx,
                self.get_cells().len()
            );
        }
        self.cells[idx]
    }

    /// Check if cell at 1D index is within the map
    pub fn idx_in_map(&self, idx: usize) -> bool {
        idx < self.cells.len()
    }

    /// Check if cell at 1D index is traversable
    pub fn idx_is_traversable(&self, idx: usize) -> bool {
        self.get_val_idx(idx) < INSCRIBED_INFLATED_OBSTACLE
    }

    /// Set cell value at 2D position
    pub fn set_val_xy(&mut self, val: u8, pos: (u32, u32)) {
        self.set_val_idx(val, self.xy_to_idx(pos))
    }

    /// Get cell value at 2D position
    pub fn get_val_xy(&self, pos: (u32, u32)) -> u8 {
        self.get_val_idx(self.xy_to_idx(pos))
    }

    /// Check if cell at 2D position is within the map
    pub fn xy_in_map(&self, pos: (u32, u32)) -> bool {
        self.idx_in_map(self.xy_to_idx(pos))
    }

    /// Check if cell at 2D position is traversable
    // TODO: Change xy_in_freespace to xy_is_traversable
    pub fn xy_is_traversable(&self, pos: (u32, u32)) -> bool {
        self.idx_is_traversable(self.xy_to_idx(pos))
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

    /// Convert from 2D position to 1D index
    pub fn xy_to_idx(&self, pos: (u32, u32)) -> usize {
        (pos.1 * (self.width) + pos.0) as usize
    }

    pub fn idx_to_xy(&self, idx: u32) -> (u32, u32) {
        (idx % self.width, idx / self.width)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// Test getting of gridmap attributes
    fn test_get_attributes() {
        let gridmap = Gridmap::new(4, 6, 0.05);

        assert_eq!(gridmap.get_width(), 4);
        assert_eq!(gridmap.get_height(), 6);
        assert_eq!(gridmap.get_resolution(), 0.05);
        assert_eq!(gridmap.get_cells().len(), 24);
    }

    #[test]
    /// Test conversion between 2D and 1D coordinates
    fn test_xy_idx_conversion() {
        let gridmap = Gridmap::new(4, 6, 0.05);

        assert_eq!(gridmap.xy_to_idx((0, 0)), 0);
        assert_eq!(gridmap.xy_to_idx((1, 0)), 1);
        assert_eq!(gridmap.xy_to_idx((0, 1)), 4);
        assert_eq!(gridmap.xy_to_idx((1, 1)), 5);
        assert_eq!(gridmap.xy_to_idx((2, 3)), 14);
        assert_eq!(gridmap.xy_to_idx((3, 5)), 23);
    }

    #[test]
    /// Test traversability of cells based on assigned values
    fn test_traversable_check() {
        let mut gridmap = Gridmap::new(4, 6, 0.05);

        for i in 0..gridmap.get_cells().len() {
            assert_eq!(gridmap.get_val_idx(i), FREE_SPACE);
            assert_eq!(gridmap.idx_is_traversable(i), true);
        }

        gridmap.set_val_idx(INSCRIBED_INFLATED_OBSTACLE, 4);
        assert_eq!(gridmap.get_val_idx(4), INSCRIBED_INFLATED_OBSTACLE);
        assert_eq!(gridmap.xy_is_traversable((0, 1)), false);
    }

    #[test]
    #[should_panic]
    /// Test that getting indices out of map should panic!
    fn test_get_in_map_check() {
        let gridmap = Gridmap::new(4, 6, 0.05);

        gridmap.get_val_idx(24);
    }

    #[test]
    #[should_panic]
    /// Test that setting indices out of map should panic!
    fn test_set_in_map_check() {
        let mut gridmap = Gridmap::new(4, 6, 0.05);

        gridmap.set_val_idx(LETHAL_OBSTACLE, 24);
    }
}
