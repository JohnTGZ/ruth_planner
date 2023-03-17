// TODO: Add doc comments here

use std::collections::HashSet;

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
    pub fn get_cell_centroid(&self, xy: (f32, f32)) -> (f32, f32) {
        (
            (xy.0 as f32) * self.resolution + (self.resolution / 2.0),
            (xy.1 as f32) * self.resolution + (self.resolution / 2.0),
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
    /// Returns true if cell is within map, and false otherwise. 
    pub fn idx_in_map(&self, idx: usize) -> bool {
        idx < self.cells.len()
    }

    /// Check if cell at 1D index is traversable
    pub fn idx_is_traversable(&self, idx: usize, max_cost: &u8) -> bool {
        self.get_val_idx(idx) < *max_cost
    }

    /// Set cell value at 2D position
    pub fn set_val_xy(&mut self, val: u8, pos: (u32, u32)) {
        self.set_val_idx(val, self.xy_to_idx(pos))
    }

    /// Get cell value at 2D position
    pub fn get_val_xy(&self, pos: (u32, u32)) -> u8 {
        self.get_val_idx(self.xy_to_idx(pos))
    }

    /// Check if cell at 2D position is within the map. 
    /// Returns true if cell is within map, and false otherwise. 
    pub fn xy_in_map(&self, pos: (u32, u32)) -> bool {
        self.idx_in_map(self.xy_to_idx(pos))
    }

    /// Check if cell at 2D position is traversable
    pub fn xy_is_traversable(&self, pos: &(u32, u32), max_cost: &u8) -> bool {
        self.idx_is_traversable(self.xy_to_idx(*pos), max_cost)
    }

    pub fn xy_is_traversable_f32(&self, pos: &(f32, f32), max_cost: &u8) -> bool {
        let pos_u32 = (pos.0.round() as u32, pos.1.round() as u32);
        if !self.xy_in_map(pos_u32){
            return false;
        }
        self.xy_is_traversable(&pos_u32, max_cost)
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

    /// Render the gripmap
    pub fn get_unique_values(&self) -> Vec<u8> {
        self.cells.clone().into_iter()
            .collect::<HashSet<_>>()
            .into_iter()
            .collect()
    }

    /// Convert from 2D position to 1D index
    pub fn xy_to_idx(&self, pos: (u32, u32)) -> usize {
        (pos.1 * (self.width) + pos.0) as usize
    }

    /// Convert from 1D index to 2D position
    pub fn idx_to_xy(&self, idx: u32) -> (u32, u32) {
        (idx % self.width, idx / self.width)
    }

    /// Checks if the line from pos_a to pos_b is in collision with an obstacle
    /// Returns true if obstacle is within the line, and false otherwise. 
    pub fn check_collision_line_f32(&self, pos_a: &(f32, f32), pos_b: &(f32, f32), max_cost: &u8) -> bool {

        let (dx, dy) = (pos_b.0 - pos_a.0, pos_b.1 - pos_a.1);

        // Starting and ending position is the same
        if dx.abs() <= f32::EPSILON && dy.abs() <= f32::EPSILON {
            return false;
        }

        // x and y increment
        let x_inc = (1.0 as f32).copysign(dx); 
        let y_inc = (1.0 as f32).copysign(dy); 

        // Get sign of gradient
        let m_sign = match (dx,dy) {
            (dx,dy) if (dx.is_sign_negative() != dy.is_sign_negative()) => -1.0,
            _ => 1.0
        };

        // Get value of gradient (set to be between 0 and 1)
        let mut m = f32::INFINITY;
        if dx.abs() > f32::EPSILON { //Only calculate m if x is a positive value
            m = dy/dx;
            if dy.abs() > dx.abs(){ // if absolute value of m > 1, then invert it
                m = 1.0/m;
            }
        }

        // Value of x and y used to track if it has reached position b
        let (mut x, mut x_track) = (pos_a.0, pos_a.0);
        let (mut y, mut y_track) = (pos_a.1, pos_a.1);

        if !self.xy_is_traversable_f32(&(x, y), max_cost) {
            return true;
        }
        println!("  ({}, {})", x,y);

        if dx.abs() <= f32::EPSILON {
            let x_b = (pos_b.0).abs();

            while x_track < x_b {
                x_track += x_inc.abs();
                x += x_inc;
                if !self.xy_is_traversable_f32(&(x, y), max_cost) {
                    return true;
                }
            }

            println!("  ({}, {})", x,y);
        }
        else if dy.abs() <= f32::EPSILON {
            let y_b = (pos_b.1).abs();

            while y_track < y_b {
                y_track += y_inc.abs();
                y += y_inc;
                if !self.xy_is_traversable_f32(&(x, y), max_cost) {
                    return true;
                }
            }

            println!("  ({}, {})", x,y);
        }
        else if dy.abs() > dx.abs() { // 1 < gradient < INF
            let y_b = (pos_b.1).abs();

            let mut err = 0.0;
            
            while y_track < y_b {
                y_track += y_inc.abs();
                y += y_inc;
                if m_sign * (err + m) < 0.5 {
                    err += m;
                }
                else {
                    err += m - m_sign;
                    x += x_inc;
                }
                if !self.xy_is_traversable_f32(&(x, y), max_cost) {
                    return true;
                }
                println!("  ({}, {})", x,y);
            }
        }
        else { // 0 < gradient < 1
            let x_b = (pos_b.0).abs();

            let mut err = 0.0;
            
            while x_track < x_b {
                x_track += x_inc.abs();
                x += x_inc;
                if m_sign * (err + m) < 0.5 {
                    err += m;
                }
                else {
                    err += m - m_sign;
                    y += y_inc;
                }
                if !self.xy_is_traversable_f32(&(x, y), max_cost) {
                    return true;
                }
                println!("  ({}, {})", x,y);
            }
        }

        return false;
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
            assert_eq!(gridmap.idx_is_traversable(i, &INSCRIBED_INFLATED_OBSTACLE), true);
        }

        gridmap.set_val_idx(INSCRIBED_INFLATED_OBSTACLE, 4);
        assert_eq!(gridmap.get_val_idx(4), INSCRIBED_INFLATED_OBSTACLE);
        assert_eq!(gridmap.xy_is_traversable(&(0, 1), &INSCRIBED_INFLATED_OBSTACLE), false);
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

    #[test]
    fn test_check_collision_line_f32() {
        let mut gridmap = Gridmap::new(10, 10, 0.05);
        let mut collision = gridmap.check_collision_line_f32(&(0.0, 0.0), &(9.0, 9.0), &INSCRIBED_INFLATED_OBSTACLE);
        assert_eq!(collision, false);

        gridmap.set_val_xy(255, (3,3));
        gridmap.set_val_xy(255, (4,2));
        gridmap.set_val_xy(255, (3,2));
        let mut collision = gridmap.check_collision_line_f32(&(0.0, 0.0), &(4.0, 3.0), &INSCRIBED_INFLATED_OBSTACLE);

        assert_eq!(collision, true);
    }
}
