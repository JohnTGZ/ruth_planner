use std::path::Path;

use image::{GenericImageView, GrayImage};
use plotters::prelude::*;

use super::gridmap::*;
use crate::planners::planner_common::MotionPlan;

/// Builds a Gridmap struct from a PGM Image
pub fn build_gridmap_from_pgm(file_path: &str) -> Option<Gridmap> {
  let img = 
    image::open(&Path::new(file_path)).unwrap();

  let (width, height) = img.dimensions();

  let mut gridmap = Gridmap::build(width, height);

  let img_grayscale = img.as_luma8().unwrap();
  
  // 0 is black, 255 is white (free space)
  for (x, y, pixel) in img_grayscale.enumerate_pixels(){
    gridmap.set_val_xy(pixel.0[0], (x, y));
  }

  Some(gridmap)
}

/// Builds a gridmap struct from a 2d vector of u8 values
pub fn build_gridmap_from_2d_arr(arr_map: &Vec<Vec<u8>>) -> Option<Gridmap> {

  if arr_map.len() == 0 || arr_map[0].len() == 0{
    return None;
  }

  let (width, height) = (arr_map[0].len() as u32, arr_map.len() as u32);

  let mut gridmap = 
    Gridmap::build(width, height);

  for y in 0..height{
    for x in 0..width{
      gridmap.set_val_xy(arr_map[y as usize][x as usize], (x, y));
    }
  }

  Some(gridmap)
}

/// Saves gridmap as PGM file
pub fn save_gridmap(gridmap: &Gridmap, file_path: &str) -> () {
  let mut imgbuf = 
    GrayImage::new(gridmap.get_width(), gridmap.get_height());

  for (x, y, pixel) in imgbuf.enumerate_pixels_mut() {
    *pixel = image::Luma([gridmap.get_val_xy((x,y))]);
  }

  imgbuf.save(&Path::new(file_path)).unwrap();
}

/// Plot gridmap
pub fn plot_gridmap(
  gridmap: &Gridmap, 
  start_cell: &(u32, u32), goal_cell: &(u32, u32),
  motion_plan: &MotionPlan,
  file_path: &str) 
-> Result<(), Box<dyn std::error::Error>> {

  // TODO: Resolution should scale with gridmap width and height
  let root = 
  BitMapBackend::new(file_path, (640, 640)).into_drawing_area();

  root.fill(&WHITE)?;

  let mut chart = ChartBuilder::on(&root)
    .caption("Gridmap Test", ("sans-serif", 30))
    .margin(5)
    .set_label_area_size(LabelAreaPosition::Left, 40)
    .set_label_area_size(LabelAreaPosition::Bottom, 40)
    .build_cartesian_2d(
      -0.5..(gridmap.get_width() as f64 - 0.5), 
      -0.5..(gridmap.get_height() as f64 - 0.5))?;

  chart.configure_mesh().draw().unwrap();

  // Fill obstacle regions with black
  for y in 0..gridmap.get_height(){
    for x in 0..gridmap.get_width(){
      let cell_val = gridmap.get_val_xy(gridmap.coord_flip_y_u32((x, y)));
      if cell_val >= 253 { //TODO: Change to a constant value
        let (x_f, y_f) = (x as f64, y as f64);
        chart.draw_series([
        Rectangle::new([(x_f-0.5 , y_f-0.5), (x_f+0.5 , y_f+0.5)], BLACK.filled())])?;
      }
    }
  }

  // Color visited cells (YELLOW)
  {
    for cell in &motion_plan.closed_list{
      let (x_f, y_f) = gridmap.coord_flip_y_f64((cell.0, cell.1));
      chart.draw_series([
        Circle::new((x_f , y_f), 20, RGBAColor(255, 255, 0, 0.5).filled())
      ])?;
    }
  }

  // Fill path with blue
  // for point in &motion_plan.path {
  //     let (x, y) = gridmap.flip_coord_y((point.0, point.1));
  //     let (x_f, y_f) = (x as f64, y as f64);
  //     chart.draw_series([
  //       Circle::new((x_f , y_f), 10, RGBAColor(0, 0, 255, 0.5).filled())
  //     ])?;
  // }

  chart.draw_series(
    motion_plan.path
      .iter()
      .map(|xy| gridmap.coord_flip_y_f64(*xy))
      .map(|xy| Circle::new(xy, 10, RGBAColor(0, 0, 255, 0.5).filled()))
  )?;


  chart.draw_series(LineSeries::new(
    motion_plan.path
      .iter()
      .map(|xy| gridmap.coord_flip_y_f64(*xy)), BLUE
  ))?;

  // Color the start (CYAN)
  {
    let (x_f, y_f) = gridmap.coord_flip_y_f64((start_cell.0, start_cell.1));
    chart.draw_series([
      Circle::new((x_f , y_f), 20, CYAN.filled())
    ])?;
  }

  // Color the goal (GREEN)
  {
    let (x_f, y_f) = gridmap.coord_flip_y_f64((goal_cell.0, goal_cell.1));
    chart.draw_series([
      Circle::new((x_f , y_f), 20, GREEN.filled())
    ])?;
  }

  // Legend
  // chart.configure_series_labels()
  //   .border_style(&BLACK)
  //   .background_style(&WHITE.mix(0.8))
  //   .draw()
  //   .unwrap();

  Ok(())

}