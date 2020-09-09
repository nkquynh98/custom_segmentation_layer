// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

#include <custom_segmentation_layer/custom_segmentation_layer.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>

PLUGINLIB_EXPORT_CLASS(custom_segmentation_layer::CustomSegmentationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace custom_segmentation_layer
{

CustomSegmentationLayer::CustomSegmentationLayer() {}

void CustomSegmentationLayer::onInitialize()
{
  std::string segmentation_topic;

  segmentation_topic = "/segmentation/data";
  
  ros::NodeHandle nh("~/" + name_);

  
  current_ = true;
  new_data = false;
  default_value_ = NO_INFORMATION;
  matchSize();
  x_range_min=1.5;
  x_range_max=5;
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &CustomSegmentationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);


  data_sub_ = nh.subscribe<sensor_msgs::PointCloud>(segmentation_topic, 1, &CustomSegmentationLayer::dataCB, this);

}

//Listen and convert the points to map frame
void CustomSegmentationLayer::dataCB(const sensor_msgs::PointCloud::ConstPtr &msg)
{
  //Clear obstacle
  freepath_converted.points.clear();
  obstacle_converted.points.clear();
  human_converted.points.clear();
  raw_data=*msg;
  new_data = true;
}

void CustomSegmentationLayer::convert_points(double robot_x, double robot_y, double robot_yaw, sensor_msgs::PointCloud data)
{
  double cos_th = cos(robot_yaw);
  double sin_th = sin(robot_yaw);
  for (int i=0; i<data.points.size(); i++)
  {
      double point_x=data.points[i].y;
      double point_y=data.points[i].x;
      if((point_x>x_range_min) && (point_x<x_range_max))
      {
        geometry_msgs::Point32 temp_point;
        temp_point.z=0;
        temp_point.x=robot_x + (point_x*cos_th - point_y*sin_th);
        temp_point.y=robot_y + (point_x*sin_th + point_y*cos_th);
        switch(int(data.points[i].z)){
          case 0:
            obstacle_converted.points.push_back(temp_point);
            break;
          case 1:
            freepath_converted.points.push_back(temp_point);
            break;
          case 2:
            human_converted.points.push_back(temp_point);
            break;        
        }
      }
  }
}
  
void CustomSegmentationLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
	    master->getOriginX(), master->getOriginY());
}
  
  // allows the plugin to dynamically change the configuration of the costmap
void CustomSegmentationLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

  // determines the area of the costmap that is potentially going to be changed
void CustomSegmentationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  if (!new_data)
    return;
  matchSize();
  convert_points(robot_x, robot_y, robot_yaw, raw_data);
  ROS_INFO("Robot_X: %f, Robot_y: %f, Robot_yaw: %f", robot_x, robot_y, robot_yaw);
  for (int i=0; i<human_converted.points.size(); i++)
  {
    unsigned int mx, my;
    double mark_x=human_converted.points[i].x;
    double mark_y=human_converted.points[i].y;
	  if(worldToMap(mark_x, mark_y, mx, my)){
	    setCost(mx, my, LETHAL_OBSTACLE);
      //ROS_INFO("Mark X is %f, Mark Y is %f  , value is %d", mark_x, mark_y, costmap_[getIndex(mx, my)]);
	  }
  }
  // REVIEW: potentially make this configurable, or calculated?
  *min_x = -20; // 20 meters, max size
  *min_y = -20;
  *max_x = 20;
  *max_y = 20;
  

  new_data = false;
}

  // actually update the costs within the bounds
void CustomSegmentationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
	continue;
      master_grid.setCost(i, j, costmap_[index]);
    }
  }
}

} // end namespace
