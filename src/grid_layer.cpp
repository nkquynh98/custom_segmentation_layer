#include <custom_segmentation_layer/grid_layer.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_layer.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace simple_layer_namespace
{

GridLayer::GridLayer() {}

void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION; 
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void GridLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
  ROS_INFO("Origin X: %f, Origin Y: %f", master->getOriginX(), master->getOriginY());
}


void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void GridLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  //matchSize(); //For matching the size again
  double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
  //ROS_INFO("Mark X is %f, Mark Y is %f", mark_x, mark_y);
  unsigned int mx;
  unsigned int my;
  if(worldToMap(mark_x, mark_y, mx, my)){
    setCost(mx, my, LETHAL_OBSTACLE);
    //ROS_INFO("Mark X is %f, Mark Y is %f  , value is %d", mark_x, mark_y, costmap_[getIndex(mx, my)]);
  }
  
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
  ROS_INFO("Min X: %f, Max X: %f, Min Y: %f, Max Y: %f", *min_x, *max_x, *min_y, *max_y);
}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
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
 
  //updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  ROS_INFO("Size X: %d, Size Y: %d", min_i,max_i);
}

} // end namespace