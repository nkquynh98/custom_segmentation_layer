// Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
// Full license terms provided in LICENSE.md file.

#include <custom_segmentation_layer/custom_segmentation_layer.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <array>


PLUGINLIB_EXPORT_CLASS(custom_segmentation_layer::CustomSegmentationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

namespace custom_segmentation_layer
{

CustomSegmentationLayer::CustomSegmentationLayer() {}

void CustomSegmentationLayer::onInitialize()
{
  isInitializing_=true;
  ros::NodeHandle nh("~/" + name_);
  std::string segmentation_topic = "/segmentation/data";
  nh.param("segmentation_topic", segmentation_topic, segmentation_topic);
  ROS_INFO_STREAM(segmentation_topic);
  std::string odom_topic="/odom";
  nh.param("odom_topic", odom_topic, odom_topic);
  std::string dynamicObstacle_topic="/move_base/TebLocalPlannerROS/obstacles";
  nh.param("obstacles_topic", dynamicObstacle_topic, dynamicObstacle_topic);
  isDynamicPublished_=true;
  nh.param("publish_dynamic_obstacle",isDynamicPublished_,isDynamicPublished_);
  x_range_min=1.5;
  x_range_max=5;
  nh.param("max_distance",x_range_max,x_range_max);
  nh.param("min_distance",x_range_min,x_range_min);
  std::string object_list;
  nh.param("object_list",object_list,std::string(""));
  segmentationTrust_offset=0.5;
  segmentationTrust_base=10;
  nh.param("segmentationTrust_offset",segmentationTrust_offset,segmentationTrust_offset); //rad/s
  nh.param("segmentationTrust_base",segmentationTrust_base,segmentationTrust_base);
  fading_factor=1;
  nh.param("obstacle_fading_factor",fading_factor,fading_factor);
  //get value from object_list
  std::stringstream ss(object_list);
  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);
    bool isPublish, isDynamic, isObstacle;
    int object_id;
    std::string object_name;
    source_node.param("name", object_name, source);
    source_node.param("id", object_id, -1);
    source_node.param("publish", isPublish, false);
    source_node.param("dynamic", isDynamic, false);
    source_node.param("obstacle", isObstacle, false);
    if(object_id!=-1)
    {
      objectList_.push_back(SegmentationObject(object_name, object_id, isPublish, isDynamic, isObstacle));
      ROS_INFO("Created object %s with id %d", source.c_str(),object_id);
    }
    else
    {
      ROS_INFO("Fail to create object %s", source.c_str());
    }
  }

  current_ = true;
  new_data = false;
  default_value_ = NO_INFORMATION;
  matchSize();

/* 
  objectList_.push_back(SegmentationObject("freepath", 1, true, true, true));
  objectList_.push_back(SegmentationObject("human", 2, true, true, true));
  objectList_.push_back(SegmentationObject("obstacles", 0, false, false, false)); */

   
  //ROS_INFO_STREAM(master->getSizeInCellsX());
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &CustomSegmentationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);


  data_sub_ = nh.subscribe<sensor_msgs::PointCloud>(segmentation_topic, 1, &CustomSegmentationLayer::dataCB, this);
  odom_sub_ = nh.subscribe(odom_topic, 1, &CustomSegmentationLayer::odomCB, this);
  staticVel_pub_ = nh.advertise<nav_msgs::Odometry>("Visualize_Velocity", 10);
  if (isDynamicPublished_)
  {
    dyn_pub_ = nh.advertise<costmap_converter::ObstacleArrayMsg>(dynamicObstacle_topic, 10);
  }
  initial_time=ros::Time::now();
}

//Listen and convert the points to map frame
void CustomSegmentationLayer::dataCB(const sensor_msgs::PointCloud::ConstPtr &msg)
{
  //Clear obstacle
/*   freepath_converted.points.clear();
  obstacle_converted.points.clear();
  human_converted.points.clear(); */
  for (int i=0; i<objectList_.size(); i++)
      {
        objectList_[i].clearPoints();
      }
  raw_data=*msg;
  
  new_data = true;
}


void CustomSegmentationLayer::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO_ONCE("CostmapToDynamicObstacles: odom received.");

  tf::Quaternion pose;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, pose);

  tf::Vector3 twistLinear;
  tf::vector3MsgToTF(msg->twist.twist.linear, twistLinear);

  // velocity of the robot in x, y and z coordinates
  tf::Vector3 vel = tf::quatRotate(pose, twistLinear);
  current_vel_.x = vel.x();
  current_vel_.y = vel.y();

  nav_msgs::Odometry temp_vel;
  temp_vel.header.stamp=ros::Time::now();
  temp_vel.twist.twist.linear.x=vel.x();
  temp_vel.twist.twist.linear.y=vel.y();
  staticVel_pub_.publish(temp_vel);
  //  if(vel.x()!=0) ROS_INFO("Vel X: %f, Vel Y: %f", vel.x(), vel.y()); 
  
  current_vel_.z = msg->twist.twist.angular.z;
}

void CustomSegmentationLayer::convert_points(double robot_x, double robot_y, double robot_yaw, sensor_msgs::PointCloud data)
{
  double cos_th = cos(robot_yaw);
  double sin_th = sin(robot_yaw);
  for (int i=0; i<data.points.size(); i++)
  {
      double point_x=data.points[i].y;
      double point_y=-data.points[i].x;
      ROS_INFO_STREAM(data.points[i].z);
      if((point_x>x_range_min) && (point_x<x_range_max))
      {
        geometry_msgs::Point32 temp_point;
        temp_point.z=0;
        temp_point.x=robot_x + (point_x*cos_th - point_y*sin_th);
        temp_point.y=robot_y + (point_x*sin_th + point_y*cos_th);
      for (int j=0; j<objectList_.size(); j++)
      {
        if (objectList_[j].obstacleID_==data.points[i].z)
        {
          objectList_[j].addPoints(temp_point);
        }
      }
/*         switch(int(data.points[i].z)){
          case 0:
            obstacle_converted.points.push_back(temp_point);
            break;
          case 1:
            freepath_converted.points.push_back(temp_point);
            break;
          case 2:
            human_converted.points.push_back(temp_point);
            break;        
        } */
      }
  }
  //ROS_INFO("Callback finish");
}
  
void CustomSegmentationLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
	    master->getOriginX(), master->getOriginY());
}

void CustomSegmentationLayer::publish_dynamicObstacle()
{
  if(!isDynamicPublished_)
    return;
  dynamicObstacles_.header.stamp = ros::Time::now();
  dynamicObstacles_.header.frame_id = "map"; //Global frame /map
  dynamicObstacles_.obstacles.clear();
  int obstacle_id=0;
  for (int i=0; i<objectList_.size(); i++)
  {
    if(objectList_[i].isDynamic())
    {
      objectList_[i].update_CVcostmap();
      objectList_[i].compute_tracking(current_vel_);
      for(int j=0; j<objectList_[i].obstacles_.size(); j++)
      {
        objectList_[i].obstacles_[j].id=obstacle_id;
        dynamicObstacles_.obstacles.push_back(objectList_[i].obstacles_[j]);
        obstacle_id++;
      }
      if(dynamicObstacles_.obstacles.size()==0)
      {
        costmap_converter::ObstacleMsg temp_obstacle;
        geometry_msgs::Polygon polygon;
        
        polygon.points.emplace_back();
        polygon.points.back().x = 0;
        polygon.points.back().y = 0;
        polygon.points.back().z = 0;
        temp_obstacle.polygon = polygon;
        temp_obstacle.velocities.twist.linear.x = 0;
        temp_obstacle.velocities.twist.linear.y = 0 ; 
        dynamicObstacles_.obstacles.push_back(temp_obstacle);
      }
    }
  }
  dyn_pub_.publish(dynamicObstacles_);
}

void CustomSegmentationLayer::publishCostMap()
{
  for (int i=0; i<objectList_.size(); i++)
  {
    if (!objectList_[i].isPublishedCostmap()) continue;
    objectList_[i].publish_costmap();
  }
    
}
void CustomSegmentationLayer::matchSize_costmapObject()
{
  static ros::Time old_time=ros::Time::now();
  if (isInitializing_)
  {
    for (int i=0; i<objectList_.size(); i++)
    {
      if (!objectList_[i].isPublishedCostmap()) continue;
      unsigned char value=-1;
      if (objectList_[i].isDynamic())
      {
        value = 0;
      }
      objectList_[i].InitializeCostmap(this->getSizeInCellsX(), this->getSizeInCellsY(), this->getResolution(),this->getOriginX(), this->getOriginY(), value);
    }
    isInitializing_=false;
  }
  else
  {
    double delta_time=ros::Time::now().toSec()-old_time.toSec();
    //ROS_INFO_STREAM(delta_time);
    for (int i=0; i<objectList_.size(); i++)
    {
      if (!objectList_[i].isPublishedCostmap()) continue;
      if (objectList_[i].isDynamic()){
              objectList_[i].SegmentationCostmaps_->resizeMap(this->getSizeInCellsX(), this->getSizeInCellsY(), this->getResolution(),
	    this->getOriginX(), this->getOriginY());
      }
      else{
        //ROS_INFO_STREAM(delta_time);
        if(delta_time<fading_factor)
        {
          objectList_[i].SegmentationCostmaps_->updateOrigin(this->getOriginX(), this->getOriginY());
        }
        else
        {
          objectList_[i].SegmentationCostmaps_->resizeMap(this->getSizeInCellsX(), this->getSizeInCellsY(), this->getResolution(),
	    this->getOriginX(), this->getOriginY()); 
        }

/*         unsigned char* temp_costmap=objectList_[i].SegmentationCostmaps_->getCharMap();
        int costmap_length=objectList_[i].SegmentationCostmaps_->getSizeInCellsX()*objectList_[i].SegmentationCostmaps_->getSizeInCellsY();
        //ROS_INFO_STREAM(costmap_length);  
        for (int i=0; i<costmap_length; i++)
        {
          if (temp_costmap[i]==255) continue;
          else temp_costmap[i]=std::max(0,temp_costmap[i]-fading_factor); 
          if(temp_costmap[i]!=0)
          {
            //ROS_INFO_STREAM(int(temp_costmap[i]));
          }
          
          //temp_costmap[i]=temp_costmap[i]*fading_factor; 
          
        } */
      }

    }
    if (delta_time>fading_factor) old_time=ros::Time::now();
  }
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
  {
    return;
  }
  matchSize();
  matchSize_costmapObject();
 
  
  //convert_points(robot_x, robot_y, robot_yaw, raw_data);
  //ROS_INFO_STREAM(objectList_[1].SegmentationCostmaps_->getOriginX());
  
  //objectList_[1].publish_costmap();
  
  //ROS_INFO_STREAM(objectList_[1].SegmentationCostmaps_->getOriginX());
  //ROS_INFO_STREAM(objectList_[1].SegmentationCostmaps_->getResolution());
  //ROS_INFO("Robot_X: %f, Robot_y: %f, Robot_yaw: %f", robot_x, robot_y, robot_yaw);
/*   for (int i=0; i<human_converted.points.size(); i++)
  {
    unsigned int mx, my;
    double mark_x=human_converted.points[i].x;
    double mark_y=human_converted.points[i].y;
	  if(worldToMap(mark_x, mark_y, mx, my)){
	    setCost(mx, my, LETHAL_OBSTACLE);
      //ROS_INFO("Mark X is %f, Mark Y is %f  , value is %d", mark_x, mark_y, costmap_[getIndex(mx, my)]);
	  }
  }
 */
  //ROS_INFO("Object List size is : %d", objectList_.size());
/*   for (int i=0; i<objectList_.size(); i++)
  {
    //ROS_INFO_STREAM(objectList_[1].obstaclePoints_.points.size());
    //ROS_INFO("Object List size is : %d",objectList_[i].obstaclePoints_.points.size());
    for (int j=0; j<objectList_[i].obstaclePoints_.points.size(); j++)
    {
     // ROS_INFO_STREAM(objectList_[i].obstaclePoints_.points.size());
      unsigned int mx, my;
      double mark_x=objectList_[i].obstaclePoints_.points[j].x;
      double mark_y=objectList_[i].obstaclePoints_.points[j].y;
      touch(mark_x, mark_y, min_x, min_y,max_x,max_y);
      //ROS_INFO("%f, %f, %f, %f",*min_x, *min_y, *max_x, *max_y);
      if(objectList_[i].isPublishedCostmap())
      {
        //ROS_INFO_STREAM(mark_x);
        if(worldToMap(mark_x, mark_y, mx, my)){
          //ROS_INFO_STREAM(objectList_[i].getName());
          //ROS_INFO_STREAM(objectList_[i].isObstacle());
          if(objectList_[i].isObstacle())
          {
            objectList_[i].SegmentationCostmaps_->setCost(mx, my, LETHAL_OBSTACLE);
            //ROS_INFO_STREAM(objectList_[i].getName());
            setCost(mx, my, LETHAL_OBSTACLE);
          }
          else 
          {
            objectList_[i].SegmentationCostmaps_->setCost(mx, my, FREE_SPACE);
            setCost(mx, my, FREE_SPACE);
          }
        //ROS_INFO("Mark X is %f, Mark Y is %f  , value is %d", mark_x, mark_y, costmap_[getIndex(mx, my)]);
      }
      }
    }
  } */

  double cos_th = cos(robot_yaw);
  double sin_th = sin(robot_yaw);
  //ROS_INFO_STREAM(raw_data.points.size());
  for (int i=0; i<raw_data.points.size(); i++)
  {
      double point_x=raw_data.points[i].y;
      double point_y=-raw_data.points[i].x;

      int segmenation_trust=int(254/std::pow(segmentationTrust_base,std::max(0.0,std::abs(current_vel_.z)-segmentationTrust_offset)));
      //ROS_INFO_STREAM(segmenation_trust);
      //ROS_INFO_STREAM(data.points[i].z);
      if((point_x>x_range_min) && (point_x<x_range_max))
      {
        double mark_x=robot_x + (point_x*cos_th - point_y*sin_th);
        double mark_y=robot_y + (point_x*sin_th + point_y*cos_th);
        touch(mark_x, mark_y, min_x, min_y,max_x,max_y);
        for (int j=0; j<objectList_.size(); j++)
        {
          if (!objectList_[j].isPublishedCostmap()) continue;
          if (objectList_[j].obstacleID_==raw_data.points[i].z)
          {
            //objectList_[j].addPoints(temp_point);
            unsigned int mx, my;
            if(worldToMap(mark_x, mark_y, mx, my))
            {
              //ROS_INFO_STREAM(objectList_[i].getName());
              //ROS_INFO_STREAM(objectList_[i].isObstacle());
              int obstacle_cellvalue= LETHAL_OBSTACLE;
              //ROS_INFO_STREAM(int(LETHAL_OBSTACLE));
              if (!objectList_[j].isDynamic()){obstacle_cellvalue=segmenation_trust;}
              if(objectList_[j].isObstacle())
              {
                objectList_[j].SegmentationCostmaps_->setCost(mx, my, obstacle_cellvalue);
              }
              else 
              {
                objectList_[j].SegmentationCostmaps_->setCost(mx, my, FREE_SPACE);
              }
            //ROS_INFO("Mark X is %f, Mark Y is %f  , value is %d", mark_x, mark_y, costmap_[getIndex(mx, my)]);
            }
          }
        }
      }
  }
  publish_dynamicObstacle();
  //publishCostMap();

  //objectList_[1].publish_costmap();

  // REVIEW: potentially make this configurable, or calculated?
/*   *min_x = -20; // 20 meters, max size
  *min_y = -20;
  *max_x = 20;
  *max_y = 20; */
  

  new_data = false;
  need_update=true;
}

  // actually update the costs within the bounds
void CustomSegmentationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  if (!need_update)
    return;
/*   for (int k=0; k<objectList_.size(); k++)
  {
    if (!objectList_[k].isPublishedCostmap()) continue;
    objectList_[k].SegmentationCostmaps_->useUpdateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
  } */
  unsigned char* master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      unsigned char old_value = master[index];
      int cell_value=-1;
      if (old_value!=NO_INFORMATION) cell_value=int(old_value);
      //ROS_INFO_STREAM(cell_value);
      for (int k=0; k<objectList_.size(); k++)
      {
        if (!objectList_[k].isPublishedCostmap()) continue;
        unsigned char object_value=objectList_[k].SegmentationCostmaps_->getCost(i,j);
        //ROS_INFO_STREAM(int(object_value));
        if (object_value == NO_INFORMATION) continue;
        cell_value=std::max(cell_value,int(object_value));
        //if(cell_value!=0) ROS_INFO_STREAM(cell_value);
      }
      if (cell_value!=-1)
      {
        master[index]=cell_value;
      }

      
      //master_grid.setCost(i, j, cell_value);
      
/*       if (costmap_[index] == NO_INFORMATION)
	continue;
      master_grid.setCost(i, j, costmap_[index]); */
    }
  }
  need_update=false;
}

} // end namespace
