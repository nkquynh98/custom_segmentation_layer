#ifndef CUSTOM_SEGMENTATION_LAYER_H_
#define CUSTOM_SEGMENTATION_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include <custom_segmentation_layer/seg_obstacles.h>
#include <nav_msgs/Odometry.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <custom_segmentation_layer/multitarget_tracker/Ctracker.h>
namespace custom_segmentation_layer
{

class CustomSegmentationLayer : public costmap_2d::CostmapLayer
{
public:
  CustomSegmentationLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  bool isDiscretized()
  {
    return true;
  }
  virtual void matchSize();



private:
  std::set<int> obstacle_ids;
  std::set<int> path_ids;
  float x_range_min;
  float x_range_max;
  float y_range;
  sensor_msgs::PointCloud freepath_converted;
  sensor_msgs::PointCloud human_converted;
  sensor_msgs::PointCloud obstacle_converted;
  sensor_msgs::PointCloud raw_data;
  float m_per_pixel;
  int costmap_height;
  int costmap_width;
  bool new_data;
  bool need_update;
  cv::Mat warped;
  cv::Mat cropped;
  cv::Mat h;
  ros::Subscriber data_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher dyn_pub_;
  bool isDynamicPublished_;
  bool isInitializing_;
  void publish_dynamicObstacle();
  void publishCostMap();
  void matchSize_costmapObject();
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  void dataCB(const sensor_msgs::PointCloud::ConstPtr &msg);
  void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  void segNetCb(const sensor_msgs::Image::ConstPtr &msg);
  void parseHomographyConstants(const std::string &homography_folder);
  void parseIntSet(const std::string &raw_list, std::set<int> &int_set);
  void convert_points(double robot_x, double robot_y, double robot_yaw, sensor_msgs::PointCloud data);
  std::vector<SegmentationObject> objectList_;
  costmap_converter::ObstacleArrayMsg dynamicObstacles_;
  Point_t current_vel_; 

};
}
#endif
