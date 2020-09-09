#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
sensor_msgs::PointCloud obstacle_converted;
bool new_data;
double robot_x_=100;
double robot_y_=200;
double robot_yaw_=90;
void convert_points(double robot_x, double robot_y, double robot_yaw, const sensor_msgs::PointCloud::ConstPtr &data)
{
  double cos_th = cos(robot_yaw);
  double sin_th = sin(robot_yaw);
  for (int i=0; i<data->points.size(); i++)
  {
    if (data->points[i].z==1)
    {

      double point_x=data->points[i].x;
      double point_y=data->points[i].y;
      geometry_msgs::Point32 temp_point;
      temp_point.z=0;
      temp_point.x=robot_x + (point_x*cos_th - point_y*sin_th);
      temp_point.y=robot_y + (point_x*sin_th + point_y*sin_th);
      obstacle_converted.points.push_back(temp_point);
    }
  }
}

void dataCB(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    obstacle_converted.points.clear();
    convert_points(robot_x_,robot_y_,robot_yaw_,msg);

/*     for (int i=0; i<msg->points.size(); i++)
    {   
        
        if( msg->points[i].z==1)
        {
            
            obstacle_converted.points.push_back(msg->points[i]);
            //ROS_INFO("%f", obstacle_converted.points[0].x);
        }
    } */
    new_data=true;
}



int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener_pointcloud");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;
  ros::Subscriber data_sub_ = nh.subscribe<sensor_msgs::PointCloud>("/segmentation/data", 1, dataCB);
  ROS_INFO("to here");
  ros::Publisher data_pub=nh.advertise<sensor_msgs::PointCloud>("converted_points", 10);
  ros::Rate rate(10.0);
  
  while (ros::ok())
  {
    //if (!new_data) continue;
    data_pub.publish(obstacle_converted);
    ros::spinOnce();
    new_data= false;
    rate.sleep();
    
  }

  return 0;
}