#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  double number_to_get;
  std::vector<std::string> my_list;
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
/*       nh.getParam("/number_float", number_to_get);
      nh.getParam("/dictionary",my_list);
      for (unsigned i=0; i<my_list.size();i++)
      {
          ROS_INFO_STREAM(my_list[i]);
      }
      //ROS_INFO_STREAM(number_to_get); */
      unsigned char i=-1;
      ROS_INFO_STREAM(int(i));
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
  
}