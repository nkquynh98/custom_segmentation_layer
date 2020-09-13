#ifndef SEG_OBSTACLE_H_
#define SEG_OBSTACLE_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <costmap_converter/ObstacleArrayMsg.h>
// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
//Custom Tracker
#include <custom_segmentation_layer/multitarget_tracker/Ctracker.h>
#include <custom_segmentation_layer/blob_detector.h>

class SegmentationObject
{
public:
    SegmentationObject() {}
    ~SegmentationObject() {}
    SegmentationObject(const std::string& name, int id, bool isPublish, bool isDynamic, bool isObstacle):
        obstacleNames_(name),
        obstacleID_(id),
        isPublishedCostmap_(isPublish),
        isDynamic_(isDynamic),
        isObstacle_(isObstacle)
    {
        std::string topic_name= name + "/costmap";
        ros::NodeHandle nh("~/" + name);
        //visualize_costmap_pub_ = new costmap_2d::Costmap2DPublisher(&nh, SegmentationCostmaps_, "/map", topic_name, false);

    }

    void InitializeCostmap(unsigned int cells_size_x, unsigned int cells_size_y, double resolution, double origin_x, double origin_y, unsigned char default_value)
    {
        SegmentationCostmaps_= new costmap_2d::Costmap2D(cells_size_x,cells_size_y,resolution, origin_x, origin_y, default_value);
        ROS_INFO_STREAM(cells_size_x);
    }

    void publish_costmap()
    {
        visualize_costmap_pub_->publishCostmap();
        
    }
    std::string getName()
    {
        return obstacleNames_;
    }
    void clearPoints()
    {
        obstaclePoints_.points.clear();
    }

    void addPoints(geometry_msgs::Point32 temp_point)
    {
        obstaclePoints_.points.push_back(temp_point);
    }
    bool isDynamic()
    {
        return isDynamic_;
    }

    bool isPublishedCostmap()
    {
        return isPublishedCostmap_;
    }
    
    bool isObstacle()
    {
        return isObstacle_;
    }

    
    sensor_msgs::PointCloud obstaclePoints_;
    int obstacleID_;
    costmap_2d::Costmap2D* SegmentationCostmaps_;
    costmap_2d::Costmap2DPublisher* visualize_costmap_pub_;

protected:
    bool isDynamic_;
    bool isPublishedCostmap_;
    bool isObstacle_;
    
    std::string obstacleNames_;
    cv::Ptr<BlobDetector> blob_det_;
    ros::Publisher costmap_publish_;
    int costmap_height;
    int costmap_width;
    

};

#endif