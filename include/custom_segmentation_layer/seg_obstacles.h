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
//Obstacle Message
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>

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
        
        nh=ros::NodeHandle("~/" + obstacleNames_);
        InitializeBlobDetector();
        //InitializeTracker();
    }

    void InitializeBlobDetector()
    {
        // Blob detection parameters
        BlobDetector::Params blob_det_params;

        blob_det_params.filterByColor = true; // actually filterByIntensity, always true
        blob_det_params.blobColor = 255;      // Extract light blobs
        blob_det_params.thresholdStep = 256;  // Input for blob detection is already a binary image
        blob_det_params.minThreshold = 127;
        blob_det_params.maxThreshold = 255;
        blob_det_params.minRepeatability = 1;

        blob_det_params.minDistBetweenBlobs = 10;
        blob_det_params.filterByArea = true;
        blob_det_params.minArea = 3; // Filter out blobs with less pixels
        blob_det_params.maxArea = 300;
        blob_det_params.filterByCircularity = true; // circularity = 4*pi*area/perimeter^2
        blob_det_params.minCircularity = 0; //0.2
        blob_det_params.maxCircularity = 1; // maximal 1 (in case of a circle)
        blob_det_params.filterByInertia = true; // Filter blobs based on their elongation
        blob_det_params.minInertiaRatio = 0; //0.2  // minimal 0 (in case of a line)
        blob_det_params.maxInertiaRatio = 1;    // maximal 1 (in case of a circle)
        blob_det_params.filterByConvexity = false; // Area of the Blob / Area of its convex hull
        blob_det_params.minConvexity = 0;          // minimal 0

        blob_det_params.maxConvexity = 1;          // maximal 1

        blob_det_ = BlobDetector::create(blob_det_params);
    }
    void InitializeCostmap(unsigned int cells_size_x, unsigned int cells_size_y, double resolution, double origin_x, double origin_y, unsigned char default_value)
    {
        SegmentationCostmaps_= new costmap_2d::Costmap2D(cells_size_x,cells_size_y,resolution, origin_x, origin_y, default_value);
        std::string topic_name= obstacleNames_ + "/costmap";
        visualize_costmap_pub_ = new costmap_2d::Costmap2DPublisher(&nh, SegmentationCostmaps_, "/map", topic_name, false);
    }

/* 
    void InitializeTracker()
    {
        // Tracking parameters
        CTracker::Params tracker_params;
        tracker_params.dt = 0.2;

        tracker_params.dist_thresh = 60.0;

        tracker_params.max_allowed_skipped_frames = 3;

        tracker_params.max_trace_length = 10;

        //tracker_ = std::unique_ptr<CTracker>(new CTracker(tracker_params));
    } */

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

    void compute_tracking()
    {
        /////////////////////////////// Blob detection /////////////////////////////////////
        // Centers and contours of Blobs are detected
        blob_det_->detect(costmap_mat_, keypoints_);
        std::vector<std::vector<cv::Point>> contours = blob_det_->getContours();

        cv::Mat im_with_keypoints;
        cv::drawKeypoints(costmap_mat_, keypoints_, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        cvVisualize_costmap("Keypoints", im_with_keypoints);
 /*        ////////////////////////////// Tracking ////////////////////////////////////////////
        // Objects are assigned to objects from previous frame based on Hungarian Algorithm
        // Object velocities are estimated using a Kalman Filter
        std::vector<Point_t> detected_centers(keypoints_.size());
        for (int i = 0; i < keypoints_.size(); i++)
        {
            detected_centers.at(i).x = keypoints_.at(i).pt.x;
            detected_centers.at(i).y = keypoints_.at(i).pt.y;
            detected_centers.at(i).z = 0; // Currently unused!
        }

        tracker_->Update(detected_centers, contours); */
    }

    void update_CVcostmap()
    {
        if (!SegmentationCostmaps_->getMutex())
            {
                ROS_ERROR("Cannot update costmap since the mutex pointer is null");
                return;
            }
        costmap_2d::Costmap2D::mutex_t::scoped_lock lock(*SegmentationCostmaps_->getMutex());
        costmap_mat_ = cv::Mat(SegmentationCostmaps_->getSizeInCellsX(), SegmentationCostmaps_->getSizeInCellsY(), CV_8UC1, SegmentationCostmaps_->getCharMap());
        //std::cout<<costmap_mat_;
        //cvVisualize_costmap(obstacleNames_, costmap_mat_);
    }
    void cvVisualize_costmap(const std::string& name, const cv::Mat& image)
    {
        if (!image.empty())
        {
            //cv::Mat im(100,100, CV_8UC1);
            cv::Mat im = image.clone();
            cv::flip(im, im, 0);
            cv::resize(im,im,cv::Size(),5,5);
            cv::imshow(name, im);
            cv::waitKey(1);
        }
    }
    sensor_msgs::PointCloud obstaclePoints_;
    int obstacleID_;
    costmap_2d::Costmap2D* SegmentationCostmaps_;
    costmap_2d::Costmap2DPublisher* visualize_costmap_pub_;
    cv::Mat costmap_mat_;
    ros::NodeHandle nh;
    costmap_converter::ObstacleArrayMsg obstacles_;
    std::vector<cv::KeyPoint> keypoints_;
    //std::unique_ptr<CTracker> tracker_;

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