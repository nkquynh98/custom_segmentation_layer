#include<custom_layers/occgrid_to_costmap_layer.h>

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2/LinearMath/Transform.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <message_filters/subscriber.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(occgrid_to_costmap_layer_namespace::OTCLayer, costmap_2d::Layer)


using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace occgrid_to_costmap_layer_namespace
{

    OTCLayer::OTCLayer() : dsrv_(NULL) {}

    OTCLayer::~OTCLayer()
    {
        if (dsrv_){
            delete dsrv_;
        }
    }

    void OTCLayer::onInitialize()
    {
      ros::NodeHandle nh("~/" + name_), g_nh;
      current_ = true;

      global_frame_ = layered_costmap_->getGlobalFrameID();

      nh.param("use_max_value_when_combining", use_max_value_, false);
      nh.param("map_topic", map_topic,  std::string("map"));
      nh.param("marking", marking_, false);
      nh.param("clearing", clearing_, false);

      // Only resubscribe if topic has changed
      if (map_sub_.getTopic() != ros::names::resolve(map_topic))
      {
            // we'll subscribe to the latched topic that the map server uses
            ROS_INFO("Requesting the map from topic %s", map_topic.c_str());
            map_sub_ = g_nh.subscribe(map_topic, 1, &OTCLayer::incomingMap, this);
            map_received_ = false;
            has_updated_data_ = false;

            ros::Rate r(10);
            while (!map_received_ && g_nh.ok())
            {
                ROS_INFO("Waiting for OccGrid...");
                ros::spinOnce();
                r.sleep();
            }

            ROS_INFO("Received a %d X %d map at %f m/pix", getSizeInCellsX(), getSizeInCellsY(), getResolution());

            if (subscribe_to_updates_)
            {
                ROS_INFO("Subscribing to updates");
                map_update_sub_ = g_nh.subscribe(map_topic + "_updates", 10, &OTCLayer::incomingUpdate, this);
            }
      }
      else
      {
            has_updated_data_ = true;
      }

      if (dsrv_)
      {
            delete dsrv_;
      }

      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
      dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
          &OTCLayer::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
    }

    void OTCLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
      if (config.enabled != enabled_)
      {
            enabled_ = config.enabled;
            has_updated_data_ = true;
            x_ = y_ = 0;
            width_ = size_x_;
            height_ = size_y_;
      }
    }

    void OTCLayer::matchSize()
    {
      // If we are using rolling costmap, the static map size is
      //   unrelated to the size of the layered costmap
      if (!layered_costmap_->isRolling())
      {
            Costmap2D* master = layered_costmap_->getCostmap();
            resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                  master->getOriginX(), master->getOriginY());
      }
    }


    void OTCLayer::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
    {
      ROS_INFO("In OTCLayer::incomingMap");
      unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

      ROS_DEBUG("Received a %d X %d map at %f m/pix", size_x, size_y, new_map->info.resolution);

      // resize costmap if size, resolution or origin do not match
      Costmap2D* master = layered_costmap_->getCostmap();
     
      if (size_x_ != size_x || size_y_ != size_y ||
               resolution_ != new_map->info.resolution ||
               origin_x_ != new_map->info.origin.position.x ||
               origin_y_ != new_map->info.origin.position.y)
      {
            // only update the size of the costmap stored locally in this layer
            ROS_INFO("Resizing static layer to %d X %d at %f m/pix", size_x, size_y, new_map->info.resolution);
            resizeMap(size_x, size_y, new_map->info.resolution,
                  new_map->info.origin.position.x, new_map->info.origin.position.y);
      }

      unsigned int index = 0;

      // initialize the costmap with static data
      for (unsigned int i = 0; i < size_y; ++i)
      {
            for (unsigned int j = 0; j < size_x; ++j)
            {
            unsigned char value = new_map->data[index];
            if(marking_ && !clearing_ && value!=-1 && value!=0)
            {
                //if only marking = true and value is an obstacle -> set costmap-value to obstacle
                costmap_[index] = LETHAL_OBSTACLE;
            }
            else if(!marking_ && clearing_ && value==0)
                {
                    //if only clearing = true and value is free_space -> set costmap-value to free_space
                costmap_[index] = FREE_SPACE;
            }
            else if(marking_ && clearing_ && value!=-1)
                {
                    //if marking = true and clearing = true and value is not no_information -> set costmap-value to either obstacle or free_space
                if(value==0){
                        costmap_[index] = FREE_SPACE;
                    }
                    else{
                        costmap_[index] = LETHAL_OBSTACLE;
                    }
            }
            ++index;
            }
      }
      map_frame_ = new_map->header.frame_id;

      // we have a new map, update full size of map
      x_ = y_ = 0;
      width_ = size_x_;
      height_ = size_y_;
      map_received_ = true;
      has_updated_data_ = true;
      ROS_INFO("In OTCLayer::incomingMap - finished.");
    }

    void OTCLayer::incomingUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
    {
      unsigned int di = 0;
      for (unsigned int y = 0; y < update->height ; y++)
      {
            unsigned int index_base = (update->y + y) * size_x_;
            for (unsigned int x = 0; x < update->width ; x++)
            {
                unsigned int index = index_base + x + update->x;
                unsigned char value = update->data[di++];
                if(marking_ && !clearing_ && value!=-1 && value!=0){
                    //if only marking = true and value is an obstacle -> set costmap-value to obstacle
                    costmap_[index] = LETHAL_OBSTACLE;
                }
                else if(!marking_ && clearing_ && value==0){
                    //if only clearing = true and value is free_space -> set costmap-value to free_space
                    costmap_[index] = FREE_SPACE;
                }
                else if(marking_ && clearing_ && value!=-1){
                    //if marking = true and clearing = true and value is not no_information -> set costmap-value to either obstacle or free_space
                    if(value==0){
                        costmap_[index] = FREE_SPACE;
                    }
                    else{
                        costmap_[index] = LETHAL_OBSTACLE;
                    }
                }
            }
      }
      x_ = update->x;
      y_ = update->y;
      width_ = update->width;
      height_ = update->height;
      has_updated_data_ = true;
    }

    void OTCLayer::activate()
    {
      onInitialize();
    }

    void OTCLayer::deactivate()
    {
      map_sub_.shutdown();
      if (subscribe_to_updates_)
        {
            map_update_sub_.shutdown();
        }
    }

    void OTCLayer::reset()
    {
       onInitialize();
    }

    void OTCLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                   double* max_x, double* max_y)
    {
      if( !layered_costmap_->isRolling() )
        {
            if (!map_received_ || !(has_updated_data_ || has_extra_bounds_))
            {
              ROS_ERROR("IN OTCLayer::updateBounds: FAILED TO RECEIVE MAP!");
              return;
            }
      }
      useExtraBounds(min_x, min_y, max_x, max_y);

      double wx, wy;

      mapToWorld(x_, y_, wx, wy);
      *min_x = std::min(wx, *min_x);
      *min_y = std::min(wy, *min_y);

      mapToWorld(x_ + width_, y_ + height_, wx, wy);
      *max_x = std::max(wx, *max_x);
      *max_y = std::max(wy, *max_y);

      has_updated_data_ = false;
    }

    void OTCLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        ROS_INFO("In OTCLayer::updateCosts");
        if (!map_received_)
        {
            ROS_ERROR("IN OTCLayer::updateCosts: FAILED TO RECEIVE MAP!");
            return;
        }

        if (!enabled_)
        {
            return;
        }
      
        if(use_max_value_)
        {
            ROS_INFO("In OTCLayer::updateCosts - updating master_grid using max_value.");
            updateWithMax(master_grid, min_i, min_j, max_i, max_j);
        }
        else
        {
            ROS_INFO("In OTCLayer::updateCosts - updating master_grid using overwrite.");
            updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
        }
    }

}  // namespace occgrid_to_costmap_layer_namespace