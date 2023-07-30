/* All rights reserved.
 *
 *  
 * Software License Agreement (BSD License 2.0)
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE. 
 * 
 * Copyright Singapore University of Technology and Design (2023)
 * Author: chhathuranga@gmail.com
 */

#include <chrono>
#include <string>
#include <list>
#include <mutex>
#include <map>
#include<string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "rrt_explore/mtrand.h"
#include "rrt_explore/msg/exploration_state.hpp"
#include "rrt_explore/functions.hpp"



#include "tf2_ros/transform_broadcaster.h"




#define DEBUG
#define DETECT_END

namespace exploration{
  class RRT : public rclcpp::Node
  {
    public:
      enum VisualizationType 
      { 
        POINTS = 0,
        LINE = 1,
        SPHERES = 2
      };

      struct Pixel
      {
        int x = 0;
        int y = 0;

        Pixel() {};
        Pixel(int x, int y): x(x), y(y) {};
        inline bool operator==(const Pixel& p) { return ((p.x == x) && (p.y == y)); };
      };

      RRT();
 
      
    private:
      void map_callback(const nav_msgs::msg::OccupancyGrid msg);
      void costmap_callback(const nav_msgs::msg::OccupancyGrid msg);
      void enable_exploration_callback(const std_msgs::msg::Bool msg);

      void set_costmap_data(nav_msgs::msg::OccupancyGrid costmapData);
      nav_msgs::msg::OccupancyGrid get_costmap_data();
      void set_map_data(nav_msgs::msg::OccupancyGrid mapData);
      nav_msgs::msg::OccupancyGrid get_map_data();
      bool map_data_available(void);

      void explore();
      bool get_ros_parameters(void);
      bool get_transform(std::string target_frame, std::string source_frame, geometry_msgs::msg::TransformStamped &transform);
      void publish_exploration_state(void);
      visualization_msgs::msg::Marker create_visualization_msg(int type, double lifetime);
      void find_frontiers(nav_msgs::msg::OccupancyGrid mapData, nav_msgs::msg::OccupancyGrid costmapData, std::vector<Pixel> &targets);
      double mapValue(const nav_msgs::msg::OccupancyGrid & mapDataIn, const geometry_msgs::msg::PointStamped & point);
      double informationGain(const nav_msgs::msg::OccupancyGrid& mapDataIn, const geometry_msgs::msg::PointStamped & point, const double r);
      double informationRectangleGain(const nav_msgs::msg::OccupancyGrid& mapDataIn, const geometry_msgs::msg::PointStamped & point, const double r);
      double norm2Dpoints(const double& point1_x, const double& point1_y, const geometry_msgs::msg::PointStamped & point2);

      // Parameters
      std::string map_topic_, costmap_topic_; 
      std::string robot_base_frame_, map_frame_, robot_frame_prefix_;
      float rate_;
      int robot_count_;
      uint robot_id_;
      std::vector<std::string> robot_base_frames_;  // Fully qualified frame names
      double costmap_pixel_threshold_, info_radius_;
      float hysteresis_radius_, hysteresis_gain_, eta_;

      // ROS Subscribers, Publishers and Action clients
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
      rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_cmd_subscriber_;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_publisher_;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr end_detection_publisher_;
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rrt_segments_publisher_;
      rclcpp::Publisher<rrt_explore::msg::ExplorationState>::SharedPtr exploration_state_publisher_;
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_client_;

      // ROS TF2
      std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

      // Attributes
      rclcpp::TimerBase::SharedPtr timer_main_;
      rclcpp::TimerBase::SharedPtr timer_exploration_state_publisher_;
      std::shared_ptr<MTRand_int32> irand_; // 32-bit int generator
      std::shared_ptr<MTRand> drand_; // double in [0, 1) generator

      // Shared variables
      nav_msgs::msg::OccupancyGrid mapData_, costmapData_;
      nav2_msgs::action::NavigateToPose_Goal robot_goal_;
      rrt_explore::msg::ExplorationState exploration_state_;
      std::mutex mtx_map; 
      std::mutex mtx_costmap; 
      std::mutex mtx_exploration_state;
      std::mutex _mt;

      // Internal counters / state registers
      int iteration_count_ = 0;
      int iterations_to_end_detection_ = 0;
      int zero_target_detections_count_ = 0;
      bool rrt_origin_initialized_ = false;
      std::vector<std::vector<float>> V_;
      std::vector<geometry_msgs::msg::PointStamped> frontiers_;
      visualization_msgs::msg::Marker rrt_segment_markers_;
      geometry_msgs::msg::PointStamped previous_goal_;
      int detectFrontierNum_;
      std::vector<geometry_msgs::msg::TransformStamped> robot_transforms_;

      // Constants
      const int MAP_PIXEL_OCCUPIED = 100;
      const int MAP_PIXEL_UNKNOWN = -1;
      const int MAP_PIXEL_FREE = 0;

      const int INFLATION_RADIUS_CELL_COUNT = 4;
      const float THRESHOLD_TRANSFORM  = 0.5;
  };
}