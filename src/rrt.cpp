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

#include "rrt_explore/rrt.hpp"

using namespace exploration;
using std::placeholders::_1;

RRT::RRT() : Node("RRT_node")
{
    /*------- Fetch parameters ------*/
    if (!get_ros_parameters()) return; // Exit if parameters fetching failure

    /*------- Initialize Exploration state ------*/
    exploration_state_.robot_id = robot_id_;
    exploration_state_.status = rrt_explore::msg::ExplorationState::INACTIVE;

    /*------- Create navigation_stack action client ------*/
    navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    /*------- Initialize TF listener ------*/
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    /*------- Create Subscribers and publishers ------*/
    map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_topic_, 10, std::bind(&RRT::map_callback, this, _1));
    costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 20, std::bind(&RRT::costmap_callback, this, _1));
    exploration_cmd_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(std::string(this->get_name()) + "/enable", 10, std::bind(&RRT::enable_exploration_callback, this, _1));
    target_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("frontier_targets", 1);
    end_detection_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("end_detection_frontiers", 1);
    rrt_segments_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_segments", 1);
    exploration_state_publisher_ = this->create_publisher<rrt_explore::msg::ExplorationState>("/exploration_state", 1);
    
    /*------- Create timers ------*/
    // timer_exploration_state_publisher_ = this->create_wall_timer( std::chrono::duration<double>(1.0 / rate_), std::bind(&RRT::publish_exploration_state, this));

    /*------- Create random number generator ------*/
 	// this is an example of initializing by an array
	// you may use MTRand(seed) with any 32bit integer as a seed for a simpler initialization
	unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
    irand_ = std::make_shared<MTRand_int32>(init, length);
    drand_ = std::make_shared<MTRand>(); // already init

    /*------- Robot Transforms initialization ------*/
    robot_transforms_.resize(robot_count_);
    for (auto &transform : robot_transforms_)
    {
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
    }
}

void RRT::explore()
{
    /*------- Return if maps are not available ------*/
    if (!map_data_available()) return;

    /*------- Return if self map to base_foot_print transform is not available ------*/
    geometry_msgs::msg::TransformStamped map_to_baseframe;
    if (!get_transform(map_frame_, robot_base_frame_, map_to_baseframe)) return;

    /*------- Set RRT origin, if not initialized already ------*/
    if (!rrt_origin_initialized_) {
        std::vector<float> xnew;
        xnew.push_back(map_to_baseframe.transform.translation.x + 0.3);
        xnew.push_back(map_to_baseframe.transform.translation.y);
        V_.push_back(xnew);
        rrt_origin_initialized_ = true;

        // Initialize the previous goal point
        previous_goal_.header.frame_id = map_frame_;
        previous_goal_.point.x = xnew[0] + mapData_.info.resolution * 3;
        previous_goal_.point.y = xnew[1] + mapData_.info.resolution * 3;
        previous_goal_.point.z = 0.0;
        RCLCPP_INFO_STREAM(this->get_logger(), "RRT Tree starts growing origin: (" << xnew[0] << ", "  << xnew[1] << ")");

        /*------- Initialize Visualization variables ------*/
        rrt_segment_markers_ = create_visualization_msg(VisualizationType::LINE, (1.0 / rate_));
    }

    /*------- Exploration state thread safety lock ---------*/
    std::unique_lock<std::mutex> exploration_state_lock(mtx_exploration_state);

    /*-------  Fetch external data------*/
    nav_msgs::msg::OccupancyGrid mapData = get_map_data();  
    nav_msgs::msg::OccupancyGrid costmapData = get_costmap_data(); 

    iteration_count_++;
    #ifdef DETECT_END
    /*-------  Find Frontiers for every 100th iteration------*/
    if (iterations_to_end_detection_ <= 0 ) {
        iterations_to_end_detection_ = 100;
        std::vector<Pixel> targets;
        find_frontiers(mapData, costmapData, targets);
        if (targets.size() < 2) 
        {
            zero_target_detections_count_++;
            if (zero_target_detections_count_ >= 5) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Exploration Done!");
                this->navigation_client_->async_cancel_all_goals();
                this->timer_main_->cancel();
                exploration_state_.status = rrt_explore::msg::ExplorationState::DONE;
            }
        } 
        else 
        {
            zero_target_detections_count_ = 0;

            /*-------------- Display Targets -------------*/
            visualization_msgs::msg::Marker frontier_markers = create_visualization_msg(VisualizationType::POINTS, (100.0 / rate_));
            for (auto &target : targets)
            {
                frontier_markers.points.push_back(geometry_msgs::msg::Point().set__x(
                            target.x * mapData.info.resolution + mapData.info.origin.position.x).set__y(
                            target.y * mapData.info.resolution + mapData.info.origin.position.y));
            }
            end_detection_publisher_->publish(frontier_markers);
        }
    }
    iterations_to_end_detection_--;
    #endif // DETECT_END
    
    int map_height = mapData.info.height;
    int map_width = mapData.info.width;
    std::vector<float> x_rand;

    float xr=((*drand_.get())() * map_width * mapData.info.resolution) + mapData.info.origin.position.x;
    float yr=((*drand_.get())() * map_height * mapData.info.resolution) + mapData.info.origin.position.y;
    x_rand.push_back(xr); x_rand.push_back(yr);

    /*------- Find Nearest ------*/
    std::vector<float> x_nearest = Nearest(V_, x_rand);

    /*------- Steer ------*/
    std::vector<float> x_new = Steer(x_nearest, x_rand, eta_);

    /*------- Find Frontiers ------*/
    uint cellType = ObstacleFree(x_nearest, x_new, mapData);
    if (cellType == MapGridCellType::FRONTIER)
    {
        geometry_msgs::msg::PointStamped exploration_goal;
        exploration_goal.header.stamp = rclcpp::Time(0);
        exploration_goal.header.frame_id = mapData.header.frame_id;
        exploration_goal.point.set__x(x_new[0]).set__y(x_new[1]).set__z(0.0);
        frontiers_.push_back(exploration_goal);

        // Add to long running segment store
        rrt_segment_markers_.points.push_back(exploration_goal.point);
        rrt_segment_markers_.points.push_back(geometry_msgs::msg::Point().set__x(x_nearest[0]).set__y(x_nearest[1]).set__z(0.0));
    }
    else if (cellType == MapGridCellType::FREE)
    {
        V_.push_back(x_new);
        rrt_segment_markers_.points.push_back(geometry_msgs::msg::Point().set__x(x_new[0]).set__y(x_new[1]).set__z(0.0));
        rrt_segment_markers_.points.push_back(geometry_msgs::msg::Point().set__x(x_nearest[0]).set__y(x_nearest[1]).set__z(0.0));
    }
    rrt_segments_publisher_->publish(rrt_segment_markers_);

    /*------- Add Previous goal ------*/
    if(mapValue(mapData, previous_goal_)!=-1 || mapValue(costmapData, previous_goal_) > costmap_pixel_threshold_){
    } else {
        frontiers_.push_back(previous_goal_);
    }

    /*------- Remove old frontiers whose places have already been explored & the invalid frontiers ------*/
    for(int i= frontiers_.size()-1; i>-1;i--){
        if(mapValue(mapData, frontiers_[i])!=-1 || mapValue(costmapData, frontiers_[i]) > costmap_pixel_threshold_){
            frontiers_.erase(frontiers_.begin() + i);
        }
    }

    if( ((frontiers_.size() - detectFrontierNum_) > 25) || (iteration_count_ > 500) ){
        // Clear detection's iteration recording.
        iteration_count_ = 0; 

        // Initializes current goal's information.
        double current_goal_score = -500.0, current_goal_infoGain = -500.0;
        int current_goal_idx   = -1;
        (void)current_goal_infoGain; // silence unused warnings

        // Remove frontiers lying out of the valid map (uncertainty of map will cause some frontiers lie out of valid map).
        for(int i= (int)frontiers_.size()-1; i>-1;i--){
            double x_,y_;
            int tempResult[4];
            x_ = frontiers_[i].point.x;
            y_ = frontiers_[i].point.y;

            // Check map bounds
            int cell_x = (x_ - mapData.info.origin.position.x) / mapData.info.resolution;
            int cell_y = (y_ - mapData.info.origin.position.y) / mapData.info.resolution;
            if ((cell_x >= (int64_t)mapData.info.width) || (cell_y >= (int64_t)mapData.info.height)){
                frontiers_.erase(frontiers_.begin() + i);
                continue;
            }

            // Check surrounding cells
            int idx = (floor((y_-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((x_+mapData.info.resolution-mapData.info.origin.position.x)/mapData.info.resolution));
            tempResult[0] = mapData.data[idx];
            idx = floor((y_+mapData.info.resolution-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width+floor((x_-mapData.info.origin.position.x)/mapData.info.resolution);
            tempResult[1] = mapData.data[idx];
            idx = (floor((y_-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((x_-mapData.info.resolution-mapData.info.origin.position.x)/mapData.info.resolution));
            tempResult[2] = mapData.data[idx];
            idx = (floor((y_-mapData.info.resolution-mapData.info.origin.position.y)/mapData.info.resolution)*mapData.info.width)+(floor((x_-mapData.info.origin.position.x)/mapData.info.resolution));
            tempResult[3] = mapData.data[idx];
            // if surring pixels are all unknown, elminate it.
            if( (tempResult[0]+tempResult[1]+tempResult[2]+tempResult[3]) == -4){
                frontiers_.erase(frontiers_.begin() + i);
            }
        }
        detectFrontierNum_ = frontiers_.size();

        // Start a new iteration if no frontiers remain
        if (frontiers_.size() == 0){
            return;
        }

        /*------- Display Valid Frontiers ------*/
        visualization_msgs::msg::Marker target_frontiers = create_visualization_msg(VisualizationType::POINTS, 1.0);
        for(int i= frontiers_.size()-1; i>-1;i--){
            target_frontiers.points.push_back(frontiers_[i].point);
        }
        target_publisher_->publish(target_frontiers);

        /*------- Find the goal which has the highest score ------*/
        for(uint i=0; i<frontiers_.size(); i++){
            double score, travel_distance, infoGain;
            infoGain = informationRectangleGain(mapData, frontiers_[i], info_radius_);
            // if(infoGain<0.2 || pixel > costmap_pixel_threshold){continue;}
            travel_distance =  norm2Dpoints(map_to_baseframe.transform.translation.x, 
                                            map_to_baseframe.transform.translation.y, 
                                            frontiers_[i]);
            // hysteresis here is to reduce overlap as much as possible
            if(travel_distance <= hysteresis_radius_){infoGain*=hysteresis_gain_;}
            score = 3*infoGain - travel_distance;
            // std::cout << ns <<  "score: "<< score << std::endl;
            if(score > current_goal_score){
                current_goal_score    = score;
                current_goal_infoGain = infoGain;
                current_goal_idx      = i;
            }
        }

        if(current_goal_idx == -1){
            return;
        }

        /*------- Send Goal ------*/
        robot_goal_.pose.header.stamp = rclcpp::Time(0);
        robot_goal_.pose.pose.position.x = frontiers_[current_goal_idx].point.x;
        robot_goal_.pose.pose.position.y = frontiers_[current_goal_idx].point.y;
        if(abs(previous_goal_.point.x - robot_goal_.pose.pose.position.x) + 
            abs(previous_goal_.point.y - robot_goal_.pose.pose.position.y) < 3 * mapData.info.resolution){
                return;
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(), "Goal: " << robot_goal_.pose.pose.position.x << ", " << robot_goal_.pose.pose.position.y);
            this->navigation_client_->async_send_goal(robot_goal_);
            previous_goal_ = frontiers_[current_goal_idx];
        }
    }

    if(robot_count_>1){
        bool reset_rrt = false;
        for (int i = 0; i < robot_count_; i++)
        {
            // Skip for current robot
            if ((uint)(i + 1) == robot_id_) continue;

            geometry_msgs::msg::TransformStamped transform;
            if (get_transform(map_frame_, robot_base_frames_[i], transform))
            {
                float dis_origin = 0;
                dis_origin = abs(transform.transform.translation.x - robot_transforms_[i].transform.translation.x);
                dis_origin += abs(transform.transform.translation.y - robot_transforms_[i].transform.translation.y);
                dis_origin += abs(transform.transform.translation.z - robot_transforms_[i].transform.translation.z);
                dis_origin += abs(transform.transform.rotation.x - robot_transforms_[i].transform.rotation.x);
                dis_origin += abs(transform.transform.rotation.y - robot_transforms_[i].transform.rotation.y);
                dis_origin += abs(transform.transform.rotation.z - robot_transforms_[i].transform.rotation.z);
                dis_origin += abs(transform.transform.rotation.w - robot_transforms_[i].transform.rotation.w);

                if (dis_origin > THRESHOLD_TRANSFORM){
                    reset_rrt = true;
                    robot_transforms_[i] = transform;
                }
            }
        }
        
        if(reset_rrt){
            RCLCPP_INFO_STREAM(this->get_logger(), "Flush frontiers");
            frontiers_.clear();
            std::vector<geometry_msgs::msg::PointStamped>().swap(frontiers_);

            rrt_segment_markers_.points.clear();
            std::vector<std::vector<float> >().swap(V_); 
            std::vector<float> xnew;
            xnew.push_back(map_to_baseframe.transform.translation.x);
            xnew.push_back(map_to_baseframe.transform.translation.y); 
            V_.push_back(xnew);
        }
    }
}

/**
 * @brief Extracts obstacle points and exploration targets given a robot map and costmap
 * 
 * @param map 
 * @param costmap 
 * @param obstacles 
 * @param targets 
 */
void RRT::find_frontiers(nav_msgs::msg::OccupancyGrid mapData, nav_msgs::msg::OccupancyGrid costmapData, std::vector<Pixel> &targets)
{
     /*------- Initialize the map ------*/
    int map_height = mapData.info.height;
    int map_width = mapData.info.width;
    std::vector<int> map(mapData.data.begin(), mapData.data.end());
    std::vector<Pixel> obstacles;
    std::list<Pixel> target_list;
    
    /*-------  Find targets & Obstacles ------*/
    // Reserve max sizes for vectors to prevent relocation
    obstacles.reserve(map_height * map_width);

    // Traverse map row, column wise while checking each pixel for free regions
    for (int i = 2; i < (map_height - 2); i++)
    {
        for (int j = 2; j < (map_width - 2); j++)
        {
            if (map[i*map_width + j] == MAP_PIXEL_OCCUPIED)
            {
                obstacles.emplace_back(j,i);
            }
            else if (map[i*map_width + j] == MAP_PIXEL_UNKNOWN)
            {
                // accessiable frontiers
                int numFree = 0, temp1 = 0;

                if (map[(i + 1)*map_width + j] == 0){
                    temp1 += (map[(i + 2)*map_width + j    ] == 0) ? 1 : 0;
                    temp1 += (map[(i + 1)*map_width + j + 1] == 0) ? 1 : 0;
                    temp1 += (map[(i + 1)*map_width + j - 1] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if (map[i*map_width + j + 1] == 0){
                    temp1 = 0;
                    temp1 += (map[      i*map_width + j + 2] == 0) ? 1 : 0;
                    temp1 += (map[(i + 1)*map_width + j + 1] == 0) ? 1 : 0;
                    temp1 += (map[(i - 1)*map_width + j + 1] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if (map[(i - 1) *map_width + j] == 0){
                    temp1 = 0;
                    temp1 += (map[(i - 1)*map_width + j + 1] == 0) ? 1 : 0;
                    temp1 += (map[(i - 1)*map_width + j - 1] == 0) ? 1 : 0;
                    temp1 += (map[(i - 2)*map_width + j    ] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if (map[i * map_width + j - 1] == 0){
                    temp1 = 0;
                    temp1 += (map[    i  *map_width + j - 2] == 0) ? 1 : 0;
                    temp1 += (map[(i + 1)*map_width + j - 1] == 0) ? 1 : 0;
                    temp1 += (map[(i - 1)*map_width + j - 1] == 0) ? 1 : 0;
                    numFree += (temp1 > 0);
                }

                if( numFree > 0 ) {
                    target_list.emplace_back(j,i);
                }
            }
        }
    }
    // Shrink obstacle vector
    obstacles.shrink_to_fit();

    // TODO Border traversal is not included in original RRT Algorithm. Hence commented
    // // Border traversal (Check if any free cell lies on map borders)
    // for (int i = 0; i < map_height; i++)
    // {
    //     if (map[i * map_width] == MAP_PIXEL_FREE) {                     // Left border
    //         targets.emplace_back(0, i);
    //     }                   
    //     if (map[map_width - 1 + i * map_width] == MAP_PIXEL_FREE) {     // Right border
    //         targets.emplace_back(map_width-1, i);            
    //     }
    // }

    // for (int j = 0; j < map_width; j++)
    // {
    //     if (map[j] == MAP_PIXEL_FREE) {                                 // Bottom border
    //         targets.emplace_back(j, 0);
    //     }                   
    //     if (map[(map_height - 1) * map_width + j] == MAP_PIXEL_FREE) {  // Top border
    //         targets.emplace_back(j, map_height - 1);            
    //     }
    // }

    /*-------  Remove targets within the inflation layer of costmap ------*/
    for (auto target=target_list.begin(); target!=target_list.end();)
    {
        float loc_x = target->x * mapData.info.resolution + mapData.info.origin.position.x;
        float loc_y = target->y * mapData.info.resolution + mapData.info.origin.position.y;
        int index_costmap = (loc_y - costmapData.info.origin.position.y) / costmapData.info.resolution * costmapData.info.width +
                            (loc_x - costmapData.info.origin.position.x)/costmapData.info.resolution;
        
        if (costmapData.data[index_costmap] > MAP_PIXEL_FREE) {
            target = target_list.erase(target);
        } else {
            target++;
        }
    }

    /*-------  Remove targets within inflation radius of obstacles ------*/
    for (auto target=target_list.begin(); target!=target_list.end();)
    {
        for (auto obstacle : obstacles) {
            if (std::abs(target->y - obstacle.y) + std::abs(target->x - obstacle.x) < INFLATION_RADIUS_CELL_COUNT) {
                target = target_list.erase(target);
                break;
            }
        }
        target++;
    }

    // Copy targets to vector
    targets.resize(target_list.size());
    std::copy(target_list.begin(), target_list.end(), targets.begin());
    return;
}

/**
 * @brief 
 * 
 * @return true     If success
 * @return false    If failure
 */
bool RRT::get_ros_parameters()
{
    this->declare_parameter("map_topic", "map"); 
    this->declare_parameter("costmap_topic", "global_costmap/costmap");
    this->declare_parameter("robot_base_frame", "base_footprint");
    this->declare_parameter("robot_frame_prefix", "robot");
    this->declare_parameter("rate", 1.0);
    this->declare_parameter("robot_count", 1);
    this->declare_parameter("info_radius", 1.0);
    this->declare_parameter("costmap_pixel_threshold", 30.0);
    this->declare_parameter("eta", 0.5);
    this->declare_parameter("hysteresis_radius", 3.0);
    this->declare_parameter("hysteresis_gain", 2.0);
    
    
    map_topic_ = this->get_parameter("map_topic").get_parameter_value().get<std::string>();
    costmap_topic_ = this->get_parameter("costmap_topic").get_parameter_value().get<std::string>();
    robot_base_frame_ = this->get_parameter("robot_base_frame").get_parameter_value().get<std::string>();
    robot_frame_prefix_ = this->get_parameter("robot_frame_prefix").get_parameter_value().get<std::string>();
    rate_ = this->get_parameter("rate").get_parameter_value().get<float>();
    robot_count_ = this->get_parameter("robot_count").get_parameter_value().get<int>();
    info_radius_ = this->get_parameter("info_radius").get_parameter_value().get<double>();
    costmap_pixel_threshold_ = this->get_parameter("costmap_pixel_threshold").get_parameter_value().get<double>();
    eta_ = this->get_parameter("eta").get_parameter_value().get<float>();
    hysteresis_radius_ = this->get_parameter("hysteresis_radius").get_parameter_value().get<float>();
    hysteresis_gain_ = this->get_parameter("hysteresis_gain").get_parameter_value().get<float>();
    
    
    // Remove any leading slash from robot_base_frame
    if (*robot_base_frame_.cbegin() == '/') robot_base_frame_.erase(0, 1);
    // Create fully qualified robot_base_frame names
    for (int i = 1; i < robot_count_ + 1; i++)
    {
        robot_base_frames_.push_back(robot_frame_prefix_ + std::to_string(i) + "/" + robot_base_frame_);
    }

    // Extract robot id from node namespace
    std::string ns = this->get_namespace();
    RCLCPP_INFO_STREAM(this->get_logger(), "Running exploration in namespace: " << ns);
    try{
        std::string id_string = ns.substr(robot_frame_prefix_.size() + 1);
        robot_id_ = std::stoi(id_string);
        RCLCPP_INFO_STREAM(this->get_logger(), "Robot ID: " << robot_id_);
    }
    catch( ... ){
        RCLCPP_ERROR_STREAM(this->get_logger(), "Cannot extract robot id from node namespace {" << ns 
                                                    << "} with prefix {" << robot_frame_prefix_ << "}");
        return false;
    }

    #ifdef DEBUG
        RCLCPP_INFO_STREAM(RRT::get_logger(), "map topic: " << map_topic_
        <<"\ncostmap_topic: " << costmap_topic_
        <<"\nrobot_base_frame: " << robot_base_frame_
        <<"\nrate: " << rate_
        <<"\nrobot_count: " << robot_count_
        <<"\nrobot_frame_prefix: " << robot_frame_prefix_
        <<"\ninfo_radius: " << info_radius_
        <<"\ncostmap_pixel_threshold: " << costmap_pixel_threshold_
        <<"\neta: " << eta_
        <<"\nhysteresis_radius: " << hysteresis_radius_
        <<"\nhysteresis_gain: " << hysteresis_gain_ 
        );
    #endif

    return true;
}

/**
 * @brief Given two frames, finds the transform from source to target frame
 * 
 * @param target_frame 
 * @param source_frame 
 * @param transform 
 * @return true     If transform is available
 * @return false    If transform is not available
 */
bool RRT::get_transform(std::string target_frame, std::string source_frame, geometry_msgs::msg::TransformStamped &transform)
{
    try{
        transform = tf_buffer_->lookupTransform(target_frame, source_frame,tf2::TimePointZero);
        return true;
    }
    catch( const tf2::TransformException & ex){
        RCLCPP_WARN_STREAM(this->get_logger(), source_frame + " to " + target_frame + " transform is not available : " + ex.what());
    }
    return false;
}

void RRT::map_callback(const nav_msgs::msg::OccupancyGrid msg)
{
    set_map_data(msg);
}

void RRT::costmap_callback(const nav_msgs::msg::OccupancyGrid msg)
{
    set_costmap_data(msg);
}

void RRT::enable_exploration_callback(const std_msgs::msg::Bool msg)
{
    if (msg.data)
    {
        if (exploration_state_.status != rrt_explore::msg::ExplorationState::ACTIVE)
        {
            /*------- Initialize Exploration state ------*/
            exploration_state_.status = rrt_explore::msg::ExplorationState::ACTIVE;
            
            RCLCPP_INFO_STREAM(this->get_logger(), "Starting Exploration!");

            /*------- Create main callback timer ------*/
            timer_main_ = this->create_wall_timer( std::chrono::duration<double>( 1.0 / rate_ ), std::bind(&RRT::explore, this));
        }
        else
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Exploration already running. Start request ignored!");
        }
    }
    else
    {
        this->timer_main_->cancel();
        this->navigation_client_->async_cancel_all_goals(); 
        RCLCPP_INFO_STREAM(this->get_logger(), "Ending exploration: Requested by User!");
        exploration_state_.status = rrt_explore::msg::ExplorationState::DONE;
    }
}

void RRT::set_costmap_data(nav_msgs::msg::OccupancyGrid costmapData)
{
    std::unique_lock<std::mutex> lck (mtx_costmap);
    costmapData_=costmapData;
}

nav_msgs::msg::OccupancyGrid RRT::get_costmap_data()
{
    std::unique_lock<std::mutex> lck (mtx_costmap);
    return costmapData_;
}

void RRT::set_map_data(nav_msgs::msg::OccupancyGrid mapData)
{
    std::unique_lock<std::mutex> lck (mtx_map);
    mapData_=mapData;
}

nav_msgs::msg::OccupancyGrid RRT::get_map_data()
{
    std::unique_lock<std::mutex> lck (mtx_map);
    return mapData_;
}

bool RRT::map_data_available(){

    if (!(get_map_data().data.size() < 1)){
        map_frame_ = get_map_data().header.frame_id;
        robot_goal_.pose.header.frame_id = map_frame_;
        robot_goal_.pose.pose.position.z = 0;
        robot_goal_.pose.pose.orientation.z = 1.0;
        return true;
    }

    RCLCPP_WARN_STREAM(this->get_logger(), "map data is not available");
    return false;
}  

void RRT::publish_exploration_state(void)
{
    std::unique_lock<std::mutex> exploration_state_lock(mtx_exploration_state);

    /*------- Update Exploration state ------*/
    geometry_msgs::msg::TransformStamped map_to_baseframe;
    if (!get_transform(map_frame_, robot_base_frame_, map_to_baseframe)) return;
    exploration_state_.header.frame_id = robot_frame_prefix_ + std::to_string(robot_id_) + "/" + map_frame_;
    exploration_state_.location.position.set__x(map_to_baseframe.transform.translation.x). set__y(
        map_to_baseframe.transform.translation.y).set__z(map_to_baseframe.transform.translation.z);
    exploration_state_.location.orientation.set__x(map_to_baseframe.transform.rotation.x).set__y(
        map_to_baseframe.transform.rotation.y).set__z(map_to_baseframe.transform.rotation.z).set__w(
        map_to_baseframe.transform.rotation.w);

    exploration_state_.header.stamp = this->get_clock()->now();
    exploration_state_publisher_->publish(exploration_state_);
}

visualization_msgs::msg::Marker RRT::create_visualization_msg(int type, double lifetime){

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = rclcpp::Time(0);
    marker.lifetime         = rclcpp::Duration::from_seconds(lifetime);

    if (type == LINE) {
        //------------------------------------- initilize the visualized lines
        marker.type				= marker.LINE_LIST;
        marker.action           = marker.ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x 			= 0.03;
        marker.scale.y			= 0.03;
        marker.color.r			= 1.0;   // 0.0/255.0;
        marker.color.g			= 0.0;   // 0.0/255.0;
        marker.color.b 			= 1.0;   // 236.0/255.0;
        marker.color.a 			= 1.0;
    }

    else if(type == POINTS) {
        //------------------------------------- initilize the visualized points
        marker.type 			= marker.POINTS;
        marker.action           = marker.ADD;
        marker.pose.orientation.w =1.0;
        marker.scale.x 			= 0.3; 
        marker.scale.y			= 0.3; 
        marker.color.r 			= 1.0;   // 255.0/255.0;
        marker.color.g 			= 0.0;   // 0.0/255.0;
        marker.color.b 			= 0.0;   // 0.0/255.0;
        marker.color.a			= 1.0;
    }

    else if(type == SPHERES) {
        //------------------------------------- initilize the visualized points
        marker.type 			= marker.SPHERE_LIST;
        marker.action           = marker.ADD;
        marker.pose.orientation.w =1.0;
        marker.scale.x 			= 0.3; 
        marker.scale.y			= 0.3; 
        marker.color.r 			= 1.0;   // 255.0/255.0;
        marker.color.g 			= 0.0;   // 0.0/255.0;
        marker.color.b 			= 0.0;   // 0.0/255.0;
        marker.color.a			= 1.0;
    }

    else{
        RCLCPP_ERROR_STREAM(RRT::get_logger(), "Undefined visualization msg type");
    }
    return marker;
 } 

double RRT::mapValue(const nav_msgs::msg::OccupancyGrid & mapDataIn, const geometry_msgs::msg::PointStamped & point){
    _mt.lock();
    double tempResult;
    int idx;
    idx = (floor((point.point.y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((point.point.x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
    tempResult = mapDataIn.data[idx];
    _mt.unlock();
    return tempResult;
}

double RRT::informationGain(const nav_msgs::msg::OccupancyGrid& mapDataIn, const geometry_msgs::msg::PointStamped & point, const double r){
    _mt.lock();
    double infoGainValue = 0, tempResult;
    int index, init_index, last_index, length, start, end, r_region, limit;
    double x = point.point.x, y = point.point.y;

    index = (floor((y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
                  
    r_region = int(r/mapDataIn.info.resolution);
    init_index = index-r_region*(mapDataIn.info.width+1);
    last_index = index+r_region*(mapDataIn.info.width+1);
    length     = 2*r_region;
    start      = int(init_index);
    end        = start + int(length);

    if((uint)last_index < mapDataIn.data.size()){
        for(int i = 0; i < 2 * r_region + 1; i++){
            
            int deltaIdx_y = abs(r_region - i);
            int deltaIdx_x = floor(std::sqrt(r_region*r_region - deltaIdx_y*deltaIdx_y));
            int temp = r_region-deltaIdx_x;
            int startIdxCircle = start + temp;
            int endIdxCircle   = end   - temp;
            
            for(int j = startIdxCircle; j < endIdxCircle; j++){
                switch(mapDataIn.data[j]){
                    case  -1: {infoGainValue++; break;}
                    case 100: {infoGainValue--; break;}
                    default: {break;}
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    else{
        for(int i = 0; i < 2 * r_region + 1; i++){

            int deltaIdx_y = abs(r_region - i);
            int deltaIdx_x = floor(std::sqrt(r_region*r_region - deltaIdx_y*deltaIdx_y));
            int temp = r_region-deltaIdx_x;
            int startIdxCircle = start + temp;
            int endIdxCircle   = end   - temp;
            (void)endIdxCircle; // silence unused warnings
            
            for(int j = startIdxCircle; j < end; j++){
                limit = ((start/mapDataIn.info.width) + 2)*mapDataIn.info.width;  // part of rectangle is outside the map
                    if(j >= 0 && j < limit && (uint)j < mapDataIn.data.size()){
                    switch(mapDataIn.data[j]){
                        case  -1: {infoGainValue++; break;}
                        case 100: {infoGainValue--; break;}
                        default: {break;}
                    }
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    tempResult = infoGainValue*(pow(mapDataIn.info.resolution,2));
    _mt.unlock();
    return tempResult;
}

double RRT::informationRectangleGain(const nav_msgs::msg::OccupancyGrid& mapDataIn, const geometry_msgs::msg::PointStamped & point, const double r){
    _mt.lock();
    double infoGainValue = 0, tempResult;
    int index, init_index, last_index, length, start, end, r_region, limit;
    double x = point.point.x, y = point.point.y;

    index = (floor((y-mapDataIn.info.origin.position.y)/mapDataIn.info.resolution)*mapDataIn.info.width)+(floor((x-mapDataIn.info.origin.position.x)/mapDataIn.info.resolution));
                  
    r_region = int(r/mapDataIn.info.resolution);
    init_index = index-r_region*(mapDataIn.info.width+1);
    last_index = index+r_region*(mapDataIn.info.width+1);
    length     = 2*r_region;
    start      = int(init_index);
    end        = start + int(length);

    if((uint)last_index < mapDataIn.data.size()){
        for(int i = 0; i < 2 * r_region + 1; i++){
            for(int j = start; j < end; j++){
                switch(mapDataIn.data[j]){
                    case  -1: {infoGainValue++; break;}
                    case 100: {infoGainValue--; break;}
                    default: {break;}
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    else{
        for(int i = 0; i < 2 * r_region + 1; i++){
            for(int j = start; j < end; j++){
                limit = ((start/mapDataIn.info.width) + 2)*mapDataIn.info.width;  // part of rectangle is outside the map
                    if(j >= 0 && j < limit && (uint)j < mapDataIn.data.size()){
                    switch(mapDataIn.data[j]){
                        case  -1: {infoGainValue++; break;}
                        case 100: {infoGainValue--; break;}
                        default: {break;}
                    }
                }
            }
            start += mapDataIn.info.width;
            end   += mapDataIn.info.width;
        }
    }
    tempResult = infoGainValue*(pow(mapDataIn.info.resolution,2));
    _mt.unlock();
    return tempResult;
}

double RRT::norm2Dpoints(const double& point1_x, const double& point1_y, const geometry_msgs::msg::PointStamped & point2)
{
    _mt.lock();
    double tempResult;
    tempResult= pow(	(pow((point2.point.x-point1_x),2)+pow((point2.point.y-point1_y),2))	,0.5);
    _mt.unlock();
    return tempResult;
}