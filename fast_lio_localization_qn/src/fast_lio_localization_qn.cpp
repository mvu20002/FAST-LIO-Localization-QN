#include "fast_lio_localization_qn.h"
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/converter_options.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <std_msgs/msg/string.hpp>
FastLioLocalizationQn::FastLioLocalizationQn(rclcpp::Node::SharedPtr &node_in):
    node_(node_in)
{
    ////// ROS params
    // temp vars, only used in constructor
    // get params

    register_params();
    /* basic */

    ////// Matching init
    map_matcher_ = std::make_shared<MapMatcher>(mm_config);

    ////// Load map
    loadMap(saved_map_path);

    ////// ROS things
    raw_odom_path_.header.frame_id = map_frame_;
    corrected_odom_path_.header.frame_id = map_frame_;
    // publishers
    odom_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/ori_odom", 10);
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/ori_path", 10);
    corrected_odom_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_odom", 10);
    corrected_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/corrected_path", 10);
    corrected_current_pcd_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_current_pcd", 10);
    map_match_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/map_match", 10);
    realtime_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/pose_stamped", 10);
    saved_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/saved_map", 10);
    debug_src_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/src", 10);
    debug_dst_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/dst", 10);
    debug_coarse_aligned_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/coarse_aligned_quatro", 10);
    debug_fine_aligned_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("/fine_aligned_nano_gicp", 10);
    // subscribers
    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(node_.get(), "/Odometry");
    sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(node_.get(), "/cloud_registered");
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(std::bind(&FastLioLocalizationQn::odomPcdCallback, this, std::placeholders::_1, std::placeholders::_2));
    // Timers at the end
    // match_timer_ = nh_.createTimer(ros::Duration(1 / map_match_hz), &FastLioLocalizationQn::matchingTimerFunc, this);
    match_timer_ = node_->create_wall_timer(std::chrono::milliseconds(1), [this]()
                                            { matchingTimerFunc(); });
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);

    // ROS_WARN("Main class, starting node...");
}

void FastLioLocalizationQn::register_params()
{
    auto &gc = mm_config.gicp_config_;
    auto &qc = mm_config.quatro_config_;

    register_and_get_params<std::string>("/basic/map_frame", map_frame_, "map");
    register_and_get_params<std::string>("/basic/saved_map", saved_map_path, "/home/mason/kitti.bag");
    register_and_get_params<double>("/basic/map_match_hz", map_match_hz, 1.0);
    register_and_get_params<double>("/basic/visualize_voxel_size", voxel_res_, 1.0);
    /* keyframe */
    register_and_get_params<double>("/keyframe/keyframe_threshold", keyframe_dist_thr_, 1.0);
    register_and_get_params<int>("/keyframe/num_submap_keyframes", mm_config.num_submap_keyframes_, 5);
    /* match */
    register_and_get_params<double>("/match/match_detection_radius", mm_config.loop_detection_radius_, 15.0);
    register_and_get_params<double>("/match/quatro_nano_gicp_voxel_resolution", mm_config.voxel_res_, 0.3);
    /* nano */
    register_and_get_params<int>("/nano_gicp/thread_number", gc.nano_thread_number_, 0);
    register_and_get_params<double>("/nano_gicp/icp_score_threshold", gc.icp_score_thr_, 10.0);
    register_and_get_params<int>("/nano_gicp/correspondences_number", gc.nano_correspondences_number_, 15);
    register_and_get_params<double>("/nano_gicp/max_correspondence_distance", gc.max_corr_dist_, 5.0);
    register_and_get_params<int>("/nano_gicp/max_iter", gc.nano_max_iter_, 32);
    register_and_get_params<double>("/nano_gicp/transformation_epsilon", gc.transformation_epsilon_, 0.01);
    register_and_get_params<double>("/nano_gicp/euclidean_fitness_epsilon", gc.euclidean_fitness_epsilon_, 0.01);
    register_and_get_params<int>("/nano_gicp/ransac/max_iter", gc.nano_ransac_max_iter_, 5);
    register_and_get_params<double>("/nano_gicp/ransac/outlier_rejection_threshold", gc.ransac_outlier_rejection_threshold_, 1.0);
    register_and_get_params<bool>("/quatro/enable", mm_config.enable_quatro_, false);
    register_and_get_params<bool>("/quatro/optimize_matching", qc.use_optimized_matching_, true);
    register_and_get_params<double>("/quatro/distance_threshold", qc.quatro_distance_threshold_, 30.0);
    register_and_get_params<int>("/quatro/max_correspondences", qc.quatro_max_num_corres_, 200);
    register_and_get_params<double>("/quatro/fpfh_normal_radius", qc.fpfh_normal_radius_, 0.02);
    register_and_get_params<double>("/quatro/fpfh_radius", qc.fpfh_radius_, 0.04);
    register_and_get_params<bool>("/quatro/estimating_scale", qc.estimat_scale_, false);
    register_and_get_params<double>("/quatro/noise_bound", qc.noise_bound_, 0.25);
    register_and_get_params<double>("/quatro/rotation/gnc_factor", qc.rot_gnc_factor_, 0.25);
    register_and_get_params<double>("/quatro/rotation/rot_cost_diff_threshold", qc.rot_cost_diff_thr_, 0.25);
    register_and_get_params<int>("/quatro/rotation/num_max_iter", qc.quatro_max_iter_, 50);
}

void FastLioLocalizationQn::odomPcdCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg)
{
    PosePcd current_frame = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_); // to be checked if keyframe or not
    //// 1. realtime pose = last TF * odom
    current_frame.pose_corrected_eig_ = last_corrected_TF_ * current_frame.pose_eig_;
    geometry_msgs::msg::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(current_frame.pose_corrected_eig_, map_frame_);
    realtime_pose_pub_->publish(current_pose_stamped_);

    geometry_msgs::msg::TransformStamped tf_msg_ = poseEigToTransformStamped(
        current_frame.pose_corrected_eig_, node_->get_clock()->now(), map_frame_, "robot");
    tf_broadcaster_->sendTransform(tf_msg_);

    // pub current scan in corrected pose frame
    corrected_current_pcd_pub_->publish(pclToPclRos(transformPcd(current_frame.pcd_, current_frame.pose_corrected_eig_), map_frame_));

    if (!is_initialized_) //// init only once
    {
        // 1. save first keyframe
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            last_keyframe_ = current_frame;
        }
        current_keyframe_idx_++;
        //// 2. vis
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            updateOdomsAndPaths(current_frame);
        }
        is_initialized_ = true;
    }
    else
    {
        //// 1. check if keyframe
        if (checkIfKeyframe(current_frame, last_keyframe_))
        {
            // 2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                last_keyframe_ = current_frame;
            }
            current_keyframe_idx_++;
            //// 3. vis
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame);
            }
        }
    }
    return;
}

void FastLioLocalizationQn::matchingTimerFunc()
{
    if (!is_initialized_)
    {
        return;
    }

    //// 1. copy not processed keyframes
    high_resolution_clock::time_point t1_ = high_resolution_clock::now();
    PosePcd last_keyframe_copy;
    {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        last_keyframe_copy = last_keyframe_;
        last_keyframe_.processed_ = true;
    }
    if (last_keyframe_copy.idx_ == 0 || last_keyframe_copy.processed_)
    {
        return; // already processed or initial keyframe
    }

    //// 2. detect match and calculate TF
    // from last_keyframe_copy keyframe to map (saved keyframes) in threshold radius, get the closest keyframe
    int closest_keyframe_idx = map_matcher_->fetchClosestKeyframeIdx(last_keyframe_copy, saved_map_from_bag_);
    if (closest_keyframe_idx < 0)
    {
        return; // if no matched candidate
    }
    // Quatro + NANO-GICP to check match (from current_keyframe to closest keyframe in saved map)
    const RegistrationOutput &reg_output = map_matcher_->performMapMatcher(last_keyframe_copy,
                                                                           saved_map_from_bag_,
                                                                           closest_keyframe_idx);

    //// 3. handle corrected results
    if (reg_output.is_valid_) // TF the pose with the result of match
    {
        // RCLCPP_INFO(node_->get_logger(), "\033[1;32mMap matching accepted. Score: %.3f\033[0m", reg_output.score_);
        RCLCPP_INFO(node_->get_logger(), "\033[1;32mMap matching accepted. Score: %.3f\033[0m", reg_output.score_);
        last_corrected_TF_ = reg_output.pose_between_eig_ * last_corrected_TF_; // update TF
        Eigen::Matrix4d TFed_pose = reg_output.pose_between_eig_ * last_keyframe_copy.pose_corrected_eig_;
        // correct poses in vis data
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_.points[last_keyframe_copy.idx_] = pcl::PointXYZ(TFed_pose(0, 3), TFed_pose(1, 3), TFed_pose(2, 3));
            corrected_odom_path_.poses[last_keyframe_copy.idx_] = poseEigToPoseStamped(TFed_pose, map_frame_);
        }
        // map matches
        matched_pairs_xyz_.push_back({corrected_odoms_.points[last_keyframe_copy.idx_], raw_odoms_.points[last_keyframe_copy.idx_]}); // for vis
        map_match_pub_->publish(getMatchMarker(matched_pairs_xyz_));
    }
    high_resolution_clock::time_point t2_ = high_resolution_clock::now();

    //// 4. vis
    debug_src_pub_->publish(pclToPclRos(map_matcher_->getSourceCloud(), map_frame_));
    debug_dst_pub_->publish(pclToPclRos(map_matcher_->getTargetCloud(), map_frame_));
    debug_coarse_aligned_pub_->publish(pclToPclRos(map_matcher_->getCoarseAlignedCloud(), map_frame_));
    debug_fine_aligned_pub_->publish(pclToPclRos(map_matcher_->getFinalAlignedCloud(), map_frame_));
    // publish odoms and paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_->publish(corrected_odom_path_);
    }
    odom_pub_->publish(pclToPclRos(raw_odoms_, map_frame_));
    path_pub_->publish(raw_odom_path_);
    // publish saved map
    if (saved_map_vis_switch_ && saved_map_pub_->get_subscription_count() > 0)
    {
        saved_map_pub_->publish(pclToPclRos(saved_map_pcd_, map_frame_));
        saved_map_vis_switch_ = false;
    }
    if (!saved_map_vis_switch_ && saved_map_pub_->get_subscription_count() == 0)
    {
        saved_map_vis_switch_ = true;
    }
    high_resolution_clock::time_point t3_ = high_resolution_clock::now();
    RCLCPP_INFO(node_->get_logger(), "Matching: %.1fms, vis: %.1fms", duration_cast<microseconds>(t2_ - t1_).count() / 1e3, duration_cast<microseconds>(t3_ - t2_).count() / 1e3);
    return;
}

void FastLioLocalizationQn::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    raw_odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                                   pose_pcd_in.pose_eig_(1, 3),
                                   pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    raw_odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
}

visualization_msgs::msg::Marker FastLioLocalizationQn::getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &match_xyz_pairs)
{
    visualization_msgs::msg::Marker edges_;
    edges_.type = 5u;
    edges_.scale.x = 0.2f;
    edges_.header.frame_id = map_frame_;
    edges_.pose.orientation.w = 1.0f;
    edges_.color.r = 1.0f;
    edges_.color.g = 1.0f;
    edges_.color.b = 1.0f;
    edges_.color.a = 1.0f;
    for (size_t i = 0; i < match_xyz_pairs.size(); ++i)
    {
        geometry_msgs::msg::Point p_, p2_;
        p_.x = match_xyz_pairs[i].first.x;
        p_.y = match_xyz_pairs[i].first.y;
        p_.z = match_xyz_pairs[i].first.z;
        p2_.x = match_xyz_pairs[i].second.x;
        p2_.y = match_xyz_pairs[i].second.y;
        p2_.z = match_xyz_pairs[i].second.z;
        edges_.points.push_back(p_);
        edges_.points.push_back(p2_);
    }
    return edges_;
}

bool FastLioLocalizationQn::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    return keyframe_dist_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}

void FastLioLocalizationQn::loadMap(const std::string &saved_map_path)
{
    std::vector<sensor_msgs::msg::PointCloud2> load_pcd_vec;
    std::vector<geometry_msgs::msg::PoseStamped> load_pose_vec;

    rosbag2_cpp::Reader reader;

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = saved_map_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    // open the bag FILE
    reader.open(storage_options, converter_options);
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pointcloud2_serialization_;
    rclcpp::Serialization<geometry_msgs::msg::PoseStamped> posestamped_serialization_;

    while (reader.has_next())
    {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

        if (msg->topic_name == "/keyframe_pcd")
        {
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            sensor_msgs::msg::PointCloud2::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

            pointcloud2_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
            load_pcd_vec.push_back(*ros_msg);
        }
        else if (msg->topic_name == "/keyframe_pose")
        {
            rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
            geometry_msgs::msg::PoseStamped::SharedPtr ros_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();

            posestamped_serialization_.deserialize_message(&serialized_msg, ros_msg.get());
            load_pose_vec.push_back(*ros_msg);
        }
    }

    if (load_pcd_vec.size() != load_pose_vec.size())
    {
        RCLCPP_ERROR(node_->get_logger(), "WRONG BAG FILE!!!!!");
    }
    for (size_t i = 0; i < load_pose_vec.size(); ++i)
    {
        saved_map_from_bag_.push_back(PosePcdReduced(load_pose_vec[i], load_pcd_vec[i], i));
        saved_map_pcd_ += transformPcd(saved_map_from_bag_[i].pcd_, saved_map_from_bag_[i].pose_eig_);
    }
    saved_map_pcd_ = *voxelizePcd(saved_map_pcd_, voxel_res_);
    // bag.close();
    return;
}
