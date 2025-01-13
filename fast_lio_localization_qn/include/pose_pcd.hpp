#ifndef FAST_LIO_LOCALIZATION_QN_POSE_PCD_HPP
#define FAST_LIO_LOCALIZATION_QN_POSE_PCD_HPP

///// coded headers
#include "utilities.hpp"

struct PosePcd
{
    pcl::PointCloud<PointType> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    int idx_;
    bool processed_ = false;
    PosePcd() {}
    PosePcd(const nav_msgs::msg::Odometry &odom_in,
            const sensor_msgs::msg::PointCloud2 &pcd_in,
            const int &idx_in);
};

struct PosePcdReduced
{
    pcl::PointCloud<PointType> pcd_;
    Eigen::Matrix4d pose_eig_ = Eigen::Matrix4d::Identity();
    int idx_;
    PosePcdReduced() {}
    PosePcdReduced(const geometry_msgs::msg::PoseStamped &pose_in,
                   const sensor_msgs::msg::PointCloud2 &pcd_in,
                   const int &idx_in);
};

inline PosePcd::PosePcd(const nav_msgs::msg::Odometry &odom_in,
                        const sensor_msgs::msg::PointCloud2 &pcd_in,
                        const int &idx_in)
{
    Eigen::Quaterniond quat(odom_in.pose.pose.orientation.w,
                            odom_in.pose.pose.orientation.x,
                            odom_in.pose.pose.orientation.y,
                            odom_in.pose.pose.orientation.z);
    pose_eig_.block<3, 3>(0, 0) = quat.toRotationMatrix();
    pose_eig_(0, 3) = odom_in.pose.pose.position.x;
    pose_eig_(1, 3) = odom_in.pose.pose.position.y;
    pose_eig_(2, 3) = odom_in.pose.pose.position.z;
    pose_corrected_eig_ = pose_eig_;
    pcl::PointCloud<PointType> tmp_pcd_;
    pcl::fromROSMsg(pcd_in, tmp_pcd_);
    pcd_ = transformPcd(tmp_pcd_,
                        pose_eig_.inverse()); // FAST-LIO publish data in world
                                              // frame, so save it in LiDAR frame
    idx_ = idx_in;
}

inline PosePcdReduced::PosePcdReduced(const geometry_msgs::msg::PoseStamped &pose_in,
                                      const sensor_msgs::msg::PointCloud2 &pcd_in,
                                      const int &idx_in)
{
    Eigen::Quaterniond quat(pose_in.pose.orientation.w,
                            pose_in.pose.orientation.x,
                            pose_in.pose.orientation.y,
                            pose_in.pose.orientation.z);
    pose_eig_.block<3, 3>(0, 0) = quat.toRotationMatrix();
    pose_eig_(0, 3) = pose_in.pose.position.x;
    pose_eig_(1, 3) = pose_in.pose.position.y;
    pose_eig_(2, 3) = pose_in.pose.position.z;
    pcl::fromROSMsg(pcd_in, pcd_);
    idx_ = idx_in;
}

#endif // FAST_LIO_LOCALIZATION_QN_POSE_PCD_HPP
