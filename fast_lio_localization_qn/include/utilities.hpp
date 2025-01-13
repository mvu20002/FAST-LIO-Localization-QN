#ifndef FAST_LIO_LOCALIZATION_QN_UTILITIES_HPP
#define FAST_LIO_LOCALIZATION_QN_UTILITIES_HPP

///// common headers
#include <string>
///// ROS
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf2/LinearMath/Transform.h>  // to Quaternion_to_euler
// #include <tf/transform_datatypes.h>   // createQuaternionFromRPY
// #include <tf_conversions/tf_eigen.h>  // tf <-> eigen
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
///// PCL
#include <pcl/point_types.h>                 //pt
#include <pcl/point_cloud.h>                 //cloud
#include <pcl/conversions.h>                 //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h> //voxelgrid
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)

using PointType = pcl::PointXYZI;

//////////////////////////////////////////////////////////////////////
///// conversions

inline geometry_msgs::msg::PoseStamped poseEigToPoseStamped(const Eigen::Matrix4d &pose_eig_in,
                                                       std::string frame_id = "map")
{
    Eigen::Quaterniond quat(pose_eig_in.block<3, 3>(0, 0));
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = pose_eig_in(0, 3);
    pose.pose.position.y = pose_eig_in(1, 3);
    pose.pose.position.z = pose_eig_in(2, 3);
    pose.pose.orientation.w = quat.w();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    return pose;
}

inline tf2::Transform poseEigToROSTf(const Eigen::Matrix4d &pose)
{
    Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
    transform.setRotation(tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
    return transform;
}

template<typename T>
inline sensor_msgs::msg::PointCloud2 pclToPclRos(pcl::PointCloud<T> cloud,
                                            std::string frame_id = "map")
{
    sensor_msgs::msg::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

///// transformation
template<typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose_tf)
{
    if (cloud_in.size() == 0)
    {
        return cloud_in;
    }
    pcl::PointCloud<T> pcl_out = cloud_in;
    pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);
    return pcl_out;
}


inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType> &pcd_in,
                                                   const float voxel_res)
{
    static pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<PointType>::Ptr pcd_in_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
    pcd_in_ptr->reserve(pcd_in.size());
    pcd_out->reserve(pcd_in.size());
    *pcd_in_ptr = pcd_in;
    voxelgrid.setInputCloud(pcd_in_ptr);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

inline pcl::PointCloud<PointType>::Ptr voxelizePcd(const pcl::PointCloud<PointType>::Ptr &pcd_in,
                                                   const float voxel_res)
{
    static pcl::VoxelGrid<PointType> voxelgrid;
    voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
    pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
    pcd_out->reserve(pcd_in->size());
    voxelgrid.setInputCloud(pcd_in);
    voxelgrid.filter(*pcd_out);
    return pcd_out;
}

#endif
