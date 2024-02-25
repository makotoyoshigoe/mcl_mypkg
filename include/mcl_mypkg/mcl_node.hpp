// SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef MCL_NODE__MCL_NODE_HPP_
#define MCL_NODE__MCL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "mcl_mypkg/LikelihoodFieldMap.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "mcl_mypkg/Mcl.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>

namespace Mcl 
{
class MclNode : public rclcpp::Node
{
    public:
    MclNode();
    ~MclNode();
    void publishParticles();
    void loop();

    protected:
    void initPubSub();
    void initParam();
    void initVariable();

    void mapCb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
    void initialposeCb(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
    void pubOdomFrame(float x, float y, float t);
    bool getOdom(float &x, float &y, float &t);

    private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr likelihood_map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;

    nav_msgs::msg::OccupancyGrid map_;

    bool init_map_;
    bool init_mcl_;
    float likelihood_range_;
    int particle_num_;
    
    std::shared_ptr<LikelihoodFieldMap> likelihood_field_map_;
    std::shared_ptr<Mcl> mcl_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};
}

#endif // MCL_MYPKG__MCL_MYPKG_HPP_
 