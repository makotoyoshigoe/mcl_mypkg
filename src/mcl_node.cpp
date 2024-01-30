// SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "mcl_mypkg/mcl_node.hpp"
#include <rclcpp/qos.hpp>
#include <iostream>
#include <algorithm>
#include <tf2_bullet/tf2_bullet.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>

namespace Mcl
{
MclNode::MclNode() : Node("mcl_node")
{
    initParam();
    initVariable();
    initPubSub();
}

MclNode::~MclNode(){}

void MclNode::initPubSub()
{
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 2, std::bind(&MclNode::mapCb, this, std::placeholders::_1));
    initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 2, std::bind(&MclNode::initialposeCb, this, std::placeholders::_1));

    likelihood_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map/likelihood", rclcpp::QoS(10));
    particles_pub_  = this->create_publisher<geometry_msgs::msg::PoseArray>("particles", rclcpp::QoS(10));
}

void MclNode::initParam()
{
    this->declare_parameter("likelihood_range", 1.0);
    this->get_parameter("likelihood_range", likelihood_range_);
    this->declare_parameter("particle_num", 1000);
    this->get_parameter("particle_num", particle_num_);
}

void MclNode::initVariable()
{
    init_map_ = false;
    init_mcl_ = false;
    mcl_.reset(new Mcl(particle_num_));
}

void MclNode::mapCb(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
    if(!init_map_){
        RCLCPP_INFO(this->get_logger(), "received map");
        map_ = *msg;
        likelihood_field_map_.reset(new LikelihoodFieldMap(map_, likelihood_range_));
        likelihood_map_pub_->publish(likelihood_field_map_->getLikelihoodFieldMap());
        init_map_ = true;
    }
}

void MclNode::initialposeCb(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received initialpose");
    float x = static_cast<float>(msg->pose.pose.position.x);
    float y = static_cast<float>(msg->pose.pose.position.y);
    float t = static_cast<float>(tf2::getYaw(msg->pose.pose.orientation));
    float w = 1. / static_cast<float>(particle_num_);
    mcl_->initParticles(x, y, t, w);
    init_mcl_ = true;
}

void MclNode::publishParticles()
{
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = "map";
    mcl_->particles2PoseArray(msg);
    msg.header.stamp = now();
    particles_pub_->publish(msg);
}

void MclNode::loop()
{
    if(init_map_ && init_mcl_){
        publishParticles();
    }
}

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mcl::MclNode>();
    rclcpp::Rate loop_rate(20);
    while(rclcpp::ok()){
        node->loop();
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}