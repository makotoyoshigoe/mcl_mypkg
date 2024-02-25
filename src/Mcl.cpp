// SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "mcl_mypkg/Mcl.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace Mcl
{
Mcl::Mcl(int particle_num, std::vector<double> motion_noises)
{
    particle_num_ = particle_num;
    particles_.resize(particle_num_);
    pre_odom_init_ = false;
    motion_noises_ = motion_noises;
}

Mcl::~Mcl(){}

void Mcl::initParticles(float init_x, float init_y, float init_t, float init_w)
{
    for(auto &p : particles_){
        p.pose.x_ = init_x;
        p.pose.y_ = init_y;
        p.pose.t_ = init_t;
        p.w_ = init_w;
    }
}

void Mcl::particles2PoseArray(geometry_msgs::msg::PoseArray &pose_array)
{
    for(auto &p : particles_){
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.pose.x_;
        pose.position.y = p.pose.y_;
        tf2::Quaternion q;
        q.setRPY(0., 0., p.pose.t_);
        tf2::convert(q, pose.orientation);
        pose_array.poses.push_back(pose);
    }
}

void Mcl::meanParticles(float &mean_x, float &mean_y, float &mean_t)
{
    for(auto &p : particles_){
        mean_x += p.pose.x_;
        mean_y += p.pose.y_;
        mean_t += p.pose.t_;
    }
    mean_x /= particle_num_;
    mean_y /= particle_num_;
    mean_t /= particle_num_;
}

void Mcl::motionUpdate(float x, float y, float t)
{
    Pose cur_odom = {x, y, t};
    if(pre_odom_init_ == false){
        pre_odom_ = cur_odom;
        pre_odom_init_ = true;
    }
    Pose diff = cur_odom - pre_odom_;
}

}