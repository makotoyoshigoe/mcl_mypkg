// SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "mcl_mypkg/Mcl.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace Mcl
{
Mcl::Mcl(int particle_num)
{
    particle_num_ = particle_num;
    particles_.resize(particle_num_);
}

Mcl::~Mcl(){}

void Mcl::initParticles(float init_x, float init_y, float init_t, float init_w)
{
    for(auto &p : particles_){
        p.setPose(init_x, init_y, init_t);
        p.setWeight(init_w);
    }
}

void Mcl::particles2PoseArray(geometry_msgs::msg::PoseArray &pose_array)
{
    for(auto &p : particles_){
        float x, y, t;
        p.getPose(x, y, t);
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        tf2::Quaternion q;
        q.setRPY(0., 0., t);
        tf2::convert(q, pose.orientation);
        pose_array.poses.push_back(pose);
    }
}

void Mcl::meanParticles(float &mean_x, float &mean_y, float &mean_t)
{
    float x, y, t;
    for(auto &p : particles_){
        p.getPose(x, y, t);
        mean_x += x;
        mean_y += y;
        mean_t += t;
    }
    mean_x /= particle_num_;
    mean_y /= particle_num_;
    mean_t /= particle_num_;
}

}