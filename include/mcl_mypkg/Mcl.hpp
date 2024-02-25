// SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef MCL__MCL_HPP_
#define MCL__MCL_HPP_

#include <vector>
#include <geometry_msgs/msg/pose_array.hpp>

namespace Mcl
{
class Mcl
{
    private:
        int particle_num_;
        bool pre_odom_init_;

        struct Pose{
            float x_;
            float y_;
            float t_;
            Pose operator-(const Pose &other) {
                Pose pose;
                pose.x_ = x_ - other.x_;
                pose.y_ = y_ - other.y_;  
                return pose;
            }
        };
        struct Particle{
            Pose pose;
            float w_;
        };

        std::vector<Particle> particles_;
        std::vector<double> motion_noises_;
        Pose pre_odom_;

    public:
        Mcl(int particle_num, std::vector<double> motion_noises);
        ~Mcl();
        void initParticles(float init_x, float init_y, float init_t, float init_w);
        void particles2PoseArray(geometry_msgs::msg::PoseArray &pose_array);
        void meanParticles(float &mean_x, float &mean_y, float &mean_t);
        void motionUpdate(float x, float y, float t);
};
}

#endif // MCL__MCL_HPP_