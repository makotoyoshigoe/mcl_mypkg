// SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef PARTICLE__PARTICLE_HPP_
#define PARTICLE__PARTICLE_HPP_

#include <geometry_msgs/msg/pose.hpp>

namespace Mcl
{
class Particle
{
    private:
    float x_, y_, t_, w_;

    public:
    Particle();
    ~Particle();
    void getPose(float &x, float &y, float &t);
    float getWeight();
    void setPose(float x, float y, float t);
    void setWeight(float w);
};
}

#endif // PARTICLE__PARTICLE_HPP_