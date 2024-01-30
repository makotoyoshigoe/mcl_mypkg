// SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "mcl_mypkg/Particle.hpp"

namespace Mcl
{
Particle::Particle()
{
}

Particle::~Particle(){}

void Particle::getPose(float &x, float &y, float &t)
{
    x = x_;
    y = y_;
    t = t_;
}

float Particle::getWeight()
{
    return w_;
}

void Particle::setPose(float x, float y, float t)
{
    x_ = x;
    y_ = y;
    t_ = t;
}

void Particle::setWeight(float w)
{
    w_ = w;
}

}