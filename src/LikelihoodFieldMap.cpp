// SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "mcl_mypkg/LikelihoodFieldMap.hpp"
#include <rclcpp/rclcpp.hpp>

namespace Mcl
{
LikelihoodFieldMap::LikelihoodFieldMap(nav_msgs::msg::OccupancyGrid map, float likelihood_range)
{   
    likelihood_range_ = likelihood_range;
    likelihood_map_ = map;
    width_ = likelihood_map_.info.width;
    height_ = likelihood_map_.info.height;
    resolution_ = likelihood_map_.info.resolution;
    x_lim_ = width_ * resolution_;
    y_lim_ = height_ * resolution_;
    createLikelihoodFieldMap();
}

LikelihoodFieldMap::~LikelihoodFieldMap()
{
}

void LikelihoodFieldMap::createLikelihoodFieldMap()
{
    RCLCPP_INFO(rclcpp::get_logger("mcl_node"), "Create likelihood map");

    for(int y=0; y<height_; ++y){
        for(int x=0; x<width_; ++x){
            if(likelihood_map_.data[x+width_*y] == 100){
                setLikelihood(x, y);
            }
        }
    }
}

void LikelihoodFieldMap::setLikelihood(int x, int y)
{
    float grid = likelihood_range_ / resolution_;
    float likelihoods[static_cast<int>(grid)+1];
    for(int i=0; i<=grid; ++i) likelihoods[i] = 100 * (1 - i / grid);

    for(int i=-grid; i<=grid; ++i){
        for(int j=-grid; j<=grid; ++j){
            int index = (x+j)+width_*(y+i);
            if(index >= 0 && index < height_*width_){
                likelihood_map_.data[index] = std::max(
                    static_cast<float>(likelihood_map_.data[index]), 
                    std::min(likelihoods[abs(i)], likelihoods[abs(j)])
                );
            }
        }
    }
}

float LikelihoodFieldMap::getLikelihood(float x, float y)
{
    int i = xy2index(x, y);
    if(i >= 0 && i < height_*width_){
        return likelihood_map_.data[x+width_*y];
    } else {
        return 0.;
    }
}

int LikelihoodFieldMap::xy2index(float x, float y){ return static_cast<int>(x / resolution_ + width_ *  y / resolution_); }

nav_msgs::msg::OccupancyGrid LikelihoodFieldMap::getLikelihoodFieldMap(){ return likelihood_map_; }

}