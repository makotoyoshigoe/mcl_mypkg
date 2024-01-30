// SPDX-FileCopyrightText: 2024 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#ifndef LIKELIHOOD_FIELD_MAP__LIKELIHOOD_FIELD_MAP_HPP_
#define LIKELIHOOD_FIELD_MAP__LIKELIHOOD_FIELD_MAP_HPP_

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace Mcl
{
class LikelihoodFieldMap
{
    private:
    nav_msgs::msg::OccupancyGrid likelihood_map_;
    int width_;
    int height_;
    float resolution_;
    float likelihood_range_;
    float x_lim_;
    float y_lim_;

    public:
    LikelihoodFieldMap(nav_msgs::msg::OccupancyGrid map, float likelihood_range);
    ~LikelihoodFieldMap();
    nav_msgs::msg::OccupancyGrid getLikelihoodFieldMap();
    float getLikelihood(float x, float y);
    void createLikelihoodFieldMap();
    void setLikelihood(int x, int y);
    int xy2index(float x, float y);
};
}

#endif // LIKELIHOOD_FIELD_MAP__LIKELIHOOD_FIELD_MAP_HPP_