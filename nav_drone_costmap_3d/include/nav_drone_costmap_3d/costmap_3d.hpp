// Copyright (c) 2022 Eric Slaghuis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <float.h>
#include <math.h>
#include <Eigen/Dense>
#include <vector>

namespace nav_drone_costmap_3d {

// Be very careful choosing the resolution! Valid resolutions must fullfill:
// 180 % (2 * ALPHA_RES) = 0
// Examples of valid resolution values: 1, 3, 5, 6, 10, 15, 18, 30, 45, 60
const int ALPHA_RES = 6;
const int GRID_LENGTH_Z = 360 / ALPHA_RES;
const int GRID_LENGTH_E = 180 / ALPHA_RES;

class Costmap3D {
  int resolution_;
  int z_dim_;
  int e_dim_;
  Eigen::MatrixXf weight_;

  /**
  * @brief     wraps elevation and azimuth indeces around the costmap
  * @param     x, elevation angle index
  * @param     y, azimuth angle index
  **/
  inline void wrap_index(int &x, int &y) const {
    x = x % e_dim_;
    if (x < 0) x += e_dim_;
    y = y % z_dim_;
    if (y < 0) y += z_dim_;
  }

 public:
  Costmap3D(const int res);
  ~Costmap3D() = default;
  
  inline int z_dim() { return z_dim_; }
  inline int e_dim() { return e_dim_; }

  /**
  * @brief     getter method for costmap cell weight
  * @param[in] x, elevation angle index
  * @param[in] y, azimuth angle index
  * @returns   weight to the vehicle of obstacle mapped to (x, y) cell [m]
  **/
  inline float get_weight(int x, int y) const {
    wrap_index(x, y);
    return weight_(x, y);
  }
  
  inline bool path_available(int x, int y) const {
    return weight_(x, y) < 0.001;
  }  

  /**
  * @brief     setter method for costmap cell weight
  * @param[in] x, elevation angle index
  * @param[in] y, azimuth angle index
  * @param[in] value, weight to the vehicle of obstacle mapped to (x, y) cell
  *[m]
  **/
  inline void set_weight(int x, int y, float value) { weight_(x, y) = value; }

  /**
  * @brief     adder method for costmap cell weight
  * @param[in] x, elevation angle index
  * @param[in] y, azimuth angle index
  * @param[in] value, weight to the vehicle of obstacle mapped to (x, y) cell
  *[m]
  **/
  inline void add_weight(int x, int y, float value) { weight_(x, y) += value; }
 
  /**
  * @brief     Compute the upsampled version of the costmap
  * @param[in] This object. Needs to be a costmap the larger bin size
  *(ALPHA_RES * 2)
  * @details   The costmap is upsampled to get the same costmap at regular
  *bin size (ALPHA_RES).
  *            This means the costmap matrix will be double the size in each
  *dimension
  * @returns   Modifies the object it is called from to have regular resolution
  * @warning   Can only be called from a large bin size costmap
  **/
  void upsample();

  /**
  * @brief     Compute the downsampled version of the costmap
  * @param[in] this object. Needs to be a costmap with regular bin size
  *(ALPHA_RES)
  * @details   The costmap is downsampled to get the same costmap at larger
  *bin size (ALPHA_RES/2).
  *            This means the costmap matrix will be half the size in each
  *dimension
  * @returns   Modifies the object it is called from to have larger bins
  * @warning   Can only be called from a regular bin size costmap
  **/
  void downsample();

  /**
  * @brief     resets all costmap cells age and weight to zero
  **/
  void set_zero();

  /**
  * @brief     determines whether the costmap is empty (weight layer
  *            contains no weight bigger than zero)
  * @returns   whether costmap is empty
  **/
  bool is_empty() const;
  
  /**
  * @brief     translates the weights to either 0 or 1 depending on the 
  *            existing weight values)
  **/
  void go_binary(float theta_low, float theta_high);
  
  

};
}  // namespace nav_drone_costmap_d3