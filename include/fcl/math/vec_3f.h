/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Jia Pan */

#ifndef FCL_VEC_3F_H
#define FCL_VEC_3F_H

# include <Eigen/Core>
# include <Eigen/Geometry>
# include "fcl/config.h"
# include "fcl/data_types.h"

namespace fcl
{
  typedef Eigen::Matrix <FCL_REAL, 3, 1> Vec3f;

  inline Vec3f min(const Vec3f& x, const Vec3f& y)
  {
    return Vec3f(x.cwiseMin (y));
  }

  inline Vec3f max(const Vec3f& x, const Vec3f& y)
  {
    return Vec3f(x.cwiseMax (y));
  }

  inline FCL_REAL triple(const Vec3f& x, const Vec3f& y,
			 const Vec3f& z)
  {
    return x.dot(y.cross(z));
  }
  inline void generateCoordinateSystem(const Vec3f& w, Vec3f& u, Vec3f& v)
  {
    FCL_REAL inv_length;
    if(std::abs(w[0]) >= std::abs(w[1])) {
      inv_length = (FCL_REAL)1.0 / sqrt(w[0] * w[0] + w[2] * w[2]);
      u[0] = -w[2] * inv_length;
      u[1] = (FCL_REAL)0;
      u[2] = w[0] * inv_length;
      v[0] = w[1] * u[2];
      v[1] = w[2] * u[0] - w[0] * u[2];
      v[2] = -w[1] * u[0];
    } else {
      inv_length = (FCL_REAL)1.0 / sqrt(w[1] * w[1] + w[2] * w[2]);
      u[0] = (FCL_REAL)0;
      u[1] = w[2] * inv_length;
      u[2] = -w[1] * inv_length;
      v[0] = w[1] * u[2] - w[2] * u[1];
      v[1] = -w[0] * u[2];
      v[2] = w[0] * u[1];
    }
  }
}
#endif
