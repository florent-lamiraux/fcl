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

#ifndef FCL_MATRIX_3F_H
#define FCL_MATRIX_3F_H

#include <Eigen/Eigenvalues>
#include <fcl/math/vec_3f.h>

namespace fcl
{

  typedef Eigen::Matrix <FCL_REAL, 3, 3>  Matrix3f;

  inline void hat (Matrix3f& mat, const Vec3f& vec)
  {
    mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;
  }

  inline void relativeTransform(const Matrix3f& R1, const Vec3f& t1,
				const Matrix3f& R2, const Vec3f& t2,
				Matrix3f& R, Vec3f& t)
  {
    R = R1.transpose () * R2;
    t = R1.transpose () * (t2 - t1);
  }

  inline void eigen (Matrix3f& m, FCL_REAL sigma [3], Vec3f axis [3])
  {
    Eigen::SelfAdjointEigenSolver <Matrix3f> es (m);
    sigma [0] = es.eigenvalues () [0];
    sigma [1] = es.eigenvalues () [1];
    sigma [2] = es.eigenvalues () [2];
    axis  [0] = es.eigenvectors ().col (0);
    axis  [1] = es.eigenvectors ().col (1);
    axis  [2] = es.eigenvectors ().col (2);
  }

  /// @brief Class for variance matrix in 3d
  class Variance3f
  {
  public:
    /// @brief Variation matrix
    Matrix3f Sigma;

    /// @brief Variations along the eign axes
    FCL_REAL sigma[3];

    /// @brief Eigen axes of the variation matrix
    Vec3f axis[3];

    Variance3f() {}

  Variance3f(const Matrix3f& S) : Sigma(S)
    {
      init();
    }

    /// @brief init the Variance
    void init()
    {
      eigen (Sigma, sigma, axis);
    }

    /// @brief Compute the sqrt of Sigma matrix based on the eigen decomposition result, this is useful when the uncertainty matrix is initialized as a square variation matrix
    Variance3f& sqrt()
      {
	for(std::size_t i = 0; i < 3; ++i)
	  {
	    if(sigma[i] < 0) sigma[i] = 0;
	    sigma[i] = std::sqrt(sigma[i]);
	  }


	Sigma.setZero();
	for(std::size_t i = 0; i < 3; ++i)
	  {
	    for(std::size_t j = 0; j < 3; ++j)
	      {
		Sigma(i, j) += sigma[0] * axis[0][i] * axis[0][j];
		Sigma(i, j) += sigma[1] * axis[1][i] * axis[1][j];
		Sigma(i, j) += sigma[2] * axis[2][i] * axis[2][j];
	      }
	  }

	return *this;
      }
  };

  /// @brief Set the matrix from euler angles YPR around ZYX axes
  /// @param eulerX Roll about X axis
  /// @param eulerY Pitch around Y axis
  /// @param eulerZ Yaw aboud Z axis
  /// @retval res resulting rotation matrix
  ///
  /// These angles are used to produce a rotation matrix. The euler
  /// angles are applied in ZYX order. I.e a vector is first rotated
  /// about X then Y and then Z
  inline void setEulerZYX(FCL_REAL eulerX, FCL_REAL eulerY, FCL_REAL eulerZ,
			  Matrix3f& res)
  {
    FCL_REAL ci(cos(eulerX));
    FCL_REAL cj(cos(eulerY));
    FCL_REAL ch(cos(eulerZ));
    FCL_REAL si(sin(eulerX));
    FCL_REAL sj(sin(eulerY));
    FCL_REAL sh(sin(eulerZ));
    FCL_REAL cc = ci * ch;
    FCL_REAL cs = ci * sh;
    FCL_REAL sc = si * ch;
    FCL_REAL ss = si * sh;

    res << cj * ch, sj * sc - cs, sj * cc + ss,
      cj * sh, sj * ss + cc, sj * cs - sc,
      -sj,     cj * si,      cj * ci;

  }

  /// @brief Set the matrix from euler angles using YPR around YXZ respectively
  /// @param yaw Yaw about Y axis
  /// @param pitch Pitch about X axis
  /// @param roll Roll about Z axis
  /// @retval res resulting rotation matrix
  inline void setEulerYPR(FCL_REAL yaw, FCL_REAL pitch, FCL_REAL roll,
			  Matrix3f& res)
  {
    setEulerZYX(roll, pitch, yaw, res);
  }
}
#endif
