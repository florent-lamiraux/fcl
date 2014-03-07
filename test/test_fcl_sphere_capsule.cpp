/*
 *  Software License Agreement (BSD License)
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

/** \author Martin Felis <martin.felis@iwr.uni-heidelberg.de> */

#define BOOST_TEST_MODULE "FCL_SPHERE_CAPSULE"
#include <boost/test/unit_test.hpp>

#include "fcl/collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace fcl;

BOOST_AUTO_TEST_CASE(Sphere_Capsule_Intersect_test_separated_z)
{
  Vec3f v1; v1 << 0., 0., -50;
  Vec3f v2; v2 << 0., 0., 200;

	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (v1);

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (v2);

	BOOST_CHECK (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL, NULL, NULL));
}

BOOST_AUTO_TEST_CASE(Sphere_Capsule_Intersect_test_separated_z_negative)
{
  Vec3f v3; v3 << 0., 0., 50;
  Vec3f v4; v4 << 0., 0., -200;
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (v3);

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (v4);

	BOOST_CHECK (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL, NULL, NULL));
}

BOOST_AUTO_TEST_CASE(Sphere_Capsule_Intersect_test_separated_x)
{
  Vec3f v1; v1 << 0., 0., -50;
  Vec3f v5; v5 << 150., 0., 0.;
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (v1);

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (v5);

	BOOST_CHECK (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL, NULL, NULL));
}

BOOST_AUTO_TEST_CASE(Sphere_Capsule_Intersect_test_separated_capsule_rotated)
{
  Vec3f v1; v1 << 0., 0., -50;
  Vec3f v5; v5 << 150., 0., 0.;
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (v1);

	Capsule capsule (50, 200.);
	Matrix3f rotation;
	setEulerZYX (M_PI * 0.5, 0., 0., rotation);
	Transform3f capsule_transform (rotation, v5);

	BOOST_CHECK (!solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, NULL, NULL, NULL));
}

BOOST_AUTO_TEST_CASE(Sphere_Capsule_Intersect_test_penetration_z)
{
  Vec3f zero; zero << 0., 0., 0.;
  Vec3f v1; v1 << 0., 0., -50;
  Vec3f v6; v6 << 0., 0., 125;
  Vec3f v7; v7 << 0., 0., 1.;
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (v1);

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (v6);

	FCL_REAL penetration = 0.;
	Vec3f contact_point;
	Vec3f normal;

	bool is_intersecting = solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, &contact_point, &penetration, &normal);

	BOOST_CHECK (is_intersecting);
	BOOST_CHECK (penetration == 25.);
	BOOST_CHECK (v7 == (normal));
	BOOST_CHECK (zero == (contact_point));
}

BOOST_AUTO_TEST_CASE(Sphere_Capsule_Intersect_test_penetration_z_rotated)
{
  Vec3f v7; v7 << 0., 0., 1.;
  Vec3f v8; v8 << 0., 50., 75;
  Vec3f v9; v9 << 0., 0., 50.;
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (Vec3f (0., 0., 0));

	Capsule capsule (50, 200.);
	Matrix3f rotation;
	setEulerZYX (M_PI * 0.5, 0., 0., rotation);
	Transform3f capsule_transform (rotation, v8);

	FCL_REAL penetration = 0.;
	Vec3f contact_point;
	Vec3f normal;

	bool is_intersecting = solver.shapeIntersect(sphere1, sphere1_transform, capsule, capsule_transform, &contact_point, &penetration, &normal);

	BOOST_CHECK (is_intersecting);
	BOOST_CHECK_CLOSE (25, penetration, solver.collision_tolerance);
	BOOST_CHECK (v7 == (normal));
	BOOST_CHECK ((v9 - contact_point).norm () <
		     solver.collision_tolerance);
}

BOOST_AUTO_TEST_CASE(Sphere_Capsule_Distance_test_collision)
{
  Vec3f v1; v1 << 0., 0., -50;
  Vec3f v10; v10 << 0., 0., 100;
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (v1);

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (v10);

	FCL_REAL distance;

	BOOST_CHECK (!solver.shapeDistance(sphere1, sphere1_transform, capsule, capsule_transform, &distance));

}

BOOST_AUTO_TEST_CASE(Sphere_Capsule_Distance_test_separated)
{
  Vec3f v1; v1 << 0., 0., -50;
  Vec3f v11; v11 << 0., 0., 175;
	GJKSolver_libccd solver;

	Sphere sphere1 (50);
	Transform3f sphere1_transform;
	sphere1_transform.setTranslation (v1);

	Capsule capsule (50, 200.);
	Transform3f capsule_transform (v11);

	FCL_REAL distance = 0.;
	Vec3f p1;
	Vec3f p2;
	bool is_separated = solver.shapeDistance(sphere1, sphere1_transform, capsule, capsule_transform, &distance);

	BOOST_CHECK (is_separated);
	BOOST_CHECK (distance == 25.);
}
