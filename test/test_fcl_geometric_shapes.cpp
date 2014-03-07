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


#define BOOST_TEST_MODULE "FCL_GEOMETRIC_SHAPES"
#include <boost/test/unit_test.hpp>

#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "test_fcl_utility.h"
#include "fcl/ccd/motion.h"
#include <iostream>

using namespace fcl;

FCL_REAL extents [6] = {0, 0, 0, 10, 10, 10};

GJKSolver_libccd solver1;
GJKSolver_indep solver2;

static  Vec3f zero, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13,
  v14, v15, v16, v17, v18, v19, v20, v21, v22, v23, v24, v25, v26, v27, v28,
  v29, v30, v31, v32, v33, v34, v35, v36, v37, v38, v39, v40, v41, v42, v43,
  v44, v45, v46, v47, v48, v49, v50, v51, v52, v53, v54, v55, v56, v57, v58,
  v59, v60, v61, v62, v63, v64, v65, v66, v67, v68, v69, v70, v71, v72, v73,
  v74, v75, v76, v77, v78, v79, v80, v81, v82, v83, v84, v85, v86, v87, v88;

void init ()
{
  zero.setZero ();
  v1 << -1, 0, 0;
  v2 << 1, 0, 0;
  v3 << 5, 0, 0;
  v4 << -5, 0, 0;
  v5 << -10.1, 0, 0;
  v6 << 10.1, 0, 0;
  v7 << -1.25, 0, 0;
  v8 << 1.25, 0, 0;
  v9 << 2.51, 0, 0;
  v10 << -2.51, 0, 0;
  v11 << -2.5, 0, 0;
  v12 << -2.5, 0, -2.5;
  v13 << -0.625, 0, 0;
  v14 << -1.875, 0, 0;
  v15 << 0.005, 0, 0;
  v16 << 2.5, 0, 0;
  v17 << -3.75, 0, 0;
  v18 << 5.1, 0, 0;
  v19 << 2.5, 0, -2.5;
  v20 << -5.1, 0, 0;
  v21 << 0, 1, 0;
  v22 << 0, -1, 0;
  v23 << 0, 2.5, 0;
  v24 << 0, 2.5, -2.5;
  v25 << 0, -2.5, 0;
  v26 << 0, -2.5, -2.5;
  v27 << 0, 5.1, 0;
  v28 << 0, -5.1, 0;
  v29 << 0, 0, 1;
  v30 << 0, 0, -1;
  v31 << 0, 0, 2.5;
  v32 << 0, 0, -2.5;
  v33 << 0, 0, 10.1;
  v34 << 9.9, 0, 0;
  v35 << 10, 0, 0;
  v36 << 0, 0, 9.9;
  v37 << 0, 0, 10;
  v38 << -20.0, -20.0, -20.0;
  v39 << 20.0, 20.0, 20.0;
  v40 << 40, 0, 0;
  v41 << 30, 0, 0;
  v42 << 30.01, 0, 0;
  v43 << 29.9, 0, 0;
  v44 << -29.9, 0, 0;
  v45 << -30, 0, 0;
  v46 << -30.01, 0, 0;
  v47 << 15, 0, 0;
  v48 << 15.01, 0, 0;
  v49 << 0, 0, -1;
  v50 << 22.5, 0, 0;
  v51 << 22.501, 0, 0;
  v52 << 22.4, 0, 0;
  v53 << 9.9, 0, 0;
  v54 << 10.01, 0, 0;
  v55 << 10.001, 0, 0;
  v56 << 0, 0, 9.9;
  v57 << 0, 0, 10.01;
  v58 << -7.5, 0, 0;
  v59 << 0.05, 0, 0;
  v60 << 0, -1.25, 0;
  v61 << 0, -3.75, 0;
  v62 << 0, 0.05, 0;
  v63 << 0, 0, -5;
  v64 << 0, 0, -3.75;
  v65 << 0, 0, -6.25;
  v66 << 0, 0, -10.1;
  v67 << 0, 0, 0.05;
  v68 << 0, 0, -5.1;
  v69 << 0, 0, -1.25;
  v70 << 0, 0, 5.1;
  v71 << -2.5, 0, -5;
  v72 << -1.25, 0, -5;
  v73 << -3.75, 0, -5;
  v74 << 0.05, 0, -5;
  v75 << 0, -2.5, -5;
  v76 << 0, -1.25, -5;
  v77 << 0, -3.75, -5;
  v78 << 0, 0.05, -5;
  v79 << 0, 40, 0;
  v80 << 30.1, 0, 0;
  v81 << 20.1, 0, 0;
  v82 << 0, 20.1, 0;
  v83 << 10.1, 10.1, 0;
  v84 << 15.1, 0, 0;
  v85 << 20, 0, 0;
  v86 << 22.6, 0, 0;
  v87 << 0, 0, 40;
  v88 << 22.51, 0, 0;
  } // end init

#define BOOST_CHECK_FALSE(p) BOOST_CHECK(!(p))

BOOST_AUTO_TEST_CASE(gjkcache)
{
  init ();
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  CollisionRequest request;
  request.enable_cached_gjk_guess = true;
  request.gjk_solver_type = GST_INDEP;

  Transform3f init (v38);
  Transform3f end (v39);
  TranslationMotion motion (init, end);

  int N = 1000;  
  FCL_REAL dt = 1.0 / (N - 1);

  /// test exploiting spatial coherence
  Timer timer1;
  timer1.start();
  std::vector<bool> result1(N);
  for(int i = 0; i < N; ++i)
  {
    motion.integrate(dt * i);
    Transform3f tf;
    motion.getCurrentTransform(tf);

    CollisionResult result;

    collide(&s1, Transform3f(), &s2, tf, request, result);
    result1[i] = result.isCollision();
    request.cached_gjk_guess = result.cached_gjk_guess; // use cached guess
  }

  timer1.stop();

  /// test without exploiting spatial coherence
  Timer timer2;
  timer2.start();
  std::vector<bool> result2(N);
  request.enable_cached_gjk_guess = false;
  for(int i = 0; i < N; ++i)
  {
    motion.integrate(dt * i);
    Transform3f tf;
    motion.getCurrentTransform(tf);

    CollisionResult result;

    collide(&s1, Transform3f(), &s2, tf, request, result);
    result2[i] = result.isCollision();
  }

  timer2.stop();

  std::cout << timer1.getElapsedTime() << " " << timer2.getElapsedTime() << std::endl;

  for(std::size_t i = 0; i < result1.size(); ++i)
  {
    BOOST_CHECK(result1[i] == result2[i]);
  }
}

BOOST_AUTO_TEST_CASE(shapeIntersection_spheresphere)
{
  init ();
  Sphere s1(20);
  Sphere s2(10);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;
  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v40), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v40), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v40), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v40), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v41), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v41), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v42), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v42), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v43), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v43), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v43), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v43), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v44), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v44), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v44), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v44), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v45), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v45), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v46), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v46), request, result) > 0);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_boxbox)
{
  init ();
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;

  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v47), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v47), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v48), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v48), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  Quaternion3f q;
  q.fromAxisAngle(v29, (FCL_REAL)3.140 / 6);
  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(q), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(q), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(q), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(q), request, result) > 0);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_spherebox)
{
  init ();
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;

  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, request, result) > 0);
  BOOST_CHECK(res);


  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v50), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v50), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v51), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v51), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v52), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v52), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v52), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v52), request, result) > 0);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_cylindercylinder)
{
  init ();
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;

  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v53), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v53), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v53), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v53), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v35), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v35), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v54), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v54), request, result) > 0);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_conecone)
{
  init ();
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;

  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v53), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v53), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v53), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v53), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v55), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v55), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v55), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v55), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v56), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v56), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v56), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v56), request, result) > 0);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_conecylinder)
{
  init ();
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  CollisionRequest request;
  CollisionResult result;

  bool res;

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v53), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v53), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v53), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v53), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v54), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v35), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v54), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v54), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v56), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v56), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v56), NULL, NULL, NULL);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v56), request, result) > 0);
  BOOST_CHECK(res);

  res = solver1.shapeIntersect(s1, Transform3f(), s2, Transform3f(v57), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v37), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s1, transform, s2, transform * Transform3f(v57), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v57), request, result) > 0);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_spheretriangle)
{
  init ();
  Sphere s(10);
  Vec3f t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3f transform;
  generateRandomTransform(extents, transform);
  Transform3f identity;

  bool res;

  res = solver1.shapeTriangleIntersect(s, Transform3f(), t[0], t[1], t[2], NULL, NULL, NULL);
  BOOST_CHECK(res);

  res =  solver1.shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, NULL, NULL, NULL);
  BOOST_CHECK(res);


  t[0] << 30, 0, 0;
  t[1] << 9.9, -20, 0;
  t[2] << 9.9, 20, 0;
  res = solver1.shapeTriangleIntersect(s, Transform3f(), t[0], t[1], t[2], NULL, NULL, NULL);
  BOOST_CHECK(res);

  res =  solver1.shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacesphere)
{
  init ();
  Sphere s(10);
  Halfspace hs(v2, 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v4));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v4)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v3), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 15) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v11));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v3), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 15) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v11)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v4), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v58));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v4), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v58)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v5), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v5), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v6), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 20.1) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v59));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v6), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 20.1) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v59)));
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planesphere)
{
  init ();
  Sphere s(10);
  Plane hs(v2, 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10) < 0.001);
  BOOST_CHECK((normal == v1) || (normal == v2));
  BOOST_CHECK(contact.isZero ());

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10) < 0.001);
  BOOST_CHECK(normal == transform.getQuatRotation().transform(v1)
	      || normal == transform.getQuatRotation().transform(v2));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v3), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == v2);
  BOOST_CHECK(contact == v3);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v3), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v2)));
  BOOST_CHECK(contact == (transform.transform(v3)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v4), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v4));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v4), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v4)));


  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v5), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v5), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v6), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v6), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacebox)
{
  init ();
  Box s(5, 10, 20);
  Halfspace hs(v2, 0);
  
  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v7));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v7)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v8), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 3.75) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v13));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v8), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 3.75) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v13)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v7), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 1.25) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v14));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v7), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 1.25) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v14)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v9), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5.01) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v15));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v9), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5.01) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v15)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v10), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v10), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(transform.getQuatRotation()), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planebox)
{
  init ();
  Box s(5, 10, 20);
  Plane hs(v2, 0);
  
  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v1) || normal == (v2));
  BOOST_CHECK(contact == (zero));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)) || normal == (transform.getQuatRotation().transform(v2)));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v8), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 1.25) < 0.001);
  BOOST_CHECK(normal == (v2));
  BOOST_CHECK(contact == (v8));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v8), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 1.25) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v2)));
  BOOST_CHECK(contact == (transform.transform(v8)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v7), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 1.25) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v7));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v7), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 1.25) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v7)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v9), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v9), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v10), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v10), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(transform.getQuatRotation()), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacecapsule)
{
  init ();
  Capsule s(5, 10);
  Halfspace hs(v2, 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v11));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v11)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v7));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v7)));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v17));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v17)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v59));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v59)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Halfspace(v21, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v25));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v25)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v60));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v60)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v61));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v61)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v62));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v62)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Halfspace(v29, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v63));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v63)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 12.5) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v64));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 12.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v64)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v65));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v65)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v33), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 20.1) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v67));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v33), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 20.1) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v67)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v66), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v66), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planecapsule)
{
  init ();
  Capsule s(5, 10);
  Plane hs(v2, 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v1) || normal == (v2));
  BOOST_CHECK(contact == (zero));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)) || normal == (transform.getQuatRotation().transform(v2)));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v2));
  BOOST_CHECK(contact == (v16));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v2)));
  BOOST_CHECK(contact == (transform.transform(v16)));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v11));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v11)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Plane(v21, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v22) || normal == (v21));
  BOOST_CHECK(contact == (zero));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)) || normal == (transform.getQuatRotation().transform(v21)));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v21));
  BOOST_CHECK(contact == (v23));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v21)));
  BOOST_CHECK(contact == (transform.transform(v23)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v25));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v25)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Plane(v29, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10) < 0.001);
  BOOST_CHECK(normal == (v49) || normal == (v29));
  BOOST_CHECK(contact == (zero));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)) || normal == (transform.getQuatRotation().transform(v29)));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v29));
  BOOST_CHECK(contact == (v31));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v29)));
  BOOST_CHECK(contact == (transform.transform(v31)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v32));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v32)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v33), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v33), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v66), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v66), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacecylinder)
{
  init ();
  Cylinder s(5, 10);
  Halfspace hs(v2, 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v11));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v11)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v7));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v7)));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v17));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v17)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v59));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v59)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Halfspace(v21, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v25));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v25)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v60));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v60)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v61));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v61)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v62));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v62)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Halfspace(v29, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v32));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v32)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v69));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v69)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v64));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v64)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v70), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v67));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v70), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v67)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v68), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v68), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planecylinder)
{
  init ();
  Cylinder s(5, 10);
  Plane hs(v2, 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v1) || normal == (v2));
  BOOST_CHECK(contact == (zero));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)) || normal == (transform.getQuatRotation().transform(v2)));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v2));
  BOOST_CHECK(contact == (v16));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v2)));
  BOOST_CHECK(contact == (transform.transform(v16)));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v11));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v11)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Plane(v21, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v22) || normal == (v21));
  BOOST_CHECK(contact == (zero));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)) || normal == (transform.getQuatRotation().transform(v21)));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v21));
  BOOST_CHECK(contact == (v23));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v21)));
  BOOST_CHECK(contact == (transform.transform(v23)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v25));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v25)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Plane(v29, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v49) || normal == (v29));
  BOOST_CHECK(contact == (zero));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)) || normal == (transform.getQuatRotation().transform(v29)));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v29));
  BOOST_CHECK(contact == (v31));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v29)));
  BOOST_CHECK(contact == (transform.transform(v31)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v32));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v32)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v33), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v33), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v66), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v66), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);
}


BOOST_AUTO_TEST_CASE(shapeIntersection_halfspacecone)
{
  init ();
  Cone s(5, 10);
  Halfspace hs(v2, 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v71));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v71)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v72));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v72)));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v73));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v73)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v74));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v74)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Halfspace(v21, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v75));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v75)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v76));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v76)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v77));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v77)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v78));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v78)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Halfspace(v29, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v32));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v32)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v69));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 7.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v69)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v64));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v64)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v70), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (v49));
  BOOST_CHECK(contact == (v67));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v70), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 10.1) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v49)));
  BOOST_CHECK(contact == (transform.transform(v67)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v68), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v68), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersection_planecone)
{
  init ();
  Cone s(5, 10);
  Plane hs(v2, 0);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL depth;
  Vec3f normal;
  bool res;

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v1) || normal == (v2));
  BOOST_CHECK(contact == (zero));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)) || normal == (transform.getQuatRotation().transform(v2)));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v2));
  BOOST_CHECK(contact == (v19));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v16), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v2)));
  BOOST_CHECK(contact == (transform.transform(v19)));
  
  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v1));
  BOOST_CHECK(contact == (v12));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v11), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v1)));
  BOOST_CHECK(contact == (transform.transform(v12)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v18), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v20), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Plane(v21, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v22) || normal == (v21));
  BOOST_CHECK(contact == (zero));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)) || normal == (transform.getQuatRotation().transform(v21)));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v21));
  BOOST_CHECK(contact == (v24));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v23), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v21)));
  BOOST_CHECK(contact == (transform.transform(v24)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v22));
  BOOST_CHECK(contact == (v26));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v25), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v22)));
  BOOST_CHECK(contact == (transform.transform(v26)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v27), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v28), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);




  hs = Plane(v29, 0);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (v30) || normal == (v29));
  BOOST_CHECK(contact == (zero));

  res = solver1.shapeIntersect(s, transform, hs, transform, &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v30)) || normal == (transform.getQuatRotation().transform(v29)));
  BOOST_CHECK(contact == (transform.transform(zero)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v29));
  BOOST_CHECK(contact == (v31));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v31), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v29)));
  BOOST_CHECK(contact == (transform.transform(v31)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (v30));
  BOOST_CHECK(contact == (v32));

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v32), &contact, &depth, &normal);
  BOOST_CHECK(res);
  BOOST_CHECK(std::abs(depth - 2.5) < 0.001);
  BOOST_CHECK(normal == (transform.getQuatRotation().transform(v30)));
  BOOST_CHECK(contact == (transform.transform(v32)));

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v33), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v33), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, Transform3f(), hs, Transform3f(v66), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeIntersect(s, transform, hs, transform * Transform3f(v66), &contact, &depth, &normal);
  BOOST_CHECK_FALSE(res);
}



BOOST_AUTO_TEST_CASE(shapeDistance_spheresphere)
{  
  init ();
  Sphere s1(20);
  Sphere s2(10);

  Transform3f transform;
  //generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist = -1;
  Vec3f closest_p1, closest_p2;
  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v79), &dist, &closest_p1, &closest_p2);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v80), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v43), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(v40), s2, Transform3f(), &dist);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(v80), s2, Transform3f(), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(v43), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);


  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v40), &dist);
  // this is one problem: the precise is low sometimes
  BOOST_CHECK(fabs(dist - 10) < 0.1);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v80), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.06);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v43), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform * Transform3f(v40), s2, transform, &dist);
  BOOST_CHECK(fabs(dist - 10) < 0.1);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform * Transform3f(v80), s2, transform, &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.1);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform * Transform3f(v43), s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeDistance_boxbox)
{
  init ();
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);
  Vec3f closest_p1, closest_p2;

  Transform3f transform;
  //generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  std::cerr << " SOVLER NUMBER 1" << std::endl;
  res = solver1.shapeDistance(s2, Transform3f(), s2, Transform3f(v6), &dist, &closest_p1, &closest_p2);
  std::cerr << "computed points in box to box" << closest_p1 << " & " << closest_p2 << "with dist: " << dist<< std::endl;
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s2, Transform3f(), s2, Transform3f(v81), &dist, &closest_p1, &closest_p2);
  std::cerr << "computed points in box to box" << closest_p1 << " & " << closest_p2 << "with dist: " << dist<< std::endl;
  BOOST_CHECK(fabs(dist - 10.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s2, Transform3f(), s2, Transform3f(v82), &dist, &closest_p1, &closest_p2);
  std::cerr << "computed points in box to box" << closest_p1 << " & " << closest_p2 << "with dist: " << dist<< std::endl;
  BOOST_CHECK(fabs(dist - 10.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s2, Transform3f(), s2, Transform3f(v83), &dist, &closest_p1, &closest_p2);
  std::cerr << "computed points in box to box" << closest_p1 << " & " << closest_p2 << "with dist: " << dist<< std::endl;
  BOOST_CHECK(fabs(dist - std::sqrt(.1*.1 + .1*.1)) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s2, Transform3f(), s2, Transform3f(v6), &dist, &closest_p1, &closest_p2);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s2, Transform3f(), s2, Transform3f(v81), &dist, &closest_p1, &closest_p2);
  BOOST_CHECK(fabs(dist - 10.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s2, Transform3f(), s2, Transform3f(v82), &dist, &closest_p1, &closest_p2);
  BOOST_CHECK(fabs(dist - 10.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s2, Transform3f(), s2, Transform3f(v83), &dist, &closest_p1, &closest_p2);
  BOOST_CHECK(fabs(dist - std::sqrt(.1*.1 + .1*.1)) < 0.001);
  BOOST_CHECK(res);


  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v84), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v85), &dist);
  BOOST_CHECK(fabs(dist - 5) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v85), &dist);
  BOOST_CHECK(fabs(dist - 5) < 0.001);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeDistance_boxsphere)
{
  init ();
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v86), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);
  
  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v86), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.05);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 17.5) < 0.001);
  BOOST_CHECK(res);
  
  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 17.5) < 0.001);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeDistance_cylindercylinder)
{
  init ();
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v6), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v6), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);
}



BOOST_AUTO_TEST_CASE(shapeDistance_conecone)
{
  init ();
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v6), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v6), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v87), &dist);
  BOOST_CHECK(fabs(dist - 30) < 1);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v87), &dist);
  BOOST_CHECK(fabs(dist - 30) < 1);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeDistance_conecylinder)
{
  init ();
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, transform, s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v6), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.01);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v6), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.02);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, Transform3f(), s2, Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 30) < 0.01);
  BOOST_CHECK(res);

  res = solver1.shapeDistance(s1, transform, s2, transform * Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 30) < 0.1);
  BOOST_CHECK(res);
}



BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_spheresphere)
{
  init ();
  Sphere s1(20);
  Sphere s2(10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  CollisionRequest request;
  CollisionResult result;

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  request.gjk_solver_type = GST_INDEP; // use indep GJK solver

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v40), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res); 
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v40), &contact, &penetration_depth, &normal);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v40), request, result) > 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v40), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v40), &contact, &penetration_depth, &normal);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v40), request, result) > 0);
  BOOST_CHECK_FALSE(res);


  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v41), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v41), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v41), request, result) > 0);
  BOOST_CHECK(res);


  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v42), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v42), &contact, &penetration_depth, &normal);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v42), request, result) > 0);
  BOOST_CHECK_FALSE(res);


  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v43), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v43), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v43), request, result) > 0);
  BOOST_CHECK(res);


  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v43), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v43), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v43), request, result) > 0);
  BOOST_CHECK(res);


  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(), request, result) > 0);
  BOOST_CHECK(res);


  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform, request, result) > 0);
  BOOST_CHECK(res);


  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v44), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v44), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v44), request, result) > 0);
  BOOST_CHECK(res);



  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v44), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v44), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v44), request, result) > 0);
  BOOST_CHECK(res);


  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v45), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v45), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
  result.clear();
  res = (collide(&s1, Transform3f(), &s2, Transform3f(v45), request, result) > 0);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v46), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v46), &contact, &penetration_depth, &normal);
  BOOST_CHECK_FALSE(res);
  result.clear();
  res = (collide(&s1, transform, &s2, transform * Transform3f(v46), request, result) > 0);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_boxbox)
{
  init ();
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v47), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v47), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v48), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v48), &contact, &penetration_depth, &normal);
  BOOST_CHECK_FALSE(res);

  Quaternion3f q;
  q.fromAxisAngle(v29, (FCL_REAL)3.140 / 6);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(q), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(q), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
  
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(q), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(q), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_spherebox)
{
  init ();
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v50), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v50), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v88), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v88), &contact, &penetration_depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v52), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v52), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v52), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v52), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_cylindercylinder)
{
  init ();
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v53), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v53), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v53), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v53), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v35), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v35), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v6), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v6), &contact, &penetration_depth, &normal);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_conecone)
{
  init ();
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v53), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v53), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v53), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v53), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v6), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v6), &contact, &penetration_depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v6), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v6), &contact, &penetration_depth, &normal);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v56), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v56), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v56), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v56), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_conecylinder)
{
  init ();
  Cylinder s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  Vec3f contact;
  FCL_REAL penetration_depth;
  Vec3f normal;  
  bool res;

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform, &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v34), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v34), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v34), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v34), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v35), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v35), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v35), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v35), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v36), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v36), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v36), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v36), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v37), NULL, NULL, NULL);
  BOOST_CHECK(res);
  res = solver2.shapeIntersect(s1, Transform3f(), s2, Transform3f(v37), &contact, &penetration_depth, &normal);
  BOOST_CHECK(res);

  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v33), NULL, NULL, NULL);
  BOOST_CHECK_FALSE(res);
  res = solver2.shapeIntersect(s1, transform, s2, transform * Transform3f(v33), &contact, &penetration_depth, &normal);
  BOOST_CHECK_FALSE(res);
}


BOOST_AUTO_TEST_CASE(shapeIntersectionGJK_spheretriangle)
{
  init ();
  Sphere s(10);
  Vec3f t[3];
  t[0] << 20, 0, 0;
  t[1] << -20, 0, 0;
  t[2] << 0, 20, 0;

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;

  res = solver2.shapeTriangleIntersect(s, Transform3f(), t[0], t[1], t[2], NULL, NULL, NULL);
  BOOST_CHECK(res);

  res =  solver2.shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, NULL, NULL, NULL);
  BOOST_CHECK(res);

  t[0] << 30, 0, 0;
  t[1] << 9.9, -20, 0;
  t[2] << 9.9, 20, 0;
  res = solver2.shapeTriangleIntersect(s, Transform3f(), t[0], t[1], t[2], NULL, NULL, NULL);
  BOOST_CHECK(res);

  res =  solver2.shapeTriangleIntersect(s, transform, t[0], t[1], t[2], transform, NULL, NULL, NULL);
  BOOST_CHECK(res);
}




BOOST_AUTO_TEST_CASE(spheresphere)
{  
  init ();
  Sphere s1(20);
  Sphere s2(10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist = -1;
  
  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v80), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v43), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(v40), s2, Transform3f(), &dist);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(v80), s2, Transform3f(), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(v43), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);


  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v80), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v43), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, transform * Transform3f(v40), s2, transform, &dist);
  BOOST_CHECK(fabs(dist - 10) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform * Transform3f(v80), s2, transform, &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform * Transform3f(v43), s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);
}

BOOST_AUTO_TEST_CASE(boxbox)
{                   
  init ();
  Box s1(20, 40, 50);
  Box s2(10, 10, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v84), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v84), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v85), &dist);
  BOOST_CHECK(fabs(dist - 5) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v85), &dist);
  BOOST_CHECK(fabs(dist - 5) < 0.001);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(boxsphere)
{
  init ();
  Sphere s1(20);
  Box s2(5, 5, 5);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v86), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.01);
  BOOST_CHECK(res);
  
  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v86), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.01);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 17.5) < 0.001);
  BOOST_CHECK(res);
  
  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 17.5) < 0.001);
  BOOST_CHECK(res);
}

BOOST_AUTO_TEST_CASE(cylindercylinder)
{
  init ();
  Cylinder s1(5, 10);
  Cylinder s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v6), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v6), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v40), &dist);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);
}



BOOST_AUTO_TEST_CASE(conecone)
{
  init ();
  Cone s1(5, 10);
  Cone s2(5, 10);

  Transform3f transform;
  generateRandomTransform(extents, transform);

  bool res;
  FCL_REAL dist;

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(), &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, transform, s2, transform, &dist);
  BOOST_CHECK(dist < 0);
  BOOST_CHECK_FALSE(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v6), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v6), &dist);
  BOOST_CHECK(fabs(dist - 0.1) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, Transform3f(), s2, Transform3f(v87), &dist);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);

  res = solver2.shapeDistance(s1, transform, s2, transform * Transform3f(v87), &dist);
  BOOST_CHECK(fabs(dist - 30) < 0.001);
  BOOST_CHECK(res);
}




