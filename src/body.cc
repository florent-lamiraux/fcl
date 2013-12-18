///
/// Copyright (c) 2013, 2014 CNRS
/// Author: Florent Lamiraux
///
///
// This file is part of hpp-model
// hpp-model is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-model is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-model  If not, see
// <http://www.gnu.org/licenses/>.

#include <fcl/distance.h>
#include <fcl/collision.h>
#include <hpp/util/debug.hh>
#include <hpp/model/body.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/collision-object.hh>
#include <hpp/model/object-factory.hh>

namespace fcl {
  HPP_PREDEF_CLASS (CollisionGeometry);
} // namespace fcl

namespace hpp {
  namespace model {
    static ObjectVector_t::iterator
    findObject (ObjectVector_t& vector,
		const fcl::CollisionObjectConstShPtr& object)
    {
      ObjectVector_t::iterator it;
      for (it = vector.begin (); it != vector.end (); it++) {
	const CollisionObjectShPtr& local = *it;
	if (local->fcl ()->collisionGeometry () ==
	    object->collisionGeometry ()) return it;
      }
      return it;
    }

    Body:: Body () : collisionInnerObjects_ (), collisionOuterObjects_ (),
		     distanceInnerObjects_ (), distanceOuterObjects_ (),
		     joint_ (0x0), name_ (), localCom_ (), inertiaMatrix_ (),
		     mass_ (0)
    {
    }

    //-----------------------------------------------------------------------

    void Body::addInnerObject (const CollisionObjectShPtr& object,
			       bool collision, bool distance)
    {
      if (collision) {
	if (findObject (collisionInnerObjects_,	object->fcl ()) ==
	    collisionInnerObjects_.end ()) {
	  if (joint () == 0) {
	    throw std::runtime_error ("Body should be connected to a joint "
				      "before inserting objects.");
	  }
	  object->joint (joint ());
	  collisionInnerObjects_.push_back (object);
	}
      }
      if (distance) {
	if (findObject (distanceInnerObjects_, object->fcl ()) ==
	    distanceInnerObjects_.end ()) {
	  if (joint () == 0) {
	    throw std::runtime_error ("Body should be connected to a joint "
				      "before inserting objects.");
	  }
	  object->joint (joint ());
	  distanceInnerObjects_.push_back (object);
	  if (!joint ()->getRobot ()) {
	    throw std::runtime_error ("Body should be connected to a robot "
				      "before inserting inner objects.");
	  }
	  joint ()->getRobot ()->updateDistances ();
	}
      }
    }

    //-----------------------------------------------------------------------

    void Body::addOuterObject (const CollisionObjectShPtr& object,
			       bool collision, bool distance)
    {
      if (collision) {
	if (findObject (collisionOuterObjects_,	object->fcl ()) ==
	    collisionOuterObjects_.end ()) {
	  hppDout (info, "adding " << object->name () << " to body "
		   << this->name_ << " for collision");
	  collisionOuterObjects_.push_back (object);
	}
      }
      if (distance) {
	if (findObject (distanceOuterObjects_, object->fcl ()) ==
	    distanceOuterObjects_.end ()) {
	  hppDout (info, "adding " << object->name () << " to body "
		   << this->name_ << " for distance");
	  distanceOuterObjects_.push_back (object);
	  if (!joint ()->getRobot ()) {
	    throw std::runtime_error ("Body should be connected to a robot "
				      "before inserting outer objects.");
	  }
	  joint ()->getRobot ()->updateDistances ();
	}
      }
    }

    //-----------------------------------------------------------------------

    void Body::removeInnerObject (const CollisionObjectShPtr& object,
				  bool collision, bool distance)
    {
      if (collision) {
	ObjectVector_t::iterator it =
	  findObject (collisionInnerObjects_, object->fcl ());
	if (it != collisionInnerObjects_.end ())
	  collisionInnerObjects_.erase (it);
      }
      if (distance) {
	ObjectVector_t::iterator it =
	  findObject (distanceInnerObjects_, object->fcl ());
	if (it != distanceInnerObjects_.end ())
	  distanceInnerObjects_.erase (it);
      }
    }

    //-----------------------------------------------------------------------

    void Body::removeOuterObject (const CollisionObjectShPtr& object,
				  bool collision, bool distance)
    {
      if (collision) {
	ObjectVector_t::iterator it =
	  findObject (collisionOuterObjects_, object->fcl ());
	if (it != collisionOuterObjects_.end ()) {
	  collisionOuterObjects_.erase (it);
	}
      }
      if (distance) {
	ObjectVector_t::iterator it =
	  findObject (distanceOuterObjects_, object->fcl ());
	if (it != distanceOuterObjects_.end ()) {
	  distanceOuterObjects_.erase (it);
	}
      }
    }

    //-----------------------------------------------------------------------

    const ObjectVector_t& Body::innerObjects (Request_t type) const
    {
      switch (type) {
      case COLLISION:
	return collisionInnerObjects_;
      case DISTANCE:
	return distanceInnerObjects_;
      default:
	throw std::runtime_error
	  ("Please choose between COLLISION and DISTANCE.");
      }
    }

    //-----------------------------------------------------------------------

    const ObjectVector_t& Body::outerObjects (Request_t type) const
    {
      switch (type) {
      case COLLISION:
	return collisionOuterObjects_;
      case DISTANCE:
	return distanceOuterObjects_;
      default:
	throw std::runtime_error
	  ("Please choose between COLLISION and DISTANCE.");
      }
    }

    bool Body::collisionTest () const
    {
      fcl::CollisionRequest collisionRequest;
      fcl::CollisionResult collisionResult;
      for (ObjectVector_t::const_iterator itInner =
	     collisionInnerObjects_.begin ();
	   itInner != collisionInnerObjects_.end (); itInner++) {
	for (ObjectVector_t::const_iterator itOuter =
	       collisionOuterObjects_.begin ();
	     itOuter != collisionOuterObjects_.end (); itOuter++) {
	  if (fcl::collide ((*itInner)->fcl ().get (),
			    (*itOuter)->fcl ().get (),
			    collisionRequest, collisionResult) != 0) {
	    return true;
	  }
	}
      }
      return false;
    }

    void Body::computeDistances (DistanceResults_t& results,
				 DistanceResults_t::size_type& offset)
    {
      fcl::DistanceRequest distanceRequest (true);
      for (ObjectVector_t::iterator itInner = distanceInnerObjects_.begin ();
	   itInner != distanceInnerObjects_.end (); itInner++) {
	// Compute global position if inner object
	fcl::Transform3f globalPosition = joint ()->currentTransformation ()*
	  (*itInner)->positionInJointFrame ();
	(*itInner)->fcl ()->setTransform (globalPosition);
	for (ObjectVector_t::iterator itOuter = distanceOuterObjects_.begin ();
	     itOuter != distanceOuterObjects_.end (); itOuter++) {
	  // Compute global position if inner object
	  fcl::distance ((*itInner)->fcl ().get (), (*itOuter)->fcl ().get (),
			 distanceRequest, results [offset].fcl);
	  results [offset].innerObject = *itInner;
	  results [offset].outerObject = *itOuter;
	  assert (results [offset].fcl.o1 == 0 || results [offset].fcl.o1 ==
		  (*itInner)->fcl ()->collisionGeometry ().get ());
	  assert (results [offset].fcl.o1 == 0 || results [offset].fcl.o1 ==
		  (*itInner)->fcl ()->collisionGeometry ().get ());
	  offset++;
	}
      }
    }
  } // namespace model
} // namespace hpp
