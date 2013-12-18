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

#include <hpp/util/debug.hh>

#include <hpp/model/collision-object.hh>
#include <hpp/model/device.hh>
#include <hpp/model/fcl-to-eigen.hh>
#include <hpp/model/object-factory.hh>

namespace hpp {
  namespace model {

    static Transform3f I4;

    Device::Device(const std::string& name) :
	name_ (name), distances_ (), jointByName_ (),
	jointVector_ (), rootJoint_ (0x0), numberDof_ (0),
	configSize_ (0), currentConfiguration_ (configSize_),
	currentVelocity_ (numberDof_), 	currentAcceleration_ (numberDof_),
	com_ (), jacobianCom_ (3, 0), mass_ (0), upToDate_ (false),
	computationFlag_ (ALL),	weakPtr_ ()
    {
      com_.setZero ();
      I4.setIdentity ();
    }


    // ========================================================================

    Device::~Device()
    {
    }

    // ========================================================================

    DeviceShPtr Device::create(std::string name)
    {
      Device* ptr = new Device (name);
      DeviceShPtr shPtr (ptr);

      ptr->init (shPtr);
      return shPtr;
    }

    // ========================================================================

    DeviceShPtr Device::createCopy(const DeviceShPtr& device)
    {
      Device* ptr = new Device(*device);
      DeviceShPtr shPtr(ptr);

      ptr->init (shPtr);
      return shPtr;
    }

    // ========================================================================

    DeviceShPtr Device::clone() const
    {
      return Device::createCopy(weakPtr_.lock());
    }

    // ========================================================================

    void Device::init(const DeviceWkPtr& weakPtr)
    {
      weakPtr_ = weakPtr;
    }

    // ========================================================================

    void Device::addOuterObject (const CollisionObjectShPtr &object,
				 bool collision, bool distance)
    {
      JointVector_t jv = getJointVector ();
      for (JointVector_t::iterator it = jv.begin (); it != jv.end (); it++) {
	Joint* joint = *it;
	Body* body = joint->linkedBody ();
	if (body) {
	  body->addOuterObject (object, collision, distance);
	}
      }
    }

    // ========================================================================

    void Device::removeOuterObject (const CollisionObjectShPtr& object,
				    bool collision, bool distance)
    {
      JointVector_t jv = getJointVector ();
      for (JointVector_t::iterator it = jv.begin (); it != jv.end (); it++) {
	Joint* joint = *it;
	Body* body = joint->linkedBody ();
	if (body) {
	  body->removeOuterObject (object, collision, distance);
	}
      }
    }

    // ========================================================================

      ObjectIterator Device::objectIterator (Request_t type)
      {
	return ObjectIterator (*this, type);
      }


    // ========================================================================

      ObjectIterator Device::objectIteratorEnd (Request_t type)
      {
	ObjectIterator iterator (*this, type);
	iterator.setToEnd ();
	return iterator;
      }

    // ========================================================================

    void Device::updateDistances ()
    {
       JointVector_t joints = getJointVector ();
       JointVector_t::size_type size = 0;
       for (JointVector_t::iterator it = joints.begin (); it != joints.end ();
	    it++) {
	 Body* body = (*it)->linkedBody ();
	 if (body) {
	   size += body->innerObjects (DISTANCE).size () *
	     body->outerObjects (DISTANCE).size ();
	 }
	 distances_.resize (size);
       }
    }

    // ========================================================================

    void Device::computeDistances ()
    {
      JointVector_t joints = getJointVector ();
      JointVector_t::size_type offset = 0;
       for (JointVector_t::iterator it = joints.begin (); it != joints.end ();
	    it++) {
	 Body* body = (*it)->linkedBody ();
	 if (body) {
	   body->computeDistances (distances_, offset);
	   assert (offset <= distances_.size ());
	 }
       }
    }

    // ========================================================================

    bool Device::collisionTest () const
    {
      for (JointVector_t::const_iterator itJoint = jointVector_.begin ();
	   itJoint != jointVector_.end (); itJoint++) {
	Body* body = (*itJoint)->linkedBody ();
	if (body != 0x0) {
	  if (body->collisionTest ()) {
	    return true;
	  }
	}
      }
      return false;
    }

    // ========================================================================

    void Device::computeForwardKinematics ()
    {
      if (upToDate_) return;
      computeJointPositions ();
      if (computationFlag_ | JACOBIAN) {
	computeJointJacobians ();
      }
      if (computationFlag_ | COM) {
	computePositionCenterOfMass ();
      }
      if (computationFlag_ | COM && computationFlag_ | JACOBIAN) {
	computeJacobianCenterOfMass ();
      }
      // Update positions of bodies from position of joints.
      JointVector_t jv = getJointVector ();
      for (JointVector_t::iterator itJoint = jv.begin (); itJoint != jv.end ();
	   itJoint++) {
	Body* body = (*itJoint)->linkedBody ();
	if (body) {
	  const ObjectVector_t& cbv =
	    body->innerObjects (COLLISION);
	  for (ObjectVector_t::const_iterator itInner = cbv.begin ();
	       itInner != cbv.end (); itInner++) {
	    // Compute global position if inner object
	    fcl::Transform3f globalPosition =
	      (*itJoint)->currentTransformation ()*
	      (*itInner)->positionInJointFrame ();
	    (*itInner)->fcl ()->setTransform (globalPosition);
	  }
	  const ObjectVector_t& dbv =
	    body->innerObjects (DISTANCE);
	  for (ObjectVector_t::const_iterator itInner = dbv.begin ();
	       itInner != dbv.end (); itInner++) {
	    // Compute global position if inner object
	    fcl::Transform3f globalPosition =
	      (*itJoint)->currentTransformation ()*
	      (*itInner)->positionInJointFrame ();
	    (*itInner)->fcl ()->setTransform (globalPosition);
	  }
	}
      }
      upToDate_ = true;
    }

    // ========================================================================

    void Device::registerJoint (Joint* joint)
    {
      jointVector_.push_back (joint);
      joint->rankInConfiguration_ = configSize_;
      joint->rankInVelocity_ = numberDof_;
      numberDof_ += joint->numberDof ();
      configSize_ += joint->configSize ();
      currentConfiguration_.resize (configSize_);
      currentVelocity_.resize (numberDof_);
      currentAcceleration_.resize (numberDof_);
      currentConfiguration_.setZero ();
      currentVelocity_.setZero ();
      currentAcceleration_.setZero ();
      jointByName_ [joint->name ()] = joint;
      resizeJacobians ();
      computeMass ();
    }

    void Device::rootJoint (Joint* joint)
    {
      rootJoint_ = joint;
      registerJoint (joint);
      joint->setRobot (this);
    }

    JointPtr_t Device::rootJoint () const
    {
      return rootJoint_;
    }

    const JointVector_t& Device::getJointVector () const
    {
      return jointVector_;
    }

    Joint* Device::getJointByName (const std::string& name)
    {
      return jointByName_ [name];
    }

    void Device::computeJointPositions ()
    {
      rootJoint_->computePosition (currentConfiguration_, I4);
    }

    void Device::computeJointJacobians ()
    {
      rootJoint_->computeJacobian ();
      for (JointVector_t::const_iterator it = jointVector_.begin ();
	   it != jointVector_.end (); it++) {
	hppDout (info, "Joint " << (*it)->name ());
	hppDout (info, (*it)->currentTransformation ());
	hppDout (info, (*it)->jacobian ());
      }
    }

    void Device::computeMass ()
    {
      mass_ = rootJoint_->computeMass ();
    }
    void Device::computePositionCenterOfMass ()
    {
      rootJoint_->computeMassTimesCenterOfMass ();
      com_ = (1/mass_) * rootJoint_->massCom_;
    }
    
    void Device::computeJacobianCenterOfMass ()
    {
      for (JointVector_t::iterator it = jointVector_.begin ();
	   it != jointVector_.end (); it++) {
	(*it)->writeComSubjacobian (jacobianCom_, mass ());
      }
    }

    void Device::resizeJacobians ()
    {
      jacobianCom_.resize (3, numberDof_);
      for (JointVector_t::iterator itJoint = jointVector_.begin ();
	   itJoint != jointVector_.end () ; itJoint++) {
	(*itJoint)->jacobian_.resize (6, numberDof_);
	(*itJoint)->jacobian_.setZero ();
      }
    }

  } // namespace model
} // namespace hpp

std::ostream& operator<<(std::ostream& os, hpp::model::Device& device)
{
  os << "Device: " << device.name() << std::endl;
  os << std::endl;
  os << " Current configuration: " << device.currentConfiguration ()
     << std::endl;
  os << std::endl;
  os << " Writing kinematic chain" << std::endl;

  //
  // Go through joints and output each joint
  //
  hpp::model::Joint* joint = device.rootJoint();

  if (joint) {
    os << *joint << std::endl;
  }
  // Get position of center of mass
  hpp::model::vector3_t com = device.positionCenterOfMass ();

  //debug
  os << "total mass " << device.mass() << ", COM: "
     << com [0] <<", "<< com [1] << ", " << com [2] <<std::endl;
  return os;
}
