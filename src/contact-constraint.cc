#include <cmath>
#include <jrl/mal/matrixabstractlayer.hh>
#include <hpp/gik/tools.hh>

#include <jrl_qp_controller/contact-constraint.hh>

#include <ros/console.h>

namespace jrl_qp_controller { 

  ContactConstraint::ContactConstraint(CjrlDynamicRobot *i_robot, 
				       CjrlJoint *i_joint,
				       const vector3d &i_contact_point,
				       const vector3d &i_contact_normal,
				       double i_friction_coefficient)
    :robot_(i_robot),
     joint_(i_joint),
     contact_point_(i_contact_point),
     epsilon_(1e-6),
     last_robot_update_(),
     first_call_(true)
  {
    jacobian_.resize(3,robot_->numberDof());
    for(unsigned int i = 0; i < 3; ++i)
      for(unsigned int j = 0; j < robot_->numberDof(); ++j)
	jacobian_(i,j) = 0;

    normal_.resize(1,3);
    for(unsigned int j = 0; j < 3; ++j)
      normal_(0,j) = i_contact_normal(j);

    normal_jacobian_.resize(1,robot_->numberDof());
    for(unsigned int j = 0; j < robot_->numberDof(); ++j)
	normal_jacobian_(0,j) = 0;

    last_jacobian_.resize(1,robot_->numberDof());
    for(unsigned int j = 0; j < robot_->numberDof(); ++j)
	last_jacobian_(0,j) = 0;

    d_jacobian_.resize(1,robot_->numberDof());
    for(unsigned int j = 0; j < robot_->numberDof(); ++j)
	d_jacobian_(0,j) = 0;

    friction_basis_.resize(3,3);
    dyn_mat_.resize(robot_->numberDof(),3);
    for(unsigned int i = 0; i < robot_->numberDof(); ++i)
      for(unsigned int j = 0; j < 3; ++j)
	dyn_mat_(i,j) = 0;

    compute_friction_basis(i_contact_normal, i_friction_coefficient);
  }

  ContactConstraint::~ContactConstraint()
  {
  }

  void
  ContactConstraint::compute_friction_basis(const vector3d &i_contact_normal,
					    double i_friction_coefficient)
  {
    /* 
       e0 is the contact normal unitized
       e1,e2,e3 are vectors tangent to the contact plane, defining the
       linearized convex cone. 
    */    
    vector3d e0,e1,e2,e3;

    e0 = i_contact_normal / MAL_S3_VECTOR_NORM(i_contact_normal);
    
    /*
      Arbitrarily choose e1
    */
    if ( abs ( pow(e0(1),2) + pow(e0(2),2)) < epsilon_) { //e0 is k.(x)
      e1 = vector3d(0,1,0);
    }
    else {
      e1 = vector3d(1,0,0) -  e0 * e0(0);
      e1 =  e1 /MAL_S3_VECTOR_NORM(e1);
    }

    matrix3d temp_rot;
    matrixNxP temp_rot_n(3,3);
    vectorN e0_n(3);
    ChppGikTools::Vector3toUblas(e0, e0_n);
    ChppGikTools::Rodrigues(e0_n, 2*M_PI/3, temp_rot_n);
    ChppGikTools::UblastoMatrix3(temp_rot_n,temp_rot);

    /*
      temp_rot is a rotation of 2*pi/3 around e0.
      Set: 
      * e2 = temp_rot.e1
      * e3 = temp_rot.e2
    */
    e2 = temp_rot*e1;
    e3 = temp_rot*e2;
      
    /*
      Fill the linearized friction basis:
      -- e0 + mu.e1
      -- e0 + mu.e2
      -- e0 + mu.e3
    */
    for(unsigned int i = 0; i < 3; ++i)
      friction_basis_(i,0) = e0(i) + i_friction_coefficient * e1(i);

    for(unsigned int i = 0; i < 3; ++i)
      friction_basis_(i,1) = e0(i) + i_friction_coefficient * e2(i);

    for(unsigned int i = 0; i < 3; ++i)
      friction_basis_(i,2) = e0(i) + i_friction_coefficient * e3(i);
  }

  void
  ContactConstraint::update_jacobian()
  {
    ros::Time now = ros::Time::now();

    last_jacobian_ = normal_jacobian_;
    robot_->getPositionJacobian(*(robot_->rootJoint()),*joint_,contact_point_,jacobian_);
    noalias(normal_jacobian_) = prod(normal_,jacobian_);
    if(!first_call_) {
      ros::Duration dt = now - last_robot_update_;
      if (dt.toSec())
	noalias(d_jacobian_) = 1/dt.toSec() * (normal_jacobian_ - last_jacobian_);
    }
    last_robot_update_ = now;
    first_call_ = false;

    noalias(dyn_mat_) = prod(MAL_RET_TRANSPOSE(jacobian_), friction_basis_);

    //ROS_DEBUG_STREAM("contact: " << this);
    //ROS_DEBUG_STREAM("jacobian: " << jacobian_);
    
  }

} // end of namespace jrl_qp_controller
