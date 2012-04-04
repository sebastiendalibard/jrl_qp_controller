#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/contact-constraint.hh>

namespace jrl_qp_controller {

  ContactConstraint::ContactConstraint(CjrlDynamicRobot *i_robot, 
				       CjrlJoint *i_joint,
				       vector3d i_contact_point,
				       std::vector<vector3d> &i_polygon,
				       matrix4d &i_contact_transformation)
    :robot_(i_robot),
     joint_(i_joint),
     contact_point_(i_contact_point),
     contact_polygon_(i_polygon),
     contact_transformation_(i_contact_transformation),
     last_robot_update_(),
     first_call_(true)
  {
    jacobian_.resize(6,robot_->numberDof());
    last_jacobian_.resize(6,robot_->numberDof());
    d_jacobian_.resize(6,robot_->numberDof(),0);
  }

  ContactConstraint::~ContactConstraint()
  {
  }

  void
  ContactConstraint::update_jacobian()
  {
    ros::Time now = ros::Time::now();

    last_jacobian_ = jacobian_;
    robot_->getJacobian(*(robot_->rootJoint()),*joint_,contact_point_,jacobian_);
    if(!first_call_) {
      ros::Duration dt = now - last_robot_update_;
      d_jacobian_ = dt.toSec() * (jacobian_ - last_jacobian_);
    }
    last_robot_update_ = now;
    first_call_ = false;
  }

} // end of namespace jrl_qp_controller
