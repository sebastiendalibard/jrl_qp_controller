#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl_qp_controller/gik-feature-task.hh>

#include <ros/console.h>

namespace jrl_qp_controller {


  GikFeatureTask::GikFeatureTask(CjrlDynamicRobot * i_robot,
				 CjrlGikStateConstraint* i_gik_constraint)
    :FeatureTask(i_robot),
     gik_constraint_(i_gik_constraint)
  {
    if (gik_constraint_) {
      value_.resize(gik_constraint_->dimension());
      jacobian_.resize(gik_constraint_->dimension(),robot_->numberDof());
      last_jacobian_.resize(gik_constraint_->dimension(),robot_->numberDof());
      d_jacobian_.resize(gik_constraint_->dimension(),robot_->numberDof());
    }
  }

  GikFeatureTask::~GikFeatureTask()
  {
  }

  void
  GikFeatureTask::set_gik_constraint(CjrlGikStateConstraint* i_gik_constraint)
  {
    gik_constraint_ = i_gik_constraint;
    if (gik_constraint_) {
      value_.resize(gik_constraint_->dimension());
      jacobian_.resize(gik_constraint_->dimension(),robot_->numberDof());
      last_jacobian_.resize(gik_constraint_->dimension(),robot_->numberDof());
      d_jacobian_.resize(gik_constraint_->dimension(),robot_->numberDof());
    }
  }

  void
  GikFeatureTask::update_jacobian_and_value(double time_step)
  {
    last_jacobian_ = jacobian_;

    /* Get jacobian and value from gik task */
    gik_constraint_->computeJacobian();
    gik_constraint_->computeValue();
    jacobian_ = gik_constraint_->jacobian();
    value_ = gik_constraint_->value();

    if(!first_call_) {
      noalias(d_jacobian_) = jacobian_ - last_jacobian_;
      if (time_step)
	d_jacobian_ /= time_step;
    }
    first_call_ = false;

    ROS_DEBUG_STREAM("task: " << this);
    ROS_DEBUG_STREAM("jacobian: " << jacobian_);
    ROS_DEBUG_STREAM("value: " << value_);
    ROS_DEBUG_STREAM("value norm: " << norm_2(value_));
  }
  
} // end of namespace jrl_qp_controller
