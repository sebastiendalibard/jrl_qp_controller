#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl_qp_controller/gik-feature-task.hh>


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
  GikFeatureTask::update_jacobian_and_value()
  {
    ros::Time now = ros::Time::now();

    last_jacobian_ = jacobian_;

    /* Get jacobian and value from gik task */
    gik_constraint_->computeJacobian();
    gik_constraint_->computeValue();
    jacobian_ = gik_constraint_->jacobian();
    value_ = gik_constraint_->value();

    if(!first_call_) {
      ros::Duration dt = now - last_robot_update_;
      d_jacobian_ = dt.toSec() * (jacobian_ - last_jacobian_);
    }
    last_robot_update_ = now;
    first_call_ = false;
  }
  
} // end of namespace jrl_qp_controller
