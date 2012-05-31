#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl_qp_controller/feature-task.hh>

#include <ros/console.h>

namespace jrl_qp_controller {

  FeatureTask::FeatureTask(CjrlDynamicRobot * i_robot)
    :Task(i_robot,Task::ACCELERATION,true,true),
     gain_(1.),
     first_call_(true)
  {
  }

  FeatureTask::~FeatureTask()
  {
  }

  void
  FeatureTask::compute_objective(double time_step)
  {
    update_jacobian_and_value(time_step);

    vectorN desired_acceleration =
      - gain_ * value_ - 2 * sqrt(gain_) * MAL_RET_A_by_B(jacobian_,robot_->currentVelocity());

    noalias(D_) = 2*prod(MAL_RET_TRANSPOSE(jacobian_), jacobian_);

    noalias(c_) = 2*prod(MAL_RET_TRANSPOSE(jacobian_),
			 prod(d_jacobian_,robot_->currentVelocity())
			 - desired_acceleration) ;
  }

  void
  FeatureTask::set_gain(double i_gain)
  {
    gain_ = i_gain;
  }

  vectorN
  FeatureTask::computed_acceleration(const vectorN & robot_acceleration, 
				     const vectorN & robot_velocity)
  {
    vectorN acc(value_.size());
    acc = prod(jacobian_, robot_acceleration) + prod(d_jacobian_,robot_velocity);
    return acc;
  }


} // end of namespace jrl_qp_controller

