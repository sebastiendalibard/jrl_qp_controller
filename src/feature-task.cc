#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/feature-task.hh>

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
  FeatureTask::compute_objective()
  {
    update_jacobian_and_value();

    vectorN desired_acceleration = 
      - gain_ * value_ - 2 * sqrt(gain_) * MAL_RET_A_by_B(jacobian_,robot_->currentVelocity());

    noalias(D_) = prod(MAL_RET_TRANSPOSE(jacobian_), 2*jacobian_);

    noalias(c_) = 2*prod(MAL_RET_TRANSPOSE(jacobian_),
			 prod(d_jacobian_,robot_->currentVelocity()) 
			 - desired_acceleration) ;
  }
  
  void
  FeatureTask::set_gain(double i_gain)
  {
    gain_ = i_gain;
  }


} // end of namespace jrl_qp_controller
