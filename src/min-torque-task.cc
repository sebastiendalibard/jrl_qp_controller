#include <assert.h>

#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/min-torque-task.hh>

namespace  jrl_qp_controller {

  MinTorqueTask::MinTorqueTask(CjrlDynamicRobot * i_robot)
    :Task(i_robot,Task::TORQUES,true,false),
     objective_computed_(false)
  {
    joint_torque_weights_.resize(robot_->getActuatedJoints().size(),1);
    MAL_MATRIX_FILL(D_,0);
  }

  MinTorqueTask::~MinTorqueTask()
  {
  }

  void
  MinTorqueTask::set_joint_torque_weights(std::vector<double> &i_weights)
  {
    assert(i_weights.size() == joint_torque_weights_.size());
    for(unsigned int i = 0; i < i_weights.size(); ++i)
      joint_torque_weights_[i] = i_weights[i];
    objective_computed_ = false;
  }

  void
  MinTorqueTask::compute_objective(double time_step)
  {
    if (objective_computed_) return;
    
    for(unsigned int i = 0; i < joint_torque_weights_.size(); ++i)
      D_(i,i) = joint_torque_weights_[i];

    objective_computed_ = true;
  }


} //end of namespace jrl_qp_controller
