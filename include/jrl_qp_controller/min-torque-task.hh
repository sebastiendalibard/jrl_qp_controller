#ifndef JRL_QP_CONTROLLER_MIN_TORQUE_TASK_HH
#define JRL_QP_CONTROLLER_MIN_TORQUE_TASK_HH

#include <jrl_qp_controller/task.hh>

namespace jrl_qp_controller {
  /*
    Task minimizing the sum of the square
    of joint torques. It is possible to weight
    differently each joint.
  */
  class MinTorqueTask: public Task {
  public:
    MinTorqueTask(CjrlDynamicRobot * i_robot);
    ~MinTorqueTask();

    virtual void compute_objective();

    virtual void set_joint_torque_weights(std::vector<double> &i_weights);

  protected:
    std::vector<double> joint_torque_weights_;

  };
} //end of namespace jrl_qp_controller


#endif // JRL_QP_CONTROLLER_MIN_TORQUE_TASK_HH
