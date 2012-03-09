#ifndef JRL_QP_CONTROLLER_ANGULAR_MOMENTUM_TASK_HH
#define JRL_QP_CONTROLLER_ANGULAR_MOMENTUM_TASK_HH

#include <jrl_qp_controller/task.hh>

namespace jrl_qp_controller {

  /* 
     Task minimizing the angular momentum around the
     center of mass of the robot.
     Not implemented yet.
  */
  class AngularMomentumTask: public Task {
  public:
    AngularMomentumTask(CjrlDynamicRobot* i_robot);
    ~AngularMomentTask();

    virtual void compute_objective();
  };
} //end of namespace jrl_qp_controller

#endif // JRL_QP_CONTROLLER_ANGULAR_MOMENTUM_TASK_HH
