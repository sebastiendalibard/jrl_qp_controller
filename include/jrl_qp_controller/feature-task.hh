#ifndef JRL_QP_CONTROLLER_FEATURE_TASK_HH
#define JRL_QP_CONTROLLER_FEATURE_TASK_HH

#include <jrl_qp_controller/task.hh>

namespace jrl_qp_controller {

  /* 
     Task servoing a feature to a certain target.
     A feature is defined as a differentiable
     function of the robot configuration, with value 
     in R^m.
  */
  class FeatureTask: public Task {
  public:
    FeatureTask(CjrlDynamicRobot * i_robot);

    ~FeatureTask();
    
    virtual void compute_objective();

  protected:
    matrixNxP jacobian_;
    vectorN value_;
  };
} // end of namespace jrl_qp_controller

#endif // JRL_QP_CONTROLLER_FEATURE_TASK_HH
