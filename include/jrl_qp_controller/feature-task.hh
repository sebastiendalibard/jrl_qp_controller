#ifndef JRL_QP_CONTROLLER_FEATURE_TASK_HH
#define JRL_QP_CONTROLLER_FEATURE_TASK_HH

#include <jrl_qp_controller/task.hh>
#include <ros/ros.h>

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

    virtual void update_jacobian_and_value() = 0;

    void set_gain(double i_gain);

  protected:
    double gain_;
    matrixNxP jacobian_;
    vectorN value_;
    matrixNxP last_jacobian_;
    matrixNxP d_jacobian_;
    ros::Time last_robot_update_;
    bool first_call_;

  };
} // end of namespace jrl_qp_controller

#endif // JRL_QP_CONTROLLER_FEATURE_TASK_HH
