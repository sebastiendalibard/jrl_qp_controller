#ifndef JRL_QP_CONTROLLER_NAO_HIP_TASK_HH
#define JRL_QP_CONTROLLER_NAO_HIP_TASK_HH

#include <jrl_qp_controller/feature-task.hh>

namespace jrl_qp_controller {

  class NaoHipTask: public FeatureTask {
  public:
    NaoHipTask(CjrlDynamicRobot * i_robot);
    ~NaoHipTask();

    virtual void update_jacobian_and_value(double time_step);

  private:
    unsigned int index_joint_1;
    unsigned int index_joint_2;

  };
} //end of namespace jrl_qp_controller

#endif
