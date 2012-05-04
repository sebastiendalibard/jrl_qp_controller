#ifndef JRL_QP_CONTROLLER_CONFIGURATION_TASK_HH
#define JRL_QP_CONTROLLER_CONFIGURATION_TASK_HH

#include <jrl_qp_controller/gik-feature-task.hh>
#include <hpp/gik/constraint/configuration-constraint.hh>

namespace jrl_qp_controller {

  /*
    Task servoing the robot to a specific configuration
  */

  class ConfigurationTask: public GikFeatureTask {
  public:
    ConfigurationTask(CjrlDynamicRobot * i_robot,
		      const vectorN &i_target,
		      const vectorN &i_mask);
    ~ConfigurationTask();

    void target(const vectorN &i_target);
  private:
    ChppGikConfigurationConstraint* gik_configuration_constraint_;
  };
} // end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_CONFIGURATION_TASK_HH
