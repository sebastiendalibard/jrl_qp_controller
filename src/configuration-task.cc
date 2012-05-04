#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/configuration-task.hh>

namespace jrl_qp_controller {

  ConfigurationTask::ConfigurationTask(CjrlDynamicRobot * i_robot,
				       const vectorN &i_target,
				       const vectorN &i_mask)
    :GikFeatureTask(i_robot),
     gik_configuration_constraint_(NULL)
  {
    gik_configuration_constraint_ =
      new ChppGikConfigurationConstraint(*i_robot,i_target,i_mask);
    set_gik_constraint(gik_configuration_constraint_);
  }

  ConfigurationTask::~ConfigurationTask()
  {
    if(gik_configuration_constraint_)
      delete gik_configuration_constraint_;
  }

  void
  ConfigurationTask::target(const vectorN &i_target)
  {
    gik_configuration_constraint_->target(i_target);
  }

} // end of namespace jrl_qp_controller
