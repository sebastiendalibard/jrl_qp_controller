#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/com-task.hh>

namespace jrl_qp_controller {

  ComTask::ComTask(CjrlDynamicRobot * i_robot,
		   const vector3d &i_target)
    :GikFeatureTask(i_robot),
     gik_com_constraint_(NULL)
  {
    gik_com_constraint_ = 
      new ChppGikComConstraint(*i_robot,i_target(0),i_target(1));
    target_.resize(3);
    target(i_target);
    set_gik_constraint(gik_com_constraint_);
  }

  ComTask::~ComTask()
  {
    if (gik_com_constraint_)
      delete gik_com_constraint_;
  }

  void
  ComTask::target(const vector3d &i_target)
  {
    for(unsigned int i = 0; i < 3; ++i) target_(i) = i_target(i); 
    gik_com_constraint_->targetXYZ(target_);
  }

} // end of namespace jrl_qp_controller
