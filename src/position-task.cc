#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/position-task.hh>

namespace jrl_qp_controller {

  PositionTask::PositionTask(CjrlDynamicRobot * i_robot,
			     CjrlJoint* i_joint,
			     const vector3d &i_local_point,
			     const vector3d &i_target)
    :GikFeatureTask(i_robot),
     gik_position_constraint_(NULL)
  {
    gik_position_constraint_ = 
      new ChppGikPositionConstraint(*i_robot,*i_joint,i_local_point,i_target);
    set_gik_constraint(gik_position_constraint_);
  }

  PositionTask::~PositionTask()
  {
    if (gik_position_constraint_)
      delete gik_position_constraint_;
  }

  void
  PositionTask::target(const vector3d &i_target)
  {
    gik_position_constraint_->worldTarget(i_target);
  }

} // end of namespace jrl_qp_controller
