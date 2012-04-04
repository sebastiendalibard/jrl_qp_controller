#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/transformation-task.hh>

namespace jrl_qp_controller {

  TransformationTask::TransformationTask(CjrlDynamicRobot * i_robot,
					 CjrlJoint* i_joint,
					 const vector3d &i_local_point,
					 const matrix4d &i_target)
    :GikFeatureTask(i_robot),
     gik_transformation_constraint_(NULL)
  {
    gik_transformation_constraint_ =
      new ChppGikTransformationConstraint(*i_robot,*i_joint,i_local_point,i_target);
    set_gik_constraint(gik_transformation_constraint_);
  }

  TransformationTask::~TransformationTask()
  {
    if(gik_transformation_constraint_)
      delete gik_transformation_constraint_;
  }

  void
  TransformationTask::target(const matrix4d &i_target)
  {
    gik_transformation_constraint_->targetTransformation(i_target);
  }

} // end of namespace jrl_qp_controller
