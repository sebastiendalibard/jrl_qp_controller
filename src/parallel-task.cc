#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/parallel-task.hh>

namespace jrl_qp_controller {

  ParallelTask::ParallelTask(CjrlDynamicRobot * i_robot,
		 CjrlJoint* i_joint,
		 const vector3d &i_local_direction,
		 const vector3d &i_target)
    :GikFeatureTask(i_robot),
     gik_parallel_constraint_(NULL)
  {
    gik_parallel_constraint_ = new ChppGikParallelConstraint(*i_robot,*i_joint,i_local_direction,i_target);
    set_gik_constraint(gik_parallel_constraint_);
  }

  ParallelTask::~ParallelTask()
  {
    if(gik_parallel_constraint_)
      delete gik_parallel_constraint_;
  }
  
  void
  ParallelTask::target(const vector3d& i_target)
  {
    gik_parallel_constraint_->targetVector(i_target);
  }

} //end of namespace jrl_qp_controller
