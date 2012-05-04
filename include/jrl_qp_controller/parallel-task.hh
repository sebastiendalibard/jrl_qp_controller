#ifndef JRL_QP_CONTROLLER_PARALLEL_TASK_HH
#define JRL_QP_CONTROLLER_PARALLEL_TASK_HH

#include <jrl_qp_controller/gik-feature-task.hh>
#include <hpp/gik/constraint/parallel-constraint.hh>

namespace jrl_qp_controller {

    /* 
     Task servoing a joint of the robot to keep a 
     fixed orientation in world frame.
    */

  class ParallelTask: public GikFeatureTask {
  public:
    ParallelTask(CjrlDynamicRobot * i_robot,
		 CjrlJoint* i_joint,
		 const vector3d &i_local_direction,
		 const vector3d &i_target);
    ~ParallelTask();

    void target(const vector3d& i_target);

  private:
    ChppGikParallelConstraint* gik_parallel_constraint_;
  };
} //end ofnamespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_PARALLEL_TASK_HH
