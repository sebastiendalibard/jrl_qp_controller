#ifndef JRL_QP_CONTROLLER_POSITION_TASK_HH
#define JRL_QP_CONTROLLER_POSITION_TASK_HH

#include <jrl_qp_controller/gik-feature-task.hh>
#include <hpp/gik/constraint/position-constraint.hh>


namespace jrl_qp_controller {

  /* 
     Task servoing a point of the robot to a certain
     3D target.
  */
  
  class PositionTask: public GikFeatureTask {
  public:
    PositionTask(CjrlDynamicRobot * i_robot,
		 CjrlJoint* i_joint,
		 const vector3d &i_local_point,
		 const vector3d &i_target);
    ~PositionTask();
    
    void target(const vector3d &i_target);

  private:
    ChppGikPositionConstraint* gik_position_constraint_;
  };
} // end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_POSITION_TASK_HH
