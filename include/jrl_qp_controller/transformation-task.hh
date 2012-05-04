#ifndef JRL_QP_CONTROLLER_TRANSFORMATION_TASK_HH
#define JRL_QP_CONTROLLER_TRANSFORMATION_TASK_HH

#include <jrl_qp_controller/gik-feature-task.hh>
#include <hpp/gik/constraint/transformation-constraint.hh>

namespace jrl_qp_controller {

  /* 
     Task servoing a point of the robot to a certain
     6D target.
  */
  
  class TransformationTask: public GikFeatureTask {
  public:
    TransformationTask(CjrlDynamicRobot * i_robot,
		       CjrlJoint* i_joint,
		       const vector3d &i_local_point,
		       const matrix4d &i_target);
    ~TransformationTask();

    void target(const matrix4d &i_target);
    
    void target(const vector3d &i_target);
  private:
    ChppGikTransformationConstraint* gik_transformation_constraint_;
  };
} // end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_TRANSFORMATION_TASK_HH
