#ifndef JRL_QP_CONTROLLER_PLANE_TASK_HH
#define JRL_QP_CONTROLLER_PLANE_TASK_HH

#include <jrl_qp_controller/gik-feature-task.hh>
#include <hpp/gik/constraint/plane-constraint.hh>

namespace jrl_qp_controller {

  /* 
     Task servoing a point of the robot to stay on a plane.
     The plane is defined by a 3d point and a plane normal vector.
  */
  class PlaneTask: public GikFeatureTask {
  public:
    PlaneTask(CjrlDynamicRobot * i_robot,
	      CjrlJoint* i_joint,
	      const vector3d &i_local_point,
	      const vector3d &i_world_plane_point,
	      const vector3d &i_plane_normal);
    ~PlaneTask();

  private:
    ChppGikPlaneConstraint* gik_plane_constraint_;
  };
}//end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_PLANE_TASK_HH
