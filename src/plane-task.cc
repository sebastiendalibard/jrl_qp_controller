#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/plane-task.hh>

namespace jrl_qp_controller {

  PlaneTask::PlaneTask(CjrlDynamicRobot * i_robot,
		       CjrlJoint* i_joint,
		       const vector3d &i_local_point,
		       const vector3d &i_world_plane_point,
		       const vector3d &i_plane_normal)
    :GikFeatureTask(i_robot),
     gik_plane_constraint_(NULL)
  {
    gik_plane_constraint_ =
      new ChppGikPlaneConstraint(*i_robot, *i_joint, i_local_point,i_world_plane_point,i_plane_normal);
    set_gik_constraint(gik_plane_constraint_);
  }

  PlaneTask::~PlaneTask()
  {
    if(gik_plane_constraint_)
      delete gik_plane_constraint_;
  }

}//end of namespace jrl_qp_controller
