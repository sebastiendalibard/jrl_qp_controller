#ifndef JRL_QP_CONTROLLER_CONTACT_CONSTRAINT_HH
#define JRL_QP_CONTROLLER_CONTACT_CONSTRAINT_HH

#include <vector>
#include <ros/ros.h>

namespace jrl_qp_controller {

  class ContactConstraint {
  public:
    ContactConstraint(CjrlDynamicRobot *i_robot, 
		      CjrlJoint *i_joint,
		      vector3d i_contact_point,
		      std::vector<vector3d> &i_polygon,
		      vector3d i_contact_normal);
    ~ContactConstraint();
    
    /*
       Robot 
    */
    CjrlDynamicRobot * robot_;

    /* 
       Joint whose attached body is in contact 
    */
    CjrlJoint * joint_;

    /* 
       Point where the 6D contact force is computed.
       It is assumed that this point belongs to the contact surface.
       Expressed in joint_ frame. 
    */
    vector3d contact_point_;

    /*
      Global transformation of the contact point.
      This frame z is orthogonal to the contact plane.
    */
    matrix4d contact_transformation_;

    /* 
       Points defining the convex hull in which the
       center of pressure has to stay.
       Expressed in contact_transformation_ frame.
    */
    std::vector<vector3d> contact_polygon;

    /*
      contact jacobian
    */
    matrixNxP jacobian_;

    /*
      previous contact jacobian
    */
    matrixNxP last_jacobian_;

    /*
      jacobian time derivative
    */
    matrixNxP d_jacobian_;

    ros::Time last_robot_update_;

  };
} //end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_CONTACT_CONSTRAINT_HH
