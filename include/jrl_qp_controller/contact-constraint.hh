#ifndef JRL_QP_CONTROLLER_CONTACT_CONSTRAINT_HH
#define JRL_QP_CONTROLLER_CONTACT_CONSTRAINT_HH

#include <vector>
#include <ros/ros.h>
#include <abstract-robot-dynamics/dynamic-robot.hh>

namespace jrl_qp_controller {

  class ContactConstraint {
  public:
    ContactConstraint(CjrlDynamicRobot *i_robot, 
		      CjrlJoint *i_joint,
		      const vector3d &i_contact_point,
		      const vector3d &i_contact_normal,
		      double i_friction_coefficient = 0.5);
    ~ContactConstraint();
    
    void update_jacobian();

    void compute_friction_basis(const vector3d &i_contact_normal,
				double i_friction_coefficient);


    /*
       Robot 
    */
    CjrlDynamicRobot * robot_;

    /* 
       Joint whose attached body is in contact 
    */
    CjrlJoint * joint_;

    /* 
       Contact point, expressed in joint_ frame.
    */
    vector3d contact_point_;

    /*
      Linearized friction cone basis.
    */
    matrixNxP friction_basis_;

    /*
      contact jacobian
    */
    matrixNxP jacobian_;

    /*
      transpose of the contact normal
    */
    matrixNxP normal_;

    /*
      Jacobian of the point along the contact normal.
    */
    matrixNxP normal_jacobian_;

    /*
      previous contact jacobian (along the normal)
    */
    matrixNxP last_jacobian_;

    /*
      jacobian (along the normal) time derivative
    */
    matrixNxP d_jacobian_;

    /*
      Part of the system's dynamic equation corresponding to this contact.
      dyn_mat_ = transpose(jacobian_)*friction_basis_
      Stored here to avoid multiple allocations.
    */
    matrixNxP dyn_mat_;

    

    double epsilon_;

    ros::Time last_robot_update_;
    bool first_call_;
  };
} //end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_CONTACT_CONSTRAINT_HH
