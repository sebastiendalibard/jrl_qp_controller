#ifndef JRL_QP_CONTROLLER_CONTACT_CONSTRAINT_HH
#define JRL_QP_CONTROLLER_CONTACT_CONSTRAINT_HH

#include <vector>

namespace jrl_qp_controller {

  class ContactConstraint {
  public:
    ContactConstraint(CjrlDynamicRobot *i_robot, 
		      CjrlJoint *i_joint,
		      vector3d i_contact_point,
		      std::vector<vector3d> &i_polygon,
		      vector3d i_contact_normal);
    ~ContactConstraint();
    
  private:
    /*
       Robot 
    */
    CjrlDynamicRobot * robot_;

    /* 
       Joint whose attached body is in contact 
    */
    CjrlJoint * joint_;

    /* 
       Point where the contact forces are computed,
       expressed in the joint_ frame 
    */
    vector3d contact_point_;

    /* 
       Points defining the convex hull in which the
       center of pressure has to stay 
    */
    std::vector<vector3d> contact_polygon;
  };
} //end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_CONTACT_CONSTRAINT_HH
