#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/min-contact-force.hh>

#include <ros/console.h>

namespace jrl_qp_controller {

  MinContactForce::MinContactForce(CjrlDynamicRobot * i_robot,
				   const std::vector<ContactConstraint *> &i_contacts,
				   const std::vector<double> &i_weights)
    :Task(i_robot,Task::CONTACT_FORCES,false,true,i_contacts.size()),
     contacts_(i_contacts),
     contact_weights_(i_weights),
     objective_computed_(false)
  {
  }

  MinContactForce::~MinContactForce()
  {
  }

  void
  MinContactForce::compute_objective(double time_step)
  {
    ROS_DEBUG_STREAM("MinContactForce::compute_objective. c_: " << c_ );
    
    if (objective_computed_)
      return;
    
    for(unsigned int i = 0; i < c_.size(); ++i) c_(i) = 0;
    unsigned int contact_position = 0;
    for(std::vector<ContactConstraint *>::iterator contact_it = contacts_.begin();
	contact_it != contacts_.end();
	++contact_it) {
      matrixNxP tmp_mat = contact_weights_[contact_position] *
	prod((*contact_it)->normal_,(*contact_it)->friction_basis_); 
      for(unsigned int i = 0; i < 3; ++i) c_(3*contact_position + i) = tmp_mat(0,i);
      ++contact_position;
    }
    objective_computed_ = true;
  }

  void
  MinContactForce::set_contact_weights(const std::vector<double> &i_weights)
  {
    contact_weights_ = i_weights;
    objective_computed_ = false;
  }

  void
  MinContactForce::set_contacts(const std::vector<ContactConstraint *> &i_contacts)
  {
    contacts_ = i_contacts;
    unsigned int size = contacts_.size();
    c_.resize(size,false);
    objective_computed_ = false;
  }

} //end of namespace jrl_qp_controller
