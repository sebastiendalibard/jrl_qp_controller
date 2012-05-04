#ifndef JRL_QP_CONTROLLER_MIN_CONTACT_FORCE_HH
#define JRL_QP_CONTROLLER_MIN_CONTACT_FORCE_HH

#include <jrl_qp_controller/task.hh>
#include <jrl_qp_controller/contact-constraint.hh>

namespace jrl_qp_controller {
  /*
    Task minimizing the normal reaction force on
    a body of the robot in contact with the
    environment.
    Note: The constraints of the QP system enforce
    strict positivity of contact forces.
  */
  class MinContactForce: public Task {
  public: 
    MinContactForce(CjrlDynamicRobot * i_robot,
		    const std::vector<ContactConstraint *> &i_contacts,
		    const std::vector<double> &i_weights);
    ~MinContactForce();

    virtual void compute_objective(double time_step);

    virtual void set_contact_weights(const std::vector<double> &i_weights);

    virtual void set_contacts(const std::vector<ContactConstraint *> &i_contacts);

  protected:
    std::vector<ContactConstraint *> contacts_;
    std::vector<double> contact_weights_;
    bool objective_computed_;
  };
} //end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_MIN_CONTACT_FORCE_HH
