#ifndef JRL_QP_CONTROLLER_TASK_HH
#define JRL_QP_CONTROLLER_TASK_HH

#include <vector>

#include <abstract-robot-dynamics/dynamic-robot.hh>

namespace jrl_qp_controller {

  class Task {
  public:
    enum EVariables
      {ACCELERATION,
       TORQUES,
       BOTH,
       CONTACT_FORCES};
    
    Task(CjrlDynamicRobot * i_robot,
	 EVariables i_variables=BOTH,
	 bool i_has_quadratic_part=true,
	 bool i_has_linear_part=true,
	 unsigned int i_count_contacts = 0)
      :robot_(i_robot),
       concerned_variables_(i_variables),
       has_quadratic_part_(i_has_quadratic_part),
       has_linear_part_(i_has_linear_part),
       weight_(1.)
    {
      if(!robot_) {
	return;
      }
      unsigned int nb_dofs = robot_->numberDof();
      unsigned int nb_actuated_dofs = robot_->getActuatedJoints().size();
      unsigned int size;
      switch (concerned_variables_) {
      case ACCELERATION:
	size = nb_dofs; break;
      case TORQUES:
	size = nb_actuated_dofs; break;
      case BOTH:
	size = nb_dofs + nb_actuated_dofs; break;
      case CONTACT_FORCES:
	size = 3*i_count_contacts;
      }
      if(has_quadratic_part_) {
	D_.resize(size,size,false);
      } 
      if(has_linear_part_) {
	c_.resize(size,0);
      }
    }

    virtual ~Task()
    {
      D_.clear();
      c_.clear();
    }

    const matrixNxP & get_d() const
    { return D_; }

    const vectorN & get_c() const
    { return c_; }

    virtual void compute_objective(double time_step) = 0;

    EVariables concerned_variables() const
    { return concerned_variables_; }

    double weight() const
    { return weight_; }

    void weight(double i_weight)
    { weight_ = i_weight; }

    bool has_quadratic_part() const
    { return has_quadratic_part_; }

    bool has_linear_part() const
    { return has_linear_part_; }
    
  protected:
    CjrlDynamicRobot * robot_;
    EVariables concerned_variables_;
    bool has_quadratic_part_;
    bool has_linear_part_;
    matrixNxP D_;
    vectorN c_;
    double weight_;
  };

} // end of namespace jrl_qp_controller


#endif // JRL_QP_CONTROLLER_TASK_HH
