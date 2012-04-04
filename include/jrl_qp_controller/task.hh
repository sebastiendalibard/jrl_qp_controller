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
       BOTH};
    
    Task(CjrlDynamicRobot * i_robot,
	 EVariables i_variables=BOTH,
	 bool i_has_quadratic_part=true,
	 bool i_has_linear_part=true)
      :robot_(i_robot),
       concerned_variables_(i_variables),
       has_quadratic_part_(i_has_quadratic_part),
       has_linear_part_(i_has_linear_part),
       weight_(1.)
    {
      if(has_quadratic_part_) {
	if (concerned_variables_ == BOTH)
	  D_.resize(2*robot_->numberDof(),2*robot_->numberDof(),false);
      	else
	  D_.resize(robot_->numberDof(),robot_->numberDof(),false);
      } 
      if(has_linear_part_) {
	if (concerned_variables_ == BOTH)
	  c_.resize(2*robot_->numberDof(),0);
	else
	  c_.resize(robot_->numberDof(),0);
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

    virtual void compute_objective() = 0;

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
