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
    {}

    virtual ~Task()
    {}

    const std::vector< std::vector<double> > & get_d() const
    { return D_; }

    const std::vector<double> & get_c() const
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
    std::vector< std::vector<double> > D_;
    std::vector<double> c_;
    double weight_;
  };

} // end of namespace jrl_qp_controller


#endif // JRL_QP_CONTROLLER_TASK_HH
