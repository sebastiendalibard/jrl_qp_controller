#ifndef JRL_QP_CONTROLLER_PROBLEM_H
#define JRL_QP_CONTROLLER_PROBLEM_H

#include <vector>
#include <map>

#include <boost/config.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include <qpOASES.hpp>

#include <abstract-robot-dynamics/dynamic-robot.hh>

#include <jrl_qp_controller/fwd.hh>

namespace jrl_qp_controller {
  /*
    Class representing a control problem
    to be solved by a qp solver.
  */
  class ProblemOases {
  public:
    ProblemOases(CjrlDynamicRobot * i_robot);
    ~ProblemOases();
    
    /*
      Objective task management
    */
    void add_task(Task * i_task);
    void remove_task(Task * i_task);
    void reset_tasks();
    /*
      Contact constraint management
    */
    void add_contact(ContactConstraint * i_contact);
    void remove_contact(ContactConstraint * i_contact);
    void reset_contacts();
    const std::map<ContactConstraint *,bool>& get_contacts() const;
    unsigned int count_activated_contacts() const;
    
    void set_torque_limits(std::vector<double> &i_l,
			   std::vector<double> &i_u);

    /*
      Called once the contacts and constraints are defined.
      No need to call it again if the dimension of the problem
      does not change.
    */
    void resize_data();

    /*
      Sets the QP data to default: 0 everywhere except for the
      parts that do not depend on the robot state.
    */
    void initialize_data();

    /* 
       Fills QP Problem data
    */
    void build_dynamic_equation();
    void set_contact_constraints();
    void compute_objective();
    void compute_acceleration_bounds(double time_step = 0.005);

    void solve();

    /*
      Convenient way of calling all the functions corresponding
      to one iteration.
     */
    void compute_one_step(double time_step = 0.005);

    /*
      Access to the computed solution.
    */
    std::vector<double>& solution();
    std::vector<double>& solution_torques();

  protected:
    CjrlDynamicRobot* robot_;
    std::vector<Task*> tasks_;
    /* contacts_[c] indicates if c is activated */
    std::map<ContactConstraint*, bool> contacts_;
     
    /*
      QP Problem data.
      Notations:
      find x such that 
      bl <= A.x <= bu 
      l <= x <= u 
      that minimizes tr(x).D.x + tr(c).x
    */
    unsigned int n_rows_;
    unsigned int n_columns_;
    boost::numeric::ublas::matrix<double,
				  boost::numeric::ublas::row_major> A_;
    std::vector<double> bl_;
    std::vector<double> bu_;
    std::vector<double> l_;
    std::vector<double> u_;
    std::vector<double> c_;
    boost::numeric::ublas::matrix<double,
				  boost::numeric::ublas::row_major> D_;

    /*
      Solution vectors
    */
    std::vector<double> solution_;
    std::vector<double> solution_torques_;

    /*
      qp solver
    */
    qpOASES::SQProblem * problem_impl_;
    
    /*
      First call to the qp solver
    */
    bool first_qp_solve_;
  };
} //end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_PROBLEM_H
