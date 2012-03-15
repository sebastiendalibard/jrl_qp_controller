#ifndef JRL_QP_CONTROLLER_PROBLEM_H
#define JRL_QP_CONTROLLER_PROBLEM_H

#include <vector>

#ifndef CGAL_QP_NO_ASSERTIONS
#define CGAL_QP_NO_ASSERTIONS
#endif

#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/Gmpz.h>
#include <CGAL/number_utils.h>

#include <abstract-robot-dynamics/dynamic-robot.hh>

#include <jrl_qp_controller/fwd.hh>

namespace jrl_qp_controller {
  /*
    Class representing a control problem
    to be solved by a qp solver.
  */
  class Problem {
  public:
    typedef CGAL::Quadratic_program_from_iterators
    < std::vector< std::vector<double>::iterator >::iterator, //A
      std::vector<double>::iterator, //b
      std::vector<CGAL::Comparison_result>::iterator, //r
      std::vector<bool>::iterator, //fl
      std::vector<double>::iterator, //l
      std::vector<bool>::iterator, //fu
      std::vector<double>::iterator, //u
      std::vector< std::vector<double>::iterator >::iterator, //D
      std::vector<double>::iterator > //c 
    Program;
  
    typedef CGAL::Quadratic_program_solution<CGAL::Gmpzf> 
    Solution;

  public:
    Problem(CjrlDynamicRobot * i_robot);
    ~Problem();

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
    void compute_acceleration_bounds();

    void solve();

    std::vector<double>& solution();
    std::vector<double>& solution_torques();

  protected:
    CjrlDynamicRobot* robot_;
    std::vector<Task*> tasks_;
    std::vector<ContactConstraint*> contacts_;
    std::vector<bool> actuated_dofs_;
     
    /*
       QP Problem data.
       Following CGAL notations:
       find x such that 
       A.x r b (r is a vector of relations: =, <, >)
       l < x < u (respectively if fl or fu)
       that minimizes tr(x).D.x + tr(c).x
    */
    unsigned int n_rows_;
    unsigned int n_columns_;
    unsigned int n_vertices_;
    std::vector< std::vector<double> > A_;
    std::vector<double> b_;
    std::vector<CGAL::Comparison_result> r_;
    std::vector<bool> fl_;
    std::vector<double> l_;
    std::vector<bool> fu_;
    std::vector<double> u_;
    std::vector<double> c_;
    std::vector< std::vector<double> > D_;

    /*
      Solution vector
    */
    std::vector<double> solution_;
    std::vector<double> solution_torques_;
  };
} //end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_PROBLEM_H
