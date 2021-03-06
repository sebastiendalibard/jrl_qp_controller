#ifndef JRL_QP_CONTROLLER_PROBLEM_H
#define JRL_QP_CONTROLLER_PROBLEM_H

#include <vector>
#include <set>

#ifndef CGAL_QP_NO_ASSERTIONS
#define CGAL_QP_NO_ASSERTIONS
#endif

#include <boost/config.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/numeric/ublas/matrix.hpp>

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
  struct column_iterator {
    typedef const double* result_type;
    result_type operator()(const double &d) const
    { return  &d; }
  };
  
  class Problem {
  public:
    /* 
       Rationale: When dereferenced, 
       matrix_iterator takes an iterator over columns of a matrix, 
       and transforms it on an iterator over the elements of the column.
    */
    
    typedef boost::transform_iterator
    <column_iterator,
     boost::numeric::ublas::matrix<double,
				   boost::numeric::ublas::column_major>::iterator2 >
    matrix_iterator;
    
    
    typedef CGAL::Quadratic_program_from_iterators
    < matrix_iterator, //A
      std::vector<double>::iterator, //b
      std::vector<CGAL::Comparison_result>::iterator, //r
      std::vector<bool>::iterator, //fl
      std::vector<double>::iterator, //l
      std::vector<bool>::iterator, //fu
      std::vector<double>::iterator, //u
      matrix_iterator, //D
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
    const std::set<ContactConstraint *>& get_contacts() const;
    
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
    std::set<ContactConstraint*> contacts_;
     
    /*
      QP Problem data.
      Following CGAL notations:
      find x such that 
      A.x r b (r is a vector of relations: =, <=, >=)
      l <= x <= u (respectively if fl or fu)
      that minimizes tr(x).D.x + tr(c).x
    */
    unsigned int n_rows_;
    unsigned int n_columns_;
    boost::numeric::ublas::matrix<double,
				  boost::numeric::ublas::column_major> A_;
    std::vector<double> b_;
    std::vector<CGAL::Comparison_result> r_;
    std::vector<bool> fl_;
    std::vector<double> l_;
    std::vector<bool> fu_;
    std::vector<double> u_;
    std::vector<double> c_;
    boost::numeric::ublas::matrix<double,
				  boost::numeric::ublas::column_major> D_;

    /*
      Solution vectors
    */
    std::vector<double> solution_;
    std::vector<double> solution_torques_;
  };
} //end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_PROBLEM_H
