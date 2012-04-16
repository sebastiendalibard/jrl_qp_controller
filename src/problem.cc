# include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>

#include "CGAL/exceptions.h"

#include <ros/console.h>
#include <sstream>

#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/problem.hh>
#include <jrl_qp_controller/contact-constraint.hh>
#include <jrl_qp_controller/task.hh>

using std::vector;

namespace jrl_qp_controller {

  Problem::Problem(CjrlDynamicRobot * i_robot)
    :robot_(i_robot)
  {
  }

  Problem::~Problem()
  {
  }

  void
  Problem::add_task(Task * i_task)
  {
    tasks_.push_back(i_task);
  }

  void
  Problem::remove_task(Task * i_task)
  {
    std::remove< vector<Task*>::iterator, Task*>
      (tasks_.begin(),tasks_.end(),i_task);
  }

  void
  Problem::reset_tasks()
  {
    tasks_.clear();
  }

  void
  Problem::add_contact(ContactConstraint * i_contact)
  {
    contacts_.push_back(i_contact);
  }
  
  void
  Problem::remove_contact(ContactConstraint * i_contact)
  {
    std::remove< vector<ContactConstraint*>::iterator, ContactConstraint*>
      (contacts_.begin(),contacts_.end(),i_contact);
  }

  void
  Problem::reset_contacts()
  {
    contacts_.clear();
  }

  void
  Problem::resize_data()
  {
    unsigned int n_dofs = robot_->numberDof();
    unsigned int n_actuated_dofs = robot_->getActuatedJoints().size();
    unsigned int n_contacts = contacts_.size();
    
    /*
      Unknowns:  
      * acceleration (n_dofs)
      * torques (n_actuated_dofs)
      * contact forces (3*n_contacts)

      Constraints: 
      * dynamic equation (n_dofs)
      * positive acceleration (1D) of contact point (n_contacts)
   
      */
    n_rows_ = n_dofs + n_contacts ;
    n_columns_ = n_dofs + n_actuated_dofs + 3*n_contacts;

    A_.resize(n_rows_,n_columns_,false);
    b_.resize(n_rows_,0);
    r_.resize(n_rows_);
    fl_.resize(n_columns_,false);
    l_.resize(n_columns_,0);
    fu_.resize(n_columns_,false);
    u_.resize(n_columns_,0);
    c_.resize(n_columns_,0);  
    D_.resize(n_columns_,n_columns_,false);
    solution_.resize(n_columns_,0);
    solution_torques_.resize(n_actuated_dofs);
  }

  void
  Problem::initialize_data()
  {
    unsigned int n_dofs = robot_->numberDof();
    unsigned int n_contacts = contacts_.size();

    for(unsigned int i = 0; i < n_rows_; ++i) {
      for(unsigned int j = 0; j < n_columns_; ++j) {
	A_(i,j) = 0;
      }
      b_[i] = 0;
    }
    
    for(unsigned int i = 0; i < n_columns_; ++i) {
      for(unsigned int j = 0; j < n_columns_; ++j) {
	D_(i,j) = 0;
      }
      c_[i] = 0;
    }

    /* Set equality/inequality constraints */
    /* -- Dynamic equation (including contact forces) */
    for(unsigned int i = 0; i < n_dofs;	++i) {
      r_[i] = CGAL::EQUAL;
    }
    /* -- Positive acceleration along frictional cone basis at each contact point */
    for(unsigned int i = n_dofs; i < n_rows_; ++i) {
      r_[i] = CGAL::LARGER;
    }

    /* Actuated joint selection matrix */
    vector<CjrlJoint *> actuated_joints = robot_->getActuatedJoints();
    for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
      for(unsigned int j = 0; j < actuated_joints.size(); ++j) {
	A_(i,robot_->numberDof() + j) = 
	  (i == actuated_joints[j]->rankInConfiguration())? -1 : 0;
      }
    }

    /* Positive contact reactions */    
    unsigned int current_column = n_dofs + actuated_joints.size();
    for(unsigned int i = 0; i < 3*n_contacts; ++i) {
      fl_[current_column + i] = true;
      l_[current_column + i] = 0;
    }
  }

  void
  Problem::build_dynamic_equation()
  {
    /* 
       H.ddot{q} - S.torques - tr(J_c).V_c.f_c = -drift
       Notes: 
       -- S has been set in initialize()
       -- f_c is expressed in the contact friction basis V_c
       -- the product tr(J_c).V_c has already been computed in 
          the corresponding contact constraint object.
    */

    matrixNxP inertia_matrix = robot_->inertiaMatrix();
    vectorN dynamic_drift = robot_->currentJointTorques();

    for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
      for(unsigned int j = 0; j < robot_->numberDof(); ++j) {
	A_(i,j) = inertia_matrix(i,j);
      }
    }
    unsigned int current_column = robot_->numberDof() + robot_->getActuatedJoints().size();
    for (vector<ContactConstraint*>::iterator contact_it = contacts_.begin();
	 contact_it != contacts_.end(); 
	 ++contact_it) {
      for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
	for (unsigned int j = 0; j < 3; ++j) {
	  A_(i,current_column + j) = -(*contact_it)->dyn_mat_(i,j);
	}
      }
      current_column += 3;
    }

    for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
      b_[i] = -dynamic_drift(i);
    } 
  }

  void
  Problem::set_torque_limits(vector<double> &i_l,
			     vector<double> &i_u)
  {
    for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
      fl_[robot_->numberDof() + i] = true;
      l_[robot_->numberDof() + i] = i_l[i];
      fu_[robot_->numberDof() + i] = true;
      u_[robot_->numberDof() + i] = i_u[i];
    }
  }

  void
  Problem::set_contact_constraints()
  {
    /*
      For each contact point, the acceleration of the robot point
      along the contact normal should be positive.
    */
    vectorN current_velocity = robot_->currentVelocity();
    double drift;
    unsigned int current_row = robot_->numberDof();
    for (vector<ContactConstraint*>::iterator contact_it = contacts_.begin();
	 contact_it != contacts_.end(); 
	 ++contact_it) {
      drift = 0; // drift is -dot{J}.dot{q} (both are 1D)
      for(unsigned int i = 0; i < robot_->numberDof(); ++i) 
	drift -= (*contact_it)->d_jacobian_(0,i)*current_velocity(i);
      b_[current_row] = drift;

      for (unsigned int j = 0; j < 3; ++j) {
	A_(current_row,j) = (*contact_it)->normal_jacobian_(0,j);
      }
      ++current_row;
     }
  }

  void
  Problem::compute_objective()
  {
    for(vector<Task *>::iterator task_it = tasks_.begin(); 
	task_it != tasks_.end();
	++task_it) {

      double weight = (*task_it)->weight();
      unsigned int nb_actuated_dofs = robot_->getActuatedJoints().size();
      /* Set D_ */
      if ((*task_it)->has_quadratic_part()) {
	const matrixNxP task_D = (*task_it)->get_d();
	
	switch((*task_it)->concerned_variables()) {
	case Task::ACCELERATION:
	  for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
	    for(unsigned int j = 0; j < robot_->numberDof(); ++j) {
	      D_(i,j) += weight * task_D(i,j);
	    }
	  }
	  break;
	case Task::TORQUES:
	  for(unsigned int i = 0; i < nb_actuated_dofs; ++i) {
	    for(unsigned int j = 0; j < nb_actuated_dofs; ++j) {
	      D_(robot_->numberDof() + i,robot_->numberDof() + j) += weight * task_D(i,j);
	    }
	  }
	  break;
	case Task::BOTH:
	  for(unsigned int i = 0; i < robot_->numberDof() + nb_actuated_dofs; ++i) {
	    for(unsigned int j = 0; j < robot_->numberDof() + nb_actuated_dofs; ++j) {
	      D_(i,j) += weight * task_D(i,j);
	    }
	  }	  
	}
      }
      /* Set c_ */
      if ((*task_it)->has_linear_part()) {
	const vectorN task_c = (*task_it)->get_c();
	switch((*task_it)->concerned_variables()) {
	case Task::ACCELERATION:
	  for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
	    c_[i] += weight * task_c(i);
	  }
	  break;
	case Task::TORQUES:
	  for(unsigned int i = 0; i < nb_actuated_dofs; ++i) {
	    c_[robot_->numberDof() + i] += weight * task_c(i);
	  }
	  break;
	case Task::BOTH:
	   for(unsigned int i = 0; i < robot_->numberDof() + nb_actuated_dofs; ++i) {
	     c_[i] += weight * task_c(i);
	   }
	}
      }
    }
  }

  void
  Problem::compute_acceleration_bounds()
  {
    //TODO
  }

  void
  Problem::solve()
  {
    //Make sure D is definite positive
    for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
      D_(i,i) += 1e-9;
    }

    Program qp(n_columns_,n_rows_,
	       matrix_iterator(A_.begin2()),
	       b_.begin(),
	       r_.begin(),
	       fl_.begin(),
	       l_.begin(),
	       fu_.begin(),
	       u_.begin(),
	       matrix_iterator(D_.begin2()),
	       c_.begin());

    CGAL::Quadratic_program_options options;
    //options.set_verbosity(1);
    options.set_pricing_strategy (CGAL::QP_PARTIAL_FILTERED_DANTZIG);
   
    try {
      boost::posix_time::ptime start, end;
      start = boost::posix_time::microsec_clock::local_time();

      Solution s = CGAL::solve_quadratic_program(qp, CGAL::Gmpzf(),options);

      end = boost::posix_time::microsec_clock::local_time();
      boost::posix_time::time_duration duration = end - start;
      long uDuration = duration.total_microseconds();

      ROS_DEBUG_STREAM("QP Solving time: " << ((double) uDuration * 1e-6) );

      std::stringstream qp_stream;
      CGAL::print_quadratic_program(qp_stream,qp);
      ROS_DEBUG_STREAM("QP Problem: " << qp_stream.str());
      
      ROS_DEBUG_STREAM("QP Solution: " << s);

      //Store result
      unsigned int i = 0;
      Solution::Variable_value_iterator it = s.variable_values_begin();
      while (it != s.variable_values_end()) {
	solution_[i] = to_double(*it);
	++it;
	++i;
      }
      vector<double>::iterator begin_torques = 
	solution_.begin() + robot_->numberDof();
      vector<double>::iterator end_torques =
	begin_torques + robot_->getActuatedJoints().size();
      std::copy(begin_torques,end_torques,solution_torques_.begin());
    }
    catch (CGAL::Assertion_exception e) {
      std::cout << "Invalid problem: " << e.expression() << std::endl;
    }
  }

  std::vector<double>& 
  Problem::solution()
  {
    return solution_;
  }

  std::vector<double>&
  Problem::solution_torques()
  {
    return solution_torques_;
  }

  void
  Problem::compute_one_step()
  {
    for(vector<ContactConstraint*>::iterator contact_it = contacts_.begin();
	contact_it != contacts_.end(); 
	++contact_it) {
      (*contact_it)->update_jacobian();
    }
    for(vector<Task *>::iterator task_it = tasks_.begin(); 
	task_it != tasks_.end();
	++task_it) {
      (*task_it)->compute_objective();
    }
    initialize_data();
    compute_objective();
    build_dynamic_equation();
    set_contact_constraints();
    solve();
  }

}
