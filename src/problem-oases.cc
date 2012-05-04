# include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>

#include <ros/console.h>
#include <sstream>

#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/problem-oases.hh>
#include <jrl_qp_controller/contact-constraint.hh>
#include <jrl_qp_controller/task.hh>

using std::vector;
using std::map;

namespace jrl_qp_controller {

  ProblemOases::ProblemOases(CjrlDynamicRobot * i_robot)
    :robot_(i_robot),
     problem_impl_(NULL),
     first_qp_solve_(true)
  {
  }

  ProblemOases::~ProblemOases()
  {
  }

  void
  ProblemOases::add_task(Task * i_task)
  {
    tasks_.push_back(i_task);
  }

  void
  ProblemOases::remove_task(Task * i_task)
  {
    std::remove< vector<Task*>::iterator, Task*>
      (tasks_.begin(),tasks_.end(),i_task);
  }

  void
  ProblemOases::reset_tasks()
  {
    tasks_.clear();
  }

  void
  ProblemOases::add_contact(ContactConstraint * i_contact)
  {
    contacts_[i_contact] = true;
  }
  
  void
  ProblemOases::remove_contact(ContactConstraint * i_contact)
  {
    contacts_[i_contact] = false;
  }

  void
  ProblemOases::reset_contacts()
  {
    contacts_.clear();
  }
  
  const std::map<ContactConstraint *,bool>&
  ProblemOases::get_contacts() const
  {
    return contacts_;
  }
  
  unsigned int
  ProblemOases::count_activated_contacts() const
  {
    unsigned int res = 0;
    for(map<ContactConstraint*,bool>::const_iterator contact_it = contacts_.begin();
	contact_it != contacts_.end();
	++contact_it) {
      if((*contact_it).second) ++res;
    }
    return res;
  }


  void
  ProblemOases::resize_data()
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
    bl_.resize(n_rows_,0);
    bu_.resize(n_rows_,0);
    l_.resize(n_columns_,0);
    u_.resize(n_columns_,0);
    c_.resize(n_columns_,0);  
    D_.resize(n_columns_,n_columns_,false);
    solution_.resize(n_columns_,0);
    solution_torques_.resize(n_actuated_dofs);

    if (problem_impl_) delete problem_impl_;
    problem_impl_ = new qpOASES::SQProblem(n_columns_,n_rows_,qpOASES::HST_SEMIDEF);
    first_qp_solve_ = true;
    qpOASES::Options options;
    options.setToDefault();
    options.printLevel = qpOASES::PL_LOW;
    problem_impl_->setOptions(options);
  }

  void
  ProblemOases::initialize_data()
  {
    unsigned int n_dofs = robot_->numberDof();
    unsigned int n_contacts = contacts_.size();

    for(unsigned int i = 0; i < n_rows_; ++i) {
      for(unsigned int j = 0; j < n_columns_; ++j) {
	A_(i,j) = 0;
      }
      bl_[i] = -std::numeric_limits<double>::infinity();
      bu_[i] = std::numeric_limits<double>::infinity();
    }
    
    for(unsigned int i = 0; i < n_columns_; ++i) {
      for(unsigned int j = 0; j < n_columns_; ++j) {
	D_(i,j) = 0;
      }
      c_[i] = 0;
      l_[i] = -std::numeric_limits<double>::infinity();
      u_[i] = std::numeric_limits<double>::infinity();
    }

    /* Actuated joint selection matrix */
    vector<CjrlJoint *> actuated_joints = robot_->getActuatedJoints();
    for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
      for(unsigned int j = 0; j < actuated_joints.size(); ++j) {
	A_(i,n_dofs + j) = 
	  (i == j+6)? -1 : 0;
      }
    }

    /* Positive contact reactions */
    unsigned int current_column = n_dofs + actuated_joints.size();
    for(unsigned int i = 0; i < 3*n_contacts; ++i) {
      l_[current_column + i] = 0;
    }
  }

  void
  ProblemOases::build_dynamic_equation()
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
    for (map<ContactConstraint*,bool>::iterator contact_it = contacts_.begin();
	 contact_it != contacts_.end(); 
	 ++contact_it) {
      if ((*contact_it).second) { //contact is activated
	for (unsigned int j = 0; j < 3; ++j) {
	  for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
	    A_(i,current_column + j) = -((*contact_it).first)->dyn_mat_(i,j);
	  }
	  u_[current_column +j] = std::numeric_limits<double>::infinity();
	}
      }
      else {
	  for (unsigned int j = 0; j < 3; ++j) {
	    for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
	      A_(i,current_column + j) = 0;
	    }
	    u_[current_column +j] = 0;
	  }
      }
      current_column += 3;
    }

    for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
      bl_[i] = -dynamic_drift(i);
      bu_[i] = -dynamic_drift(i);
    } 
  }

  void
  ProblemOases::set_torque_limits(vector<double> &i_l,
			     vector<double> &i_u)
  {
    for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
      l_[robot_->numberDof() + i] = i_l[i];
      u_[robot_->numberDof() + i] = i_u[i];
    }
  }

  void
  ProblemOases::set_contact_constraints()
  {
    /*
      For each contact point, the acceleration of the robot point
      along the contact normal should be positive.
    */
    vectorN current_velocity = robot_->currentVelocity();
    double drift;
    unsigned int current_row = robot_->numberDof();
    for (map<ContactConstraint*,bool>::iterator contact_it = contacts_.begin();
	 contact_it != contacts_.end(); 
	 ++contact_it) {
      if((*contact_it).second) { //contact is activated
	drift = 0; // drift is -dot{J}.dot{q} (both are 1D)
	for(unsigned int i = 0; i < robot_->numberDof(); ++i) 
	  drift -= ((*contact_it).first)->d_jacobian_(0,i)*current_velocity(i);
	bl_[current_row] = drift;
	
	for (unsigned int j = 0; j < 3; ++j) {
	  A_(current_row,j) = ((*contact_it).first)->normal_jacobian_(0,j);
	}	
      }
      else {
	bl_[current_row] = -std::numeric_limits<double>::infinity();
	for (unsigned int j = 0; j < 3; ++j) {
	  A_(current_row,j) = 0;
	}
      }
      ++current_row;
    }
  }

  void
  ProblemOases::compute_objective()
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
	  break;
	case Task::CONTACT_FORCES:
	  unsigned int current_row = robot_->numberDof() + nb_actuated_dofs;
	  for(unsigned int i = 0; i < 3*contacts_.size(); ++i) {
	    for(unsigned int j = 0; j < 3*contacts_.size(); ++j) {
	      D_(current_row + i, current_row + j) += weight * task_D(i,j);
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
	  break;
	case Task::CONTACT_FORCES:
	  unsigned int current_row = robot_->numberDof() + nb_actuated_dofs;
	  for(unsigned int i = 0; i < 3*contacts_.size(); ++i) {
	    c_[current_row + i] += weight * task_c(i);
	  }
	}
      }
    }
  }

  void
  ProblemOases::compute_acceleration_bounds(double time_step)
  {
    vector<CjrlJoint*> actuated_joints = robot_->getActuatedJoints();
    for(vector<CjrlJoint*>::const_iterator joint_it = actuated_joints.begin();
	joint_it != actuated_joints.end();
	++joint_it) {
      unsigned int pos = (*joint_it)->rankInConfiguration();
      double qmin = (*joint_it)->lowerBound(0);
      double qmax = (*joint_it)->upperBound(0);
      double q = robot_->currentConfiguration()(pos);
      double dq = robot_->currentVelocity()(pos);
      double ddq_max = 2/time_step*((qmax - q)/time_step - dq);
      double ddq_min = 2/time_step*((qmin - q)/time_step - dq);
      l_[pos] = ddq_min;
      u_[pos] = ddq_max;
    }
  }

  void
  ProblemOases::solve()
  {
    if(!problem_impl_) {
      std::cerr << "Problem has not be initialized. Have you called Problem::resize_data()?"
		<< std::endl;
      return;
    }

    //Make sure D is semi definite positive
    for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
      D_(i,i) += 1e-9;
    }
    int wsr = 10000;

    boost::posix_time::ptime start, end;
    start = boost::posix_time::microsec_clock::local_time();
    qpOASES::returnValue qp_res;
    if (first_qp_solve_) {
      qp_res = problem_impl_->init(&(D_.data()[0]),
				   &(*c_.begin()),
				   &(A_.data()[0]),
				   &(*l_.begin()),
				   &(*u_.begin()),
				   &(*bl_.begin()),
				   &(*bu_.begin()),
				   wsr);
    }
    else {
      qp_res = problem_impl_->hotstart(&(D_.data()[0]),
				       &(*c_.begin()),
				       &(A_.data()[0]),
				       &(*l_.begin()),
				       &(*u_.begin()),
				       &(*bl_.begin()),
				       &(*bu_.begin()),
				       wsr);
    }
    end = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration duration = end - start;
    long uDuration = duration.total_microseconds();

    ROS_DEBUG_STREAM("QP Solving time: " << ((double) uDuration * 1e-6) );

    //Store result
    if(qp_res == qpOASES::SUCCESSFUL_RETURN) {
      problem_impl_->getPrimalSolution(&(*solution_.begin()));
      first_qp_solve_ = false;
    }
    else {
      first_qp_solve_ = true;
    }
    //DEBUG
    vectorN solution_ublas(solution_.size());
    for(unsigned int i = 0; i < solution_.size(); ++i) solution_ublas(i) = solution_[i];
    ROS_DEBUG_STREAM("QP Solution: " << solution_ublas);

    vector<double>::iterator begin_torques = 
      solution_.begin() + robot_->numberDof();
    vector<double>::iterator end_torques =
      begin_torques + robot_->getActuatedJoints().size();
    std::copy(begin_torques,end_torques,solution_torques_.begin());

  }

  std::vector<double>& 
  ProblemOases::solution()
  {
    return solution_;
  }

  std::vector<double>&
  ProblemOases::solution_torques()
  {
    return solution_torques_;
  }

  void
  ProblemOases::compute_one_step(double time_step)
  {
    for(map<ContactConstraint*,bool>::iterator contact_it = contacts_.begin();
	contact_it != contacts_.end(); 
	++contact_it) {
      if((*contact_it).second) ((*contact_it).first)->update_jacobian(time_step);
    }
    for(vector<Task *>::iterator task_it = tasks_.begin(); 
	task_it != tasks_.end();
	++task_it) {
      (*task_it)->compute_objective(time_step);
    }
    initialize_data();
    compute_objective();
    build_dynamic_equation();
    set_contact_constraints();
    compute_acceleration_bounds(time_step);
    solve();
  }

}
