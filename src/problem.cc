#include <algorithm>

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
    unsigned int n_contacts = contacts_.size();
    n_vertices_ = 0;
    
    for( vector<ContactConstraint*>::iterator it = contacts_.begin();
	 it != contacts_.end(); 
	 it++) {
      n_vertices_ += (*it)->contact_polygon.size();
    }

    /*
      Unknowns:  
      * acceleration (n_dofs)
      * torques (n_dofs)
      * contact forces (6*n_contacts)
      * f_orth at the contact polygon vertices (n_vertices_)

      Constraints: 
      * dynamic equation (n_dofs)
      * contact forces (3*n_contacts)
      * static contact points (6*n_contacts)
      * f_orth positive (n_vertices_)
   
      */
    n_rows_ = n_dofs + 3*n_contacts + 6*n_contacts + n_vertices_;
    n_columns_ = 2*n_dofs + 6*n_contacts + n_vertices_;

    A_.resize(n_rows_);
    for(vector< vector<double> >::iterator it = A_.begin();
	it != A_.end();
	it++) {
      (*it).resize(n_columns_,0);
    }
    b_.resize(n_rows_,0);
    r_.resize(n_rows_);
    fl_.resize(n_rows_,false);
    l_.resize(n_rows_,0);
    fu_.resize(n_rows_,false);
    u_.resize(n_rows_,0);
    c_.resize(n_rows_,0);

    
    D_.resize(n_columns_);
    for(vector< vector<double> >::iterator it = D_.begin();
	it != D_.end();
	it++) {
      (*it).resize(n_columns_);
    }
  }

  void
  Problem::initialize_data()
  {
    unsigned int n_dofs = robot_->numberDof();
    unsigned int n_contacts = contacts_.size();

    for(unsigned int i = 0; i < n_rows_; ++i) {
      for(unsigned int j = 0; j < n_columns_; ++j) {
	A_[i][j] = 0;
      }
      b_[i] = 0;
    }
    
    for(unsigned int i = 0; i < n_columns_; ++i) {
      for(unsigned int j = 0; j < n_columns_; ++j) {
	D_[i][j] = 0;
      }
      c_[i] = 0;
    }

    /* Set equality/inequality constraints */
    for(unsigned int i = 0;
	i < n_dofs + 3*n_contacts + 6*n_contacts;
	++i) {
      r_[i] = CGAL::EQUAL;
    }
    for(unsigned int i = n_dofs + 3*n_contacts + 6*n_contacts;
	i < n_rows_;
	++i) {
      r_[i] = CGAL::LARGER;
    }

    /* Actuated joint selection matrix */
    for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
      for(unsigned int j = 0; j < robot_->numberDof(); ++j) {
	A_[i][robot_->numberDof() + j] = ((i==j)&&(actuated_dofs_[i]))? -1 : 0;
      }
    }
    /* Orthogonal contact forces are positive at the contact polygon vertices */
    unsigned int current_row = n_dofs + 3*n_contacts + 6*n_contacts;
    unsigned int current_column = 2*n_dofs + 6*n_contacts;
    for(unsigned int i = 0; i < n_vertices_; ++i) {
      for(unsigned int j = 0; j < n_vertices_; ++j) {
	A_[current_row + i][current_column + j] = (i==j)? 1 : 0;
      }
    }
  }

  void
  Problem::build_dynamic_equation()
  {
    /* 
       H.ddot{q} - S.torques - tr(J_c).phi_c = b
       Note: S has been set in initialize()
    */

    matrixNxP inertia_matrix = robot_->inertiaMatrix();
    vectorN dynamic_drift = robot_->currentJointTorques();

    for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
      for(unsigned int j = 0; j < robot_->numberDof(); ++j) {
	A_[i][j] = inertia_matrix(i,j);
      }
    }

    for (unsigned int c = 0; c < contacts_.size(); ++c) {
      unsigned int current_column = 2*robot_->numberDof() + c*6;
      for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
	for (unsigned int j = 0; j < 6; ++j) {
	  A_[i][current_column + j] = -contacts_[c]->jacobian_(j,i);
	}
      }
    }

    for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
      b_[i] = dynamic_drift(i);
    } 
  }

  void
  Problem::set_torque_limits(std::vector<double> &i_l,
			     std::vector<double> &i_u)
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
    unsigned int n_vertices = 0;

    /* 
       Link between contact forces and orthogonal forces
       at the contact polygon vertices
    */ 
    for(unsigned int c = 0; c < contacts_.size(); ++c) {
      ContactConstraint* current_contact = contacts_[c];
      unsigned int current_row = robot_->numberDof() + c*3;
      unsigned int current_column_1 = //corresponding to phi_c
	2*robot_->numberDof() + c*6; 
      unsigned int current_column_2 = //orthogonal forces at contact polygon vertices
	2*robot_->numberDof() + contacts_.size()*6 + n_vertices_; 

      A_[current_row][current_column_1 + 2] = -1;
      A_[current_row + 1][current_column_1 + 3] = -1;
      A_[current_row + 2][current_column_1 + 4] = -1;

      for(unsigned int p = 0;
	  p < current_contact->contact_polygon.size();
	  ++p) {
	A_[current_row][current_column_2 + p] = 1;
	A_[current_row + 1][current_column_2 + p] = current_contact->contact_polygon[p](1);
	A_[current_row + 2][current_column_2 + p] = - current_contact->contact_polygon[p](0);
      }
      n_vertices += current_contact->contact_polygon.size();
    }
    
    /*
      Enforce static contact
      Jc.ddot(q) = - dot(Jc).dot(q)
    */
    for(unsigned int c = 0; c < contacts_.size(); ++c) {
      ContactConstraint* current_contact = contacts_[c];
      unsigned int current_row =  robot_->numberDof() + 3*contacts_.size() + 6*c;
      vectorN temp = -MAL_RET_A_by_B(current_contact->d_jacobian_,robot_->currentVelocity());
      for(unsigned int i = 0; i < 6; ++i) {
	for(unsigned int j = 0; j < robot_->numberDof(); ++j) {
	  A_[current_row + i][j] = current_contact->jacobian_(i,j);
	}
	b_[current_row + i] = temp(i);
      }
    }
  }

  void
  Problem::compute_objective()
  {
    for(vector<Task *>::iterator task_it = tasks_.begin(); 
	task_it != tasks_.end();
	task_it++) {
      double weight = (*task_it)->weight();

      /* Set D_ */
      if ((*task_it)->has_quadratic_part()) {
	const std::vector< std::vector<double> > task_D = (*task_it)->get_d();
	switch((*task_it)->concerned_variables()) {
	case Task::ACCELERATION:
	  for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
	    for(unsigned int j = 0; j < robot_->numberDof(); ++j) {
	      D_[i][j] += weight * task_D[i][j];
	    }
	  }
	  break;
	case Task::TORQUES:
	  for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
	    for(unsigned int j = 0; j < robot_->numberDof(); ++j) {
	      D_[robot_->numberDof() + i][robot_->numberDof() + j] += weight * task_D[i][j];
	    }
	  }
	  break;
	case Task::BOTH:
	  for(unsigned int i = 0; i < 2*robot_->numberDof(); ++i) {
	    for(unsigned int j = 0; j < 2*robot_->numberDof(); ++j) {
	      D_[i][j] += weight * task_D[i][j];
	    }
	  }	  
	}
      }
      /* Set c_ */
      if ((*task_it)->has_linear_part()) {
	const std::vector<double> task_c = (*task_it)->get_c();
	switch((*task_it)->concerned_variables()) {
	case Task::ACCELERATION:
	  for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
	    c_[i] += weight * task_c[i];
	  }
	  break;
	case Task::TORQUES:
	  for(unsigned int i = 0; i < robot_->numberDof(); ++i) {
	    c_[robot_->numberDof() + i] += weight * task_c[i];
	  }
	  break;
	case Task::BOTH:
	   for(unsigned int i = 0; i < 2*robot_->numberDof(); ++i) {
	     c_[i] += weight * task_c[i];
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
    vector< vector<double>::iterator > A_it;
    for(unsigned int i = 0; i < A_.size(); ++i) {
      A_it.push_back(A_[i].begin());
    }
    vector< vector<double>::iterator > D_it;
    for(unsigned int i = 0; i < D_.size(); ++i) {
      D_it.push_back(D_[i].begin());
    }
    
    Program qp(n_columns_,n_rows_,
	       A_it.begin(),
	       b_.begin(),
	       r_.begin(),
	       fl_.begin(),
	       l_.begin(),
	       fu_.begin(),
	       u_.begin(),
	       D_it.begin(),
	       c_.begin());
    Solution s = CGAL::solve_quadratic_program(qp, CGAL::Gmpzf());
    
    //Store result
    unsigned int i = 0;
    Solution::Variable_value_iterator it = s.variable_values_begin();
    while (it != s.variable_values_end()) {
      solution_[i] = to_double(*it);
      it++;
      ++i;
    }
    vector<double>::iterator begin_torques = 
      solution_.begin() + robot_->numberDof();
    vector<double>::iterator end_torques =
      solution_.begin() + 2*robot_->numberDof();
    solution_torques_ = vector<double>(begin_torques,end_torques);
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

}
