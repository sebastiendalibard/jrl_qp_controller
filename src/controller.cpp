
#include <iostream>
#include <cassert>

#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

#include "controller.h"

using jrl_controller_manager::JrlControl;
using sensor_msgs::JointState;

//Exact type for cgal qp solver
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpzf ET;
//Cgal qp solver and solution types
typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

namespace jrl_qp_controller {

  Controller::Controller(CjrlDynamicRobot * i_robot,
			 jrl_controller_manager::JrlControl * i_msg)
    :robot_(i_robot),
     msg_(i_msg),
     did_allocate_msg_(false),
     control_size_(0),
     pos_jacobian_computed_(false)
  {
    if (!robot_) {
      std::cerr << "Controller started with no robot\n";
      return;
    }
    
    if (!msg_) {
      msg_ = new JrlControl;
      did_allocate_msg_ = true;
    }

    control_size_ = robot_->getActuatedJoints().size();
    current_configuration_.resize(robot_->numberDof());
    current_velocity_.resize(robot_->numberDof());
    current_acceleration_.resize(robot_->numberDof(),0.);
  }

  Controller::~Controller() 
  {
    if (did_allocate_msg_) {
      delete msg_;
    }
  }

  void Controller::fill_joints(const sensor_msgs::JointStateConstPtr& msg)
  {
    if (msg->position.size() != control_size_) {
      std::cerr << "jrl_qp_controller received joint_state message of invalid size.\n";
      return;
    }
    std::vector<CjrlJoint*> actuated_joints = robot_->getActuatedJoints();
    for(unsigned int i = 0; i < control_size_; ++i) {
      unsigned int rank = actuated_joints[i]->rankInConfiguration();
      current_configuration_(rank) = msg->position[i];
      current_velocity_(rank) = msg->velocity[i];
    }
  }

  void Controller::fill_free_flyer(const jrl_controller_manager::FreeFlyerStateConstPtr& msg)
  {
    if (msg->position.size() != 6) {
      std::cerr << "jrl_qp_controller received free_flyer_state message of invalid size.\n";
      return;
    }
    for(unsigned int i = 0; i < 6; ++i) {
      current_configuration_(i) = msg->position[i];
      current_velocity_(i) = msg->velocity[i];
    }
  }

  void Controller::compute_robot_dynamics() {
    robot_->currentConfiguration(current_configuration_);
    robot_->currentVelocity(current_velocity_);
    robot_->currentAcceleration(current_acceleration_); 
    // Note: Acceleration fixed to 0, used to compute dynamic drift
    //       with jrl-dynamics free-floating inverse dynamics
    robot_->computeForwardKinematics();
    robot_->computeInertiaMatrix();
  }

  void Controller::compute_objective() {
    // desired acceleration for feature x_ddot_desired
    // x_ddot_desired = k ( error ) - 2 sqrt(k) x_dot
    // D_, c_, c0_
    //Objective: minimize distance between desired acceleration 
    //           and PD given acceleration of joint effector
    // careful: specify 2d
    // 2d = 2 J^T J
    // c = 2 (Jdot qdot - xddot_desired)^T J
    // c0 = (Jdot qdot - xddot_desired)^2

    double k = 10; //gain

    matrixNxP pos_jacobian;
    pos_jacobian.resize(6,robot_->numberDof());
    robot_->getPositionJacobian(*robot_->rootJoint(),
				*constrained_joint_,
				local_point_,
				pos_jacobian);

    if (pos_jacobian_computed_) {
      derivative_pos_jacobian_ = 1/dt_ * (pos_jacobian - last_pos_jacobian_);
      last_pos_jacobian_ = pos_jacobian;
    }
    else {
      last_pos_jacobian_ = pos_jacobian;
      pos_jacobian_computed_;
      derivative_pos_jacobian_.resize(6,robot_->numberDof());
      for(unsigned int i=0;i<6;i++) 
	for(unsigned int j=0;j<robot_->numberDof();j++)
	  derivative_pos_jacobian_(i,j) = 0;
    }

    matrix4d joint_transformation = constrained_joint_->currentTransformation();
    vector4d temp_local_point (local_point_(0),local_point_(1),local_point_(2),1);
    vector4d translation4 = joint_transformation * temp_local_point;
    vectorN  translation;
    translation.resize(3);
    for(unsigned int i=0;i<3;++i) translation(i) = translation4(i);

    vectorN x; //error
    x = target_  - translation;

    vectorN x_dot = MAL_RET_A_by_B(pos_jacobian, robot_->currentConfiguration());
    vectorN x_ddot_desired = k*x - 2*sqrt(k)*x_dot;
    
    D_ = 2* MAL_RET_A_by_B (MAL_RET_TRANSPOSE(pos_jacobian), pos_jacobian);
    vectorN temp = 
      MAL_RET_A_by_B (derivative_pos_jacobian_, robot_->currentVelocity()) 
      - x_ddot_desired;
    //c0_ = MAL_RET_TRANSPOSE(temp) * temp;
    c_ = 2 * MAL_RET_A_by_B ( MAL_RET_TRANSPOSE(pos_jacobian), temp);
  }

  void Controller::compute_control()
  {
    //Constraints corresponding to dynamic equation
    //inertia matrix
    matrixNxP inertia_matrix = robot_->inertiaMatrix();

    //dynamic drift
    vectorN dynamic_drift = robot_->currentJointTorques();

    //Positive contact force constraints
    // Note: for simple pendulum model, point of contact is linked to free-flyer joint.
    // Will make it cleaner later **TODO**
    matrixNxP contact_jacobian;
    
    //PD over the end-effector position
    matrixNxP pos_jacobian;
    
    //Build qp problem
    Program qp(CGAL::EQUAL,false,0,false,0);
    unsigned int current_line = 0;
    unsigned int current_row = 0;
    //Dynamic equation
    // H ddot{q} - S tau - J_c^T phi_c = - drift
    for ( unsigned int i = 0; i < robot_->numberDof(); ++i) {
      for ( unsigned int j = 0; j < robot_->numberDof(); ++j) {
	qp.set_a(current_line + i,j, inertia_matrix(i,j));
      }
    }
    current_row += robot_->numberDof();
    //TODO: specific to inverse pendulum, only one actuated joint
    qp.set_a(robot_->numberDof()-1,current_row + robot_->numberDof()-1,-1);
    current_row += robot_->numberDof();

    for ( unsigned int i = 0; i < robot_->numberDof(); ++i) {
      for ( unsigned int j = 0; j < 6; ++j) {
	qp.set_a(i,j + current_row, -contact_jacobian(j,i));
      }
    }
    
    for( unsigned int i = 0; i < robot_->numberDof(); ++i) {
      qp.set_b(i,dynamic_drift(i));
    }
    current_line += robot_->numberDof();
    //Contact forces: link between phi_c and f_orth
    //For inverse pendulum, 4 contact points (p[i])
    //TODO: adapt for generic support polygon
    double **p = new double*[4];
    for(unsigned int i = 0; i < 4; ++i) p[i] = new double[2];
    p[0][0] = 0.5; p[0][1] = 0.5;
    p[1][0] = 0.5; p[1][1] = -0.5;
    p[2][0] = -0.5; p[2][1] = 0.5;
    p[3][0] = -0.5; p[3][1] = -0.5;

    current_row = 2*robot_->numberDof();

    qp.set_a(current_line, current_row + 2, -1);
    current_row += 6;
    for(unsigned int i = 0; i < 4; ++i) {
      qp.set_a(current_line,current_row + i,1);
    }
    current_line+=1;
    current_row = 2*robot_->numberDof();
    qp.set_a(current_line, current_row + 3, -1);
    for(unsigned int i = 0; i < 4; ++i) {
      qp.set_a(current_line,current_row + i,p[i][1]);
    }
    current_line+=1;
    current_row = 2*robot_->numberDof();
    qp.set_a(current_line, current_row + 4, -1);
    for(unsigned int i = 0; i < 4; ++i) {
      qp.set_a(current_line,current_row + i,p[i][0]);
    }
    current_line+=1;

    //Contact point should not move
    for(unsigned int i = 0; i < 6; ++i) {
      for (unsigned int j = 0; j < robot_->numberDof(); ++j) {
	qp.set_a(current_line + i,j, contact_jacobian(i,j));
      }
    }
    for(unsigned int i = 0; i < 6; ++i)
      qp.set_b(current_line + i, 0); // TODO should be dot{J} dot{q}
    current_line += 6;

    //Inequality: f_orth > 0
    current_row = 2 * robot_->numberDof() + 6;
    for(unsigned int i = 0; i < 4; ++i) {
      qp.set_a(current_line + i, current_row + i, 1);
      qp.set_r(current_line + i, CGAL::LARGER);
    }
     
    for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
      for (unsigned int j = 0; j < robot_->numberDof(); ++j) {
	qp.set_d(i,j,D_(i,j));
      }
    }
    for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
      qp.set_c(i,c_(i));
    }
    qp.set_c0(0);

    
    Solution s = CGAL::solve_quadratic_program(qp, ET());
    unsigned int i = 0;
    Solution::Variable_value_iterator it = s.variable_values_begin();
    command_[i] = to_double(*it); //TMP


  }

}//end of namespace jrl_qp_controller

int main(int argc, char* argv[]) {


  //Build robot model

  //subscribe to configuration and free_flyer messages


  //spin and publish torques

  return 0;
}
