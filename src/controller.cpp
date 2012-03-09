
#include <iostream>
#include <cassert>

#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

#include <jrl/dynamics/urdf/parser.hh>

#include "controller.h"

using jrl_controller_manager::JrlControl;
using jrl_controller_manager::FreeFlyerState;
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

  void Controller::initialize_task(){
    local_point_ = vector3d(0,0,0.95);

    std::vector<CjrlJoint *> joint_vector = robot_->getActuatedJoints();
    constrained_joint_ =  joint_vector[0];

    target_ .resize(3);
    target_(0)=0; target_(1)=0.2; target_(2)=1;
    
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

    double k = 1; //gain

    matrixNxP pos_jacobian;
    pos_jacobian.resize(3,robot_->numberDof());
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
      derivative_pos_jacobian_.resize(3,robot_->numberDof());
      for(unsigned int i=0;i<3;i++) 
	for(unsigned int j=0;j<robot_->numberDof();j++)
	  derivative_pos_jacobian_(i,j) = 0;
    }

    matrix4d joint_transformation = constrained_joint_->currentTransformation();

    std::cout << "----------------------------" << std::endl
	      << "Time: "
	      << ros::Time::now().toSec() << std::endl;

    std::cout << "Current config: "  << robot_->currentConfiguration()  << std::endl;


    vector4d temp_local_point (local_point_(0),local_point_(1),local_point_(2),1);
    vector4d translation4 = joint_transformation * temp_local_point;
    vectorN  translation;
    translation.resize(3);
    for(unsigned int i=0;i<3;++i) translation(i) = translation4(i);

    vectorN x; //error
    x = target_  - translation;
    std::cout << "task error: " << x << std::endl;

    vectorN x_dot = MAL_RET_A_by_B(pos_jacobian, robot_->currentVelocity());
    
    std::cout << "x_dot: " << x_dot << std::endl;

    vectorN x_ddot_desired = k*x - 2*sqrt(k)*x_dot;

    std::cout << "x_ddot_desired: " << x_ddot_desired << std::endl;
    
    D_ = 2* MAL_RET_A_by_B (MAL_RET_TRANSPOSE(pos_jacobian), pos_jacobian);

    //Make sure D_ is semi-definite positive
    for(unsigned int i = 0; i < robot_->numberDof(); ++i)
      D_(i,i) += 1e-9;

    
    std::cout << "Position jacobian: " << pos_jacobian << std::endl;

    std::cout << "2D: " << D_ << std::endl;


    vectorN temp = 
      MAL_RET_A_by_B (derivative_pos_jacobian_, robot_->currentVelocity()) 
      - x_ddot_desired;
    //c0_ = MAL_RET_TRANSPOSE(temp) * temp;
    c_ = 2 * MAL_RET_A_by_B ( MAL_RET_TRANSPOSE(pos_jacobian), temp);

    std::cout << "c: " << c_ << std::endl;


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
    contact_jacobian.resize(6,robot_->numberDof());
    for(unsigned int i = 0; i < 6; ++i) {
      for (unsigned int j = 0; j < robot_->numberDof(); ++j) {
	contact_jacobian(i,j) = (i==j) ? 1 : 0;
      }
    } 
    
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
    // 	<box size="0.2 0.2 0.1"/>
    //TODO: adapt for generic support polygon
    double **p = new double*[4];
    for(unsigned int i = 0; i < 4; ++i) p[i] = new double[2];
    p[0][0] = 0.1; p[0][1] = 0.1;
    p[1][0] = 0.1; p[1][1] = -0.1;
    p[2][0] = -0.1; p[2][1] = 0.1;
    p[3][0] = -0.1; p[3][1] = -0.1;

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
 
    //objectives
    for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
      for (unsigned int j = 0; j <= i ; ++j) {
	qp.set_d(i,j,D_(i,j));
      }
    }
    //Minimize torques
    for(unsigned int i =  robot_->numberDof(); i < 2*robot_->numberDof(); ++i)
      qp.set_d(i,i,1);
  

    for (unsigned int i = robot_->numberDof(); i < 2*robot_->numberDof()+10; ++i)
      qp.set_d(i,i,0);
    for (unsigned int i = 0; i < robot_->numberDof(); ++i) {
      qp.set_c(i,c_(i));
    }
    qp.set_c0(0);
    
    Solution s = CGAL::solve_quadratic_program(qp, ET());
    std::cout << s ;
    Solution::Variable_value_iterator it = s.variable_values_begin();
    //TMP command size is One
    for (unsigned int i =0; i<2* robot_->numberDof() -1; ++i) it++;

    std::cout << "Computed control: " << to_double(*it) << std::endl;

    msg_->torques[0] = to_double(*it);

  }

  void Controller::set_time_step(double in_time_step)
  {
    dt_ = in_time_step;
  }


}//end of namespace jrl_qp_controller

int main(int argc, char* argv[]) 
{
  using namespace jrl_qp_controller;

  ros::init(argc,argv,"qp_controller");

  JrlControl msg;


  ros::NodeHandle n;

  //Build robot model
  std::string urdf_param_name("robot_description");
  std::string urdf_string;
  if (n.getParam(urdf_param_name,urdf_string)) {
    ROS_DEBUG("found upstream\n%s\n------\n%s\n",
	      urdf_param_name.c_str(),
	      urdf_string.c_str());
  }
  else {
    std::cerr << "Unable to fetch robot model.\n";
    return 1;
  }
  
  ROS_DEBUG("gazebo jrl plugin got urdf file from param server, parsing it...");
  jrl::dynamics::urdf::Parser parser;
  CjrlDynamicRobot * jrl_dynamic_robot = parser.buildFromXmlString(urdf_string,"free_flyer_joint");

  if (!jrl_dynamic_robot) {
    std::cerr << "Error: Failed to parse robot\n";
    return 1;
  }
  
  std::string property,value;
  property="ComputeZMP"; value="false";
  jrl_dynamic_robot->setProperty ( property,value );
  property="TimeStep"; value="0.005";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeAccelerationCoM"; value="false";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeBackwardDynamics"; value="true";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeMomentum"; value="true";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeAcceleration"; value="false";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeVelocity"; value="true";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeSkewCom"; value="true";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeCoM"; value="true";
  jrl_dynamic_robot->setProperty ( property,value );

  msg.torques.resize(jrl_dynamic_robot->getActuatedJoints().size());
  
  //Create controller
  std::cout << "Creating controller\n";
  Controller controller(jrl_dynamic_robot,
			&msg);

  //Initialize position task on end-effector
  controller.initialize_task();

  //subscribe to configuration and free_flyer messages
  std::cout << "Subscribing to joint state messages\n";
  ros::Subscriber joint_state_sub = n.subscribe<JointState>("joint_states",
							    1,
							    &Controller::fill_joints,
							    &controller);
  ros::Subscriber free_flyer_sub = n.subscribe<FreeFlyerState>("free_flyer_state",
							       1,
							       &Controller::fill_free_flyer,
							       &controller);


  //spin and publish torques
  ros::Rate loop_rate(200);
  controller.set_time_step(loop_rate.cycleTime().toSec());

  ros::Publisher control_pub = n.advertise<JrlControl>("command",10);


  std::cout << "Entering spinner\n";
  while (ros::ok()) {
    //Receive a joint state message before computing control
    ros::spinOnce();

    controller.compute_robot_dynamics();
    controller.compute_objective();
    controller.compute_control();

    control_pub.publish(msg);
    loop_rate.sleep();
  }

  return 0;
}
