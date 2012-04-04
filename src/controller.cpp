
#include <iostream>
#include <cassert>

#include <jrl/dynamics/urdf/parser.hh>

#include "controller.h"

#include <jrl_qp_controller/position-task.hh>
#include <jrl_qp_controller/transformation-task.hh>
#include <jrl_qp_controller/contact-constraint.hh>
#include <jrl_qp_controller/problem.hh>
#include <jrl_qp_controller/min-torque-task.hh>

using jrl_controller_manager::JrlControl;
using jrl_controller_manager::FreeFlyerState;
using sensor_msgs::JointState;


namespace jrl_qp_controller {

  Controller::Controller(CjrlDynamicRobot * i_robot,
			 jrl_controller_manager::JrlControl * i_msg)
    :robot_(i_robot),
     msg_(i_msg),
     did_allocate_msg_(false),
     control_size_(0)
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
    current_acceleration_.resize(robot_->numberDof());
    for(unsigned int i = 0; i < robot_->numberDof(); ++i)
      current_acceleration_(i) = 0;
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

    /*
    std::cout << "Robot dynamics:" << std::endl;
    std::cout << "Config: " << robot_->currentConfiguration() << std::endl;
    std::cout << "Velocity: " << robot_->currentVelocity() << std::endl;
    std::cout << "Acceleration: " << robot_->currentAcceleration() << std::endl;
    std::cout << "Torque drift: " << robot_->currentJointTorques()  << std::endl;
    */
    robot_->computeInertiaMatrix();
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
  CjrlDynamicRobot * jrl_dynamic_robot = 
    parser.parseStream(urdf_string,"free_flyer_joint");

  if (!jrl_dynamic_robot) {
    std::cerr << "Error: Failed to parse robot\n";
    return 1;
  }
  
  std::string property,value;
  property="ComputeZMP"; value="true";
  jrl_dynamic_robot->setProperty ( property,value );
  property="TimeStep"; value="0.005";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeAccelerationCoM"; value="true";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeBackwardDynamics"; value="true";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeMomentum"; value="true";
  jrl_dynamic_robot->setProperty ( property,value );
  property="ComputeAcceleration"; value="true";
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


  Problem problem(jrl_dynamic_robot);
  //contact
  matrix4d contact_transformation;
  std::vector<vector3d> polygon;
  polygon.push_back(vector3d(0.1,0.1,0));
  polygon.push_back(vector3d(0.1,-0.1,0));
  polygon.push_back(vector3d(-0.1,0.1,0));
  polygon.push_back(vector3d(-0.1,-0.1,0));
  ContactConstraint contact(jrl_dynamic_robot,jrl_dynamic_robot->rootJoint(),
			    vector3d(0,0,0),polygon,contact_transformation);
  problem.add_contact(&contact);
  //fix contact
  TransformationTask transformation_task(jrl_dynamic_robot,
					 jrl_dynamic_robot->rootJoint(),
					 vector3d(0,0,0),
					 contact_transformation);
  transformation_task.weight(1);
  transformation_task.set_gain(10);
  problem.add_task(&transformation_task);

  //position task
  PositionTask position_task(jrl_dynamic_robot,jrl_dynamic_robot->jointVector()[1],
			     vector3d(0,0,0.95),vector3d(0,0.2,1));
  position_task.weight(1);
  position_task.set_gain(10);
  problem.add_task(&position_task);
  //min torque task
  MinTorqueTask min_torque_task(jrl_dynamic_robot);
  min_torque_task.weight(1);
  //problem.add_task(&min_torque_task);
  //initialize
  problem.resize_data();
 

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

  ros::Publisher control_pub = n.advertise<JrlControl>("command",10);


  std::cout << "Entering spinner\n";
  while (ros::ok()) {
    //Receive a joint state message before computing control
    ros::spinOnce();

    controller.compute_robot_dynamics();

    problem.initialize_data();
    problem.set_contact_constraints(); 
    problem.build_dynamic_equation();
    problem.compute_objective();
    problem.solve();
    std::vector<double> solution = problem.solution_torques();

    //std::cout << "old implementation: " << msg.torques[0] << std::endl;

    /*
    std::cout << "new implementation: " ;
    for(std::vector<double>::iterator it = solution.begin();it != solution.end(); it++)
      std::cout << *it << " , ";
    std::cout << std::endl;
    */
    msg.torques[0] =  solution[6];
     
    control_pub.publish(msg);
    loop_rate.sleep();
  }

  return 0;
}
