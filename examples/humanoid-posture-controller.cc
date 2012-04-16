#include <iostream>
#include <cassert>

#include <jrl/dynamics/urdf/parser.hh>

#include <ros/console.h>

#include <jrl_qp_controller/position-task.hh>
#include <jrl_qp_controller/transformation-task.hh>
#include <jrl_qp_controller/contact-constraint.hh>
#include <jrl_qp_controller/problem.hh>
#include <jrl_qp_controller/min-torque-task.hh>
#include <jrl_qp_controller/com-task.hh>
#include <jrl_qp_controller/controller.hh>

using jrl_controller_manager::JrlControl;
using sensor_msgs::JointState;

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

  CjrlHumanoidDynamicRobot * jrl_humanoid_dynamic_robot = 
    parser.parseStream(urdf_string,"free_flyer_joint");

  if (!jrl_humanoid_dynamic_robot) {
    std::cerr << "Error: Failed to parse robot\n";
    return 1;
  }
  
  std::string property,value;
  property="ComputeZMP"; value="true";
  jrl_humanoid_dynamic_robot->setProperty ( property,value );
  property="TimeStep"; value="0.005";
  jrl_humanoid_dynamic_robot->setProperty ( property,value );
  property="ComputeAccelerationCoM"; value="true";
  jrl_humanoid_dynamic_robot->setProperty ( property,value );
  property="ComputeBackwardDynamics"; value="true";
  jrl_humanoid_dynamic_robot->setProperty ( property,value );
  property="ComputeMomentum"; value="true";
  jrl_humanoid_dynamic_robot->setProperty ( property,value );
  property="ComputeAcceleration"; value="true";
  jrl_humanoid_dynamic_robot->setProperty ( property,value );
  property="ComputeVelocity"; value="true";
  jrl_humanoid_dynamic_robot->setProperty ( property,value );
  property="ComputeSkewCom"; value="true";
  jrl_humanoid_dynamic_robot->setProperty ( property,value );
  property="ComputeCoM"; value="true";
  jrl_humanoid_dynamic_robot->setProperty ( property,value );

  msg.torques.resize(jrl_humanoid_dynamic_robot->getActuatedJoints().size());
  
  //Create controller
  ROS_DEBUG_STREAM("Creating controller\n");
  Controller controller(jrl_humanoid_dynamic_robot,
			&msg);

  Problem problem(jrl_humanoid_dynamic_robot);

  //Set contacts // for nao: <box size="0.16 0.06 0.015"/>
  // -- left foot
  CjrlJoint * left_ankle = jrl_humanoid_dynamic_robot->leftAnkle();
  if (!left_ankle) {
    ROS_ERROR_STREAM("No left ankle found");
    return -1;
  }
    
  ContactConstraint contact_l_1(jrl_humanoid_dynamic_robot, left_ankle,
				vector3d(0.08,0.03,-0.015),vector3d(0,0,1));
  problem.add_contact(&contact_l_1);
  ContactConstraint contact_l_2(jrl_humanoid_dynamic_robot, left_ankle,
				vector3d(-0.08,0.03,-0.015),vector3d(0,0,1));
  problem.add_contact(&contact_l_2);
  ContactConstraint contact_l_3(jrl_humanoid_dynamic_robot, left_ankle,
				vector3d(-0.08,-0.03,-0.015),vector3d(0,0,1));
  problem.add_contact(&contact_l_3);
  ContactConstraint contact_l_4(jrl_humanoid_dynamic_robot, left_ankle,
				vector3d(0.08,-0.03,-0.015),vector3d(0,0,1));
  problem.add_contact(&contact_l_4);

  // -- right foot
  CjrlJoint * right_ankle = jrl_humanoid_dynamic_robot->rightAnkle();
  if (!right_ankle) {
    ROS_ERROR_STREAM("No right ankle found");
    return -1;
  }

  ContactConstraint contact_r_1(jrl_humanoid_dynamic_robot, right_ankle,
				vector3d(0.08,0.03,-0.015),vector3d(0,0,1));
  problem.add_contact(&contact_r_1);
  ContactConstraint contact_r_2(jrl_humanoid_dynamic_robot, right_ankle,
				vector3d(-0.08,0.03,-0.015),vector3d(0,0,1));
  problem.add_contact(&contact_r_2);
  ContactConstraint contact_r_3(jrl_humanoid_dynamic_robot, right_ankle,
				vector3d(-0.08,-0.03,-0.015),vector3d(0,0,1));
  problem.add_contact(&contact_r_3);
  ContactConstraint contact_r_4(jrl_humanoid_dynamic_robot, right_ankle,
				vector3d(0.08,-0.03,-0.015),vector3d(0,0,1));
  problem.add_contact(&contact_r_4);

  //min torque task
  MinTorqueTask min_torque_task(jrl_humanoid_dynamic_robot);
  min_torque_task.weight(0.1);
  problem.add_task(&min_torque_task);

  //com task
  ComTask com_task(jrl_humanoid_dynamic_robot, vector3d(0.01,0,0.3));
  com_task.weight(1);
  problem.add_task(&com_task);

  //initialize
  problem.resize_data();

  //subscribe to configuration and free_flyer messages
  ROS_DEBUG_STREAM("Subscribing to joint state messages\n");
  ros::Subscriber joint_state_sub = n.subscribe<JointState>("joint_states",
							    1,
							    &Controller::fill_joints,
							    &controller);

  //spin and publish torques
  ros::Rate loop_rate(200);
  ros::Publisher control_pub = n.advertise<JrlControl>("command",10);
  ROS_DEBUG_STREAM("Entering spinner\n");

  while (ros::ok()) {
    //Receive a joint state message before computing control
    ros::spinOnce();

    controller.compute_robot_dynamics();

    ROS_DEBUG_STREAM("CoM Position: " << jrl_humanoid_dynamic_robot->positionCenterOfMass() );

    problem.compute_one_step();
    std::vector<double> solution = problem.solution_torques();
    std::copy(solution.begin(),solution.end(), msg.torques.begin());
    control_pub.publish(msg);
    loop_rate.sleep();
  }

  return 0;
}


