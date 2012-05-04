#include <iostream>
#include <cassert>

#include <tf/transform_broadcaster.h>
#include <jrl/dynamics/urdf/parser.hh>

#include <ros/console.h>

#include <hpp/gik/robot/mask-factory.hh>

#include <jrl_qp_controller/position-task.hh>
#include <jrl_qp_controller/transformation-task.hh>
#include <jrl_qp_controller/configuration-task.hh>
#include <jrl_qp_controller/contact-constraint.hh>
#include <jrl_qp_controller/min-contact-force.hh>
#include <jrl_qp_controller/plane-task.hh>
#include <jrl_qp_controller/problem.hh>
#include <jrl_qp_controller/min-torque-task.hh>
#include <jrl_qp_controller/com-task.hh>
#include <jrl_qp_controller/controller.hh>

#include "nao-config.hh"

using jrl_controller_manager::JrlControl;
using sensor_msgs::JointState;
using std::ostream; using std::vector;
using nav_msgs::Odometry;



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

  //Put robot in half sitting position
  vectorN half_sitting_config(jrl_humanoid_dynamic_robot->numberDof());
  vectorN zero_velocity(jrl_humanoid_dynamic_robot->numberDof());
  
  nao_config::set_joint_map();
  for(unsigned int i = 0; i < 6; ++i)
    half_sitting_config(i) = nao_config::base_free_flyer[i];

  for(std::map<std::string,double>::iterator dof_it = nao_config::base_config_degrees.begin();
      dof_it != nao_config::base_config_degrees.end();
      ++dof_it) {
    CjrlJoint* joint = parser.mapJrlJoint()[(*dof_it).first];
    if(!joint) {
      
      throw std::runtime_error("");
    }
    half_sitting_config(joint->rankInConfiguration()) = M_PI /180.0 * ((*dof_it).second);
  }

  for(unsigned int i = 0; i < jrl_humanoid_dynamic_robot->numberDof(); ++i)
    zero_velocity(i) = 0;

  jrl_humanoid_dynamic_robot->currentConfiguration(half_sitting_config);
  jrl_humanoid_dynamic_robot->currentVelocity(zero_velocity);
  jrl_humanoid_dynamic_robot->currentAcceleration(zero_velocity);
  jrl_humanoid_dynamic_robot->computeForwardKinematics();

  //Set contacts 
  // for nao: <origin rpy="0 0 0" xyz="0.02 0 0.0075"/> 
  //          <box size="0.16 0.06 0.015"/>
  // -- left foot
  CjrlJoint * left_ankle = jrl_humanoid_dynamic_robot->leftAnkle();
  if (!left_ankle) {
    ROS_ERROR_STREAM("No left ankle found");
    return -1;
  }
    
  ContactConstraint contact_l_1(jrl_humanoid_dynamic_robot, left_ankle,
				vector3d(0.10,0.03,-0.0075),vector3d(0,0,1),0.5);
  problem.add_contact(&contact_l_1);
  ContactConstraint contact_l_2(jrl_humanoid_dynamic_robot, left_ankle,
				vector3d(-0.06,0.03,-0.0075),vector3d(0,0,1),0.5);
  problem.add_contact(&contact_l_2);
  ContactConstraint contact_l_3(jrl_humanoid_dynamic_robot, left_ankle,
				vector3d(-0.06,-0.03,-0.0075),vector3d(0,0,1),0.5);
  problem.add_contact(&contact_l_3);
  ContactConstraint contact_l_4(jrl_humanoid_dynamic_robot, left_ankle,
				vector3d(0.10,-0.03,-0.0075),vector3d(0,0,1),0.5);
  problem.add_contact(&contact_l_4);

  // -- right foot
  CjrlJoint * right_ankle = jrl_humanoid_dynamic_robot->rightAnkle();
  if (!right_ankle) {
    ROS_ERROR_STREAM("No right ankle found");
    return -1;
  }

  ContactConstraint contact_r_1(jrl_humanoid_dynamic_robot, right_ankle,
				vector3d(0.10,0.03,-0.0075),vector3d(0,0,1),0.5);
  problem.add_contact(&contact_r_1);
  ContactConstraint contact_r_2(jrl_humanoid_dynamic_robot, right_ankle,
				vector3d(-0.06,0.03,-0.0075),vector3d(0,0,1),0.5);
  problem.add_contact(&contact_r_2);
  ContactConstraint contact_r_3(jrl_humanoid_dynamic_robot, right_ankle,
				vector3d(-0.06,-0.03,-0.0075),vector3d(0,0,1),0.5);
  problem.add_contact(&contact_r_3);
  ContactConstraint contact_r_4(jrl_humanoid_dynamic_robot, right_ankle,
				vector3d(0.10,-0.03,-0.0075),vector3d(0,0,1),0.5);
  problem.add_contact(&contact_r_4);

  //Tasks on foot position
  matrix4d r_ankle_t = right_ankle->currentTransformation();
  matrix4d l_ankle_t = left_ankle->currentTransformation();

  TransformationTask r_ankle_task(jrl_humanoid_dynamic_robot, right_ankle,
				  vector3d(0,0,0), r_ankle_t);
  TransformationTask l_ankle_task(jrl_humanoid_dynamic_robot, left_ankle,
				  vector3d(0,0,0), l_ankle_t);
  r_ankle_task.set_gain(1000);
  l_ankle_task.set_gain(1000);
  r_ankle_task.weight(10);
  l_ankle_task.weight(10);
  problem.add_task(&r_ankle_task);
  problem.add_task(&l_ankle_task);

  //Configuration task
  vectorN wb_mask(jrl_humanoid_dynamic_robot->numberDof());
  for (unsigned int i = 0; i < 6; ++i) wb_mask(i) = 0;
  for (unsigned int i = 6; i < jrl_humanoid_dynamic_robot->numberDof(); ++i) wb_mask(i) = 1;
  ConfigurationTask config_task(jrl_humanoid_dynamic_robot,half_sitting_config,wb_mask);
  config_task.set_gain(10);
  config_task.weight(0.01);
  problem.add_task(&config_task);

  //min torque task
  MinTorqueTask min_torque_task(jrl_humanoid_dynamic_robot);
  min_torque_task.weight(0.01);
  //problem.add_task(&min_torque_task);

  //plane task
  PlaneTask plane_task(jrl_humanoid_dynamic_robot,
		       jrl_humanoid_dynamic_robot->gazeJoint(),
		       vector3d(0,0,0),
		       vector3d(0,0,0.34),
		       vector3d(1,0,0));
  plane_task.set_gain(50);
  plane_task.weight(1);
  problem.add_task(&plane_task);



  //com task
  ComTask com_task(jrl_humanoid_dynamic_robot, vector3d(0.004,0.0,0.301));
  com_task.set_gain(50);
  com_task.weight(1);
  problem.add_task(&com_task);

  //contact force task
  const std::set<ContactConstraint *> contacts = problem.get_contacts();
  std::vector<double> contact_weights(contacts.size());
  for(unsigned int i = 0; i < 4; ++i) {
    contact_weights[i] = 0;
    contact_weights[i + 4] = 1;
  }
  //MinContactForce contact_task(jrl_humanoid_dynamic_robot, contacts, contact_weights);
  //contact_task.weight(0.01);
  //problem.add_task(&contact_task);

  //initialize
  problem.resize_data();


  ros::Publisher joint_state_pub = n.advertise<JointState>("joint_states",10);
  ros::Publisher odom_pub = n.advertise<Odometry>("odom",10);
  tf::TransformBroadcaster odom_broadcaster;
  
  double time = 0;
  
  ROS_DEBUG_STREAM("Entering main loop");

  while (ros::ok()) {
    /* Publish robot state, including FF */
    controller.fill_joint_state_message();
    joint_state_pub.publish(*(controller.joint_state_msg_));
    odom_pub.publish(*(controller.odom_msg_));
    odom_broadcaster.sendTransform(*(controller.odom_tf_));
    
    controller.compute_robot_dynamics();

    problem.compute_one_step();
    std::vector<double> solution = problem.solution();
    std::vector<double> acceleration(solution.begin(),solution.begin() 
				     + jrl_humanoid_dynamic_robot->numberDof());
    controller.update_robot(acceleration,0.005);


    time += 0.005;
    com_task.target(vector3d(0.004 + 0.05*sin(time),0,0.301));
    ROS_DEBUG_STREAM("Simulated time: " << time);
  }

  return 0;
}


