#include <iostream>
#include <cassert>

#include <jrl/dynamics/urdf/parser.hh>

#include <ros/console.h>
#include <tf/transform_broadcaster.h>

#include <jrl_qp_controller/position-task.hh>
#include <jrl_qp_controller/transformation-task.hh>
#include <jrl_qp_controller/contact-constraint.hh>
#include <jrl_qp_controller/problem-oases.hh>
#include <jrl_qp_controller/min-torque-task.hh>
#include <jrl_qp_controller/angular-momentum-task.hh>
#include <jrl_qp_controller/controller.hh>

using jrl_controller_manager::JrlControl;
using sensor_msgs::JointState;
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
  ROS_DEBUG_STREAM("Creating controller\n");
  Controller controller(jrl_dynamic_robot,
			&msg);

  ProblemOases problem(jrl_dynamic_robot);
  //contact
  matrix4d contact_transformation;
  contact_transformation.setIdentity();
  ContactConstraint contact1(jrl_dynamic_robot,jrl_dynamic_robot->rootJoint(),
			     vector3d(0.1,0.1,-0.5),vector3d(0,0,1),50);
  problem.add_contact(&contact1);
  ContactConstraint contact2(jrl_dynamic_robot,jrl_dynamic_robot->rootJoint(),
			     vector3d(0.1,-0.1,-0.5),vector3d(0,0,1),50);
  problem.add_contact(&contact2);
  ContactConstraint contact3(jrl_dynamic_robot,jrl_dynamic_robot->rootJoint(),
			     vector3d(-0.1,0.1,-0.5),vector3d(0,0,1),50);
  problem.add_contact(&contact3);
  ContactConstraint contact4(jrl_dynamic_robot,jrl_dynamic_robot->rootJoint(),
			     vector3d(-0.1,-0.1,-0.5),vector3d(0,0,1),50);
  problem.add_contact(&contact4);

  //fix contact
  TransformationTask transformation_task(jrl_dynamic_robot,
					 jrl_dynamic_robot->rootJoint(),
					 vector3d(0,0,0),
					 contact_transformation);
  transformation_task.weight(1);
  transformation_task.set_gain(100);
  problem.add_task(&transformation_task);

  //position task
  PositionTask position_task(jrl_dynamic_robot,jrl_dynamic_robot->jointVector()[1],
			     vector3d(0,0,0.95),vector3d(0,0.1,0.9));
  position_task.weight(1);
  position_task.set_gain(10);
  problem.add_task(&position_task);

  //min torque task
  MinTorqueTask min_torque_task(jrl_dynamic_robot);
  min_torque_task.weight(0.1);
  problem.add_task(&min_torque_task);


  //Angular momentum task
  AngularMomentumTask am_task(jrl_dynamic_robot);
  problem.add_task(&am_task);

  //initialize
  problem.resize_data();

  ros::Rate loop_rate(200);
  ros::Publisher joint_state_pub = n.advertise<JointState>("joint_states",10);
  ros::Publisher odom_pub = n.advertise<Odometry>("odom",10);
  tf::TransformBroadcaster odom_broadcaster;

  while (ros::ok()) {
    controller.compute_robot_dynamics();

    problem.compute_one_step();
    std::vector<double> solution = problem.solution();
    std::vector<double> acceleration(solution.begin(),solution.begin() 
				     + jrl_dynamic_robot->numberDof());
    controller.update_robot(acceleration,0.005);
    controller.fill_joint_state_message();

    joint_state_pub.publish(*(controller.joint_state_msg_));
    odom_pub.publish(*(controller.odom_msg_));
    odom_broadcaster.sendTransform(*(controller.odom_tf_));

    position_task.target(vector3d(0,0.1*sin(ros::Time::now().toSec()),1));


  }

}
