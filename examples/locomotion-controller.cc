# include <boost/date_time/posix_time/posix_time.hpp>

#include <tf/transform_broadcaster.h>
#include <jrl/dynamics/urdf/parser.hh>
#include <ros/console.h>

#include <jrl_qp_controller/locomotion-controller.hh>
#include <jrl_qp_controller/LocomotionParameters.h>

#include "nao-config.hh"

using jrl_qp_controller::LocomotionParameters;
using sensor_msgs::JointState;
using std::ostream; using std::vector;


int main(int argc, char* argv[])
{
using namespace jrl_qp_controller;

  ros::init(argc,argv,"locomotion_controller");
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

  //Create controller
  ROS_DEBUG_STREAM("Creating controller\n");
  LocomotionController controller(jrl_humanoid_dynamic_robot);

  //Initialize controller
   vectorN half_sitting_config(jrl_humanoid_dynamic_robot->numberDof());

   nao_config::set_joint_map();
   for(unsigned int i = 0; i < 6; ++i)
     half_sitting_config(i) = nao_config::base_free_flyer[i];

   for(std::map<std::string,double>::iterator dof_it = nao_config::base_config_degrees.begin();
       dof_it != nao_config::base_config_degrees.end();
       ++dof_it) {
     CjrlJoint* joint = parser.mapJrlJoint()[(*dof_it).first];
     if(!joint) {
      throw std::runtime_error("incoherent joint map");
    }
     double value = M_PI /180.0 * ((*dof_it).second);
     half_sitting_config(joint->rankInConfiguration()) = value;
   }

   controller.initialize(half_sitting_config);
   controller.time_step(0.001);

   ros::Subscriber locomotion_parameters_sub = 
     n.subscribe<LocomotionParameters>("locomotion_parameters",
				       1,
				       &LocomotionController::update_locomotion_parameters,
				       &controller);

   ros::Publisher joint_state_pub = n.advertise<JointState>("joint_states",10);
   tf::TransformBroadcaster odom_broadcaster;
   
   boost::posix_time::ptime start, end;

   ros::AsyncSpinner spinner(1);
   spinner.start();

   ROS_DEBUG_STREAM("Entering main loop");
   while (ros::ok()) {
     /* Publish robot state, including FF */
     joint_state_pub.publish(*(controller.joint_state_msg_));
     odom_broadcaster.sendTransform(*(controller.odom_tf_));
     start = boost::posix_time::microsec_clock::local_time();
     controller.compute_one_step();
     end = boost::posix_time::microsec_clock::local_time();
     boost::posix_time::time_duration duration = end - start;
     long uDuration = duration.total_microseconds();
     ROS_DEBUG_STREAM("Iteration time: " << ((double) uDuration * 1e-6) );
   }
   controller.close_stream();
   spinner.stop();
   return 0;
}
