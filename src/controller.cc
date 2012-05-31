#include <sstream>
#include <string>

#include <tf/transform_datatypes.h>

#include <jrl/mal/matrixabstractlayer.hh>
#include <hpp/gik/tools.hh>

#include <jrl_qp_controller/controller.hh>

using jrl_controller_manager::JrlControl;
using sensor_msgs::JointState;
using nav_msgs::Odometry;
using geometry_msgs::TransformStamped;

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

    unsigned int nb_dofs = robot_->numberDof();
    joint_state_msg_ = new JointState;
    joint_state_msg_->name.resize(nb_dofs);
    joint_state_msg_->position.resize(nb_dofs);
    joint_state_msg_->velocity.resize(nb_dofs);

    odom_msg_ = new Odometry;
    odom_tf_ = new TransformStamped;

    control_size_ = robot_->getActuatedJoints().size();
    msg_->torques.resize(control_size_);
    current_configuration_.resize(nb_dofs);
    current_velocity_.resize(nb_dofs);
    current_acceleration_.resize(nb_dofs);
    zero_acceleration_.resize(nb_dofs);

    for(unsigned int i = 0; i < nb_dofs; ++i) {
      current_configuration_(i) = 0;
      current_velocity_(i) = 0;
      current_acceleration_(i) = 0;
      zero_acceleration_(i) = 0;
    }

  }

  Controller::~Controller() 
  {
    if (did_allocate_msg_) {
      delete msg_;
    }
    delete joint_state_msg_;
    delete odom_msg_;
    delete odom_tf_;
  }

  void Controller::fill_joints(const sensor_msgs::JointStateConstPtr& msg)
  {
    if (msg->name.size() != robot_->numberDof()) {
      std::cerr << "jrl_qp_controller received joint_state message of invalid size.\n";
      return;
    }

    for(unsigned int i = 0; i < msg->name.size(); ++i) {
      current_configuration_(i) = msg->position[i];
      current_velocity_(i) = msg->velocity[i];
    }
  }

  void Controller::fill_joints(sensor_msgs::JointState& msg)
  {
    if (msg.name.size() != robot_->numberDof()) {
      std::cerr << "jrl_qp_controller received joint_state message of invalid size.\n";
      return;
    }

    for(unsigned int i = 0; i < msg.name.size(); ++i) {
      current_configuration_(i) = msg.position[i];
      current_velocity_(i) = msg.velocity[i];
    }
    robot_->currentConfiguration(current_configuration_);
    robot_->currentVelocity(current_velocity_);
  }


  void Controller::compute_robot_dynamics() {
    robot_->currentAcceleration(zero_acceleration_);

    // Note: Acceleration fixed to 0, used to compute dynamic drift
    //       with jrl-dynamics free-floating inverse dynamics
    robot_->computeForwardKinematics();
    robot_->computeInertiaMatrix();

    root_pose_ = robot_->rootJoint()->currentTransformation();

    ROS_DEBUG_STREAM("Robot dynamics:");
    ROS_DEBUG_STREAM("Config: " << robot_->currentConfiguration());
    ROS_DEBUG_STREAM("Velocity: " << robot_->currentVelocity());
    ROS_DEBUG_STREAM("Torque drift: " << robot_->currentJointTorques());
  }

  void
  Controller::update_robot(std::vector<double>& acceleration, double delta_t)
  {
    assert(acceleration.size() == robot_->numberDof());
    for(unsigned int i = 0; i < robot_->numberDof(); ++i)
      current_acceleration_(i) = acceleration[i];

    current_configuration_ = robot_->currentConfiguration();
    current_velocity_ = robot_->currentVelocity();

    current_velocity_ *= delta_t;
    current_velocity_ += 0.5*delta_t*delta_t * current_acceleration_;

    ChppGikTools::Rodrigues4d(current_velocity_, root_pose_change_);
    root_pose_ = root_pose_change_ * root_pose_;
    ChppGikTools::splitM4(root_pose_ , root_rotation_, root_translation_);
    ChppGikTools::M3toEulerZYX(root_rotation_, root_euler_angles_);

    current_configuration_ += current_velocity_;
    for(unsigned int i = 0; i < 3; ++i) {
      current_configuration_(i) = root_translation_(i);
      current_configuration_(i+3) = root_euler_angles_(i);
    }

    current_velocity_ = robot_->currentVelocity();
    current_velocity_ += delta_t * current_acceleration_;

    robot_->currentConfiguration(current_configuration_);
    robot_->currentVelocity(current_velocity_);

    /* output new velocity as the command */
    for(unsigned int i = 6; i < robot_->numberDof(); ++i)
      msg_->torques[i-6] = current_velocity_(i);
  }

  void
  Controller::fill_joint_state_message()
  {
    std::vector<CjrlJoint *> joint_vector = robot_->jointVector();

    vectorN current_configuration = robot_->currentConfiguration();
    vectorN current_velocity = robot_->currentVelocity();
  
    ros::Time now = ros::Time::now();
    /*
      ROS Joints are actually DoFs. For jrl joints with several DoFs,
      we create additional joints.
      Note: 0 DoF joints are ignored.
    */
    unsigned int ros_joint_id = 0;
    for (std::vector<CjrlJoint *>::iterator joint_it = joint_vector.begin();
	 joint_it != joint_vector.end();
	 ++joint_it) {
      unsigned int joint_position = (*joint_it)->rankInConfiguration();
      if ((*joint_it)->numberDof() > 1) {
	for(unsigned int dof_id = 0; dof_id < (*joint_it)->numberDof(); ++dof_id) {
	  std::stringstream dof_name;
	  dof_name << (*joint_it)->getName() << "-" << dof_id;
	  joint_state_msg_->name[ros_joint_id] = dof_name.str();
	  joint_state_msg_->position[ros_joint_id] =
	    current_configuration(joint_position + dof_id);
	  joint_state_msg_->velocity[ros_joint_id] =
	    current_velocity(joint_position + dof_id);
	  ros_joint_id++;
	}
      }
      else if ((*joint_it)->numberDof() == 1) {
	joint_state_msg_->name[ros_joint_id] = (*joint_it)->getName();
	joint_state_msg_->position[ros_joint_id] = current_configuration(joint_position);
	joint_state_msg_->velocity[ros_joint_id] = current_velocity(joint_position);
	ros_joint_id++;
      }
    }
    joint_state_msg_->header.stamp = now;

    /*
      Fill odometry message and tf with free-flyer position.
    */
    odom_msg_->pose.pose.position.x = current_configuration(0);
    odom_msg_->pose.pose.position.y = current_configuration(1);
    odom_msg_->pose.pose.position.z = current_configuration(2);

    tf::Quaternion orientation_tf = 
      tf::createQuaternionFromRPY(current_configuration(3),
				  current_configuration(4),
				  current_configuration(5));
    geometry_msgs::Quaternion orientation;
    tf::quaternionTFToMsg(orientation_tf,orientation);
    odom_msg_->pose.pose.orientation = orientation;
    odom_msg_->header.frame_id = "odom";
    odom_msg_->child_frame_id = "base_link";
    odom_msg_->header.stamp = now;

    odom_tf_->header.stamp = now;
    odom_tf_->header.frame_id = "odom";
    odom_tf_->child_frame_id = "base_link";

    odom_tf_->transform.translation.x = current_configuration(0);
    odom_tf_->transform.translation.y = current_configuration(1);
    odom_tf_->transform.translation.z = current_configuration(2);
    odom_tf_->transform.rotation = orientation;
  }


}//end of namespace jrl_qp_controller

