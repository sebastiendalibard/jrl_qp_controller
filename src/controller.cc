#include <jrl_qp_controller/controller.hh>

using jrl_controller_manager::JrlControl;
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
    if (msg->name.size() != robot_->numberDof()) {
      std::cerr << "jrl_qp_controller received joint_state message of invalid size.\n";
      return;
    }
    for(unsigned int i = 0; i < msg->name.size(); ++i) {
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

    ROS_DEBUG_STREAM("Robot dynamics:");
    ROS_DEBUG_STREAM("Config: " << robot_->currentConfiguration() << std::endl);
    ROS_DEBUG_STREAM("Velocity: " << robot_->currentVelocity() << std::endl);
    ROS_DEBUG_STREAM("Acceleration: " << robot_->currentAcceleration() << std::endl);
    ROS_DEBUG_STREAM("Torque drift: " << robot_->currentJointTorques()  << std::endl);
  }

}//end of namespace jrl_qp_controller

