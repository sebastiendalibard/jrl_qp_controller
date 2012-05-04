#ifndef JRL_QP_CONTROLLER_CONTROLLER_H
#define JRL_QP_CONTROLLER_CONTROLLER_H

#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <jrl_controller_manager/JrlControl.h>

#include <jrl/mal/matrixabstractlayer.hh>
#include <abstract-robot-dynamics/dynamic-robot.hh>

namespace jrl_qp_controller {
  class Controller {
  public:
    Controller(CjrlDynamicRobot * i_robot,
	       jrl_controller_manager::JrlControl * i_msg = NULL);
    ~Controller();

    void fill_joints(const sensor_msgs::JointStateConstPtr& msg);

    void update_robot(std::vector<double>& acceleration, double delta_t);

    void compute_robot_dynamics();

    void fill_joint_state_message();

    std::vector<double> command_;
    CjrlDynamicRobot * robot_;
    jrl_controller_manager::JrlControl * msg_;
    sensor_msgs::JointState * joint_state_msg_;
    nav_msgs::Odometry * odom_msg_;
    geometry_msgs::TransformStamped * odom_tf_;

  private:
    bool did_allocate_msg_;
    unsigned int control_size_;
 
    // To avoid multiple allocations
    vectorN current_configuration_;
    vectorN current_velocity_;
    vectorN current_acceleration_;
    vectorN zero_acceleration_;
    matrix4d root_pose_;
    matrix3d root_rotation_;
    vector3d root_euler_angles_;
    vector3d root_translation_;
    matrix4d root_pose_change_;

  };

}  //end of namespace jrl_qp_controller



#endif //JRL_QP_CONTROLLER_CONTROLLER_H
