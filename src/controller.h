#ifndef JRL_QP_CONTROLLER_CONTROLLER_H
#define JRL_QP_CONTROLLER_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <jrl_controller_manager/JrlControl.h>
#include <jrl_controller_manager/FreeFlyerState.h>

#include <jrl/mal/matrixabstractlayer.hh>
#include <abstract-robot-dynamics/dynamic-robot.hh>

namespace jrl_qp_controller {
  class Controller {
  public:
    Controller(CjrlDynamicRobot * i_robot,
	       jrl_controller_manager::JrlControl * i_msg = NULL);
    ~Controller();

    void fill_joints(const sensor_msgs::JointStateConstPtr& msg);
    void fill_free_flyer(const jrl_controller_manager::FreeFlyerStateConstPtr& msg);

    void compute_robot_dynamics();

			 
    std::vector<double> command_;
    jrl_controller_manager::JrlControl * msg_;
    CjrlDynamicRobot * robot_;

  private:
    bool did_allocate_msg_;
    unsigned int control_size_;
    vectorN current_configuration_;
    vectorN current_velocity_;
    vectorN current_acceleration_;

  };

}  //end of namespace jrl_qp_controller



#endif //JRL_QP_CONTROLLER_CONTROLLER_H
