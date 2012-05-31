#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl_qp_controller/nao-hip-task.hh>

#include <ros/console.h>

namespace jrl_qp_controller {

  NaoHipTask::NaoHipTask(CjrlDynamicRobot * i_robot):
    FeatureTask(i_robot)
  {
    index_joint_1 = 0;
    index_joint_2 = 0;
    jacobian_.resize(1,robot_->numberDof());
    last_jacobian_.resize(1,robot_->numberDof());
    d_jacobian_.resize(1,robot_->numberDof());
    value_.resize(1);
    for(unsigned int i = 0; i < robot_->numberDof(); ++i){
      jacobian_(0,i) = 0;
      d_jacobian_(0,i) = 0;
      last_jacobian_(0,i) = 0;
    }
    std::vector<CjrlJoint* > joints = robot_->jointVector();
    for(std::vector<CjrlJoint* >::iterator it = joints.begin();
	it != joints.end();
	++it) {
      if ((*it)->getName() == "LHipYawPitch") 
	index_joint_1 = (*it)->rankInConfiguration();
      else if  ((*it)->getName() == "RHipYawPitch")
	index_joint_2 = (*it)->rankInConfiguration();
    }
    if(!index_joint_1 || !index_joint_2)
      ROS_ERROR_STREAM("Error, did not find hip joint");
    jacobian_(0,index_joint_1) = 1;
    jacobian_(0,index_joint_2) = -1;
  }

  NaoHipTask::~NaoHipTask()
  {
  }

  void
  NaoHipTask::update_jacobian_and_value(double time_step)
  {
    vectorN config = robot_->currentConfiguration();
    value_(0) = config(index_joint_1) - config(index_joint_2);
  }
}
