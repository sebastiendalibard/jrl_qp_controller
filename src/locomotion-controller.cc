#include <jrl/mal/matrixabstractlayer.hh>
#include <jrl_qp_controller/locomotion-controller.hh>

#include <hpp/gik/tools.hh>
#include <hpp/gik/robot/mask-factory.hh>

#include <ros/console.h>

namespace jrl_qp_controller {

  LocomotionController::LocomotionController(CjrlHumanoidDynamicRobot * i_robot)
    :Controller(i_robot),
     robot_(i_robot),
     problem_(NULL),
     left_foot_task_(NULL),
     right_foot_task_(NULL),
     config_task_(NULL),
     com_task_(NULL),
     parallel_task_(NULL),
     angular_momentum_task_(NULL),
     min_torque_task_(NULL)
  {
    /*
      Set default values
    */
    time_step_ = 0.005;
    contact_precision_ = 5e-4;
    foot_flight_duration_ = 0.5;
    double_support_duration_ = 0.2;
    step_height_ = 0.001;
    step_length_ = 0.0;
    com_amplitude_ = 0.02;
    com_phase_ = 0;
    x_start_ = 0;
    x_target_ = 0;

    /*
      The following variables will be properly set in initialize()
    */
    ankle_height_ = 0;
    y_left_ = 0;
    y_right_ = 0;
  }

  LocomotionController::~LocomotionController()
  {
    if(problem_) delete problem_;
    free_contacts();
    if(left_foot_task_) delete left_foot_task_;
    if(right_foot_task_) delete right_foot_task_;
    if(config_task_) delete config_task_;
    if(com_task_) delete com_task_;
    if(parallel_task_) delete parallel_task_;
    if(angular_momentum_task_) delete angular_momentum_task_;
    if(min_torque_task_) delete min_torque_task_;
  }

  void
  LocomotionController::initialize(vectorN i_start_config)
  {
    assert(i_start_config.size() == robot_->numberDof());

    /* Set jrl properties */
    std::string property,value;
    property="ComputeZMP"; value="true";
    robot_->setProperty ( property,value );
    property="TimeStep"; value="0.005";
    robot_->setProperty ( property,value );
    property="ComputeAccelerationCoM"; value="true";
    robot_->setProperty ( property,value );
    property="ComputeBackwardDynamics"; value="true";
    robot_->setProperty ( property,value );
    property="ComputeMomentum"; value="true";
    robot_->setProperty ( property,value );
    property="ComputeAcceleration"; value="true";
    robot_->setProperty ( property,value );
    property="ComputeVelocity"; value="true";
    robot_->setProperty ( property,value );
    property="ComputeSkewCom"; value="true";
    robot_->setProperty ( property,value );
    property="ComputeCoM"; value="true";
    robot_->setProperty ( property,value );

    /* Put robot in initial configuration */
    robot_->currentConfiguration(i_start_config);
    vectorN zero_config(robot_->numberDof());
    for(unsigned int i = 0; i < zero_config.size(); ++i) zero_config(i) = 0;
    robot_->currentVelocity(zero_config);
    robot_->currentAcceleration(zero_config);
    robot_->computeForwardKinematics();

    com_position_ = robot_->positionCenterOfMass();
    com_target_ = com_position_;

    /*
      Allocate tasks.
    */
    /* -- left foot */
    matrix4d left_ankle_transformation = robot_->leftAnkle()->currentTransformation();
    left_foot_task_ = new TransformationTask(robot_, robot_->leftAnkle(),vector3d(0,0,0),left_ankle_transformation);
    left_foot_task_->set_gain(1000);
    left_foot_task_->weight(10);

    /* -- right foot */
    matrix4d right_ankle_transformation = robot_->rightAnkle()->currentTransformation();
    right_foot_task_ = new TransformationTask(robot_, robot_->rightAnkle(),vector3d(0,0,0),right_ankle_transformation);
    right_foot_task_->set_gain(1000);
    right_foot_task_->weight(10);

    /* -- config task */
    vectorN wb_mask(robot_->numberDof());
    for (unsigned int i = 0; i < 6; ++i) wb_mask(i) = 0;
    for (unsigned int i = 6; i < robot_->numberDof(); ++i) wb_mask(i) = 1;
    config_task_ = new ConfigurationTask(robot_,i_start_config,wb_mask);
    config_task_->set_gain(10);
    config_task_->weight(0.001);

    /* -- com task */
    com_task_ = new ComTask(robot_, com_target_);
    com_task_->set_gain(50);
    com_task_->weight(1);
    com_x_shift_ = com_position_(0) - right_ankle_transformation(0,3);

    /* -- parallel task */
    parallel_task_ = new ParallelTask(robot_,robot_->chest(),vector3d(0,0,1),vector3d(0,0,1));
    parallel_task_->set_gain(50);
    parallel_task_->weight(1);

    /* -- angular momentum task */
    angular_momentum_task_ = new AngularMomentumTask(robot_);
    angular_momentum_task_->set_gain(50);
    angular_momentum_task_->weight(1);

    /* -- min torque task on the arms */
    std::vector<double> weights(robot_->getActuatedJoints().size());
    for(unsigned int i = 0; i < weights.size(); ++i) weights[i] = 0;
    std::vector<CjrlJoint*> left_arm_joints = 
      robot_->jointsBetween(*(robot_->rootJoint()), *(robot_->leftWrist()));
    for(std::vector<CjrlJoint*>::const_iterator joint_it = left_arm_joints.begin();
	joint_it != left_arm_joints.end(); ++joint_it) {
      if((*joint_it)->numberDof() == 0)
	continue;
      if (*joint_it != robot_->rootJoint()) {  //ignore free flyer
	weights[(*joint_it)->rankInConfiguration() -6] = 1;
      }
    }
    std::vector<CjrlJoint*> right_arm_joints = 
      robot_->jointsBetween(*(robot_->rootJoint()), *(robot_->rightWrist()));
    for(std::vector<CjrlJoint*>::const_iterator joint_it = right_arm_joints.begin();
	joint_it != right_arm_joints.end(); ++joint_it) {
      if((*joint_it)->numberDof() == 0)
	continue;
      if (*joint_it != robot_->rootJoint()) {  //ignore free flyer
	weights[(*joint_it)->rankInConfiguration() -6] = 1;
      }
    }
    min_torque_task_ = new MinTorqueTask(robot_);
    min_torque_task_->set_joint_torque_weights(weights);
    min_torque_task_->weight(1);

    /*
      Create contact point vectors
    */
    // for nao: <origin rpy="0 0 0" xyz="0.02 0 0.0075"/>
    //          <box size="0.16 0.06 0.015"/>
    /* -- left foot */
    ContactConstraint * contact_l_1 = new ContactConstraint(robot_,robot_->leftAnkle(),vector3d(0.10,0.03,-0.0075),vector3d(0,0,1),50);
    ContactConstraint * contact_l_2 = new ContactConstraint(robot_,robot_->leftAnkle(),vector3d(0.10,-0.03,-0.0075),vector3d(0,0,1),50);
    ContactConstraint * contact_l_3 = new ContactConstraint(robot_,robot_->leftAnkle(),vector3d(-0.06,0.03,-0.0075),vector3d(0,0,1),50);
    ContactConstraint * contact_l_4 = new ContactConstraint(robot_,robot_->leftAnkle(),vector3d(-0.06,-0.03,-0.0075),vector3d(0,0,1),50);
    left_foot_contacts_.push_back(contact_l_1);
    left_foot_contacts_.push_back(contact_l_2);
    left_foot_contacts_.push_back(contact_l_3);
    left_foot_contacts_.push_back(contact_l_4);

    /* -- right foot */
    ContactConstraint * contact_r_1 = new ContactConstraint(robot_,robot_->rightAnkle(),vector3d(0.10,0.03,-0.0075),vector3d(0,0,1),50);
    ContactConstraint * contact_r_2 = new ContactConstraint(robot_,robot_->rightAnkle(),vector3d(0.10,-0.03,-0.0075),vector3d(0,0,1),50);
    ContactConstraint * contact_r_3 = new ContactConstraint(robot_,robot_->rightAnkle(),vector3d(-0.06,0.03,-0.0075),vector3d(0,0,1),50);
    ContactConstraint * contact_r_4 = new ContactConstraint(robot_,robot_->rightAnkle(),vector3d(-0.06,-0.03,-0.0075),vector3d(0,0,1),50);
    right_foot_contacts_.push_back(contact_r_1);
    right_foot_contacts_.push_back(contact_r_2);
    right_foot_contacts_.push_back(contact_r_3);
    right_foot_contacts_.push_back(contact_r_4);

    /* Set robot dependant variables */
    ankle_height_ = right_ankle_transformation(2,3);
    y_left_ = left_ankle_transformation(1,3);
    y_right_ = right_ankle_transformation(1,3);

    /* Allocate qp problem */
    problem_ = new ProblemOases(robot_);

    /* Add foot tasks */
    problem_->add_task(left_foot_task_);
    problem_->add_task(right_foot_task_);
    problem_->add_task(config_task_);
    problem_->add_task(com_task_);
    problem_->add_task(parallel_task_);
    problem_->add_task(angular_momentum_task_);
    problem_->add_task(min_torque_task_);

   ROS_DEBUG_STREAM("Ankle height: " 
		    << robot_->leftAnkle()->currentTransformation()(2,3));

    /* Check and add contacts */
    update_contacts();

    if(problem_->count_activated_contacts() == 0)
      throw(std::runtime_error("Starting locomotion controller in a flying configuration"));

    current_state_ = DOUBLE_SUPPORT_R;
    time_ = 0;
    last_state_change_ = 0;
    problem_->resize_data();
  }

  void
  LocomotionController::update_contacts()
  {
    std::vector<ContactConstraint*>::iterator contact_it;
    /*
      Check left foot
    */
    tmp_transformation_ = robot_->leftAnkle()->currentTransformation();
    ChppGikTools::splitM4(tmp_transformation_,tmp_rotation_,tmp_translation_);
    for(contact_it = left_foot_contacts_.begin(); contact_it != left_foot_contacts_.end(); ++contact_it) {
      tmp_point_ = tmp_rotation_ * (*contact_it)->contact_point_;
      tmp_point_ += tmp_translation_;
      if (tmp_point_(2) < contact_precision_)
	problem_->add_contact(*contact_it);
      else
	problem_->remove_contact(*contact_it);
    }
    /*
      Check right foot
    */
    tmp_transformation_ = robot_->rightAnkle()->currentTransformation();
    ChppGikTools::splitM4(tmp_transformation_,tmp_rotation_,tmp_translation_);
    for(contact_it = right_foot_contacts_.begin(); contact_it != right_foot_contacts_.end(); ++contact_it) {
      tmp_point_ = tmp_rotation_ * (*contact_it)->contact_point_;
      tmp_point_ += tmp_translation_;
      if (tmp_point_(2) < contact_precision_)
	problem_->add_contact(*contact_it);
      else
	problem_->remove_contact(*contact_it);
    }
    ROS_DEBUG_STREAM("Number of contact points: " << problem_->count_activated_contacts() );
  }

  void
  LocomotionController::compute_one_step()
  {
    /*
      Finite-state machine:
      DOUBLE_SUPPORT_L -> RIGHT_SUPPORT -> DOUBLE_SUPPORT_R -> LEFT_SUPPORT -> ...
    */
    switch(current_state_) {
    case DOUBLE_SUPPORT_L:
      if (time_ - last_state_change_ > double_support_duration_) {
	current_state_ = RIGHT_SUPPORT;
	last_state_change_ = time_;
	left_foot_task_->weight(1);
	x_start_ = robot_->leftAnkle()->currentTransformation()(0,3);
	x_target_ = robot_->rightAnkle()->currentTransformation()(0,3) + step_length_;
      }
      break;
    case RIGHT_SUPPORT:
      if (time_ - last_state_change_ > foot_flight_duration_) {
	current_state_ = DOUBLE_SUPPORT_R;
 	last_state_change_ = time_;
	left_foot_task_->weight(10);
      }
      break;
    case DOUBLE_SUPPORT_R:
      if (time_ - last_state_change_ > double_support_duration_) {
	current_state_ = LEFT_SUPPORT;
	last_state_change_ = time_;
	right_foot_task_->weight(1);
	x_start_ = robot_->rightAnkle()->currentTransformation()(0,3);
	x_target_ = robot_->leftAnkle()->currentTransformation()(0,3) + step_length_;
      }
      break;
    case LEFT_SUPPORT:
      if (time_ - last_state_change_ > foot_flight_duration_) {
	current_state_ = DOUBLE_SUPPORT_L;
	last_state_change_ = time_;
	right_foot_task_->weight(10);
      }
      break;
    default:
      break;
    }
    fill_joint_state_message();
    compute_robot_dynamics();
    update_contacts();
    update_foot_tasks();
    update_com_task();
    problem_->compute_one_step(time_step_);
    std::vector<double> solution = problem_->solution();
    std::vector<double> acceleration(solution.begin(),solution.begin()
				     + robot_->numberDof());
    update_robot(acceleration,time_step_);
    time_ += time_step_;
    ROS_DEBUG_STREAM("Simulated time: " << time_ );
  }

  void
  LocomotionController::update_com_task()
  {
    com_target_(0) = com_x_shift_
      + 0.5 * robot_->rightAnkle()->currentTransformation()(0,3)
      + 0.5 * robot_->leftAnkle()->currentTransformation()(0,3);
    com_target_(1) = 
      com_amplitude_ * sin ((M_PI/(foot_flight_duration_ + double_support_duration_)) * time_  + com_phase_);
    //com_task_->targetXY(com_target_(0),com_target_(1));
    com_task_->target(com_target_);
  }

  void
  LocomotionController::update_foot_tasks()
  {
    double x = cubic_interpolation(x_start_,x_target_,
				   foot_flight_duration_,time_ - last_state_change_);
    double z = (time_ - last_state_change_ <= 0.5 * foot_flight_duration_)?
      cubic_interpolation(ankle_height_, ankle_height_ + step_height_,
			  0.5*foot_flight_duration_, time_ - last_state_change_):
      cubic_interpolation(ankle_height_ + step_height_, ankle_height_,
			  0.5*foot_flight_duration_, time_ - last_state_change_ - 0.5*foot_flight_duration_);
    double weight = 1 + (foot_flight_duration_ - time_ + last_state_change_);

    switch(current_state_) {
    case RIGHT_SUPPORT:
      left_foot_task_->target(vector3d(x,y_left_,z));
      left_foot_task_->weight(weight);
      break;
    case LEFT_SUPPORT:
      right_foot_task_->target(vector3d(x,y_right_,z));
      right_foot_task_->weight(weight);
      break;
    default: break;
    }
  }


  void
  LocomotionController::free_contacts()
  {
    std::vector<ContactConstraint*>::iterator it;
    for(it = left_foot_contacts_.begin(); it != left_foot_contacts_.end(); ++it)
      delete (*it);

    for(it = right_foot_contacts_.begin(); it != right_foot_contacts_.end(); ++it)
      delete (*it);
  }

  void
  LocomotionController::trunk_angle(double i_angle)
  {
    parallel_task_->target(vector3d(sin(i_angle),0,cos(i_angle)));
  }

  void
  LocomotionController::double_support_duration(double i_duration)
  {
    double_support_duration_ = i_duration;
  }
  void
  LocomotionController::foot_flight_duration(double i_duration)
  {
    foot_flight_duration_ = i_duration;
  }

  void
  LocomotionController::step_height(double i_step_height)
  {
    step_height_ = i_step_height;
  }

  void
  LocomotionController::step_length(double i_step_length)
  {
    step_length_ = i_step_length;
  }

  void 
  LocomotionController::com_amplitude(double i_com_amplitude)
  {
    com_amplitude_ = i_com_amplitude;
  }

  void 
  LocomotionController::com_phase(double i_com_phase)
  {
    com_phase_ = i_com_phase;
  }
   
  void 
  LocomotionController::am_task_weight(double i_am_task_weight)
  {
    if(!angular_momentum_task_)
      return;
    angular_momentum_task_->weight(i_am_task_weight);
  }

  void
  LocomotionController::time_step(double i_time_step)
  {
    time_step_ = i_time_step;
  }

  void
  LocomotionController::update_locomotion_parameters(const LocomotionParametersConstPtr& i_msg)
  {
    foot_flight_duration(i_msg->flight_duration);
    double_support_duration(i_msg->double_support_duration);
    step_height(i_msg->step_height);
    step_length(i_msg->step_length);
    com_amplitude(i_msg->com_amplitude);
    com_phase(i_msg->com_phase);
    trunk_angle(i_msg->trunk_angle);
  }

} //end of namespace jrl_qp_controller
