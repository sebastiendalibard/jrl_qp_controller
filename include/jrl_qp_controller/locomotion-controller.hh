#ifndef JRL_QP_CONTROLLER_LOCOMOTION_CONTROLLER_H
#define JRL_QP_CONTROLLER_LOCOMOTION_CONTROLLER_H

#include <fstream>

#include <abstract-robot-dynamics/humanoid-dynamic-robot.hh>

#include <jrl_qp_controller/transformation-task.hh>
#include <jrl_qp_controller/configuration-task.hh>
#include <jrl_qp_controller/com-task.hh>
#include <jrl_qp_controller/parallel-task.hh>
#include <jrl_qp_controller/contact-constraint.hh>
#include <jrl_qp_controller/problem-oases.hh>
#include <jrl_qp_controller/angular-momentum-task.hh>
#include <jrl_qp_controller/min-torque-task.hh>
#include <jrl_qp_controller/nao-hip-task.hh>


#include <jrl_qp_controller/controller.hh>
#include <jrl_qp_controller/LocomotionParameters.h>

#include <jrl_controller_manager/GetCommand.h>

namespace jrl_qp_controller {
  
  inline 
  double cubic_interpolation(double xi, double xf, double period, double t)
  {
    return (xi + 3*(xf-xi)*pow(t/period,2) - 2*(xf-xi)*pow(t/period,3));
  }
  
  class LocomotionController: public Controller {
  public:
    enum ERobotState {
      DOUBLE_SUPPORT_L,
      DOUBLE_SUPPORT_R,
      RIGHT_SUPPORT,
      LEFT_SUPPORT,
      FLYING};
    
    LocomotionController(CjrlHumanoidDynamicRobot * i_robot);
    ~LocomotionController();

    void initialize(vectorN i_start_config);

    void compute_one_step();

    bool compute_command(jrl_controller_manager::GetCommand::Request &req,
			 jrl_controller_manager::GetCommand::Response &res);

    /*
      Checks which potential contact points are on the ground,
      and updates the qp problem.
      Note: only feet are considered.
     */
    void update_contacts();
    
    void update_foot_tasks();
    void update_com_task();

    void write_joints();
    void write_config();
    void close_stream();
    
    /*
      Locomotion parameter setters
    */
    void double_support_duration(double i_duration);
    void foot_flight_duration(double i_duration);
    void step_height(double i_step_height);
    void step_length(double i_step_length);
    void com_amplitude(double i_com_amplitude);
    void com_phase(double i_com_phase);
    void am_task_weight(double i_am_task_weight);
    void torque_task_weight(double i_am_task_weight);
    void trunk_angle(double i_angle);

    /*
      One call setter
    */
    void update_locomotion_parameters(const LocomotionParametersConstPtr& i_msg);

    void time_step(double i_time_step);

    void free_contacts();

    protected:
    CjrlHumanoidDynamicRobot * robot_;
    ProblemOases *problem_;
    std::vector<ContactConstraint*> left_foot_contacts_;
    std::vector<ContactConstraint*> right_foot_contacts_;

    /*
      Tasks
    */
    NaoHipTask* nao_hip_task_;
    TransformationTask* left_foot_task_;
    TransformationTask* right_foot_task_;
    ConfigurationTask* config_task_;
    ComTask* com_task_;
    ParallelTask* parallel_task_;
    AngularMomentumTask* angular_momentum_task_;
    MinTorqueTask* min_torque_task_;

    /*
      Robot state
    */
    ERobotState current_state_;
    double last_state_change_;

    /*
      Step parameters
    */
    double foot_flight_duration_;
    double double_support_duration_;
    double step_height_;
    double step_length_;
    double com_amplitude_;
    double com_phase_;
    double com_x_shift_;

    /*
      Controller parameters
    */
    double time_step_;
    double time_;
    double contact_precision_;

    /*
      Temporary variables, to avoid multiple allocations
    */
    matrix4d tmp_transformation_;
    matrix3d tmp_rotation_;
    vector3d tmp_translation_;
    vector3d tmp_point_;
    vector3d com_position_;
    vector3d com_target_;
    
    /*
      Current foot trajectories
    */
    double ankle_height_;
    double x_start_,x_target_,x_support_;
    double y_left_, y_right_;

    /*
      Seq-play file
    */
    std::fstream seqplay_file_;
 
  };
} //end of namespace jrl_qp_controller

#endif//JRL_QP_CONTROLLER_LOCOMOTION_CONTROLLER_H
