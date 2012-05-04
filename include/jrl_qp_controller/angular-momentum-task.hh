#ifndef JRL_QP_CONTROLLER_ANGULAR_MOMENTUM_TASK_HH
#define JRL_QP_CONTROLLER_ANGULAR_MOMENTUM_TASK_HH

#include <jrl_qp_controller/task.hh>

namespace jrl_qp_controller {

  /* 
     Task minimizing the angular momentum around the
     center of mass of the robot.
     Not implemented yet.
  */
  class AngularMomentumTask: public Task {
  public:
    AngularMomentumTask(CjrlDynamicRobot* i_robot);
    ~AngularMomentumTask();

    virtual void compute_objective(double time_step);

    void set_gain(double i_gain);

    //Note: this one should probably not be here.
    static void cross_product_matrix(const vector3d &i_vec, matrixNxP &o_mat);
  private:
    bool first_call_;
    /*
      The system angular momentum at the center of mass
      is a linear function of the robot velocity.
      Notations: L = K \dot{q}
      We store K and its time derivative d_K.
    */
    matrixNxP K_;
    matrixNxP d_K_;

    double gain_;

    /*
      Temporary variables to avoid multiple allocations.
    */
    matrixNxP Ki_;
    matrix3d Ri_;
    matrixNxP Ri_ublas_;
    vector3d ti_;
    matrix3d Ri_t_;
    matrix3d Ii_Ri_t_;
    matrixNxP Ii_Ri_t_ublas_;
    vector3d xi_c_;
    matrixNxP Ji_;
    matrixNxP Ki_l_;
    matrixNxP last_K_;
    vectorN desired_dL_;

  };
} //end of namespace jrl_qp_controller

#endif // JRL_QP_CONTROLLER_ANGULAR_MOMENTUM_TASK_HH
