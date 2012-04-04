#ifndef JRL_QP_CONTROLLER_GIK_FEATURE_TASK_HH
#define JRL_QP_CONTROLLER_GIK_FEATURE_TASK_HH

#include <hpp/gik/constraint/joint-state-constraint.hh>

#include <jrl_qp_controller/feature-task.hh>


namespace jrl_qp_controller {

  /*
    Task servoing a feature to a certain target.
    It delegates task value and jacobian computations
    to a CjrlGikStateConstraint object.
  */
  class GikFeatureTask: public FeatureTask {
  public:
    GikFeatureTask(CjrlDynamicRobot * i_robot,
		   CjrlGikStateConstraint* i_gik_constraint = NULL);
    ~GikFeatureTask();

    void set_gik_constraint(CjrlGikStateConstraint* i_gik_constraint);

    virtual void update_jacobian_and_value();

  protected:
    CjrlGikStateConstraint* gik_constraint_;

  };
} // end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_GIK_FEATURE_TASK_HH
