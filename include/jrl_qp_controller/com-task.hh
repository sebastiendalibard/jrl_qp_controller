#ifndef JRL_QP_CONTROLLER_COM_TASK_HH
#define JRL_QP_CONTROLLER_COM_TASK_HH

#include <jrl_qp_controller/gik-feature-task.hh>
#include <hpp/gik/constraint/com-constraint.hh>

namespace jrl_qp_controller {

  /* 
     Task servoing the center of mass of the robot
     to a certain 3D target.
  */

  class ComTask: public GikFeatureTask {
  public:
    ComTask(CjrlDynamicRobot * i_robot,
	    const vector3d &i_target);
    ~ComTask();

    void target(const vector3d &i_target);

    void targetXY(double x, double y);

  private:
    ChppGikComConstraint* gik_com_constraint_;
    vectorN target_;
    unsigned int dimension_;
  };
} // end of namespace jrl_qp_controller

#endif //JRL_QP_CONTROLLER_COM_TASK_HH
