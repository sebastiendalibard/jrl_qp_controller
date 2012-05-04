#include <jrl/mal/matrixabstractlayer.hh>

#include <jrl_qp_controller/angular-momentum-task.hh>

#include <hpp/gik/tools.hh>

#include <ros/console.h>

namespace jrl_qp_controller {

  AngularMomentumTask::AngularMomentumTask(CjrlDynamicRobot* i_robot):
    Task(i_robot,Task::ACCELERATION),
    first_call_(true),
    gain_(1)
  {
    unsigned int n = i_robot->numberDof();
    K_.resize(3,n);
    d_K_.resize(3,n);
    last_K_.resize(3,n);
    Ki_.resize(3,n);
    Ri_ublas_.resize(3,3);
    Ii_Ri_t_ublas_.resize(3,3);
    Ji_.resize(6,n);
    Ki_l_.resize(3,6);
    desired_dL_.resize(3);
    MAL_MATRIX_FILL(d_K_,0);
    MAL_MATRIX_FILL(D_,0);
    MAL_VECTOR_FILL(c_,0);
  }

  AngularMomentumTask::~AngularMomentumTask()
  {
  }

  void
  AngularMomentumTask::set_gain(double i_gain)
  {
    gain_ = i_gain;
  }

  void
  AngularMomentumTask::cross_product_matrix(const vector3d &i_vec, matrixNxP &o_mat)
  {
    o_mat(0,0) =  0;        o_mat(0,1) = -i_vec(2); o_mat(0,2) =  i_vec(1);
    o_mat(1,0) =  i_vec(2); o_mat(1,1) =  0;        o_mat(1,2) = -i_vec(0);
    o_mat(2,0) = -i_vec(1); o_mat(2,1) =  i_vec(0); o_mat(2,2) =  0;
  }

  void
  AngularMomentumTask::compute_objective(double time_step)
  {
    using std::vector;

    /*
      Compute K
    */
    MAL_MATRIX_FILL(K_,0);
    vector3d c = robot_->positionCenterOfMass();
    vector<CjrlJoint*> joints = robot_->jointVector();
    for(vector<CjrlJoint*>::iterator joint_it = joints.begin(); 
	joint_it != joints.end(); 
	++joint_it) {
      if (!(*joint_it)->linkedBody()){
	ROS_DEBUG_STREAM("angular momentum task: joint without attached body.");
	continue;
      }
      ChppGikTools::splitM4((*joint_it)->currentTransformation(),Ri_,ti_);
      MAL_MATRIX_FILL(Ki_l_,0);
      
      /* Joint jacobian at the body center of mass */
      robot_->getJacobian(*(robot_->rootJoint()),**joint_it,
			  (*joint_it)->linkedBody()->localCenterOfMass(),Ji_);

      /* linear momentum part */
      ti_ += Ri_ * (*joint_it)->linkedBody()->localCenterOfMass();
      ti_ -= c;
      ti_ *= (*joint_it)->linkedBody()->mass();
      cross_product_matrix(ti_, Ki_l_);

      /*angular momentum part */
      MAL_S3x3_TRANSPOSE_A_in_At(Ri_,Ri_t_);
      MAL_S3x3_C_eq_A_by_B(Ii_Ri_t_,(*joint_it)->linkedBody()->inertiaMatrix(),Ri_t_);
      ChppGikTools::Matrix3toUblas(Ii_Ri_t_,Ii_Ri_t_ublas_);
      ChppGikTools::Matrix3toUblas(Ri_,Ri_ublas_);
      noalias(subrange(Ki_l_,0,3,3,6)) = prod(Ri_ublas_,Ii_Ri_t_ublas_);
    
      noalias(Ki_) = prod(Ki_l_,Ji_);
      K_ += Ki_;
    }
    
    if(!first_call_) {
      noalias(d_K_) = K_ - last_K_;
      d_K_ /= time_step;
    }
    last_K_ = K_;
    first_call_ = false;

    desired_dL_ = -gain_ * prod(K_,robot_->currentVelocity());
    
    /* Fill in the qp */
    noalias(D_) = 2*prod(MAL_RET_TRANSPOSE(K_),K_);
    noalias(c_) = 2*prod(MAL_RET_TRANSPOSE(K_),
			 prod(d_K_,robot_->currentVelocity())
			 - desired_dL_);

    ROS_DEBUG_STREAM("Measured angular momentum: " <<  prod(K_,robot_->currentVelocity()));
    ROS_DEBUG_STREAM("task norm: " << norm_2(prod(K_,robot_->currentVelocity())));
    ROS_DEBUG_STREAM("D: " << D_);
    ROS_DEBUG_STREAM("c: " << c_);
  }

} //end of namespace jrl_qp_controller
