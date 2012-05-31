#ifndef NAO_CONFIG_HH
#define NAO_CONFIG_HH

#include <map>
#include <string>

namespace nao_config {
  std::map<std::string,double> base_config_degrees;
  void set_joint_map() {
    //    base_config_degrees["HeadPitch"] = -2.37548;
    base_config_degrees["HeadPitch"] = 20;
    base_config_degrees["HeadYaw"] = 0.;
    base_config_degrees["LAnklePitch"] = -20;
    base_config_degrees["LAnkleRoll"] = -1.5;
    base_config_degrees["LElbowRoll"] =  -21.27220;
    base_config_degrees["LElbowYaw"] = -52.11740;
    base_config_degrees["LHipPitch"] = -20;
    base_config_degrees["LHipRoll"] = 1.5;
    base_config_degrees["LHipYawPitch"] = 0.;
    base_config_degrees["LKneePitch"] = 40;
    base_config_degrees["LShoulderPitch"] = 90;
    base_config_degrees["LShoulderRoll"] = 18;
    base_config_degrees["LWristYaw"] = -43.06930;
    base_config_degrees["RAnklePitch"] = -20;
    base_config_degrees["RAnkleRoll"] = 1.5;
    base_config_degrees["RElbowRoll"] = 21.27220;
    base_config_degrees["RElbowYaw"] = 52.11740;
    base_config_degrees["RHipPitch"] = -20;
    base_config_degrees["RHipRoll"] = -1.5;
    base_config_degrees["RHipYawPitch"] = 0.;
    base_config_degrees["RKneePitch"] = 40;
    base_config_degrees["RShoulderPitch"] = 90;
    base_config_degrees["RShoulderRoll"] = -18;
    base_config_degrees["RWristYaw"] = 43.06930;
  }
  //  double base_free_flyer[6] = {0,0,0.37489,0,0,0};
  double base_free_flyer[6] = {0,0,0.32978,0,0,0};

}
#endif
