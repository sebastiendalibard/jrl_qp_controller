#ifndef NAO_CONFIG_HH
#define NAO_CONFIG_HH

#include <map>
#include <string>

namespace nao_config {
  std::map<std::string,double> base_config_degrees;
  void set_joint_map() {
    base_config_degrees["HeadPitch"] = -2.37548;
    base_config_degrees["HeadYaw"] = 0.;
    base_config_degrees["LAnklePitch"] = -10;
    base_config_degrees["LAnkleRoll"] = 0;
    base_config_degrees["LElbowRoll"] =  -21.27220;
    base_config_degrees["LElbowYaw"] = -52.11740;
    base_config_degrees["LHipPitch"] = -10;
    base_config_degrees["LHipRoll"] = 0;
    base_config_degrees["LHipYawPitch"] = 0.;
    base_config_degrees["LKneePitch"] = 20;
    base_config_degrees["LShoulderPitch"] = 90;
    base_config_degrees["LShoulderRoll"] = 8.440010;
    base_config_degrees["LWristYaw"] = -43.06930;
    base_config_degrees["RAnklePitch"] = -10;
    base_config_degrees["RAnkleRoll"] = 0;
    base_config_degrees["RElbowRoll"] = 21.27220;
    base_config_degrees["RElbowYaw"] = 52.11740;
    base_config_degrees["RHipPitch"] = -10;
    base_config_degrees["RHipRoll"] = 0;
    base_config_degrees["RHipYawPitch"] = 0.;
    base_config_degrees["RKneePitch"] = 20;
    base_config_degrees["RShoulderPitch"] = 90;
    base_config_degrees["RShoulderRoll"] = -8.440010;
    base_config_degrees["RWristYaw"] = 60.46710;
  }
  double base_free_flyer[6] = {0,0,0.33727977,0,0};
}
#endif
