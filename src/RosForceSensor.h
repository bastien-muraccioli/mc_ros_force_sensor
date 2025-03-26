/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

#include <mc_rbdyn/ExternalTorqueSensor.h>
#include <mc_rbdyn/VirtualTorqueSensor.h>

#ifdef MC_RTC_ROS_IS_ROS2
  #include "utils/ROS2Subscriber.h"
  #pragma message("Using ROS2Subscriber.h")
#else
  #include "utils/ROSSubscriber.h"
  #pragma message("Using ROSSubscriber.h")
#endif

namespace mc_plugin
{

struct RosForceSensor : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController & controller) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~RosForceSensor() override;

private:
  
// Force Sensor ROS
std::shared_ptr<rclcpp::Node> node; 
std::thread spinThread_;              // Thread to spin the ROS node
std::mutex mutex_;                    // Mutex to lock the Force sensor data
ROSWrenchStampedSubscriber wrench_sub_; // ROS Force subscriber
void rosSpinner(void);     // ROS spinner function
bool stop_thread = false;

// yaml config
bool verbose; // Verbose flag
std::string referenceFrame; // Reference frame for the Force sensor
double freq_;                 // Frequency of the Force sensor
std::string force_sensor_topic_; // Force sensor topic
mc_rbdyn::ExternalTorqueSensor * forceSensor;
Eigen::Vector6d externalForcesFT;

double maxTime_;              // MaxForcem time for the Force sensor
};

} // namespace mc_plugin
