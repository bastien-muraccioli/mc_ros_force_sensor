#include "RosForceSensor.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

RosForceSensor::~RosForceSensor() = default;

void RosForceSensor::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot(ctl.robots()[0].name());
  if(!ctl.controller().datastore().has("ros_spin"))
  {
     ctl.controller().datastore().make<bool>("ros_spin", false);
  }
  if(!robot.hasDevice<mc_rbdyn::ExternalTorqueSensor>("externalTorqueSensor"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[ExternalForcesEstimator][Init] No \"ExternalTorqueSensor\" with the name \"externalTorqueSensor\" found in "
        "the robot module, please add one to the robot's RobotModule.");
  }
  forceSensor = &robot.device<mc_rbdyn::ExternalTorqueSensor>("externalTorqueSensor");
  // load config
  referenceFrame = config("reference_frame", (std::string) "FT_sensor_force");
  force_sensor_topic_ = config("ros_topic_sensor", (std::string) "/bus0/ft_sensor0/ft_sensor_readings/force");
  freq_ = config("force_freq", (double) 1000);

  // config loaded
  maxTime_ = 1/freq_;

  // Initializing ROS node
  node = mc_rtc::ROSBridge::get_node_handle();
  if(!ctl.controller().datastore().get<bool>("ros_spin"))
  {
    spinThread_ = std::thread(std::bind(&RosForceSensor::rosSpinner, this));
    ctl.controller().datastore().assign("ros_spin", true);
  }
  mc_rtc::log::info("[RosForceSensor][ROS] Subscribing to {}", force_sensor_topic_);

  wrench_sub_.subscribe(node, force_sensor_topic_);
  wrench_sub_.maxTime(maxTime_);

  externalForcesFT = Eigen::Vector6d::Zero();
  ctl.setWrenches({{"EEForceSensor", sva::ForceVecd::Zero()}});

  auto fConf = mc_rtc::gui::ForceConfig();
  // fConf.color = mc_rtc::gui::Color::Blue;
  fConf.force_scale = 0.01;

  ctl.controller().gui()->addElement(
      {"Plugins", "External forces estimator"},
      mc_rtc::gui::Force(
          "EndEffector F/T sensor", fConf, [this]()
          { return sva::ForceVecd(this->externalForcesFT.segment(0, 3), this->externalForcesFT.segment(3, 3)); },
          [this, &controller]()
          {
            auto transform = controller.robot().bodyPosW(referenceFrame);
            return transform;
          }));

  ctl.controller().logger().addLogEntry("ExternalForceEstimator_FTSensor_wrench",
    [&, this]() { return this->externalForcesFT; });
  mc_rtc::log::info("RosForceSensor::init called with configuration:\n{}", config.dump(true, true));
}

void RosForceSensor::reset(mc_control::MCGlobalController & controller)
{
  stop_thread = true;
  spinThread_.join();
  mc_rtc::log::info("RosForceSensor::reset called");
}

void RosForceSensor::before(mc_control::MCGlobalController & controller)
{
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & realRobot = ctl.realRobot(ctl.robots()[0].name());

  auto wrench = wrench_sub_.data().value();
  externalForcesFT = wrench.vector();
  ctl.setWrenches({{"EEForceSensor", wrench}});

  auto sva_EF_FT = realRobot.forceSensor("EEForceSensor").wrenchWithoutGravity(realRobot);
  externalForcesFT = sva_EF_FT.vector();
  // mc_rtc::log::info("RosForceSensor::before");
}

void RosForceSensor::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RosForceSensor::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration RosForceSensor::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void RosForceSensor::rosSpinner(void)
{
  mc_rtc::log::info("[ExternalForcesEstimator][ROS Spinner] thread created for force sensor reading");
  #ifdef MC_RTC_ROS_IS_ROS2
    rclcpp::Rate r(freq_);
    while(rclcpp::ok()and !stop_thread)
    {
      rclcpp::spin_some(node);
      r.sleep();
    }
  #else
    ros::Rate r(freq_);
    while(ros::ok()and !stop_thread)
    {
      ros::spinOnce();
      r.sleep();
    }
  #endif
  mc_rtc::log::info("[ExternalForcesEstimator][ROS Spinner] spinner destroyed");
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("RosForceSensor", mc_plugin::RosForceSensor)