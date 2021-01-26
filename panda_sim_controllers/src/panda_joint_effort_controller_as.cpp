#include <panda_sim_controllers/panda_joint_effort_controller.h>
#include <pluginlib/class_list_macros.h>

template <class T>
void forward_command_controller::ForwardCommandController<T>::starting(const ros::Time& time)
{
  // Start controller with 0.0 effort
  command_buffer_.writeFromNonRT(0.0);
}


namespace panda_effort_controllers
{

bool JointEffortController::init(panda_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n)
{
  return forward_command_controller::ForwardCommandController<panda_hardware_interface::SharedJointInterface>::init(hw, n);
}
bool JointEffortController::init(panda_hardware_interface::SharedJointInterface* hw, ros::NodeHandle &n, const std::string& ctrl_type)
{
  std::string joint_name;
  if (!n.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }
  joint_ = hw->getHandle(joint_name, ctrl_type);
  // just got rid of ros Subscriber interface sub_command_;
  return true;
}

}  // namespace


PLUGINLIB_EXPORT_CLASS(panda_effort_controllers::JointEffortController,controller_interface::ControllerBase)
