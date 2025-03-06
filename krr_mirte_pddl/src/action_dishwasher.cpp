//  Copyright 2025 KAS-Lab
// 
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
// 
    //  http://www.apache.org/licenses/LICENSE-2.0
// 
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
#include "krr_mirte_pddl/action_dishwasher.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace krr_mirte_pddl
{

  UseDishWasher::UseDishWasher(const std::string & node_name,
    const std::chrono::nanoseconds & rate)
  : plansys2::ActionExecutorClient(node_name, rate)
  {
  }

  UseDishWasher::~UseDishWasher()
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  UseDishWasher::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    return plansys2::ActionExecutorClient::on_configure(previous_state);
  }

  void UseDishWasher::do_work(){
    // TODO: ADD your code to make do_work call finish(true, 1.0, "Clean cycle finished") after 10 seconds it started
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<krr_mirte_pddl::UseDishWasher>(
    "use_dishwasher", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
