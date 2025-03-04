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
#include "krr_mirte_pddl/action_pick.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace krr_mirte_pddl
{

  Pick::Pick(const std::string & node_name,
    const std::chrono::nanoseconds & rate)
  : plansys2::ActionExecutorClient(node_name, rate)
  {
  }

  Pick::~Pick()
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Pick::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    callback_group_pick_client_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    pick_cli_ = this->create_client<krr_mirte_skills_msgs::srv::PickObject>(
        "pick_object",
        rmw_qos_profile_services_default,
        callback_group_pick_client_);

    return plansys2::ActionExecutorClient::on_configure(previous_state);
  }

  void Pick::do_work(){
    if(pick_cli_->service_is_ready()){
        auto request = std::make_shared<krr_mirte_skills_msgs::srv::PickObject::Request>();
        request->object_id = get_arguments()[0];
        auto pick_result_ = pick_cli_->async_send_request(request);

        // Wait for the result.
        if (pick_result_.wait_for(1s) == std::future_status::ready)
        {
            auto result_ = pick_result_.get();
            if(!result_->success){
                finish(false, 1.0, "Failed to pick object!");
                return;
            }

            if(result_->success){
                finish(true, 1.0, "Picked object!");
                return;
            }
        }
    }
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<krr_mirte_pddl::Pick>(
    "pick", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
