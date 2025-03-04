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
#include "krr_mirte_pddl/action_place.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace krr_mirte_pddl
{

  Place::Place(const std::string & node_name,
    const std::chrono::nanoseconds & rate)
  : plansys2::ActionExecutorClient(node_name, rate)
  {
  }

  Place::~Place()
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  Place::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    callback_group_place_client_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    place_cli_ = this->create_client<krr_mirte_skills_msgs::srv::PlaceObject>(
        "place_object",
        rmw_qos_profile_services_default,
        callback_group_place_client_);

    return plansys2::ActionExecutorClient::on_configure(previous_state);
  }

  void Place::do_work(){
    if(place_cli_->service_is_ready()){
        auto request = std::make_shared<krr_mirte_skills_msgs::srv::PlaceObject::Request>();
        auto place_result_ = place_cli_->async_send_request(request);

        // Wait for the result.
        if (place_result_.wait_for(1s) == std::future_status::ready)
        {
            auto result_ = place_result_.get();
            if(!result_->success){
                finish(false, 1.0, "Failed to place object!");
                return;
            }

            if(result_->success){
                finish(true, 1.0, "Placed object!");
                return;
            }
        }
    }
  }

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<krr_mirte_pddl::Place>(
    "place", 500ms);

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
