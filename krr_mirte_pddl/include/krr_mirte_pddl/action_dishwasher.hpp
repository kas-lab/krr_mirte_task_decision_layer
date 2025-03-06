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
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "plansys2_executor/ActionExecutorClient.hpp"

namespace krr_mirte_pddl
{

class UseDishWasher : public plansys2::ActionExecutorClient
{
public:
    UseDishWasher(const std::string & node_name,
        const std::chrono::nanoseconds & rate);

    virtual ~UseDishWasher();

private:

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & previous_state);

    void do_work();
};

}  // namespace krr_mirte_pddl
