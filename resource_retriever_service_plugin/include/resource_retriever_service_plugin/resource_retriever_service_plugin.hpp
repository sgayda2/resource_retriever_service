// Copyright 2026 Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RESOURCE_RETRIEVER_SERVICE_PLUGIN__RESOURCE_RETRIEVER_SERVICE_PLUGIN_HPP_
#define RESOURCE_RETRIEVER_SERVICE_PLUGIN__RESOURCE_RETRIEVER_SERVICE_PLUGIN_HPP_

#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <resource_retriever/plugins/retriever_plugin.hpp>
#include <resource_retriever_interfaces/srv/get_resource.hpp>

namespace resource_retriever_service_plugin
{

/// Plugin for resource_retriever that loads resources from a ROS service interface.
class RosServiceResourceRetriever : public resource_retriever::plugins::RetrieverPlugin
{
  static constexpr std::string_view service_uri_prefix = "service://";
  static constexpr std::string_view service_uri_deliminator = ":";
  using GetResource = resource_retriever_interfaces::srv::GetResource;

  RosServiceResourceRetriever() = delete;

public:
  explicit RosServiceResourceRetriever(rclcpp::Node::SharedPtr ros_node);

  ~RosServiceResourceRetriever() override = default;

  std::string name() override;

  bool can_handle(const std::string & url) override;

  resource_retriever::ResourceSharedPtr get_shared(const std::string & url) override;

private:
  rclcpp::Client<GetResource>::SharedPtr getServiceClient(const std::string & service_name);

  // It should be safe to keep a shared pointer to the node here, because this
  // plugin will be destroyed with the resource retriever it is used with,
  // which should be destroyed along before the node abstraction is destroyed.
  // Also, since we're keeping callback groups and clients around, we need to
  // ensure the node stays around too.
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Maps between the server name and the client pointer that we use
  std::unordered_map<
    std::string,
    rclcpp::Client<GetResource>::SharedPtr
  > clients_;
  std::mutex clients_mutex_;

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Logger logger_;

  // Maps [service name][resource path] => pair(etag, resource).
  std::unordered_map<
    std::string,
    std::unordered_map<
      std::string,
      std::pair<std::string, resource_retriever::ResourceSharedPtr>>
  > cached_resources_;
};

}  // namespace resource_retriever_service_plugin

#endif  // RESOURCE_RETRIEVER_SERVICE_PLUGIN__RESOURCE_RETRIEVER_SERVICE_PLUGIN_HPP_
