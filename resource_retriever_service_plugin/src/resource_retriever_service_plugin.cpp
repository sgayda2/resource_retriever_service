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

#include "resource_retriever_service_plugin/resource_retriever_service_plugin.hpp"

#include <cinttypes>
#include <memory>
#include <stdexcept>
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

RosServiceResourceRetriever::RosServiceResourceRetriever(rclcpp::Node::SharedPtr ros_node)
    : ros_node_(ros_node),
      logger_(ros_node_->get_logger().get_child("ros_service_resource_retriever"))
{
  // Create a client with a custom callback group that will not be included in the main executor.
  callback_group_ = ros_node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);

  // Add the callback group to the executor so we can spin on it later.
  executor_.add_callback_group(callback_group_, ros_node_->get_node_base_interface());
}

std::string RosServiceResourceRetriever::name()
{
  return "resource_retriever_service_plugin::RosServiceResourceRetriever";
}

bool RosServiceResourceRetriever::can_handle(const std::string &url)
{
  // Check for the full format service://service_name:path where service_name.size() > 0
  if (url.find(service_uri_prefix) != 0)
  {
    return false;
  }

  const size_t first_colon = url.find(":", service_uri_prefix.size());
  if (first_colon == std::string::npos)
  {
    RCLCPP_ERROR(this->logger_, "Malformed url: %s", url.data());
    return false;
  }

  if (first_colon == service_uri_prefix.size())
  {
    RCLCPP_ERROR(this->logger_, "Malformed url: %s", url.data());
    return false;
  }

  if (url.find(":", first_colon + 1) != std::string::npos)
  {
    RCLCPP_ERROR(this->logger_, "Malformed url: %s", url.data());
    return false;
  }

  return true;
}

resource_retriever::ResourceSharedPtr
RosServiceResourceRetriever::get_shared(const std::string &url)
{
  // Extract out the service name and the resource path
  const size_t colon_index = url.find(":", service_uri_prefix.size());
  if (colon_index == std::string::npos)
  {
    RCLCPP_ERROR(this->logger_, "Malformed url: %s", url.data());
    return nullptr;
  }

  const std::string service_name(url.begin() + service_uri_prefix.size(),
                                 url.begin() + colon_index);
  if (service_name.empty())
  {
    RCLCPP_ERROR(this->logger_, "Malformed url: %s", url.data());
    return nullptr;
  }

  const std::string resource_path(url.begin() + service_uri_prefix.size(),
                                  url.begin() + colon_index);

  RCLCPP_DEBUG(
      this->logger_, "Getting resource: %s from %s", resource_path.c_str(),
      service_name.c_str());

  // First check for a cache hit.
  std::string etag;
  auto &service_cache = cached_resources_[service_name];
  auto it = service_cache.find(resource_path);
  if (it != service_cache.end())
  {
    etag = it->second.first;
    // If the etag was not set, then the server doesn't do caching, just return what we have.
    if (etag.empty())
    {
      RCLCPP_DEBUG(
          this->logger_, "Resource '%s' cached without etag, returning.",
          resource_path.c_str());
      return it->second.second;
    }
  }

  auto client = getServiceClient(service_name);
  if (!client || !client->service_is_ready())
  {
    return nullptr;
  }

  // Request the resource with an etag, if it is set.
  RCLCPP_DEBUG(
      this->logger_,
      "Requesting resource '%s'%s.",
      resource_path.c_str(),
      etag.empty() ? "" : (" with etag '" + etag + "'").c_str());
  auto req = std::make_shared<GetResource::Request>();
  req->path = resource_path;
  req->etag = etag;
  auto result = client->async_send_request(req);

  using namespace std::chrono_literals;
  auto maximum_wait_time = 3s;

  if (executor_.spin_until_future_complete(result, maximum_wait_time) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->logger_, "Timeout: Not able to call the service %s", service_name.data());
    client->remove_pending_request(result);
    return nullptr;
  }

  auto res = result.get();
  std::shared_ptr<resource_retriever::Resource> memory_resource = nullptr;
  switch (res->status_code)
  {
  case resource_retriever_interfaces::srv::GetResource::Response::OK:
    RCLCPP_DEBUG(
        this->logger_,
        "Received resource '%s' with etag '%s', caching and returning %zu bytes.",
        res->expanded_path.c_str(),
        res->etag.c_str(),
        res->body.size());
    memory_resource =
        std::make_shared<resource_retriever::Resource>(
            resource_path, res->expanded_path,
            res->body);
    service_cache.insert({resource_path, {res->etag, memory_resource}});
    return memory_resource;
  case resource_retriever_interfaces::srv::GetResource::Response::NOT_MODIFIED:
    RCLCPP_DEBUG(
        this->logger_,
        "Resource '%s' with etag '%s' was not modified, returning cached value.",
        res->expanded_path.c_str(),
        res->etag.c_str());
    if (etag != res->etag)
    {
      RCLCPP_WARN(
          this->logger_,
          "Unexpectedly got a different etag values ('%s' vs '%s') for resource '%s' "
          "with a NOT_MODIFIED status_code. This will not stop the resource "
          "from loading, but indicates some issue with the caching logic.",
          res->expanded_path.c_str(),
          etag.c_str(),
          res->etag.c_str());
    }

    // We return the cached value if we had one
    if (it == service_cache.end())
    {
      return nullptr;
    }
    else
    {
      return it->second.second;
    }
    break;
  case resource_retriever_interfaces::srv::GetResource::Response::ERROR:
    RCLCPP_DEBUG(
        this->logger_,
        "Received an unexpected error when getting resource '%s' from '%s': %s",
        resource_path.c_str(),
        service_name.c_str(),
        res->error_reason.c_str());
    return nullptr;
    break;
  default:
    RCLCPP_ERROR(
        this->logger_,
        "Unexpected status_code from resource ROS Service '%s' for resource '%s': %" PRId32,
        service_name.data(),
        resource_path.c_str(),
        res->status_code);
    return nullptr;
    break;
  }
}

rclcpp::Client<resource_retriever_interfaces::srv::GetResource>::SharedPtr
RosServiceResourceRetriever::getServiceClient(const std::string &service_name)
{
  auto &client_ptr = this->clients_[service_name];
  if (!client_ptr)
  {
    client_ptr = ros_node_->create_client<resource_retriever_interfaces::srv::GetResource>(
        service_name,
        rclcpp::ServicesQoS(),
        callback_group_);
    client_ptr->wait_for_service(std::chrono::seconds(1));
  }

  return client_ptr;
}
