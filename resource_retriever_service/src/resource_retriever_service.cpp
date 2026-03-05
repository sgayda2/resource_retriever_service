// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "resource_retriever_service/resource_retriever_service.hpp"

#include <memory>
#include <shared_mutex>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/create_service.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_logging_interface.hpp>
#include <rclcpp/node_interfaces/node_services_interface.hpp>

namespace resource_retriever_service
{

using resource_retriever_interfaces::srv::GetResource;
using ResourceRetrieverServiceNodeType =
    rclcpp::node_interfaces::NodeInterfaces<
        rclcpp::node_interfaces::NodeBaseInterface,
        rclcpp::node_interfaces::NodeLoggingInterface,
        rclcpp::node_interfaces::NodeServicesInterface>;

std::unique_ptr<ResourceRetrieverService> ResourceRetrieverService::Create(
    ResourceRetrieverService::NodeType node, std::string_view service_name) {
  std::unique_ptr<ResourceRetrieverService> service_impl(
      new ResourceRetrieverService(node));
  service_impl->Init(node, service_name);
  return service_impl;
}

ResourceRetrieverService::ResourceRetrieverService(
    ResourceRetrieverService::NodeType node)
    : logger_(node.get<rclcpp::node_interfaces::NodeLoggingInterface>()->get_logger().get_child("ros_resource_retriever_service")) {}

void ResourceRetrieverService::Init(ResourceRetrieverService::NodeType node, std::string_view service_name) {
  service_ = rclcpp::create_service<GetResource>(
      node.get<rclcpp::node_interfaces::NodeBaseInterface>(),
      node.get<rclcpp::node_interfaces::NodeServicesInterface>(),
      std::string(service_name),
      [this](const std::shared_ptr<GetResource::Request> request,
             std::shared_ptr<GetResource::Response> response) {
        this->Get(request, response);
      }, rclcpp::ServicesQoS(), /*group=*/nullptr);
}

void ResourceRetrieverService::ClearResourceData() {
  std::unique_lock<std::shared_mutex> write_lock(data_mutex_);
  data_.clear();
}

void ResourceRetrieverService::SetResourceData(
    std::unordered_map<std::string, std::vector<uint8_t>> resource_data) {
  std::unique_lock<std::shared_mutex> write_lock(data_mutex_);
  data_.clear();

  // Fill in the data map with the appropriate etags
  for (auto [resource_path, resource_blob] : resource_data) {
    data_[resource_path] =
        std::make_pair(std::to_string(next_etag_value_++), std::move(resource_blob));
  }
}

void ResourceRetrieverService::UpdateResourceData(const std::string& resource_path,
                        std::vector<uint8_t> resource_data) {
  std::unique_lock<std::shared_mutex> write_lock(data_mutex_);
  data_[resource_path] =
      std::make_pair(std::to_string(next_etag_value_++), std::move(resource_data));
}

std::optional<std::string> ResourceRetrieverService::GetEtagForResoucePath(const std::string& resource_path) const {
    std::shared_lock<std::shared_mutex> lock(data_mutex_);
  auto itr = data_.find(resource_path);
  if (itr == data_.end()) {
    return std::nullopt;
  }

  return itr->second.first;
}

void ResourceRetrieverService::Get(
    const std::shared_ptr<GetResource::Request> request,
    std::shared_ptr<GetResource::Response> response) {
  if (response == nullptr) {
    RCLCPP_INFO(this->logger_, "we got an invalid response pointer");
    return;
  }

  if (request == nullptr) {
    RCLCPP_INFO(this->logger_, "we got an invalid request pointer");
    response->status_code = GetResource::Response::ERROR;
    response->error_reason = "internal error";
    return;
  }

  RCLCPP_ERROR(this->logger_, "requested resource path: %s", request->path.c_str());
  std::shared_lock<std::shared_mutex> lock(data_mutex_);

  auto itr = data_.find(request->path);
  if (itr == data_.end()) {
    response->status_code = GetResource::Response::ERROR;
    response->error_reason = "resource path not found";
    return;
  }

  if (itr->second.first == request->etag) {
    response->etag = itr->second.first;
    response->status_code = GetResource::Response::NOT_MODIFIED;
    return;
  }

  response->etag = itr->second.first;
  response->body = itr->second.second;
  response->status_code = GetResource::Response::OK;
}

}  // namespace resource_retriever_service
