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

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <stdexcept>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace resource_retriever_service {
namespace {

using ::resource_retriever_interfaces::srv::GetResource;

TEST(ResourceRetrieverService, GoodConstruction) {
  auto node = rclcpp::Node::make_shared("test_service");
  EXPECT_NE(ResourceRetrieverService::Create(*node), nullptr);
}

class ResourceRetrieverServiceTest : public ::testing::Test {
 protected:
  void SetUp() override {
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    client_node_ = rclcpp::Node::make_shared("test_client_node");
    server_node_ = rclcpp::Node::make_shared("test_server_node");
    executor_->add_node(client_node_);
    executor_->add_node(server_node_);

    executor_thread_ = std::make_unique<std::thread>(
        [executor = executor_.get()]() { executor->spin(); });

    service_ = ResourceRetrieverService::Create(*client_node_);

    client_ = client_node_->create_client<GetResource>(
        std::string(ResourceRetrieverService::kDefaultServiceName));
  }

  void TearDown() override {
    executor_->cancel();
    executor_thread_->join();

    service_.reset();
    server_node_.reset();
    client_node_.reset();
    executor_thread_.reset();
    executor_.reset();
  }

  GetResource::Response::SharedPtr SendRequest(
      const std::string& resource_path, const std::string& resource_etag) {
    auto request = std::make_shared<GetResource::Request>();
    request->path = resource_path;
    request->etag = resource_etag;

    auto future = client_->async_send_request(request);
    return future.get();
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::unique_ptr<std::thread> executor_thread_;
  std::shared_ptr<rclcpp::Node> client_node_;
  std::shared_ptr<rclcpp::Node> server_node_;
  std::shared_ptr<ResourceRetrieverService> service_;
  rclcpp::Client<GetResource>::SharedPtr client_;
};

TEST_F(ResourceRetrieverServiceTest, ConfirmSetUp) { /* Intentionally empty */ }

TEST_F(ResourceRetrieverServiceTest, GetWithEmpty) {
  auto response = SendRequest("", "");
  ASSERT_NE(response, nullptr);

  EXPECT_THAT(response->status_code, GetResource::Response::ERROR);
  EXPECT_NE(response->error_reason, "");
}

TEST_F(ResourceRetrieverServiceTest, GetWithMissingPath) {
  auto response = SendRequest("something_not_here", "");
  ASSERT_NE(response, nullptr);

  EXPECT_THAT(response->status_code, GetResource::Response::ERROR);
  EXPECT_NE(response->error_reason, "");
}

TEST_F(ResourceRetrieverServiceTest, GetWithValidPath) {
  std::vector<uint8_t> resource_data(2u, 3);
  service_->UpdateResourceData("valid_path", resource_data);

  auto response = SendRequest("valid_path", "");
  ASSERT_NE(response, nullptr);

  EXPECT_THAT(response->status_code, GetResource::Response::OK);
  EXPECT_EQ(response->error_reason, "");
  EXPECT_NE(response->etag, "");
  EXPECT_EQ(response->body, resource_data);
}

TEST_F(ResourceRetrieverServiceTest, GetWithSameEtag) {
  std::vector<uint8_t> resource_data(2u, 3);
  service_->UpdateResourceData("valid_path", resource_data);
  auto etag = service_->GetEtagForResoucePath("valid_path");
  ASSERT_TRUE(etag.has_value());
  ASSERT_FALSE(etag->empty());

  auto response = SendRequest("valid_path", *etag);
  ASSERT_NE(response, nullptr);

  EXPECT_THAT(response->status_code, GetResource::Response::NOT_MODIFIED);
  EXPECT_EQ(response->error_reason, "");
  EXPECT_EQ(response->etag, etag);
  EXPECT_TRUE(response->body.empty());
}

TEST_F(ResourceRetrieverServiceTest, GetWithDifferentEtag) {
  std::vector<uint8_t> resource_data(2u, 3);
  service_->UpdateResourceData("valid_path", resource_data);
  auto etag = service_->GetEtagForResoucePath("valid_path");
  ASSERT_TRUE(etag.has_value());
  ASSERT_FALSE(etag->empty());
  std::string bad_etag = *etag + *etag;

  auto response = SendRequest("valid_path", bad_etag);
  ASSERT_NE(response, nullptr);

  EXPECT_THAT(response->status_code, GetResource::Response::OK);
  EXPECT_EQ(response->error_reason, "");
  EXPECT_EQ(response->etag, etag);
  EXPECT_EQ(response->body, resource_data);
}

TEST_F(ResourceRetrieverServiceTest, GetWithSameEtagAfterUpdate) {
  std::vector<uint8_t> resource_data1(2u, 3);
  std::vector<uint8_t> resource_data2(2u, 7);
  service_->UpdateResourceData("valid_path", resource_data1);

  auto response1 = SendRequest("valid_path", "");
  ASSERT_NE(response1, nullptr);

  EXPECT_THAT(response1->status_code, GetResource::Response::OK);
  EXPECT_EQ(response1->error_reason, "");
  EXPECT_NE(response1->etag, "");
  EXPECT_EQ(response1->body, resource_data1);

  service_->UpdateResourceData("valid_path", resource_data2);
  auto response2 = SendRequest("valid_path", response1->etag);
  ASSERT_NE(response2, nullptr);

  EXPECT_THAT(response2->status_code, GetResource::Response::OK);
  EXPECT_EQ(response2->error_reason, "");
  EXPECT_NE(response2->etag, response1->etag);
  EXPECT_EQ(response2->body, resource_data2);
}

TEST_F(ResourceRetrieverServiceTest, GetAfterClear) {
  std::vector<uint8_t> resource_data(2u, 3);
  service_->UpdateResourceData("valid_path", resource_data);

  auto response1 = SendRequest("valid_path", "");
  ASSERT_NE(response1, nullptr);

  EXPECT_THAT(response1->status_code, GetResource::Response::OK);
  EXPECT_EQ(response1->error_reason, "");
  EXPECT_NE(response1->etag, "");
  EXPECT_EQ(response1->body, resource_data);

  service_->ClearResourceData();
  auto response2 = SendRequest("valid_path", response1->etag);
  ASSERT_NE(response2, nullptr);

  EXPECT_THAT(response2->status_code, GetResource::Response::ERROR);
  EXPECT_NE(response2->error_reason, "");
}

}  // namespace
}  // namespace resource_retriever_service

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
