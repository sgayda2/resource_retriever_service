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

#include <stdexcept>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include <rclcpp/executors/single_threaded_executor.hpp>
#include "resource_retriever_service_plugin/resource_retriever_service_plugin.hpp"

namespace resource_retriever_service_plugin
{
namespace
{

using ::resource_retriever_interfaces::srv::GetResource;
using ::testing::_;
using ::testing::Invoke;
using ::testing::MockFunction;
using ::testing::Return;

TEST(RosServiceResourceRetriever, GoodConstruction)
{
  auto node = rclcpp::Node::make_shared("test_message_passing");
  RosServiceResourceRetriever retriever(node);
}

TEST(RosServiceResourceRetriever, BadConstruction)
{
  EXPECT_EXIT(
    RosServiceResourceRetriever retriever(nullptr),
    testing::KilledBySignal(SIGSEGV), "");
}

TEST(RosServiceResourceRetriever, CanHandleUri)
{
  auto node = rclcpp::Node::make_shared("test_get_shared");
  RosServiceResourceRetriever retriever(node);

  EXPECT_FALSE(retriever.can_handle("something"));
  EXPECT_FALSE(retriever.can_handle("http://something"));
  EXPECT_FALSE(retriever.can_handle("file://something"));
  EXPECT_FALSE(retriever.can_handle("package://something"));
  EXPECT_FALSE(retriever.can_handle("service"));
  EXPECT_FALSE(retriever.can_handle("service://"));

  EXPECT_FALSE(retriever.can_handle("service://:"));
  EXPECT_FALSE(retriever.can_handle("service://:b"));
  EXPECT_FALSE(retriever.can_handle("service://a:b:c"));

  EXPECT_TRUE(retriever.can_handle("service://a:b"));
  EXPECT_TRUE(retriever.can_handle("service://a:"));
}

TEST(RosServiceResourceRetriever, BadGetSharedCall)
{
  auto node = rclcpp::Node::make_shared("test_get_shared");
  RosServiceResourceRetriever retriever(node);

  EXPECT_EQ(nullptr, retriever.get_shared("something"));
  EXPECT_EQ(nullptr, retriever.get_shared("http://something"));
  EXPECT_EQ(nullptr, retriever.get_shared("file://something"));
  EXPECT_EQ(nullptr, retriever.get_shared("package://something"));
  EXPECT_EQ(nullptr, retriever.get_shared("service"));
  EXPECT_EQ(nullptr, retriever.get_shared("service://"));

  EXPECT_EQ(nullptr, retriever.get_shared("service://:"));
  EXPECT_EQ(nullptr, retriever.get_shared("service://:b"));
  EXPECT_EQ(nullptr, retriever.get_shared("service://a:b:c"));
}

class RosServiceResourceRetrieverTest : public ::testing::Test {
  protected:
    void SetUp() override {
      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

      client_node_ = rclcpp::Node::make_shared("test_client_node");
      server_node_ = rclcpp::Node::make_shared("test_server_node");
      executor_->add_node(client_node_);
      executor_->add_node(server_node_);

      executor_thread_ = std::make_unique<std::thread>(
        [executor = executor_.get()]() { executor->spin(); });

      service_ = server_node_->create_service<GetResource>(
        "test_service",
        [&](const std::shared_ptr<GetResource::Request> request,
            std::shared_ptr<GetResource::Response> response) {
          *response = mock_service_function_.AsStdFunction()(request->path,
                                                            request->etag);
        });

      retriever_ = std::make_unique<RosServiceResourceRetriever>(client_node_);
    }

    void TearDown() override {
      executor_->cancel();
      executor_thread_->join();

      retriever_.reset();
      server_node_.reset();
      client_node_.reset();
      executor_thread_.reset();
      executor_.reset();
    }

    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::unique_ptr<std::thread> executor_thread_;
    std::shared_ptr<rclcpp::Node> client_node_;
    std::shared_ptr<rclcpp::Node> server_node_;
    rclcpp::Service<GetResource>::SharedPtr service_;
    std::unique_ptr<RosServiceResourceRetriever> retriever_;

    MockFunction<GetResource::Response(const std::string&, const std::string&)>
        mock_service_function_;
};

TEST_F(RosServiceResourceRetrieverTest, SimpleE2EGet)
{
  std::vector<uint8_t> resource_data(2u, 3);
  GetResource::Response response;
  response.status_code = GetResource::Response::OK;
  response.body = resource_data;

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response));

  auto resource = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource);
  EXPECT_EQ(resource->data, resource_data);
}

TEST_F(RosServiceResourceRetrieverTest, ErrorStatusReturnsNull)
{
  GetResource::Response response;
  response.status_code = GetResource::Response::ERROR;

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response));

  auto resource = retriever_->get_shared("service://test_service:a");
  EXPECT_EQ(resource, nullptr);
}

TEST_F(RosServiceResourceRetrieverTest, UnknownStatusReturnsNull)
{
  GetResource::Response response;
  response.status_code = 17;

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response));

  auto resource = retriever_->get_shared("service://test_service:a");
  EXPECT_EQ(resource, nullptr);
}

TEST_F(RosServiceResourceRetrieverTest, DuplicateCallsWithoutEtagReturnCachedValue)
{
  std::vector<uint8_t> resource_data(2u, 3);
  GetResource::Response response1;
  response1.status_code = GetResource::Response::OK;
  response1.body = resource_data;
  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response1));

  auto resource1 = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource1);
  EXPECT_EQ(resource1->data, resource_data);

  auto resource2 = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource2);
  EXPECT_EQ(resource2->data, resource_data);

  EXPECT_EQ(resource1, resource2);
}

TEST_F(RosServiceResourceRetrieverTest, NotModifiedStatusReturnsCachedValue)
{
  std::vector<uint8_t> resource_data(2u, 3);
  GetResource::Response response1;
  response1.status_code = GetResource::Response::OK;
  response1.body = resource_data;
  response1.etag = "something";
  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response1));

  auto resource1 = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource1);
  EXPECT_EQ(resource1->data, resource_data);

  GetResource::Response response2;
  response2.status_code = GetResource::Response::NOT_MODIFIED;
  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response2));

  auto resource2 = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource2);
  EXPECT_EQ(resource2->data, resource_data);

  EXPECT_EQ(resource1, resource2);
}

TEST_F(RosServiceResourceRetrieverTest, NotModifiedReturnsEmptyCachedValue)
{
  GetResource::Response response;
  response.status_code = GetResource::Response::NOT_MODIFIED;
  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response));

  auto resource = retriever_->get_shared("service://test_service:a");
  EXPECT_EQ(resource, nullptr);
}

TEST_F(RosServiceResourceRetrieverTest, MultipleServices)
{
  MockFunction<GetResource::Response(const std::string&, const std::string&)>
    mock_service_function2;
  auto service2 = server_node_->create_service<GetResource>(
    "test_service2",
    [&](const std::shared_ptr<GetResource::Request> request,
        std::shared_ptr<GetResource::Response> response) {
      *response = mock_service_function2.AsStdFunction()(request->path,
                                                         request->etag);
    });

  std::vector<uint8_t> resource_data1(2u, 3);
  GetResource::Response response1;
  response1.status_code = GetResource::Response::OK;
  response1.body = resource_data1;

  std::vector<uint8_t> resource_data2(2u, 8);
  GetResource::Response response2;
  response2.status_code = GetResource::Response::OK;
  response2.body = resource_data2;

  EXPECT_CALL(mock_service_function_, Call(_, _)).WillOnce(Return(response1));

  auto resource1 = retriever_->get_shared("service://test_service:a");
  ASSERT_NE(nullptr, resource1);
  EXPECT_EQ(resource1->data, resource_data1);

  EXPECT_CALL(mock_service_function2, Call(_, _)).WillOnce(Return(response2));

  auto resource2 = retriever_->get_shared("service://test_service2:a");
  ASSERT_NE(nullptr, resource2);
  EXPECT_EQ(resource2->data, resource_data2);
}

}  // namespace
}  // namespace resource_retriever_service_plugin

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
