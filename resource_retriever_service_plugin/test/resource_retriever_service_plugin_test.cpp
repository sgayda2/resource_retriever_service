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

#include "gtest/gtest.h"

#include "resource_retriever_service_plugin/resource_retriever_service_plugin.hpp"

TEST(RosServiceResourceRetriever, GoodConstruction)
{
  auto node = rclcpp::Node::make_shared("test_message_passing");
  RosServiceResourceRetriever retriever(node);
}

TEST(RosServiceResourceRetriever, BadConstruction)
{
  EXPECT_EXIT(RosServiceResourceRetriever retriever(nullptr),
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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
