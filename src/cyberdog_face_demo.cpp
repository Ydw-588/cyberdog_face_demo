// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
#include <memory>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "protocol/srv/face_entry.hpp"
#include "protocol/srv/face_rec.hpp"
#include "protocol/msg/face_recognition_result.hpp"
#include "protocol/msg/face_entry_result.hpp"

using FaceEntry = protocol::srv::FaceEntry;
using FaceRec = protocol::srv::FaceRec;
using FaceRecResultMsg = protocol::msg::FaceRecognitionResult;
using FaceEntryResultMsg = protocol::msg::FaceEntryResult;
rclcpp::Node::SharedPtr node{nullptr};

void Face_entry_Result_Callback(FaceEntryResultMsg msg)
{
  RCLCPP_INFO(
    node->get_logger(), "Entry result: '%d', entry username: '%s'.",
    msg.result, msg.username.c_str());
  // 人脸识别
  rclcpp::CallbackGroup::SharedPtr face_callbackgroup =
    node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto face_rec_client_ = node->create_client<FaceRec>(
    "cyberdog_face_recognition_srv",
    rmw_qos_profile_services_default, face_callbackgroup);
  while (!face_rec_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }
  RCLCPP_INFO(node->get_logger(), "Face recognition coming.");
  auto face_rec_request = std::make_shared<FaceRec::Request>();
  face_rec_request->command = FaceRec::Request::COMMAND_RECOGNITION_SINGLE;
  face_rec_request->username = "David";
  face_rec_request->id = "1001";
  auto face_rec_response = face_rec_client_->async_send_request(face_rec_request);
}

void Face_Rec_Result_Callback(FaceRecResultMsg msg)
{
  RCLCPP_INFO(
    node->get_logger(), "Recognition result: '%d', Recognition username: '%s'.",
    msg.result, msg.username.c_str());
}

void Run(rclcpp::Node::SharedPtr node)
{
  rclcpp::spin(node);
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("cyberdog_face_demo");
  RCLCPP_INFO(node->get_logger(), "Create cyberdog_face_demo node.");
  std::thread process(&Run, node);
  auto face_entry_client_ = node->create_client<FaceEntry>("cyberdog_face_entry_srv");
  while (!face_entry_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  }
  // 录入人脸
  RCLCPP_INFO(node->get_logger(), "Face entry coming.");
  auto face_entry_request = std::make_shared<FaceEntry::Request>();
  face_entry_request->command = FaceEntry::Request::ADD_FACE;
  face_entry_request->username = "David";
  auto face_entry_response = face_entry_client_->async_send_request(face_entry_request);
  if (face_entry_response.wait_for(std::chrono::seconds(10)) !=
    std::future_status::ready)
  {
    RCLCPP_ERROR(node->get_logger(), "Request to entry face service failed.");
    return 1;
  }
  auto face_entry_sub_ = node->create_subscription<FaceEntryResultMsg>(
    "face_entry_msg",
    10, std::bind(Face_entry_Result_Callback, std::placeholders::_1));
  auto face_rec_sub_ = node->create_subscription<FaceRecResultMsg>(
    "face_rec_msg",
    10, std::bind(Face_Rec_Result_Callback, std::placeholders::_1));
  if (process.joinable()) {
    process.join();
  }
  return 0;
}
