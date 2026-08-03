#ifndef MICRO_ROS_ADAPTER_H
#define MICRO_ROS_ADAPTER_H

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>
#include <std_srvs/srv/trigger.h>
#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <rosidl_runtime_c/string_functions.h>

#include "Eixo.h"

class MicroRosAdapter {
private:
  Eixo* eixoPan;
  Eixo* eixoTilt;

  rcl_node_t node;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rclc_executor_t executor;

  // Pub/Sub/Services
  rcl_publisher_t joint_state_pub;
  rcl_publisher_t diagnostics_pub;
  rcl_subscription_t cmd_vel_sub;
  rcl_subscription_t cmd_pos_sub;
  rcl_service_t set_zero_srv;

  // Mensagens em memória alocada
  sensor_msgs__msg__JointState joint_state_msg;
  geometry_msgs__msg__Twist cmd_vel_msg;
  sensor_msgs__msg__JointState cmd_pos_msg;
  std_srvs__srv__Trigger_Request set_zero_req;
  std_srvs__srv__Trigger_Response set_zero_res;
  diagnostic_msgs__msg__DiagnosticArray diag_msg;

  // Buffers estáticos para o JointState
  rosidl_runtime_c__String name_buffer[2];
  double position_buffer[2];
  double velocity_buffer[2];

  static MicroRosAdapter* instance;

  static void cmdVelCallback(const void* msgin);
  static void cmdPosCallback(const void* msgin);
  static void setZeroCallback(const void* req, void* res);

  void initJointStateMessage();

public:
  MicroRosAdapter(Eixo* pan, Eixo* tilt);
  bool begin();
  void spinSome();
  void publishTelemetry();
};

#endif