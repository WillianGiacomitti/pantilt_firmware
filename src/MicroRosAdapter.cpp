#include "MicroRosAdapter.h"

// Buffers de recepção para a mensagem de comando de posição
static rosidl_runtime_c__String cmd_name_buffer[2];
static double cmd_pos_buffer[2];

MicroRosAdapter* MicroRosAdapter::instance = nullptr;

MicroRosAdapter::MicroRosAdapter(Eixo* pan, Eixo* tilt) 
  : eixoPan(pan), eixoTilt(tilt) 
{
  instance = this;
}

void MicroRosAdapter::initJointStateMessage() {
  sensor_msgs__msg__JointState__init(&joint_state_msg);
  
  joint_state_msg.name.capacity = 2;
  joint_state_msg.name.size = 2;
  joint_state_msg.name.data = name_buffer;
  
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "pan_joint");
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "tilt_joint");

  joint_state_msg.position.capacity = 2;
  joint_state_msg.position.size = 2;
  joint_state_msg.position.data = position_buffer;

  joint_state_msg.velocity.capacity = 2;
  joint_state_msg.velocity.size = 2;
  joint_state_msg.velocity.data = velocity_buffer;
}

void MicroRosAdapter::cmdVelCallback(const void* msgin) {
  const auto* msg = static_cast<const geometry_msgs__msg__Twist*>(msgin);
  
  // Conversão de rad/s para deg/s (1 rad = 57.2958 deg)
  float pan_deg_s = msg->angular.z * 57.2957795;
  float tilt_deg_s = msg->angular.y * 57.2957795;

  if (abs(pan_deg_s) < 0.01f) {
    instance->eixoPan->parar();
  } else {
    instance->eixoPan->iniciarMovimentoContinuo(pan_deg_s);
  }

  if (abs(tilt_deg_s) < 0.01f) {
    instance->eixoTilt->parar();
  } else {
    instance->eixoTilt->iniciarMovimentoContinuo(tilt_deg_s);
  }
}

void MicroRosAdapter::cmdPosCallback(const void* msgin) {
  const auto* msg = static_cast<const sensor_msgs__msg__JointState*>(msgin);
  
  for (size_t i = 0; i < msg->name.size; i++) {
    String joint_name = msg->name.data[i].data;
    float pos_deg = msg->position.data[i] * 57.2957795; // Converte rad para graus

    if (joint_name == "pan_joint") {
      instance->eixoPan->moverParaGrausAbsoluto(pos_deg);
    } else if (joint_name == "tilt_joint") {
      instance->eixoTilt->moverParaGrausAbsoluto(pos_deg);
    }
  }
}

void MicroRosAdapter::setZeroCallback(const void* req, void* res) {
  auto* response = static_cast<std_srvs__srv__Trigger_Response*>(res);
  
  instance->eixoPan->setZero();
  instance->eixoTilt->setZero();

  response->success = true;
  rosidl_runtime_c__String__assign(&response->message, "Eixos PAN e TILT zerados com sucesso.");
}

bool MicroRosAdapter::begin() {
  // 1. PRIMEIRO: Inicializa o transporte Serial nativo do micro-ROS
  set_microros_transports();

  // 2. SEGUNDO: Aguarda ativamente o Agente responder ao ping via Serial
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    delay(200);
  }

  // 3. TERCEIRO: Agora que o agente respondeu, inicializa o nó e estruturas
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "ptu_driver_node", "", &support) != RCL_RET_OK) return false;

  // Prepara a memória da mensagem de envio (/joint_states)
  initJointStateMessage();

  // ==================== NOVA MODIFICAÇÃO AQUI ====================
  // Prepara a memória da mensagem de recebimento (/ptu/cmd_pos)
  sensor_msgs__msg__JointState__init(&cmd_pos_msg);
  
  cmd_pos_msg.name.capacity = 2;
  cmd_pos_msg.name.size = 0; // O micro-ROS preencherá dinamicamente até o limite 2
  cmd_pos_msg.name.data = cmd_name_buffer;

  rosidl_runtime_c__String__init(&cmd_pos_msg.name.data[0]);
  rosidl_runtime_c__String__init(&cmd_pos_msg.name.data[1]);

  cmd_pos_msg.position.capacity = 2;
  cmd_pos_msg.position.size = 0;
  cmd_pos_msg.position.data = cmd_pos_buffer;
  // ===============================================================

  // Publisher: /joint_states
  rclc_publisher_init_default(
    &joint_state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"
  );

  // Subscriber: /ptu/cmd_vel
  rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/ptu/cmd_vel"
  );

  // Subscriber: /ptu/cmd_pos
  rclc_subscription_init_default(
    &cmd_pos_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/ptu/cmd_pos"
  );

  // Service: /ptu/set_zero
  rclc_service_init_default(
    &set_zero_srv,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "/ptu/set_zero"
  );

  // Configuração do Executor
  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &cmd_pos_sub, &cmd_pos_msg, &cmdPosCallback, ON_NEW_DATA);
  rclc_executor_add_service(&executor, &set_zero_srv, &set_zero_req, &set_zero_res, &setZeroCallback);

  return true;
}

void MicroRosAdapter::publishTelemetry() {
  // Amostragem dos encoders (Transação I2C concentrada na tarefa de telemetria)
  eixoPan->atualizarPosicaoEncoder();
  eixoTilt->atualizarPosicaoEncoder();

  // Conversão de Graus para Radianos (1 deg = 0.0174533 rad)
  position_buffer[0] = eixoPan->getAnguloEixo() * 0.0174532925;
  position_buffer[1] = eixoTilt->getAnguloEixo() * 0.0174532925;

  velocity_buffer[0] = eixoPan->getVelocidadeEixo() * 0.0174532925;
  velocity_buffer[1] = eixoTilt->getVelocidadeEixo() * 0.0174532925;

  int64_t time_ns = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec = time_ns / 1000000000;
  joint_state_msg.header.stamp.nanosec = time_ns % 1000000000;

  rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
}

void MicroRosAdapter::spinSome() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}