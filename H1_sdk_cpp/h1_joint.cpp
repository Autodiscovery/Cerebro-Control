#include <array>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>
#include <algorithm>
#include <cstring>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <fcntl.h>  // For non-blocking mode
#include <sys/socket.h>
#include <unistd.h>

#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

static const std::string kTopicArmSDK = "rt/arm_sdk";
constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;
constexpr int kPort = 8080;

enum JointIndex {
  // Right leg
  kRightHipYaw = 8,
  kRightHipRoll = 0,
  kRightHipPitch = 1,
  kRightKnee = 2,
  kRightAnkle = 11,
  // Left leg
  kLeftHipYaw = 7,
  kLeftHipRoll = 3,
  kLeftHipPitch = 4,
  kLeftKnee = 5,
  kLeftAnkle = 10,

  kWaistYaw = 6,

  kNotUsedJoint = 9,

  // Right arm
  kRightShoulderPitch = 12,
  kRightShoulderRoll = 13,
  kRightShoulderYaw = 14,
  kRightElbow = 15,
  // Left arm
  kLeftShoulderPitch = 16,
  kLeftShoulderRoll = 17,
  kLeftShoulderYaw = 18,
  kLeftElbow = 19,
};

// Define joint limits based on provided values
std::array<float, 9> joint_min_limits = {
    -kPi, 0, -kPi, -kPi_2,         // Left arm: pitch, roll, yaw, elbow
    -kPi, -kPi, -kPi, -kPi_2, // Right arm: pitch, roll, yaw, elbow
    -0.1                        // Waist yaw
};
std::array<float, 9> joint_max_limits = {
    0.1, kPi, kPi, kPi_2,     // Left arm: pitch, roll, yaw, elbow
    0.1, 0, kPi, kPi_2,             // Right arm: pitch, roll, yaw, elbow
    0.1                         // Waist yaw
};

// Array defining arm joints, moved to global scope
std::array<JointIndex, 9> arm_joints = {
    JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
    JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
    JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
    JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow, JointIndex::kWaistYaw};

std::array<float, 9> init_pos{};
std::array<float, 9> current_jpos{};
std::array<float, 9> current_tau_ff{};
float weight = 0.f;
float weight_rate = 0.2f;
float kp = 60.f;
float kd = 1.5f;
float dq = 0.f;
float control_dt = 0.02f;
float max_joint_velocity = 0.5f;

unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> arm_sdk_publisher;
unitree_go::msg::dds_::LowCmd_ msg;

// Helper function to clamp joint values to safe limits
std::array<float, 9> clamp_to_limits(const std::array<float, 9>& target_pose) {
  std::array<float, 9> clamped_pose;
  for (int i = 0; i < target_pose.size(); ++i) {
    clamped_pose[i] = std::clamp(target_pose[i], joint_min_limits[i], joint_max_limits[i]);
  }
  return clamped_pose;
}

// Function to execute a pose command
void execute_pose(const std::array<float, 9>& target_pose, const std::array<float, 9>& target_tau_ff) {
  std::array<float, 9> safe_target_pose = clamp_to_limits(target_pose);
  float max_joint_delta = max_joint_velocity * control_dt;
  auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

  std::cout << "Executing pose: ";
  for (const auto& val : safe_target_pose) {  // change to safe_target_pose to clamp limits after uncommenting line 95
    std::cout << val << " ";
  }
  std::cout << std::endl;

  for (int i = 0; i < static_cast<int>(5.f / control_dt); ++i) {
    bool movement_needed = false;
    for (int j = 0; j < init_pos.size(); ++j) {
      float delta = safe_target_pose.at(j) - current_jpos.at(j);  // change to safe_target_pose to clamp limits after uncommenting line 95
      if (std::abs(delta) > 1e-3) {
        movement_needed = true;
        current_jpos.at(j) += std::clamp(delta, -max_joint_delta, max_joint_delta);
      }
    }
    if (movement_needed) {
      for (int j = 0; j < init_pos.size(); ++j) {
        msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos.at(j));
        msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
        msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
        msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
        msg.motor_cmd().at(arm_joints.at(j)).tau(target_tau_ff.at(j));
      }
      arm_sdk_publisher->Write(msg);
      std::this_thread::sleep_for(sleep_time);
    } else {
      break;
    }
  }
}

// Release control safely
void release_control(int client_socket) {
  std::cout << "Releasing control and returning to initial position..." << std::endl;
  std::array<float, 9> zero_tau_ff = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  execute_pose(init_pos, zero_tau_ff);  // Return to initial position

  float delta_weight = weight_rate * control_dt;
  auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));
  int stop_time_steps = static_cast<int>(2.0f / control_dt);

  for (int i = 0; i < stop_time_steps; ++i) {
    weight -= delta_weight;
    weight = std::clamp(weight, 0.f, 1.f);
    msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
    arm_sdk_publisher->Write(msg);
    std::this_thread::sleep_for(sleep_time);
  }
  std::cout << "Control released." << std::endl;
}

void initialize_arms() {
  float delta_weight = weight_rate * control_dt;
  auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));
  current_jpos = init_pos;
  std::array<float, 9> tau_ff = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Initialize tau_ff

  // Gradually increase weight and set the arm joints to initial positions
  for (int i = 0; i < static_cast<int>(5.0f / control_dt); ++i) {
    weight += delta_weight;
    weight = std::clamp(weight, 0.f, 1.f);
    msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight * weight);
    std::this_thread::sleep_for(sleep_time);
  }
  execute_pose(init_pos, tau_ff); // Call execute_pose to initialize arms
  std::cout << "Socket ready, arms initialised." << std::endl;
}


// Set up a socket to receive commands from the Python script
bool setup_socket(int &server_fd, struct sockaddr_in &address) {
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        std::cerr << "Socket creation failed." << std::endl;
        return false;
    }

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
        std::cerr << "Set socket options failed." << std::endl;
        close(server_fd);
        return false;
    }

    // Disable Nagle’s Algorithm (TCP_NODELAY)
    if (setsockopt(server_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt)) == -1) {
        std::cerr << "Failed to disable Nagle’s algorithm." << std::endl;
        close(server_fd);
        return false;
    }

    // Reduce receive buffer size (prevents old data from accumulating)
    int recv_buf_size = 1024;
    if (setsockopt(server_fd, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size)) == -1) {
        std::cerr << "Failed to set receive buffer size." << std::endl;
        close(server_fd);
        return false;
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(8080);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) == -1) {
        std::cerr << "Socket bind failed." << std::endl;
        close(server_fd);
        return false;
    }

    if (listen(server_fd, 3) == -1) {
        std::cerr << "Socket listen failed." << std::endl;
        close(server_fd);
        return false;
    }

    // Set non-blocking mode (optional, prevents read from waiting)
    // fcntl(server_fd, F_SETFL, O_NONBLOCK);

    return true;
}

void receive_pose_commands(int server_fd, struct sockaddr_in &address) {
  int new_socket;
  int addrlen = sizeof(address);
  std::array<float, 9> target_pose;
  std::array<float, 9> target_tau_ff;
  char buffer[1024];

  new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t *)&addrlen);
  if (new_socket == -1) {
    std::cerr << "Socket accept failed." << std::endl;
    return;
  }
  
  std::string data_accumulator;
  
  while (true) {
    ssize_t read_bytes = read(new_socket, buffer, sizeof(buffer));
    if (read_bytes <= 0) {
      std::cerr << "Error reading data from socket or client disconnected." << std::endl;
      release_control(new_socket);
      break;
    }
    
    // Append the newly read data to our accumulator.
    data_accumulator.append(buffer, read_bytes);
    
    // Process each complete message (ending with '\n')
    size_t pos;
    while ((pos = data_accumulator.find('\n')) != std::string::npos) {
      std::string message = data_accumulator.substr(0, pos);
      data_accumulator.erase(0, pos + 1);
      
      if (message.find("exit") != std::string::npos) {
        std::cout << "Received exit command. Shutting down gracefully." << std::endl;
        release_control(new_socket);
        return;
      }
      
      std::istringstream ss(message);
      bool valid = true;
      for (int i = 0; i < 9; ++i) {
        if (!(ss >> target_pose[i])) {
          std::cerr << "Invalid input received for target_pose." << std::endl;
          valid = false;
          continue;
        }
      }
      
      if (!valid) continue;
      
      for (int i = 0; i < 9; ++i) {
        if (!(ss >> target_tau_ff[i])) {
          std::cerr << "Invalid input received for target_tau_ff." << std::endl;
          valid = false;
          continue;
        }
      }
      
      if (!valid) continue;
      
      // Print the received values
      std::cout << "Received target_pose: ";
      for (const auto& val : target_pose) {
        std::cout << val << " ";
      }
      std::cout << std::endl;
      
      std::cout << "Received target_tau_ff: ";
      for (const auto& val : target_tau_ff) {
        std::cout << val << " ";
      }
      std::cout << std::endl;
      
      execute_pose(target_pose, target_tau_ff);  // comment to check socket only and no movement with line 320
    }
  }
}


int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  arm_sdk_publisher.reset(new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(kTopicArmSDK));
  arm_sdk_publisher->InitChannel();

  initialize_arms();  // comment to check socket only and no movement

  int server_fd;
  struct sockaddr_in address;
  if (!setup_socket(server_fd, address)) {
    std::cerr << "Failed to set up socket. Exiting." << std::endl;
    return -1;
  }

  receive_pose_commands(server_fd, address);

  close(server_fd);
  return 0;
}
