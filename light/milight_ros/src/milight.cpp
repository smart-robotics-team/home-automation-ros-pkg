#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "ros/ros.h"

#include "std_msgs/Empty.h"

class MilightManager {
 public:
  /**
   * Ctor
   */
  MilightManager();

  /**
   * Dtor
   */
  ~MilightManager() {
    close(_udpSocket);
  }

 private:
  ros::NodeHandle nh; // Node handler.
  ros::Subscriber _milight_on; // Listener for light on. 
  ros::Subscriber _milight_off; // Listener for light off. 
  int _udpSocket;  // File descriptor to milight controller.
  struct sockaddr_in _destSockAddr; // Socket infos.
  std::string _address;
  int _port;
  int _milight_number;

  static char onCode[5];
  static char offCode[5];

  /**
   * milightOnCallback.
   * @brief Callback for turning milight light on.
   * @param [in] empty trigger for turning light on.
   */
  void milightOnCallback(const std_msgs::Empty::ConstPtr &empty);

  /**
   * milightOffCallback.
   * @brief Callback for turning milight light off.
   * @param [in] empty trigger for turning light off.
   */
  void milightOffCallback(const std_msgs::Empty::ConstPtr &empty);
};  // Class MilightManager

// Definition of static members
char MilightManager::onCode[5] = {0x42, 0x45, 0x47, 0x49, 0x4B};
char MilightManager::offCode[5] = {0x41, 0x46, 0x48, 0x4A, 0x4C};

MilightManager::MilightManager() {
  // Subscribing to topics. 
  // WARN: generic topic given here, they should be remaped in launch file.
  _milight_on = nh.subscribe<std_msgs::Empty>("on", 5, &MilightManager::milightOnCallback, this);
  _milight_off = nh.subscribe<std_msgs::Empty>("off", 5, &MilightManager::milightOffCallback, this);

  // Opening udp socket.
  _udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
  if (_udpSocket < 0) {
    ROS_ERROR("Error opening socket.");
  }

  nh.param("address", _address, std::string("192.168.0.11"));
  nh.param("port", _port, 8899);
  nh.param("milight_number", _milight_number, 0); 

  _destSockAddr.sin_addr.s_addr = inet_addr(_address.c_str());
  _destSockAddr.sin_port = htons(_port);
  _destSockAddr.sin_family = AF_INET;
}   // Ctor

void MilightManager::milightOnCallback(const std_msgs::Empty::ConstPtr &msg) {
  char command[3] = {onCode[_milight_number], 0x00, 0x55};
  ROS_INFO("Turning light %d on.", _milight_number);
  size_t ret = sendto(_udpSocket, 
                      command, 
                      3, 
                      0, 
                      (const struct sockaddr*)&_destSockAddr, 
                      sizeof(_destSockAddr));

  if (ret < 0) {
    ROS_ERROR("Unable to send data to milight controller.");
  }
}   // milightOnCallback

void MilightManager::milightOffCallback(const std_msgs::Empty::ConstPtr &msg) {
   char command[3] = {offCode[_milight_number], 0x00, 0x55};
  ROS_INFO("Turning light %d off.", _milight_number);
  size_t ret = sendto(_udpSocket, 
                      command, 
                      3, 
                      0, 
                      (const struct sockaddr*)&_destSockAddr, 
                      sizeof(_destSockAddr));

  if (ret < 0) {
    ROS_ERROR("Unable to send data to milight controller.");
  }
}   // milightOffCallback

/*
 * Main.
 */
int main(int argc, char **argv) {
  // Initialize ros.
  ros::init(argc, argv, "milight_control");

  // Instanciate MilightManager;
  MilightManager milightManager;

  // Loop for pumping callbacks.
  ros::spin();

  return 0;
}
