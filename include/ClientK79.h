#include <ros/ros.h>

#include <netinet/in.h>

#include <string>
#include <mutex>
#include <memory>
#include <thread>

#define MSG_LEN    28 // bytes

class ClientK79 {

public:
  ClientK79( std::string client_ip, int client_port );
  ~ClientK79();

  bool connect( void );
  void mainLoop( void );

  static uint32_t read_uint32(const char* p);
  
private:

  std::string ip_addr_;
  int port_;

  int sockfd_; // socket file descriptor
  struct sockaddr_in sockaddr_;

  bool is_running_;
  std::unique_ptr<std::thread> thread_;
  std::mutex mutex_;

  ros::NodeHandle node_handle_;
  ros::Publisher pub_;
};

