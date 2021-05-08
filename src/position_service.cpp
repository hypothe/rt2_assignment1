#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_assignment1/srv/random_position.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

class RandPosServer : public rclcpp::Node
{
public:
  RandPosServer(const rclcpp::NodeOptions & options)
  : Node("random_position_server", options)
  {
    service_ = this->create_service<RandomPosition>(
      "/position_server", std::bind(&RandPosServer::myrandom, this, _1, _2, _3)); //placeholders, at runtime they'll be
      // replaced by the received message data
  }

private:

  double randMToN(double M, double N)
  {     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

  void myrandom(
  	const std::shared_ptr<rmw_request_id_t> request_header,
  	const std::shared_ptr<RandomPosition::Request> request,
  	const std::shared_ptr<RandomPosition::Response> response)
  {
    (void)request_header;
    response->x = randMToN(request->x_min, request->x_max);
    response->y = randMToN(request->y_min, request->y_max);
    response->theta = randMToN(-3.14, 3.14);
    }
  rclcpp::Service<RandomPosition>::SharedPtr service_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::RandPosServer)
