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
/****************************************//**
* Server generating a random position
*
* This server is what serves the
* '/position_server' service.
********************************************/
class RandPosServer : public rclcpp::Node
{

public:
  /****************************************//**
  * Constructor, instanciates the server
  *
  * \param options (const rclcpp::NodeOptions &):
  *   Used to run this node as a component
  ********************************************/
  RandPosServer(const rclcpp::NodeOptions & options)
  : Node("random_position_server", options)
  {
    service_ = this->create_service<RandomPosition>(
      "/position_server", std::bind(&RandPosServer::myrandom, this, _1, _2, _3));
  }

private:
  /****************************************//**
  * Generate a bounded random number
  *
  * \param M (double):
  *   Lower bound.
  * \param N (double):
  *   Upper bound.
  *
  * \retval randMToN (double):
  *   random number, linearly distributed,
  *   between M and N.
  *
  ********************************************/
  double randMToN(double M, double N)
  {     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }
  
  /****************************************//**
  * Service callback generating a random
  * (x,y,theta) pose
  *
  * \param request_header (const std::shared_ptr<rmw_request_id_t>):
  *   Service call header (unused).
  * \param request (const std::shared_ptr<RandomPosition::Request>):
  *   Service request, containing the (x,y) ranges.
  * \param response (const std::shared_ptr<RandomPosition::Response>):
  *   Service response, containing (x,y,theta).
  *
  ********************************************/
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
  rclcpp::Service<RandomPosition>::SharedPtr service_;  /*!<  RandomPosition service  */
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::RandPosServer)
