#include <memory>
#include <chrono>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
/*  Services includes  */
#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/random_position.hpp"


using namespace std::chrono_literals;

using Command = rt2_assignment1::srv::Command;
using Position = rt2_assignment1::srv::Position;
using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{
/****************************************//**
* State machine implementation
*
* This component receives the user's requests:
* if these ask for the robot to start moving
* then a random goal pose is retrieved from
* '/position_server' and it's forwarded to
* the 'go_to_point' service; once the goal
* is reached the process is repeated, until
* the user asks for the robot to stop.
*
********************************************/
class StateMachine : public rclcpp::Node
{
public:
  /****************************************//**
  * Constructor, instanciates a server, two
  * clients and initialize the variables.
  *
  * \param options (const rclcpp::NodeOptions &):
  *   Used to run this node as a component
  ********************************************/
  StateMachine(const rclcpp::NodeOptions & options)
  : Node("state_machine", options)
  { 
  
    start = false;
    goal_reached = true;
    /*  SERVICE INIT  */
    service_c = this->create_service<Command>(
      "/user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));
      
    /*  CLIENT INIT */
    client_rp = this->create_client<RandomPosition>("/position_server");
    while (!client_rp->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service POSITION_SERVER to appear...");
    }
    
    client_p = this->create_client<Position>("/go_to_point");
    while (!client_p->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service POSITION to appear...");
    }
    request_rp = std::make_shared<RandomPosition::Request>();
    response_rp = std::make_shared<RandomPosition::Response>();
    request_p = std::make_shared<Position::Request>();
	  
	  // Init the limits of the random position only once
    request_rp->x_max = 5.0;
    request_rp->x_min = -5.0;
    request_rp->y_max = 5.0;
    request_rp->y_min = -5.0;
    
    /*  Periodically check for the goToPoint callback */
    timer_ = this->create_wall_timer(
      500ms, std::bind(&StateMachine::fetch_new_goal, this));
  }


private:
  /****************************************//**
  * Periodic timer callback
  *
  * This function is periodically called to
  * check whether a new goal should be
  * retrieved and forwarded to the go_to_point.
  *
  ********************************************/
  void fetch_new_goal(){
    
    if (!goal_reached) return; // we're still trying to reach the previous goal
    
    if (!start) return; // we reached the goal, but there's no need to fetch a new one
    // If we reached the previous goal and we should keep going then fetch the new one
    call_goToPoint();
  }
  
  /****************************************//**
  * Service callback setting the start/stop
  * robot state
  *
  * \param request_header (const std::shared_ptr<rmw_request_id_t>):
  *   Service call header (unused).
  * \param request (const std::shared_ptr<Command::Request>):
  *   Service request, containing the command (string).
  * \param response (const std::shared_ptr<Command::Response>):
  *   Service response, the value of the 'start' state (bool).
  *
  ********************************************/ 
  void user_interface(
  	const std::shared_ptr<rmw_request_id_t> request_header,
  	const std::shared_ptr<Command::Request> request,
  	const std::shared_ptr<Command::Response> response)
  {
    (void) request_header;
    if (request->command == "start"){
      start = true;
      /*
      Initially this block would call
      call_goToPoint();
      It soon became obvious this had the small problem of blocking
      everything preventing this service to return until call_goToPoint
      ended (which, being that funtion at first potentially recursive,
      was not a great idea either).
      */
    }
    else{start = false;}
    response->ok = start;
    RCLCPP_INFO(this->get_logger(), "Received request %s", request->command.c_str());
  }
  
  /****************************************//**
  * Retrieve and set a new goal
  *
  * The goal pose is retrieved from the
  * '/position_server' service and then passed
  * as the request to the '/go_to_point'
  * service in order to set the robot goal.
  * Once this last service call returns a
  * callback is issued setting the goal as
  * reached.
  * All service call are asynchronous to avoid
  * blocking.
  *
  ********************************************/ 
  void call_goToPoint(){
    
    call_randomPosition();
    
    goal_reached = false; // a new goal is received
    
    request_p->x = response_rp->x;
    request_p->y = response_rp->y;
    request_p->theta = response_rp->theta;
    
    RCLCPP_INFO(this->get_logger(), "Going to the position: x= %f y= %f theta= %f",
            request_p->x, request_p->y, request_p->theta);
    // If we reached the goal set the flag goal_reached to true
    auto point_reached_callback =
              [this](rclcpp::Client<rt2_assignment1::srv::Position>::SharedFuture future){(void)future; goal_reached = true;
               RCLCPP_INFO(this->get_logger(), "Goal reached!");};
    auto future_result = client_p->async_send_request(request_p, point_reached_callback);
  }
  
  //function to call the random position service  
  /****************************************//**
  * Retrieve the goal pose
  *
  * The goal pose is retrieved from the
  * '/position_server' service. Once the async
  * call returns the that value is stored in
  * the 'response_rp' varible, which always
  * maintains a copy of the current goal.
  *
  ********************************************/ 
  void call_randomPosition(){
    // Update the private position var once the response is received
    auto response_rp_received_callback = 
              [this](rclcpp::Client<rt2_assignment1::srv::RandomPosition>::SharedFuture future){response_rp = future.get();};
    auto future_result = client_rp->async_send_request(request_rp,response_rp_received_callback);
  }
  
  rclcpp::Service<Command>::SharedPtr service_c;          /*!<  Command service  */
  rclcpp::Client<RandomPosition>::SharedPtr client_rp;    /*!<  RandomPosition client  */
  rclcpp::Client<Position>::SharedPtr client_p;           /*!<  Position client  */

  std::shared_ptr<RandomPosition::Request> request_rp;    /*!<  RandomPosition service request  */
  std::shared_ptr<RandomPosition::Response> response_rp;  /*!<  RandomPosition service response  */
  std::shared_ptr<Position::Request> request_p;           /*!<  Position service request  */
  
  rclcpp::TimerBase::SharedPtr timer_;                    /*!<  Periodic timer used to monitor the goal state */
  
  bool start;                                             /*!<  Robot state, whether it should keep moving or not  */
  bool goal_reached;                                      /*!<  Goal state, whether it's been reached or not  */
  
};

}   // namespace close

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)



