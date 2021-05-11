#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

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
* \param req (rt2_assignment1::RandomPosition::Request &):
*   Service request, containing the (x,y) ranges.
* \param res (rt2_assignment1::RandomPosition::Response):
*   Service response, containing (x,y,theta).
*
********************************************/
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/****************************************//**
* Server generating a random position
*
* This server is what serves the
* '/position_server' service.
********************************************/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
