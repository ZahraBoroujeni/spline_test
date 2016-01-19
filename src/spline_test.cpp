/**
 * @file /src/test/cubic_splines.cpp
 *
 * @brief Unit Test for cubic splines.
 *
 * @date 20/05/2009
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
//ros-indigo-ecl-geometry
#include "ros/ros.h"

#include <string>
#include <gtest/gtest.h>
#include <ecl/containers.hpp>
#include <ecl/geometry.hpp>
#include "nav_msgs/Path.h"

/*****************************************************************************
** Using
*****************************************************************************/

//using std::cout; using std::endl;
using std::string;
using ecl::Array;
//using ecl::RightAlign;
//using ecl::Format;
using ecl::CubicPolynomial;
using ecl::CubicSpline;

/*****************************************************************************
** Main program
*****************************************************************************/

int main(int argc, char **argv){

    ros::init(argc,argv,"test_spline");

    Array<double> x_set(4);
    Array<double> y_set(4);
    x_set << 0.688792, 1.15454, 1.67894,0.8;
    y_set << -0.75, -1.2, -1.30,1.4;

    CubicSpline cubic = CubicSpline::Natural(x_set, y_set);
    const CubicPolynomial &p1 = cubic.polynomials()[0];
    const CubicPolynomial &p2 = cubic.polynomials()[1];
    const CubicPolynomial &p3 = cubic.polynomials()[2];
    std::cout << cubic << std::endl;
    std::cout << "Value         : " << cubic(0.8) << std::endl;
    std::cout << "Coffiecinets         : " << p1.coefficients()<< std::endl;
       ros::NodeHandle n;
       //ros::Publisher chatter_pub = n.advertise<std_msgs::Float64 >("/drc_vehicle_xp900/gas_pedal/cmd",1000);

       ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path", 1000);
       std::vector<geometry_msgs::PoseStamped> plan;
       nav_msgs::Path gui_path;

       //ros::Rate loop_rate(1);
      for (int i=0; i<200; i++){
        geometry_msgs::PoseStamped new_goal;
         //tf::Quaternion goal_quat = tf::createQuaternionFromRPY(0,0,1);


         new_goal.pose.position.x = i*(1.67894-0.688792)/200+0.688792;
         new_goal.pose.position.y = cubic(i*(1.67894-0.688792)/200+0.688792);


          new_goal.pose.orientation.x = 0;
          new_goal.pose.orientation.y =0;
          new_goal.pose.orientation.z = 0;
          new_goal.pose.orientation.w =1;

         plan.push_back(new_goal);
      }
      gui_path.poses.resize(plan.size());

       if(!plan.empty()){
             gui_path.header.frame_id = "map";
             gui_path.header.stamp = plan[0].header.stamp;
       }

       for(unsigned int i=0; i < plan.size(); i++){
             gui_path.poses[i] = plan[i];
       }


       while (ros::ok())
       {
           path_pub.publish(gui_path);
           ros::spinOnce();
       }
       return 0;

}

