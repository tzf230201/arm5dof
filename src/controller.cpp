#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/Float64.h"
#include "math.h"
#include "geometry_msgs/Vector3.h"

float theta1,theta2,theta3,theta4;

void chatterCallback(const geometry_msgs::Vector3& input)
{
  float x = input.x;
  float y = input.y;
  float z = -input.z;
  float L1=185,L2=165,L3=150,L4=60;

  theta1 = atan2(y,x);
  float R = sqrt(pow(x,2)+pow(y,2));
  theta3 = acos( (pow(R,2)+pow(z+L1-L4,2)-pow(L2,2)-pow(L3,2)) / (2*L2*L3) );
  theta2 = atan2(z+L1-L4,R)-atan2( L3*sin(theta3), (L2+L3*cos(theta3)) );
  theta4 = M_PI_2-(theta2+theta3);

  ROS_INFO("J1 = %f   J2 = %f   J3 = %f   J4 = %f",theta1,theta2,theta3,theta4);


  //rostopic pub -1 /arm5dof/input geometry_msgs/Vector3  '{x: 100.0,y: 0.0,z: 0.0}'
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "controller");   
  ros::NodeHandle n;   
  ros::Subscriber sub = n.subscribe("/arm5dof/input", 100, chatterCallback);
  ros::Publisher pub_J1 = n.advertise<std_msgs::Float64>("/arm5dof/J1_position_controller/command", 100);
  ros::Publisher pub_J2 = n.advertise<std_msgs::Float64>("/arm5dof/J2_position_controller/command", 100);
  ros::Publisher pub_J3 = n.advertise<std_msgs::Float64>("/arm5dof/J3_position_controller/command", 100);
  ros::Publisher pub_J4 = n.advertise<std_msgs::Float64>("/arm5dof/J4_position_controller/command", 100);
  

  ros::Rate loop_rate(10);   

    int deg = 0;
    const bool up = true;
    const bool down = false;
    bool state = up;
    std_msgs::Float64 rad[5];
  while (ros::ok())
  {

    
    float kp=0.1;
    rad[0].data = deg*M_PI/180.0;
    rad[1].data += (theta1-rad[1].data)*kp;
    rad[2].data += (theta2-rad[2].data)*kp;
    rad[3].data += (theta3-rad[3].data)*kp;
    rad[4].data += (theta4-rad[4].data)*kp;
    pub_J1.publish(rad[1]);
    pub_J2.publish(rad[2]);
    pub_J3.publish(rad[3]);
    pub_J4.publish(rad[4]);
    


    ros::spinOnce();

    loop_rate.sleep();

    if(deg>=35)
        state = down;
    else if(deg<=0)
        state = up;

    if(state == up)
        ++deg;
    else
        --deg;
  }


  ros::spin();

  return 0;
}