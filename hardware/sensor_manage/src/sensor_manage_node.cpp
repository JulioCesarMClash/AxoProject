#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
using namespace ros;

int sensor;
float date;
void arduinoCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	date = msg->data[sensor];
	cout<<"sensor_reading["<<sensor<<"]: "<<date<<endl;
}
int main(int argc, char **argv)
{
	cout<<"Initializing arduino_listener node..."<<endl;
	init(argc, argv, "arduino_listener_sensor");
	NodeHandle node("~");
	Subscriber sub=node.subscribe("/axo/hardware/arduino_sensors", 1000, arduinoCallback);
	Rate loop_rate(10);
	while(ok())
	{
		cout<<sensor<<endl;
		spinOnce();
		loop_rate.sleep();
	}
}
