#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
using namespace ros;

float mi,md,angle;

int main(int argc, char  **argv)
{
	cout<<"Initializing motors_publisher node..."<<endl;
	init(argc, argv,"motors_publisher");
	NodeHandle node("~");
/*
	if(!node.hasParam("mi"))
	{
		cout<<"mi missing"<<endl;
		return -1;
	}
	if(!node.getParam("mi",mi))
	{
		cout<<"mi invalid"<<endl;
		return -1;
	}
	if(!node.hasParam("md"))
	{
		cout<<"md missing"<<endl;
		return -1;
	}
	if(!node.getParam("md",md))
	{
		cout<<"md invalid"<<endl;
		return -1;
	}//*/
        

	Publisher pub = node.advertise<std_msgs::Float32MultiArray>("/axo/hardware/motor_speeds",1000);
	ros::Rate loop_rate(10);
        while(ok())
	{
		cout<<"value for md"<<endl;
                cin>>md;
                cout<<"value for mi"<<endl;
		cin>>mi;
		cout<<"value for angle"<<endl;
		cin>>angle;
		if(md==0.0 && mi == 0.0)
			break; 
		std_msgs::Float32MultiArray array;
		array.data.clear();
		for(int i=0; i<3; i++)
		{
			if(i==0) array.data.push_back(mi);
			if(i==1) array.data.push_back(md);
			if(i==2) array.data.push_back(angle);
		}
		pub.publish(array);
		spinOnce();
                loop_rate.sleep();
//		cout<<"mi: "<<mi<<" md:"<<md<<endl;
	}//From while loop
}


