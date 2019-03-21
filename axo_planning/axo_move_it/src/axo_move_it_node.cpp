#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>


using namespace std;
using namespace ros;

float mi,md,angle;

void visionValueCallback(const std_msgs::Int32MultiArray &msg)
{
	if (msg.data[0] == 1)
	{	
		mi=-0.65;
		md=0.29;
	}
	else if (msg.data[0] == 2)
        {       
                mi=0.99;
		md=-0.99;
        }
	else if(msg.data[0] == 3)
        {       
                mi=-0.29;
		md=0.65;
        }
	else 
	{
		mi=0.0;
		md=-0.0;
	}

}

int main(int argc, char  **argv)
{
        cout<<"Initializing motors_publisher node..."<<endl;
        init(argc, argv,"axo_move_it");
        NodeHandle node("~");
	Subscriber sub = node.subscribe("/axo/vision/vision_test",1000,visionValueCallback);
	Publisher pub = node.advertise<std_msgs::Float32MultiArray>("/axo/hardware/motor_speeds",1000);
	Rate loop_rate(10);
	angle = 0.09;
        while(ok())
        {
                //if(md==0.0 && mi == 0.0)
                        //break; 
                std_msgs::Float32MultiArray array;
                array.data.clear();
                for(int i=0; i<2; i++)
                {
                        if(i==0) array.data.push_back(mi);
                        if(i==1) array.data.push_back(md);
                        if(i==2) array.data.push_back(angle);
                }
                pub.publish(array);
                spinOnce();
                loop_rate.sleep();
	        }//From while loop
}



            
