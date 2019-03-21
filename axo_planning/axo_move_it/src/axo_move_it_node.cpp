#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>



using namespace std;
using namespace ros;

float mi,md,angle;

void IMUValueCallback(const std_msgs::Float32MultiArray &msg)
{
	if (msg.data[2]>1.0 && msg.data[2]<60.0)
	{	
		mi=-0.65;
		md=0.29;
	}
	else if (msg.data[2]>2.0 && msg.data[2]<61.0)
        {       
                mi=0.85;
		md=-0.85;
        }
	else if(msg.data[2]>1.0 && msg.data[2]<60.0)
        {       
                mi=-0.29;
		md=0.65;
        }
	else 
	{
		mi=0.0;
		md=0.0;
	}

}

int main(int argc, char  **argv)
{
        cout<<"Initializing motors_publisher node..."<<endl;
        init(argc, argv,"axo_move_it");
        NodeHandle node("~");
	Subscriber sub = node.subscribe("/axo/hardware/imu_fusion_pose",1,visionValueCallback);
	Publisher pub = node.advertise<std_msgs::Float32MultiArray>("/axo/hardware/motor_speeds",1000);
	Rate loop_rate(10);
	angle = 0.05;
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



            
