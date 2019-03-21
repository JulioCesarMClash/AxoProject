#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

	//>>>VARIABLES GLOBALES

using namespace std;
using namespace ros;


int i=0; //varaibles para contadores
// 0-5 sonares 6-7 fin de carrera
float val_Sensor[8]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}; 
//_____________________________________________________________________
	//>>>FUNCIONES

void callbackSensorsArduino(const std_msgs::Float32MultiArray::ConstPtr& dataArduino){

	for(i=0;i<8;i++)
	{ //Vaciando los sensores
		val_Sensor[i]=dataArduino->data[i]; 
		std::cout<<"Value"<<i<<":_ "<<val_Sensor[i]<< std::endl;	
	}
	
	if   

}//fin del callbackArduino

int main(int  argc, char** argv){

	cout<< "SENSOR NODE ARDUINO"<<endl;
	cout<< ">_Recolectando datos...."<<endl;

	//_>Inicialiación del nodo de ROS); //Publicar datos enconders

	init(argc,argv,"axo_obstacle");
	NodeHandle n;

	//_>Obtención de los datos proporcionados por el arduino
 	Subscriber subArd = n.subscribe("/axo/hardware/arduino_sensors", 1000, callbackSensorsArduino);

	Rate loop(10);
    	Rate r(1000);
	
	//Ciclo ROS
	while(ok()){

		
		spinOnce();
		loop.sleep();
        	cout<<""<<endl;

	} //Fin del while(ROS)

}//Fin del MAIN 
