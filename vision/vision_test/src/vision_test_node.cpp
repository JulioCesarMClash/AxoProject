#include<ros/ros.h>
#include<std_msgs/Int32MultiArray.h>
#include<opencv2/opencv.hpp>
#include<iostream>

using namespace std;
using namespace ros;

int valueBlack;

//Variables 
int maxV_black=54;
int kernelSizeGauss=15;
int deviationGauss=10;

int main(int argc, char  **argv)
{
	cout<<"Initializing node..."<<endl;
        init(argc, argv,"vision_test");
        NodeHandle node("~");
	Publisher pub_vision = node.advertise<std_msgs::Int32MultiArray>("/axo/vision/vision_test",1000);
        Rate loop_rate(10);
	
	cv::VideoCapture vidCap;

	if(!vidCap.open(1)){
		cout<<"No fue posible incializar la camara"<<endl;
		return -1;
	}
	
	vidCap.set(cv::CAP_PROP_FRAME_WIDTH,640);
	vidCap.set(cv::CAP_PROP_FRAME_HEIGHT,480);

	while(ok())
        {
		std_msgs::Int32MultiArray num;
		num.data.clear();
		
		if(!vidCap.grab()){
			std::cout<<"Cant grab frames";
			return -1;
		}
		
		//"Mat" perteneciente al cuadro actual (y conversiones):
		cv::Mat frame;
		
		//Obtención del cuadro, en un instante determinado del video:
		vidCap.retrieve(frame);

		////////////Filtrando el color negro/////////////////////////////
		
		//Máscara binaria para determinar las zonas negras:
		cv::Mat Black_Mask=cv::Mat::zeros(frame.rows,frame.cols,CV_8UC1);
		
		//cv::namedWindow("FiltroN",CV_WINDOW_AUTOSIZE);

		//cv::createTrackbar("Negro","FiltroN",&maxV_black,255,NULL);

		/// Filtro Gaussiano
		cv::Mat gaussBlur;
	
		//cv::namedWindow("gauss",CV_WINDOW_AUTOSIZE);
		
		//cv::createTrackbar("kernerS","gauss",&kernelSizeGauss,255,NULL);
		//cv::createTrackbar("deviation","gauss",&deviationGauss,255,NULL);
		
		if(kernelSizeGauss<1){
			kernelSizeGauss=1;
		}
		if(kernelSizeGauss%2==0){
			kernelSizeGauss++;
		}
		
		cv::GaussianBlur(frame, gaussBlur, cv::Size(kernelSizeGauss,kernelSizeGauss), deviationGauss, deviationGauss ); 
	
		//Conversion del frame preprocesado al espacio HSV:
		cv::Mat frameHSV1;
		cv::cvtColor(gaussBlur,frameHSV1,CV_BGR2HSV);
		
		//Se evalua sólo el canal "Value", de la conversión a hsv:
		for( int i=0; i<Black_Mask.rows; i++)
		{
			for(int j=0; j<Black_Mask.cols; j++)
			{
				uchar vVal = frameHSV1.at<cv::Vec3b>(i,j)[2];
				
				if(vVal<maxV_black)
				{
					Black_Mask.at<uchar>(i,j) = 255; 
				}
			}
		}
		
		// VContours
		std::vector< std::vector< cv::Point2i > > cont; 
		cv::findContours(  Black_Mask , cont, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
		
		double iMaxArea=0;
		int MaxX=0;
		int MaxY=0;
		for (int i = 0; i < cont.size(); ++i){
			double area = cv::contourArea( cont[i] );
			
			if(area<50)
				continue;

			cv::Moments mm = cv::moments( cont[i] );
			double cx = mm.m10 / mm.m00; 
			double cy = mm.m01 / mm.m00; 
			//std::cout << "cx:" << cx  << " , cy:" << cy << std::endl; 
			
			if(area>iMaxArea){
				iMaxArea=area;
				MaxX=round(cx);	
				MaxY=round(cy);	
			}

		}
		
		if(cont.size()>0 && MaxX>0 && MaxY>0){
			cv::circle( frame, cv::Point(MaxX, MaxY), 5, cv::Scalar(255,0,0), -1 );
		}

		if(cont.size()>0){
			if(MaxX<205){
				valueBlack=1;
			}else if(MaxX<437){
				valueBlack=2;
			}else{
				valueBlack=3;
			}
			
		}
		else{
			valueBlack=0;
		}
			

		num.data[0] = valueBlack;
		pub_vision.publish(num);
		spinOnce();
                loop_rate.sleep();
	}

}

