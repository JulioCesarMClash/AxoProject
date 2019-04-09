#include<ros/ros.h>
#include<std_msgs/Int32MultiArray.h>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<array>
#include<map>

using namespace std;
using namespace ros;

//----- Variables de visión ----//
int black_maxV=0;

int kernelSizeGauss=0;
int deviationGauss=0;
	
int red_minH=0;
int red_maxH=0; 
int red_minS=0; 
int red_minV=0; 

int blue_minH=0;
int blue_maxH=0; 
int blue_minS=0; 
int blue_minV=0; 

//------------------------------//


//********* Regiones ***********//
class Region{
	public:
		int inicio=0;
		int fin=0;
		int area=0;
		std::vector<std::array<int,2>> rangos;

		void calcularArea(){
			this->area=0;
			if(this->rangos.size()>0){
				for(int indice=0;indice<this->rangos.size();indice++){
					this->area+=(this->rangos.at(indice)[1]-this->rangos.at(indice)[0]);
				}
			}
		}
};

int colorEnRegion(cv::Mat frame,Region reg,int hMin,int hMax,int sMin,int vMin){
	int encontrado=0;
	uchar hVal;
	uchar sVal;
	uchar vVal;
	for(int i=reg.inicio;i<reg.fin;i++){
		for(int j=reg.rangos.at(i-reg.inicio)[0];j<reg.rangos.at(i-reg.inicio)[1];j++){
			hVal = frame.at<cv::Vec3b>(i,j)[0];
			sVal = frame.at<cv::Vec3b>(i,j)[1];
			vVal = frame.at<cv::Vec3b>(i,j)[2];
			if(hVal>hMin && hVal<hMax && sVal>sMin && vVal>vMin){
				encontrado++;
			}
		}
	}
	return encontrado;
}

bool cargarRegion(const char* nombre,Region *reg){
	std::fstream file=std::fstream();
	file.open(nombre,std::ios::in);
	std::string line;
	reg->rangos.clear();
	int a,b;
	bool inicio=true;
	if(file.is_open()){
		while(std::getline(file,line)){
			std::string str1=line.substr(0,line.find_first_of(' '));
			std::string str2=line.substr(line.find_first_of(' ')+1,line.find_first_of('\n'));
			a=std::stoi(str1);
			b=std::stoi(str2);
			if(inicio==true){
				reg->inicio=a;
				reg->fin=b;
				inicio=false;
			}
			else{
				reg->rangos.push_back({a,b});
			}
		}
	}
	else{
		return false;
	}
	return true;
}

//Colores:
cv::Scalar magenta=cv::Scalar(229,50,214);
cv::Scalar verde1=cv::Scalar(62,115,25);
cv::Scalar verde2=cv::Scalar(124,229,50);
cv::Scalar violeta=cv::Scalar(115,25,108);
cv::Scalar naranja=cv::Scalar(27,128,250);
cv::Scalar amarillo=cv::Scalar(27,195,250);

//Puntos de proyección:
cv::Point pointA;
cv::Point pointB;
cv::Point pointC;
cv::Point pointD;
		
//Puntos de proyección (límites):
cv::Point limA=cv::Size(0,0);
cv::Point limB=cv::Size(0,0);
cv::Point limC=cv::Size(0,0);
cv::Point limD=cv::Size(0,0);

//Proporciones triangulares, para el cálculo de proyecciones:
float prop1;
float prop2;

//Límites superiores e inferiores:
int limFmax;
int limFmin;

//Regiones para evaluar el color azul:
Region regAD;
Region regAF;
Region regAI;

//Centro del frame:
int xCenter=0;

//******************************//

//Diccionario para "cargar" configuración previa:
std::map<std::string,int *> dicVar={
	{"GaussKS",&kernelSizeGauss},
	{"GaussD",&deviationGauss},
	{"BlkMaxV",&black_maxV},
	{"BluMaxH",&blue_minH},
	{"BluMinH",&blue_maxH},
	{"BluMinS",&blue_minS},
	{"BluMinV",&blue_minV},
	{"RedMaxH",&red_minH},
	{"RedMinH",&red_maxH},
	{"RedMinS",&red_minS},
	{"RedMinV",&red_minV},
	{"limFmin",&limFmin},
	{"limFmax",&limFmax},
	{"Ax",&pointA.x},
	{"Ay",&pointA.y},
	{"Bx",&pointB.x},
	{"By",&pointB.y},
	{"Cx",&pointC.x},
	{"Cy",&pointC.y},
	{"Dx",&pointD.x},
	{"Dy",&pointD.y}
};

int main(int argc, char  **argv){
	cout<<"Initializing node..."<<endl;
        init(argc, argv,"vision_test");
        NodeHandle node("~");
	Publisher pub_vision = node.advertise<std_msgs::Int32MultiArray>("/axo/vision/vision_test",1000);
        Rate loop_rate(10);
	
	cv::VideoCapture vidCap;
	vidCap.set(CV_CAP_PROP_SETTINGS,1);
	
	while(!vidCap.open(0)){
		cout<<"Imposible incializar la cámara. Verificar conexión."<<endl;
	}
	
	//Cambio de resolución:
	int width=640;
	int height=480;
	vidCap.set(cv::CAP_PROP_FRAME_WIDTH,width);
	vidCap.set(cv::CAP_PROP_FRAME_HEIGHT,height);

	//Procesamiento de argumentos de ejecución:
	bool calibration=false;
	bool debug=false;
	std::string argumento;
	if(argc>1){
		for(int i=1;i<argc;i++){
			argumento=std::string(argv[i]);
			if(argumento=="-config"){
				calibration=true;
			}
			if(argumento=="-debug"){
				debug=true;
			}
		}
	}
	
	//Regiones de azul:
	regAD=Region();
	regAF=Region();
	regAI=Region();
	
	//Stream para manipular archivos:			
	std::fstream file;

	uchar hVal;
	uchar sVal;
	uchar vVal;
		
	//Secuencia de calibración:
	if(calibration){
		
		//Inicializando puntos de perspectiva:
		pointA=cv::Size(0,0);
        	pointB=cv::Size(0,height);
        	pointC=cv::Size(width,0);
		pointD=cv::Size(width,height);
		
		//Inicializando puntos de límite:
		limB.y=height;
		limD.y=height;
		
		int tecla=-1;
		int select=0;

		//Inicio y fin de regiones:
		int inicio,fin;
	
		//Inicialización de restricciones frontales:
		limFmax=0;
		limFmin=height;

		//Cargando configuración previa, si existe el archivo "config.txt":
		file.open("/home/pi/Desktop/config.txt",std::ios::in);
		std::string line;
		int *ptr_int;
		if(file.is_open()){
			while(std::getline(file,line)){
				std::string key=line.substr(0,line.find_first_of(':'));
				std::string value=line.substr(line.find_first_of(':')+1,line.find_first_of('\n'));
				ptr_int=dicVar[key];
				*ptr_int=std::stoi(value);
			}
		}
		file.close();
	
		//Centro del frame:
		xCenter=round(width/2);
		
		//Mats:
		cv::Mat frame;
		cv::Mat hsv_frame;
		cv::Mat gaussBlur;
		cv::Mat drawImage;
		cv::Mat limIMG;

		while(tecla!='q'){
			while( !vidCap.grab())
			{
				std::cout << "ERROR Imposible obtener el video"<<std::endl; 
			}
			
			//Obtención de la imágen:
			vidCap.retrieve(frame);

			//Copia para "dibujo" de plano:
			drawImage=frame.clone();

			//Filtro "Gaussiano":
			if(kernelSizeGauss<1){
				kernelSizeGauss=1;
			}
			if(kernelSizeGauss%2==0){
				kernelSizeGauss++;
			}
			cv::GaussianBlur(frame,gaussBlur,cv::Size(kernelSizeGauss,kernelSizeGauss),deviationGauss,deviationGauss); 
			
			//Conversión a HSV
			cv::cvtColor(gaussBlur,hsv_frame,CV_BGR2HSV);

			//Máscaras:
			cv::Mat black_Mask=cv::Mat::zeros(frame.rows,frame.cols,CV_8UC1);
			cv::Mat blue_Mask=cv::Mat::zeros(frame.rows,frame.cols,CV_8UC1);
			cv::Mat red_Mask=cv::Mat::zeros(frame.rows,frame.cols,CV_8UC1);
		
			//Ventanas:
			cv::namedWindow("Blue",CV_WINDOW_AUTOSIZE);
			cv::namedWindow("Red",CV_WINDOW_AUTOSIZE);
			cv::namedWindow("Black",CV_WINDOW_AUTOSIZE);
			cv::namedWindow("Gauss",CV_WINDOW_AUTOSIZE);
			
			//Trackbars:
			cv::createTrackbar("hMin","Blue",&blue_minH,180,NULL);
			cv::createTrackbar("hMax","Blue",&blue_maxH,180,NULL);
			cv::createTrackbar("sMin","Blue",&blue_minS,255,NULL);
			cv::createTrackbar("vMin","Blue",&blue_minV,255,NULL);

			cv::createTrackbar("hMin","Red",&red_minH,180,NULL);
			cv::createTrackbar("hMax","Red",&red_maxH,180,NULL);
			cv::createTrackbar("sMin","Red",&red_minS,255,NULL);
			cv::createTrackbar("vMin","Red",&red_minV,255,NULL);
			
			cv::createTrackbar("vMax","Black",&black_maxV,255,NULL);
		
			cv::createTrackbar("kernerS","Gauss",&kernelSizeGauss,255,NULL);
			cv::createTrackbar("deviation","Gauss",&deviationGauss,255,NULL);
		
			//Lineas de borde (laterales):
			cv::line(drawImage,pointA,pointB,verde2,2);
			cv::line(drawImage,pointC,pointD,verde2,2);
	
			//Dibujo, y cálculo, de lineas de restricción:
			limA.x=round(pointA.x+((xCenter-pointA.x)/2));
			limA.y=0;

			if(pointB.y==frame.rows){
				limB.x=round(pointB.x+((xCenter-pointB.x)/2));
				limB.y=frame.rows;
			}
			else{
				prop1=(float)pointA.x/pointB.y;
				limB.x=round(xCenter-((xCenter+(prop1*(frame.rows-pointB.y)))/2));
			}
			cv::line(drawImage,limA,limB,violeta,2);

			limC.x=xCenter+((pointC.x-xCenter)/2);
			limC.y=0;
			if(pointD.y==frame.rows){
				limD.x=xCenter+((pointD.x-xCenter)/2);
				limD.y=frame.rows;
			}
			else{
				prop2=(float)(frame.cols-pointC.x)/pointD.y;
				limD.x=round(xCenter+(((frame.cols-xCenter)+(prop2*(frame.rows-pointD.y)))/2));
			}
			cv::line(drawImage,limC,limD,violeta,2);
	
			//Dibujo y movimiento de puntos de borde:
			switch (select){
				case 1:
					//Lineas de borde (frontales):
					cv::line(drawImage,cv::Size(0,limFmax),cv::Size(frame.cols,limFmax),naranja,2);
					cv::line(drawImage,cv::Size(0,limFmin),cv::Size(frame.cols,limFmin),naranja,2);
					cv::circle(drawImage,pointA,5,magenta,-1);
					cv::circle(drawImage,pointB,5,verde1,-1);
					cv::circle(drawImage,pointC,5,verde1,-1);
					cv::circle(drawImage,pointD,5,verde1,-1);
					if(tecla==82 && pointA.x==0 && pointA.y>0){
						pointA.y-=2;	
					}
					if(tecla==84 && pointA.x==0 && pointA.y<frame.rows){
						pointA.y+=2;
					}
					if(tecla==81 && pointA.y==0 && pointA.x>0){
						pointA.x-=2;
					}
					if(tecla==83 && pointA.y==0 && pointA.x<frame.cols){
						pointA.x+=2;
					}
					break;
				case 2:
					//Lineas de borde (frontales):
					cv::line(drawImage,cv::Size(0,limFmax),cv::Size(frame.cols,limFmax),naranja,2);
					cv::line(drawImage,cv::Size(0,limFmin),cv::Size(frame.cols,limFmin),naranja,2);
					cv::circle(drawImage,pointA,5,verde1,-1);
					cv::circle(drawImage,pointB,5,magenta,-1);
					cv::circle(drawImage,pointC,5,verde1,-1);
					cv::circle(drawImage,pointD,5,verde1,-1);
					if(tecla==82 && pointB.x==0 && pointB.y>0){
						pointB.y-=2;	
					}
					if(tecla==84 && pointB.x==0 && pointB.y<frame.rows){
						pointB.y+=2;
					}
					if(tecla==81 && pointB.y==frame.rows && pointB.x>0){
						pointB.x-=2;
					}
					if(tecla==83 && pointB.y==frame.rows && pointB.x<frame.cols){
						pointB.x+=2;
					}
					break;
				case 3:
					//Lineas de borde (frontales):
					cv::line(drawImage,cv::Size(0,limFmax),cv::Size(frame.cols,limFmax),naranja,2);
					cv::line(drawImage,cv::Size(0,limFmin),cv::Size(frame.cols,limFmin),naranja,2);
					cv::circle(drawImage,pointA,5,verde1,-1);
					cv::circle(drawImage,pointB,5,verde1,-1);
					cv::circle(drawImage,pointC,5,magenta,-1);
					cv::circle(drawImage,pointD,5,verde1,-1);
					if(tecla==82 && pointC.x==frame.cols && pointC.y>0){
						pointC.y-=2;	
					}
					if(tecla==84 && pointC.x==frame.cols && pointC.y<frame.rows){
						pointC.y+=2;
					}
					if(tecla==81 && pointC.y==0 && pointC.x>0){
						pointC.x-=2;
					}
					if(tecla==83 && pointC.y==0 && pointC.x<frame.cols){
						pointC.x+=2;
					}
					break;
				case 4:
					//Lineas de borde (frontales):
					cv::line(drawImage,cv::Size(0,limFmax),cv::Size(frame.cols,limFmax),naranja,2);
					cv::line(drawImage,cv::Size(0,limFmin),cv::Size(frame.cols,limFmin),naranja,2);
					cv::circle(drawImage,pointA,5,verde1,-1);
					cv::circle(drawImage,pointB,5,verde1,-1);
					cv::circle(drawImage,pointC,5,verde1,-1);
					cv::circle(drawImage,pointD,5,magenta,-1);
					if(tecla==82 && pointD.x==frame.cols && pointD.y>0){
						pointD.y-=2;	
					}
					if(tecla==84 && pointD.x==frame.cols && pointD.y<frame.rows){
						pointD.y+=2;
					}
					if(tecla==81 && pointD.y==frame.rows && pointD.x>0){
						pointD.x-=2;
					}
					if(tecla==83 && pointD.y==frame.rows && pointD.x<frame.cols){
						pointD.x+=2;
					}
					break;
				case 5:
					//Lineas de borde (frontales):
					cv::line(drawImage,cv::Size(0,limFmax),cv::Size(frame.cols,limFmax),amarillo,2);
					cv::line(drawImage,cv::Size(0,limFmin),cv::Size(frame.cols,limFmin),naranja,2);
					if(tecla==82 && limFmax>0){
						limFmax-=2;
					}
					if(tecla==84 && limFmax<frame.rows){
						limFmax+=2;
					}
					break;
				case 6:
					//Lineas de borde (frontales):
					cv::line(drawImage,cv::Size(0,limFmax),cv::Size(frame.cols,limFmax),naranja,2);
					cv::line(drawImage,cv::Size(0,limFmin),cv::Size(frame.cols,limFmin),amarillo,2);
					if(tecla==82 && limFmin>0){
						limFmin-=2;
					}
					if(tecla==84 && limFmin<frame.rows){
						limFmin+=2;
					}
					break;
				default:
					cv::circle(drawImage,pointA,5,verde1,-1);
					cv::circle(drawImage,pointB,5,verde1,-1);
					cv::circle(drawImage,pointC,5,verde1,-1);
					cv::circle(drawImage,pointD,5,verde1,-1);

					//Lineas de borde (frontales):
					cv::line(drawImage,cv::Size(0,limFmax),cv::Size(frame.cols,limFmax),naranja,2);
					cv::line(drawImage,cv::Size(0,limFmin),cv::Size(frame.cols,limFmin),naranja,2);
					break;
			}
			
			//Selección de punto de borde:
			switch(tecla){
				case 'a':
					select=1;
					break;
				case 'b':
					select=2;
					break;
				case 'c':
					select=3;
					break;
				case 'd':
					select=4;
					break;
				case 'e':
					select=5;
					break;
				case 'f':
					select=6;
					break;
			}

			//Filtro azul:
			for( int i=0; i<frame.rows; i++){
				for(int j=0; j<frame.cols; j++){
					hVal = hsv_frame.at<cv::Vec3b>(i,j)[0];
					sVal = hsv_frame.at<cv::Vec3b>(i,j)[1];
					vVal = hsv_frame.at<cv::Vec3b>(i,j)[2];
					if(hVal>blue_minH && hVal<blue_maxH && sVal>blue_minS && vVal>blue_minV){
						blue_Mask.at<uchar>(i,j) = 255; 
					}
				}
			}
	
			//Filtro rojo:
			for( int i=0; i<frame.rows; i++){
				for(int j=0; j<frame.cols; j++){
					hVal = hsv_frame.at<cv::Vec3b>(i,j)[0];
					sVal = hsv_frame.at<cv::Vec3b>(i,j)[1];
					vVal = hsv_frame.at<cv::Vec3b>(i,j)[2];
					if((hVal<red_maxH || hVal>red_minH) && sVal>red_minS && vVal>red_minV){
						red_Mask.at<uchar>(i,j) = 255; 
					}
				}
			}

			//Filtro negro:
			for( int i=0; i<frame.rows; i++){
				for(int j=0; j<frame.cols; j++){
					vVal = hsv_frame.at<cv::Vec3b>(i,j)[2];
					if(vVal<black_maxV){
						black_Mask.at<uchar>(i,j) = 255; 
					}
				}
			}
			
			//Guardado de regiones y configuración:
			if(tecla=='s'){

				//Región de detección azul(derecha): 
				limIMG=cv::Mat(480,640,CV_8UC3,cv::Scalar(0,0,0));
				cv::line(limIMG,limC,limD,violeta,1);
				cv::line(limIMG,pointC,pointD,verde2,2);

				fin=frame.cols;
				file.open("/home/pi/Desktop/AD.txt",std::ios::out | std::ios::trunc);
				file<<limFmax<<" "<<limFmin<<std::endl;
				for(int i=limFmax;i<limFmin;i++){
					for(int j=0;j<limIMG.cols;j++){
						cv::Vec3b pixel=limIMG.at<cv::Vec3b>(cv::Point(j,i));
						//Linea verde:
						if(pixel[0]==124 && pixel[1]==229 && pixel[2]==50){
							fin=j;
						}
						//Linea violeta:
						if(pixel[0]==115 && pixel[1]==25 && pixel[2]==108){
							inicio=j;
						}
					}
					file<<inicio<<" "<<fin<<std::endl;
				}
				file.close();

				//Región de detección azul(izquierda): 
				limIMG=cv::Mat(480,640,CV_8UC3,cv::Scalar(0,0,0));
				cv::line(limIMG,pointA,pointB,verde2,1);
				cv::line(limIMG,limA,limB,violeta,1);

				inicio=0;	
				file.open("/home/pi/Desktop/AI.txt",std::ios::out | std::ios::trunc);
				file<<limFmax<<" "<<limFmin<<std::endl;
				for(int i=limFmax;i<limFmin;i++){
					for(int j=0;j<limIMG.cols;j++){
						cv::Vec3b pixel=limIMG.at<cv::Vec3b>(cv::Point(j,i));
						//Linea verde:
						if(pixel[0]==124 && pixel[1]==229 && pixel[2]==50){
							inicio=j;
						}
						//Linea violeta:
						if(pixel[0]==115 && pixel[1]==25 && pixel[2]==108){
							fin=j;
						}
					}
					file<<inicio<<" "<<fin<<std::endl;
				}
				file.close();

				//Región de detección azul(frente):
				limIMG=cv::Mat(480,640,CV_8UC3,cv::Scalar(0,0,0));
				cv::line(limIMG,limC,limD,verde2,1);
				cv::line(limIMG,limA,limB,violeta,1);

				file.open("/home/pi/Desktop/AF.txt",std::ios::out | std::ios::trunc);
				file<<limFmax<<" "<<limFmin<<std::endl;
				for(int i=limFmax;i<limFmin;i++){
					for(int j=0;j<limIMG.cols;j++){
						cv::Vec3b pixel=limIMG.at<cv::Vec3b>(cv::Point(j,i));
						//Linea verde:
						if(pixel[0]==124 && pixel[1]==229 && pixel[2]==50){
							fin=j;
						}
						//Linea violeta:
						if(pixel[0]==115 && pixel[1]==25 && pixel[2]==108){
							inicio=j;
						}
					}
					file<<inicio<<" "<<fin<<std::endl;
				}
				file.close();
				
				//Respaldo de variables:	
				file.open("/home/pi/Desktop/config.txt",std::ios::out | std::ios::trunc);
				file<<"GaussKS:"<<kernelSizeGauss<<std::endl;
				file<<"GaussD:"<<deviationGauss<<std::endl;
				file<<"BlkMaxV:"<<black_maxV<<std::endl;
				file<<"BluMaxH:"<<blue_minH<<std::endl;
				file<<"BluMinH:"<<blue_maxH<<std::endl;
				file<<"BluMinS:"<<blue_minS<<std::endl;
				file<<"BluMinV:"<<blue_minV<<std::endl;
				file<<"RedMaxH:"<<red_minH<<std::endl;
				file<<"RedMinH:"<<red_maxH<<std::endl;
				file<<"RedMinS:"<<red_minS<<std::endl;
				file<<"RedMinV:"<<red_minV<<std::endl;
				file<<"limFmin:"<<limFmin<<std::endl;
				file<<"limFmax:"<<limFmax<<std::endl;
				file<<"Ax:"<<pointA.x<<std::endl;
				file<<"Ay:"<<pointA.y<<std::endl;
				file<<"Bx:"<<pointB.x<<std::endl;
				file<<"By:"<<pointB.y<<std::endl;
				file<<"Cx:"<<pointC.x<<std::endl;
				file<<"Cy:"<<pointC.y<<std::endl;
				file<<"Dx:"<<pointD.x<<std::endl;
				file<<"Dy:"<<pointD.y<<std::endl;
				file.close();
			}
			
			//Frame con "máscaras":
			cv::Mat blue_filter;
			cv::Mat red_filter;
			cv::Mat black_filter;
			
			//Copia del frame:
			frame.copyTo(blue_filter,blue_Mask);
			frame.copyTo(red_filter,red_Mask);
			frame.copyTo(black_filter,black_Mask);
			
			//Despliegue de ventanas:
			cv::imshow("Bordes",drawImage);
			cv::imshow("Blue",blue_filter);
			cv::imshow("Red",red_filter);
			cv::imshow("Black",black_filter);
			cv::imshow("Gauss", gaussBlur);
			
			//Actualización de tecla:
			tecla=cv::waitKey(1);
		}

		cv::destroyAllWindows();
	}

	//Lectura de variables:
	file.open("/home/pi/Desktop/config.txt",std::ios::in);
	std::string line;
	int *ptr_int;
	if(file.is_open()){
		while(std::getline(file,line)){
			std::string key=line.substr(0,line.find_first_of(':'));
			std::string value=line.substr(line.find_first_of(':')+1,line.find_first_of('\n'));
			ptr_int=dicVar[key];
			*ptr_int=std::stoi(value);
		}
	}
	else{
		std::cout<<"ERROR. Archivo \"config.txt\" no encontrado."<<std::endl;
		return -1;
	}
	file.close();

	//Lectura de "regiones":
	if(!cargarRegion("/home/pi/Desktop/AI.txt",&regAI)){
		std::cout<<"ERROR. Archivo \"AI.txt\" no encontrado."<<std::endl;
		return -1;
	}
	
	if(!cargarRegion("/home/pi/Desktop/AD.txt",&regAD)){
		std::cout<<"ERROR. Archivo \"AD.txt\" no encontrado."<<std::endl;
		return -1;
	}
	
	if(!cargarRegion("/home/pi/Desktop/AF.txt",&regAF)){
		std::cout<<"ERROR. Archivo \"AF.txt\" no encontrado."<<std::endl;
		return -1;
	}

	//Cálculo de áreas de regiones:
	regAI.calcularArea();
	regAF.calcularArea();
	regAD.calcularArea();
	
	//Varibles de ROS:
	std_msgs::Int32MultiArray num;
	num.data.clear();
	for(int i=0;i<7;i++){
		num.data.push_back(0);
	}
	
	enum indices{LataX,LataY,DepositoX,DepositoY,azulIzq,azulF,azulDer};
	
	while(ros::ok() && cv::waitKey(1)!='q'){
		//Mats:
		cv::Mat frame;
		cv::Mat hsv_frame;
		cv::Mat gaussBlur;
			
		while( !vidCap.grab()){
			std::cout << "ERROR Imposible obtener el video"<<std::endl; 
		}
		
		//Obtención de la imágen:
		vidCap.retrieve(frame);

		//Filtro "Gaussiano":
		cv::GaussianBlur(frame,gaussBlur,cv::Size(kernelSizeGauss,kernelSizeGauss),deviationGauss,deviationGauss); 
		
		//Conversión a HSV
		cv::cvtColor(gaussBlur,hsv_frame,CV_BGR2HSV);

		//Máscaras:
		cv::Mat black_Mask=cv::Mat::zeros(frame.rows,frame.cols,CV_8UC1);
		cv::Mat red_Mask=cv::Mat::zeros(frame.rows,frame.cols,CV_8UC1);
			
		//Filtro rojo:
		for( int i=0; i<frame.rows; i++){
			for(int j=0; j<frame.cols; j++){
				hVal = hsv_frame.at<cv::Vec3b>(i,j)[0];
				sVal = hsv_frame.at<cv::Vec3b>(i,j)[1];
				vVal = hsv_frame.at<cv::Vec3b>(i,j)[2];
				if((hVal<red_maxH || hVal>red_minH) && sVal>red_minS && vVal>red_minV){
					red_Mask.at<uchar>(i,j) = 255; 
				}
			}
		}

		//Filtro negro:
		for( int i=0; i<frame.rows; i++){
			for(int j=0; j<frame.cols; j++){
				vVal = hsv_frame.at<cv::Vec3b>(i,j)[2];
				if(vVal<black_maxV){
					black_Mask.at<uchar>(i,j) = 255; 
				}
			}
		}
		
		int MaxX=0;
		int MaxY=0;
		double cx,cy,area;
		double iMaxArea=0;
		
		//Contornos lata:
		std::vector< std::vector< cv::Point2i > > latas; 
		cv::findContours(black_Mask,latas,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
		
		for (int i = 0; i < latas.size(); ++i){
			area = cv::contourArea( latas[i] );
			
			if(area<50)
				continue;

			cv::Moments mm = cv::moments( latas[i] );
			cx = mm.m10 / mm.m00; 
			cy = mm.m01 / mm.m00; 
			
			if(area>iMaxArea){
				iMaxArea=area;
				MaxX=round(cx);	
				MaxY=round(cy);	
			}
		}
		
		if(latas.size()>0){	
			num.data[LataX]=MaxX;
			num.data[LataY]=MaxY;
		}
		else{
			num.data[LataX]=-1;
			num.data[LataY]=-1;
		}

		//Contornos depósito:	
		std::vector< std::vector< cv::Point2i > > contenedores; 
		cv::findContours(red_Mask,contenedores,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
		
		iMaxArea=0;
		MaxX=0;
		MaxY=0;
		
		for (int i = 0; i < contenedores.size(); ++i){
			area = cv::contourArea( contenedores[i] );
			
			if(area<50)
				continue;

			cv::Moments mm = cv::moments( contenedores[i] );
			cx = mm.m10 / mm.m00; 
			cy = mm.m01 / mm.m00; 
			
			if(area>iMaxArea){
				iMaxArea=area;
				MaxX=round(cx);	
				MaxY=round(cy);	
			}
		}
		
		if(contenedores.size()>0){	
			num.data[DepositoX]=MaxX;
			num.data[DepositoY]=MaxY;
		}
		else{
			num.data[DepositoX]=-1;
			num.data[DepositoY]=-1;
		}

		//Regiones de azul:
		num.data[azulIzq]=((float)colorEnRegion(hsv_frame,regAI,blue_minH,blue_maxH,blue_minS,blue_minV)/regAI.area)*100;
		num.data[azulF]=((float)colorEnRegion(hsv_frame,regAF,blue_minH,blue_maxH,blue_minS,blue_minV)/regAF.area)*100;
		num.data[azulDer]=((float)colorEnRegion(hsv_frame,regAD,blue_minH,blue_maxH,blue_minS,blue_minV)/regAD.area)*100;
		
		//Despligue de ventana(debug):
		if(debug==true){
			if(latas.size()>0)	
				cv::circle(frame,cv::Point(num.data[LataX],num.data[LataY]),5,cv::Scalar(255,0,0),-1);
			if(contenedores.size()>1)
				cv::circle(frame,cv::Point(num.data[DepositoX],num.data[DepositoY]),5,cv::Scalar(0,255,0),-1);
			cv::imshow("Test",frame);
		}
		
		//Publicación de variables:
		pub_vision.publish(num);
		spinOnce();
                loop_rate.sleep();
	}

	//Cierre de la cámara:
	vidCap.release();
}
