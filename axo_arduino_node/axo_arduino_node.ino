//Librerías
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Servo.h>
//Biblioteca sonar
#include <HCSR04.h>

Servo myservo;
//servo angle
int angle = 0;
//FinDeCarrera
int buttonback_1 = 40;
int buttonback_2 = 41; 
//Crea un nodo en ros llamado n
ros::NodeHandle n;
//Message containing the sensor data to be published
std_msgs::Float32MultiArray msg_sensors;
//Array de sensores
float playa_sensor_readings[8] = {0,0,0,0,0,0,0,0};
//----------------------------------------
//Asignación de los pines del ARDUINO-MEGA2560
//Barredora
int barredora_a = 5;
int barredora_b = 48;
//Motores
int m_M1A = 2; //Motor1_A
int M1_pwm = 9; //Motor1_pwm
int m_M1B = 4; //Motor1_B
int m_M2A = 7; //Motor2_A
int M2_pwm = 10; //Motor2_pwm
int m_M2B = 8; //Motor2_B
//--------------------------------------
//Variables globales
int M1A=0; //Valor de dirección del Motor1_A
int M1B=0; //Valor de dirección del Motor1_B
int M2A=0; //Valor de dirección del Motor2_A
int M2B=0; //Valor de dirección del Motor2_B
int pwm_1 = 0; //Valor de dirección del Motor_1
int pwm_2 = 0; //Valor de dirección del Motor_2
// Initialize sensor that uses digital pins trigger and echo. 
UltraSonicDistanceSensor distanceSensor1(22, 23);  
UltraSonicDistanceSensor distanceSensor2(24, 25);
UltraSonicDistanceSensor distanceSensor3(26, 27);
UltraSonicDistanceSensor distanceSensor4(28, 29);
UltraSonicDistanceSensor distanceSensor5(30, 31);  
UltraSonicDistanceSensor distanceSensor6(32, 33);

// >>>>>>>>>>>>>>>>>> FUNCIONES <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void SpeedsCallback(const std_msgs::Float32MultiArray&msg){


angle = int(180 * msg.data[2]); 

if (msg.data[0] > 0 && msg.data[1]>0){
M1A = 0;
M1B = 1;
pwm_1 = int(255 * msg.data[0]);
M2A = 0;
M2B = 1;
pwm_2 = int(255 * msg.data[1]);
}
else if (msg.data[0] < 0 && msg.data[1] < 0){
M1A = 1;
M1B = 0;
pwm_1 = int(-255 * msg.data[0]);
M2A = 1;
M2B = 0;
pwm_2 = int(-255 * msg.data[1]);
}
else if (msg.data[0] > 0 && msg.data[1] < 0){
M1A = 0;
M1B = 1;
pwm_1 = int(255 * msg.data[0]);
M2A = 1;
M2B = 0;
pwm_2 = int(-255 * msg.data[1]);
}
else if (msg.data[0] < 0 && msg.data[1]>0){
M1A = 1;
M1B = 0;
pwm_1 = int(-255 * msg.data[0]);
M2A = 0;
M2B = 1;
pwm_2 = int(255 * msg.data[1]);
}
else { //ALTO
M1A = 0;
M1B = 0;
M2A = 0;
M2B = 0; }
}// Fin de SpeedsCallBack
//--------------------------------------------------------------------------
//Indica los tópicos que utilizará y su función
//Se subscribe al tópico publicado en el Raspberry para indicar la dirreción y velocidad del robot
ros::Publisher pub_sensors("/axo/hardware/arduino_sensors", &msg_sensors);
ros::Subscriber<std_msgs::Float32MultiArray> robot_Speeds("/axo/hardware/motor_speeds",&SpeedsCallback);
//________________________________________________________________________________________________________
  
//SETUP
void setup(){
n.getHardware()->setBaud(500000);

//Iniciar nodo en ros
n.initNode();

//Advertir a nodo que voy a publicar
n.advertise(pub_sensors);
n.subscribe(robot_Speeds);

msg_sensors.data_length = 8;
//Configuracion de fin de carrera
pinMode(buttonback_1,INPUT);
pinMode(buttonback_2,INPUT);

//Configuracion de los pines de salida -- Control Motores
pinMode(m_M1A, OUTPUT);
pinMode(m_M1B, OUTPUT);
pinMode(m_M2A, OUTPUT);
pinMode(m_M2B, OUTPUT);
pinMode(M1_pwm, OUTPUT);
pinMode(M2_pwm, OUTPUT);
pinMode(barredora_a,OUTPUT);
pinMode(barredora_b, OUTPUT);
myservo.attach(36);
} //Fin del SETUP
//******************************************************************************************************
void loop(){

    playa_sensor_readings[0]=distanceSensor1.measureDistanceCm();
    delay(2);
    playa_sensor_readings[1]=distanceSensor2.measureDistanceCm();
    delay(2);
    playa_sensor_readings[2]=distanceSensor3.measureDistanceCm();
    delay(2);
    playa_sensor_readings[3]=distanceSensor4.measureDistanceCm();
    delay(2);
    playa_sensor_readings[4]=distanceSensor5.measureDistanceCm();
    delay(2);
    playa_sensor_readings[5]=distanceSensor6.measureDistanceCm();
    delay(2);
    
    playa_sensor_readings[6] = digitalRead(buttonback_1);
    playa_sensor_readings[7] = digitalRead(buttonback_2);

    msg_sensors.data = playa_sensor_readings;
    pub_sensors.publish(&msg_sensors);
    n.spinOnce();
    myservo.write(angle);       
    //Barredora
    analogWrite(barredora_a,200);
    digitalWrite(barredora_b,LOW);
    //Motores
    digitalWrite(m_M1A,M1A);
    digitalWrite(m_M1B,M1B);
    analogWrite(M1_pwm, pwm_1);
    digitalWrite(m_M2A,M2A);
    digitalWrite(m_M2B,M2B);
    analogWrite(M2_pwm, pwm_2);

}
