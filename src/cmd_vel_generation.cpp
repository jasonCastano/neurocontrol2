#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Temperature.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <math.h>
#include <algorithm>
#include <thread>

using namespace std;


bool f(float x){
	if(x>=0.3){
		return true;
	}
	else{
	return false;
	}
}

bool pwm(float x){
	if(x>20){
		return true;
	}
	else{
	return false;
	}
}


// 	std::vector<float> t; // Evaluar si es nevesario t
double dt = 0.1;
double dt_ant = 0;
bool first_time = true;
double z[4][2]{};

double y[4][2]{};

double Su = 0.0; //Entrada

double Sd = 0.0;

double Sy = 0.0;


// Parametros memoria a corto plazo

const float A = 100.0;
const float B = 125.0;
const float a = 3.0;
const float b = 0.7;
const float c = 1.0;
const float tau1 = 20.0;
const float tau2 = 2000.0;

// Neuronas

double Mu[5][2]{};
double Md[5][2]{};
const float B1 = 30.0;
const float tau = 100.0;

// Acumuladores

double Au[1][2]{};
double Ad[1][2]{};

//Comparadores

double U[2][2]{};

//WTA
double O[3][2]{};
const float Oe = 1.0;

//Actividad Ciclicas

double Oc1[4][2]{};
double Oc2[4][2]{};
double Oc3[4][2]{};

const float c_Oc = 1.5;
const float d = 3.2;
const float K1 = 150.0;
const float K2 = 146.916;
const float K3 = 149.9;

const float tauOc1 = 20.0;
const float tauOc2 = 280.0;

//Acumulador Exploracion
double Ac[2][2]{};

//Memoria de aprendizaje

double Mo1[5][2]{};
double Mo2[5][2]{};

//Acumulador Aprendizaje

double Ap[2][2]{};


//Entrada de ganglios

double S1_rc = 0.0;
double S2_rc = 100 - S1_rc;

//Parametros de red de control (ganglios basales)

const float A_rc = 1.0;
const float B_rc = 1.0;
const float N_rc = 40.0;
const float M_rc = 0.3;
const float b_rc = 1.0;
const float c_rc = 1.0;
const float g_rc = 1.0;
const float h_rc = 1.0;
const float d_rc = 10.0;
const float tau1_rc = 10.0;

double n[7][2]{};
double n_z[4][2]{};

const float tau1z = 10.0;
const float tau2z = 5000.0;

ros::Publisher pub_cmd;
ros::Publisher pub_exp_mot;
ros::Publisher pub_huir_mot;

std_msgs::Float64 exp_msg;
std_msgs::Float64 huir_msg;

int max_result = 0;

const float r = 3.3; //radio de la llanta en cm
const float l = 16.0; //distancia entre llantas en cm

int get_max(double v1, double v2, double v3){
	if(v1 > v2 && v1 > v3){
		return 1;
	}
	else if(v2 > v1 && v2 > v3){
		return 2;
	}
	else{
	return 3;
    }
}

ros::Rate loop_rate(5);

geometry_msgs::Twist cmd_vel_msg;

void publish_cmd(){
    
    while(1){
        pub_cmd.publish(cmd_vel_msg);
        loop_rate.sleep();
    }

}

void calculation_thread(){
    
    while(1){

    //ganglios basales
        
    n_z[0][1] = n_z[0][0] + (dt/tau1z)*(-n_z[0][0] + (A*pow(50*S1_rc + a*n_z[2][0],2.0))/(pow(B+c*n_z[1][0],2.0) + pow(50*S1_rc+a*n_z[2][0],2.0)));
    n_z[1][1] = n_z[1][0] + (dt/tau2z)*(-n_z[1][0] + b*n_z[0][0]);                      
    n_z[2][1] = n_z[2][0] + (dt/tau1z)*(-n_z[2][0] + (A*pow(a*n_z[0][0],2.0))/(pow(B+c*n_z[3][0],2.0) + pow(a*n_z[0][0],2.0)));
    n_z[3][1] = n_z[3][0] + (dt/tau2z)*(-n_z[3][0] + b*n_z[2][0]);
    
    S2_rc = 100.0 - n_z[0][0];
    
    n[0][1]=n[0][0]+(dt/tau1_rc)*(-n[0][0]+(A_rc*pow(max(0.0,n_z[0][0]-b_rc*n[3][0]+n[5][0]),2))/(pow(N_rc,2)+pow(max(0.0,n_z[0][0]-b_rc*n[3][0]+n[5][0]),2)));
    n[5][1]=n[5][0]+(dt/tau1_rc)*(-n[5][0]+(A_rc*pow(n[0][0],2))/(pow(N_rc,2)+pow(n[0][0],2)));
    n[1][1]=n[1][0]+(dt/tau1_rc)*(-n[1][0]+(A_rc*pow(max(0.0,S2_rc-c_rc*n[2][0]+n[6][0]),2))/(pow(N_rc,2)+pow(max(0.0,S2_rc-c_rc*n[2][0]+n[6][0]),2)));
    n[6][1]=n[6][0]+(dt/tau1_rc)*(-n[6][0]+pow(A_rc*(n[1][0]),2)/(pow(N_rc,2)+pow(n[1][0],2)));
    n[2][1]=n[2][0]+(dt/tau1_rc)*(-n[2][0]+(B_rc*pow(n[0][0],2))/(pow(M_rc,2)+pow(n[0][0],2)));
    n[3][1]=n[3][0]+(dt/tau1_rc)*(-n[3][0]+(B_rc*pow(n[1][0],2))/(pow(M_rc,2)+pow(n[1][0],2)));
    n[4][1]=n[4][0]+(dt/tau1_rc)*(-n[4][0]+(A_rc)/(1.0+exp(-d_rc*(g_rc*n[0][0]-h_rc*n[1][0]))));
        
    //Memoria Mu                                                                 
    z[0][1] = z[0][0] + (dt/tau1)*(-z[0][0] + (A*pow(50*Su + a*z[2][0],2.0))/(pow(B+c*z[1][0],2.0) + pow(50*Su+a*z[2][0],2.0)));
    z[1][1] = z[1][0] + (dt/tau2)*(-z[1][0] + b*z[0][0]);                      
    z[2][1] = z[2][0] + (dt/tau1)*(-z[2][0] + (A*pow(50*Sy + a*z[0][0],2.0))/(pow(B+c*z[3][0],2.0) + pow(50*Sy+a*z[0][0],2.0)));
    z[3][1] = z[3][0] + (dt/tau2)*(-z[3][0] + b*z[2][0]);

    //Memoria Md
    y[0][1] = y[0][0] + (dt/tau1)*(-y[0][0] + (A*pow(50*Sd + a*y[2][0],2.0))/(pow(B+c*y[1][0],2.0) + pow(50*Sd+a*y[2][0],2.0)));
    y[1][1] = y[1][0] + (dt/tau2)*(-y[1][0] + b*y[0][0]);
    y[2][1] = y[2][0] + (dt/tau1)*(-y[2][0] + (A*pow(50*Sy + a*y[0][0],2.0))/(pow(B+c*y[3][0],2.0) + pow(50*Sy+a*y[0][0],2.0)));
    y[3][1] = y[3][0] + (dt/tau2)*(-y[3][0] + b*y[2][0]); 

    //Neuronas Mu
    Mu[0][1]=Mu[0][0]+(dt/tau)*(-Mu[0][0]+(A*pow(z[0][0],2.0))/(pow(B1,2.0)+pow(z[0][0],2.0)));
    Mu[1][1]=Mu[1][0]+(dt/tau)*(-Mu[1][0]+(A*pow(Mu[0][0],2.0))/(pow(B1,2.0)+pow(Mu[0][0],2.0)));
    Mu[2][1]=Mu[2][0]+(dt/tau)*(-Mu[2][0]+(A*pow(Mu[1][0],2.0))/(pow(B1,2.0)+pow(Mu[1][0],2.0)));
    Mu[3][1]=Mu[3][0]+(dt/tau)*(-Mu[3][0]+(A*pow(Mu[2][0],2.0))/(pow(B1,2.0)+pow(Mu[2][0],2.0)));
    Mu[4][1]=Mu[4][0]+(dt/tau)*(-Mu[4][0]+(A*pow(Mu[3][0],2.0))/(pow(B1,2.0)+pow(Mu[3][0],2.0)));

    //Neuronas Md
    Md[0][1]=Md[0][0]+(dt/tau)*(-Md[0][0]+(A*pow(y[0][0],2.0))/(pow(B1,2.0)+pow(y[0][0],2.0)));
    Md[1][1]=Md[1][0]+(dt/tau)*(-Md[1][0]+(A*pow(Md[0][0],2.0))/(pow(B1,2.0)+pow(Md[0][0],2.0)));
    Md[2][1]=Md[2][0]+(dt/tau)*(-Md[2][0]+(A*pow(Md[1][0],2.0))/(pow(B1,2.0)+pow(Md[1][0],2.0)));
    Md[3][1]=Md[3][0]+(dt/tau)*(-Md[3][0]+(A*pow(Md[2][0],2.0))/(pow(B1,2.0)+pow(Md[2][0],2.0)));
    Md[4][1]=Md[4][0]+(dt/tau)*(-Md[4][0]+(A*pow(Md[3][0],2.0))/(pow(B1,2.0)+pow(Md[3][0],2.0)));

    //Acumuladores
    //Exploración
    Au[0][1]=Au[0][0]+(dt/1.0)*(-Au[0][0]+(A*pow(Mu[0][0]+Mu[1][0]+Mu[2][0]+Mu[3][0]+Mu[4][0],2.0))/(pow(B1,2.0)+pow(Mu[0][0]+Mu[1][0]+Mu[2][0]+Mu[3][0]+Mu[4][0],2.0)));
    //Huir
    Ad[0][1]=Ad[0][0]+(dt/1.0)*(-Ad[0][0]+(A*pow(Md[0][0]+Md[1][0]+Md[2][0]+Md[3][0]+Md[4][0],2.0))/(pow(B1,2.0)+pow(Md[0][0]+Md[1][0]+Md[2][0]+Md[3][0]+Md[4][0],2.0)));

    //Comparadores                                                               
    U[0][1]=U[0][0]+(dt/1.0)*(-U[0][0]+(A*(Au[0][0]-Ad[0][0]))/(pow(B1,2.0)+(Au[0][0]-Ad[0][0])));
    U[1][1]=U[1][0]+(dt/1.0)*(-U[1][0]+(A*(Ad[0][0]-Au[0][0]))/(pow(B1,2.0)+(Ad[0][0]-Au[0][0])));

    //WTA
    O[0][1]=O[0][0]+(dt/tau)*(-O[0][0]+(A*pow(max(0.0,(U[0][0]-a*(O[1][0]+O[2][0]))),2.0))/(pow(B,2.0)+pow(max(0.0,(U[0][0]-a*(O[1][0]+O[2][0]))),2.0)));
    O[1][1]=O[1][0]+(dt/tau)*(-O[1][0]+(A*pow(max(0.0,(U[1][0]-a*(O[0][0]+O[2][0]))),2.0))/(pow(B,2.0)+pow(max(0.0,(U[1][0]-a*(O[0][0]+O[2][0]))),2.0)));
    O[2][1]=O[2][0]+(dt/tau)*(-O[2][0]+(100*A*pow(max(0.0,(Oe-a*(O[0][0]+O[1][0]))),2.0))/(pow(B,2.0)+pow(max(0.0,(Oe-a*(O[0][0]+O[1][0]))),2.0)));

    //Neuronas memoria Mo1
    Mo1[0][1]=Mo1[0][0]+(dt/(2*tau))*(-Mo1[0][0]+(A*pow(30*O[0][0],2.0))/(pow(B1,2.0)+pow(30*O[0][0],2.0)));
    Mo1[1][1]=Mo1[1][0]+(dt/(2*tau))*(-Mo1[1][0]+(A*pow(Mo1[0][0],2.0))/(pow(B1,2.0)+pow(Mo1[0][0],2.0)));
    Mo1[2][1]=Mo1[2][0]+(dt/(2*tau))*(-Mo1[2][0]+(A*pow(Mo1[1][0],2.0))/(pow(B1,2.0)+pow(Mo1[1][0],2.0)));
    Mo1[3][1]=Mo1[3][0]+(dt/(2*tau))*(-Mo1[3][0]+(A*pow(Mo1[2][0],2.0))/(pow(B1,2)+pow(Mo1[2][0],2.0)));
    Mo1[4][1]=Mo1[4][0]+(dt/(2*tau))*(-Mo1[4][0]+(A*pow(Mo1[3][0],2.0))/(pow(B1,2.0)+pow(Mo1[3][0],2.0)));

    //Neuronas memoria Mo2                                                       
    Mo2[0][1]=Mo2[0][0]+(dt/(2*tau))*(-Mo2[0][0]+(A*pow(30*O[1][0],2.0))/(pow(B1,2.0)+pow(30*O[1][0],2.0)));
    Mo2[2][1]=Mo2[1][0]+(dt/(2*tau))*(-Mo2[1][0]+(A*pow(Mo2[0][0],2.0))/(pow(B1,2.0)+pow(Mo2[0][0],2.0)));
    Mo2[2][1]=Mo2[2][0]+(dt/(2*tau))*(-Mo2[2][0]+(A*pow(Mo2[1][0],2.0))/(pow(B1,2.0)+pow(Mo2[1][0],2.0)));
    Mo2[3][1]=Mo2[3][0]+(dt/(2*tau))*(-Mo2[3][0]+(A*pow(Mo2[2][0],2.0))/(pow(B1,2.0)+pow(Mo2[2][0],2.0)));
    Mo2[4][1]=Mo2[4][0]+(dt/(2*tau))*(-Mo2[4][0]+(A*pow(Mo2[3][0],2.0))/(pow(B1,2.0)+pow(Mo2[3][0],2.0)));

    //Acumuladores
    Ap[0][1]=(Ap[0][0]+(dt/1.0)*(-Ap[0][0]+(A*pow(Mo1[0][0]+Mo1[1][0]+Mo1[2][0]+Mo1[3][0]+Mo1[4][0],2.0))/(pow(B1,2.0)+pow(Mo1[0][0]+Mo1[1][0]+Mo1[2][0]+Mo1[3][0]+Mo1[4][0],2.0))));
    Ap[1][1]=(Ap[1][0]+(dt/1.0)*(-Ap[1][0]+(A*pow(Mo2[0][0]+Mo2[1][0]+Mo2[2][0]+Mo2[3][0]+Mo2[4][0],2.0))/(pow(B1,2.0)+pow(Mo2[0][0]+Mo2[1][0]+Mo2[2][0]+Mo2[3][0]+Mo2[4][0],2.0))));

    //Actividad ciclica Exploración
    Oc1[0][1]=Oc1[0][0]+(dt/tauOc1)*(-Oc1[0][0]+((A-11*n[4][0])*pow(K1*f(O[0][0] + Ap[0][0] - Ap[1][0])-d*Oc1[1][0] + 0.006*Ap[0][0],2.0))/(pow(B+Oc1[2][0],2.0)+pow(K1*f(O[0][0] + Ap[0][0] - Ap[1][0])-d*Oc1[1][0] + 0.006*Ap[0][0],2.0)));
    Oc1[1][1]=Oc1[1][0]+(dt/tauOc1)*(-Oc1[1][0]+((A-11*n[4][0])*pow(K2*f(O[0][0] + Ap[0][0] - Ap[1][0])-d*Oc1[0][0] - 0.006*Ap[0][0],2.0))/(pow(B+Oc1[3][0],2.0)+pow(K2*f(O[0][0] + Ap[0][0] - Ap[1][0])-d*Oc1[0][0] - 0.006*Ap[0][0],2.0)));
    Oc1[2][1]=Oc1[2][0]+(dt/tauOc2)*(-Oc1[2][0]+c_Oc*Oc1[0][0]);
    Oc1[3][1]=Oc1[3][0]+(dt/tauOc2)*(-Oc1[3][0]+c_Oc*Oc1[1][0]);

    //Actividad ciclica Huir
    Oc2[0][1]=Oc2[0][0]+(dt/tauOc1)*(-Oc2[0][0]+((A-11*n[4][0])*pow(K2*f(O[1][0] - Ap[0][0] + Ap[1][0])-d*Oc2[1][0] - 0.006*Ap[1][0],2.0))/(pow(B+Oc2[2][0],2.0)+pow(K2*f(O[1][0] - Ap[0][0] + Ap[1][0])-d*Oc2[1][0] - 0.006*Ap[1][0],2.0)));
    Oc2[1][1]=Oc2[1][0]+(dt/tauOc1)*(-Oc2[1][0]+((A-11*n[4][0])*pow(K1*f(O[1][0] - Ap[0][0] + Ap[1][0])-d*Oc2[0][0] + 0.006*Ap[1][0],2.0))/(pow(B+Oc2[3][0],2.0)+pow(K1*f(O[1][0] - Ap[0][0] + Ap[1][0])-d*Oc2[0][0] + 0.006*Ap[1][0],2.0)));
    Oc2[2][1]=Oc2[2][0]+(dt/tauOc2)*(-Oc2[2][0]+c_Oc*Oc2[0][0]);
    Oc2[3][1]=Oc2[3][0]+(dt/tauOc2)*(-Oc2[3][0]+c_Oc*Oc2[1][0]);

    //Actividad ciclica Espera
    Oc3[0][1]=Oc3[0][0]+(dt/tauOc1)*(-Oc3[0][0]+((A-11*n[4][0])*pow(K1*f(O[2][0] - Ap[0][0] - Ap[1][0])-d*Oc3[1][0],2.0))/(pow(B+Oc3[2][0],2.0)+pow(K1*f(O[2][0] - Ap[0][0] - Ap[1][0])-d*Oc3[1][0],2.0)));
    Oc3[1][1]=Oc3[1][0]+(dt/tauOc1)*(-Oc3[1][0]+((A-11*n[4][0])*pow(K3*f(O[2][0] - Ap[0][0] - Ap[1][0])-d*Oc3[0][0],2.0))/(pow(B+Oc3[3][0],2.0)+pow(K3*f(O[2][0] - Ap[0][0] - Ap[1][0])-d*Oc3[0][0],2.0)));
    Oc3[2][1]=Oc3[2][0]+(dt/tauOc2)*(-Oc3[2][0]+c_Oc*Oc3[0][0]);
    Oc3[3][1]=Oc3[3][0]+(dt/tauOc2)*(-Oc3[3][0]+c_Oc*Oc3[1][0]);

    //Acumulador Derecho
    Ac[0][1] = Ac[0][0]+(dt/1.0)*(-Ac[0][0]+Oc1[0][0]+Oc2[0][0]+Oc3[0][0]);

    //Acumulador Izquierdo
    Ac[1][1] = Ac[1][0]+(dt/1.0)*(-Ac[1][0]+Oc1[1][0]+Oc2[1][0]+Oc3[1][0]);

    //exp_msg.data = (Ac[0][0])/1.12/40.0;
    //huir_msg.data = (Ac[1][0])/1.12/40.0;
    
    exp_msg.data = (Ac[0][0])/1.12/800.0;
    huir_msg.data = (Ac[1][0])/1.12/800.0;
    
    
    //cmd_vel_msg.linear.x = (r/2.0)*(exp_msg.data + huir_msg.data);
    
    //pub_exp_mot.publish(exp_msg);
    //pub_huir_mot.publish(huir_msg);

    max_result = get_max(O[0][0], O[1][0], O[2][0]);
    cout << O[0][0] << " " << O[1][0] << " " << O[2][0] << endl;
    if(max_result == 1){cout << "Moving to left" << endl; cmd_vel_msg.angular.z = (10*exp_msg.data - huir_msg.data);}
    else if(max_result == 2){cout << "Moving to right" << endl; cmd_vel_msg.angular.z = (exp_msg.data - 10*huir_msg.data);}
    else{cout << "Moving forward" << endl; cmd_vel_msg.angular.z = 0.0;}
    
    cmd_vel_msg.linear.x = (exp_msg.data + huir_msg.data);
    cmd_vel_msg.linear.y = 0;
    cmd_vel_msg.linear.z = 0;
    
    cmd_vel_msg.angular.x = 0;
    cmd_vel_msg.angular.y = 0;
    //cmd_vel_msg.angular.z = (r/(2.0*l))*(exp_msg.data - huir_msg.data);
    
    //pub_cmd.publish(cmd_vel_msg);
    
    z[0][0] = z [0][1];  
    z[1][0] = z [1][1];
    z[2][0] = z [2][1];
    z[3][0] = z [3][1];

    y[0][0] = y[0][1];
    y[1][0] = y[1][1];
    y[2][0] = y[2][1];
    y[3][0] = y[3][1];

    Mu[0][0]=Mu[0][1];
    Mu[1][0]=Mu[1][1];
    Mu[2][0]=Mu[2][1];
    Mu[3][0]=Mu[3][1];
    Mu[4][0]=Mu[4][1];

    Md[0][0]=Md[0][1];
    Md[1][0]=Md[1][1];
    Md[2][0]=Md[2][1];
    Md[3][0]=Md[3][1];
    Md[4][0]=Md[4][1];

    Au[0][0]=Au[0][1];
    Ad[0][0]=Ad[0][1];

    U[0][0]=U[0][1];
    U[1][0]=U[1][1];

    O[0][0]=O[0][1];
    O[1][0]=O[1][1];
    O[2][0]=O[2][1];
   
    Mo1[0][0]=Mo1[0][1];
    Mo1[1][0]=Mo1[1][1];
    Mo1[2][0]=Mo1[2][1];
    Mo1[3][0]=Mo1[3][1];
    Mo1[4][0]=Mo1[4][1];

    Mo2[0][0]=Mo2[0][1];
    Mo2[2][0]=Mo2[1][1];
    Mo2[2][0]=Mo2[2][1];
    Mo2[3][0]=Mo2[3][1];
    Mo2[4][0]=Mo2[4][1];

    Ap[0][0]=Ap[0][1];
    Ap[1][0]=Ap[1][1];

    Oc1[0][0]=Oc1[0][1];
    Oc1[1][0]=Oc1[1][1];
    Oc1[2][0]=Oc1[2][1];
    Oc1[3][0]=Oc1[3][1];

    Oc2[0][0]=Oc2[0][1];
    Oc2[1][0]=Oc2[1][1];
    Oc2[2][0]=Oc2[2][1];
    Oc2[3][0]=Oc2[3][1];

    Oc3[0][0]=Oc3[0][1];
    Oc3[1][0]=Oc3[1][1];
    Oc3[2][0]=Oc3[2][1];
    Oc3[3][0]=Oc3[3][1];

    Ac[0][0] = Ac[0][1];
    Ac[1][0] = Ac[1][1];
    
    n[0][0]=n[0][1];
    n[1][0]=n[1][1];
    n[2][0]=n[2][1];
    n[3][0]=n[3][1];
    n[4][0]=n[4][1];
    n[5][0]=n[5][1];
    n[6][0]=n[6][1];
    
    n_z[0][0]=n_z[0][1];
    n_z[1][0]=n_z[1][1];
    n_z[2][0]=n_z[2][1];
    n_z[3][0]=n_z[3][1];
    
    
    }
}

void callback(const sensor_msgs::Temperature::ConstPtr& l_data, const sensor_msgs::Temperature::ConstPtr& r_data, const sensor_msgs::Temperature::ConstPtr& g_data)
{

    Su = l_data->temperature;
    Sd = r_data->temperature;
    S1_rc = g_data->temperature;
    //S2_rc = 100.0 - S1_rc;
    

}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "neuromotor_node");
    ros::NodeHandle n;
    
    //pub_path = n.advertise<nav_msgs::Path>("robot_path", 1000);
    //ros::Subscriber sub_scan = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, callback);
    
    //pub_exp_mot = n.advertise<std_msgs::Float64>("/right_effort_controller/command", 1000);
    //pub_huir_mot = n.advertise<std_msgs::Float64>("/left_effort_controller/command", 1000);
    
    pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    
    message_filters::Subscriber<sensor_msgs::Temperature> sub_l_scan(n, "l_scan", 10);
    message_filters::Subscriber<sensor_msgs::Temperature> sub_r_scan(n, "r_scan", 10);
    message_filters::Subscriber<sensor_msgs::Temperature> sub_ganglios(n, "ganglios_signal", 10);
    message_filters::TimeSynchronizer<sensor_msgs::Temperature, sensor_msgs::Temperature, sensor_msgs::Temperature> sync(sub_l_scan, sub_r_scan, sub_ganglios, 10);   
    sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    
    std::thread (calculation_thread).detach();
    
    std::thread (publish_cmd).detach();

    ros::spin();

    return 0;
     
}
