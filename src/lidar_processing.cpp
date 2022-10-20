#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Temperature.h>  
#include <math.h>

using namespace std;

ros::Publisher pub_r_scan;
ros::Publisher pub_l_scan;
ros::Publisher pub_ganglios;

double r_scan_ant = 0.0;
double l_scan_ant = 0.0;

bool c_var = false;
int c_var_n = 0;

void callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    //cout << msg->ranges.size() << endl;
    sensor_msgs::Temperature r_scan;
    sensor_msgs::Temperature l_scan;
    sensor_msgs::Temperature ganglios_signal;
    r_scan.temperature = 0.0;
    l_scan.temperature = 0.0;
    double aux_scan_r;
    double aux_scan_l;
    for(int i = 0; i < 90; i++){
        aux_scan_r = msg->ranges[90+i];
        aux_scan_l = msg->ranges[180+i];
        if(aux_scan_r <= 0.6 && aux_scan_r != 0.0){
            r_scan.temperature -= 1;
        }
        else if(aux_scan_r > 0.6 || isinf(aux_scan_r) || aux_scan_r == 0.0){
            r_scan.temperature+=1;
        }
        if(aux_scan_l <= 0.6 && aux_scan_l != 0.0){
            l_scan.temperature -= 1;
        }
        else if(aux_scan_l > 0.6 || isinf(aux_scan_l) || aux_scan_l == 0.0){
            l_scan.temperature+=1;
        }
 
    }
    /*if(r_scan.data != 1.0){
        r_scan.data = 1 - 1.0/r_scan.data;
    }
    if(l_scan.data != 1.0){
        l_scan.data = 1 - 1.0/l_scan.data;
    } */
    //cout << "r_scan: " << r_scan << endl;
    //cout << "l_scan: " << l_scan << endl;
    if(r_scan.temperature < 0) r_scan.temperature = 0;
    if(l_scan.temperature < 0) l_scan.temperature = 0;
    
    
    if(r_scan_ant != 0 && l_scan_ant != 0)
    {
        if(std::abs(r_scan.temperature - r_scan_ant)/r_scan_ant > 0.5 || std::abs(l_scan.temperature - l_scan_ant)/l_scan_ant > 0.5){
        //Enviar señal a ganglios
        ganglios_signal.temperature = 100.0;
        c_var = true;
        }
        else{
        //Enviar 0 a ganglios
        ganglios_signal.temperature = 0.0;
            
        }   
    }
    /*
    if(c_var && c_var_n < 10){
        c_var_n++;
        ganglios_signal.temperature = 100.0;
    }
    else{
        c_var = false;
        c_var_n = 0;
    }
    */
    r_scan_ant = r_scan.temperature;
    l_scan_ant = l_scan.temperature;
    
    pub_l_scan.publish(l_scan);
    pub_r_scan.publish(r_scan);
    pub_ganglios.publish(ganglios_signal);
}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "lidar_processing_node");
    ros::NodeHandle n;
    
    //pub_path = n.advertise<nav_msgs::Path>("robot_path", 1000);
    ros::Subscriber sub_scan = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, callback);
    
    pub_l_scan = n.advertise<sensor_msgs::Temperature>("l_scan", 1000);
    pub_r_scan = n.advertise<sensor_msgs::Temperature>("r_scan", 1000);
    pub_ganglios = n.advertise<sensor_msgs::Temperature>("ganglios_signal", 1000);
    
    ros::spin();

    return 0;
     
}
