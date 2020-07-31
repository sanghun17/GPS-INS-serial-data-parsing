#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string>
#include <boost/lexical_cast.hpp>
#include <bits/stdc++.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <bits/stdc++.h> 
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>



#define NTH_BIT(b, n) ((b >> n) & 0x1)
//for IEEE754 32bit convert
typedef union UnFloatingPointIEEE754
{
    struct
    {
        unsigned int mantissa : 23;
        unsigned int exponent : 8;
        unsigned int sign : 1;
    } raw;   
    float f;  
} UFloatingPointIEEE754;

//for IEEE754 64bit convert
typedef union UnDoublePointIEEE754
{
    struct
    {
        unsigned long mantissa : 52;
        unsigned int exponent : 11;
        unsigned int sign : 1;
    } raw;   
    double d;  
} UnDoublePointIEEE754;

float ieee754decoding_32(unsigned char* str_data_arr,int i)
{
    unsigned char input[4] ={str_data_arr[i],str_data_arr[i-1],str_data_arr[i-2],str_data_arr[i-3] };
    char output_32[9];
    //char[ string shape ] -> char[ hex shape ]
    for (int j=0; j<4; j++)
    {
        sprintf((char*)(output_32 + 2*j),"%02X", input[j]);
    }
    //insert NULL at the end of the output string
    output_32[9] = '\0';

    uint32_t n;
    // char [ hex shape ] -> string (hex shape) 
    std::string hex_str(output_32);
    std::stringstream ss;
    ss << hex_str;
    // string (hex shape) -> int (hex shape)
    ss >> std::hex >> n;
    
    //apply ieee754 rule at int (hex shpae) to get int (int shape)
    UFloatingPointIEEE754 ieee754float;    unsigned int mantissa = 0;
    unsigned int exponent = 0 ;
    unsigned int sign = 0;    
   
    sign = NTH_BIT(n, 31);
    for( int ix=0; ix<8; ix++)
    exponent = (exponent | (NTH_BIT(n, (30-ix))))<<1;
    exponent = exponent>>1;
    for( int ix=0; ix<23; ix++)
    mantissa = (mantissa | (NTH_BIT(n, (22-ix))))<<1;
    mantissa = mantissa >> 1;    
   
    ieee754float.raw.sign = sign;
    ieee754float.raw.exponent = exponent;
    ieee754float.raw.mantissa = mantissa;    
    return ieee754float.f; 
}

double ieee754decoding_64(unsigned char* str_data_arr, int i)
{
    unsigned char input[8] ={str_data_arr[i],str_data_arr[i-1],str_data_arr[i-2],str_data_arr[i-3],str_data_arr[i-4],str_data_arr[i-5],str_data_arr[i-6],str_data_arr[i-7] } ;
    char output_64[17];
    for (int j=0; j<8; j++)
    {
        sprintf((char*)(output_64 + 2*j),"%02X", input[j]);
    }
    output_64[17] = '\0';
    uint64_t n; 
    std::string hex_str(output_64);
    std::stringstream ss;
    ss << hex_str;
    ss >> std::hex >> n;
    
    UnDoublePointIEEE754 ieee754double;    unsigned long mantissa = 0;
    unsigned int exponent = 0 ;
    unsigned int sign = 0;    
   
    sign = NTH_BIT(n, 63);
    for( int ix=0; ix<11; ix++)
    exponent = (exponent | (NTH_BIT(n, (62-ix))))<<1;
    exponent = exponent >> 1;
    for( int ix=0; ix<52; ix++)
    mantissa = (mantissa | (NTH_BIT(n, (51-ix))))<<1;
    mantissa = mantissa >> 1;
  
    ieee754double.raw.sign = sign;
    ieee754double.raw.exponent = exponent;
    ieee754double.raw.mantissa = mantissa;    
    return ieee754double.d; 
}

uint16_t checkcrc(unsigned char* pBuffer, int bufferSize)
{
    const uint8_t *pArray = (const uint8_t *)pBuffer;
    uint16_t poly = 0x8408;
    uint16_t crc16 = 0;
    uint8_t carry;
    uint8_t i;
    uint16_t j;
    for (j=2; j<bufferSize-3; j++)
    {
        crc16 = (crc16 ^ pBuffer[j]);
        for(i=0; i<8;i++)
        {
            carry = crc16 & 1;
            crc16 = crc16 / 2;
            if(carry)
            {
                crc16 = (crc16 ^ poly);
            }
        }
    }

    unsigned char input[2] ={pBuffer[bufferSize-2],pBuffer[bufferSize-3]};
    //unsigned char input[2] ={0xfb, 0x7b};
  
    char crc[5];
    //char[ string shape ] -> char[ hex shape ]
    for (int j=0; j<2; j++)
    {
        sprintf((char*)(crc + 2*j),"%02X", input[j]);
    }
    //insert NULL at the end of the output string
    crc[5] = '\0';
    uint32_t n;
    // char [ hex shape ] -> string (hex shape) 
    std::string hex_str(crc);
    std::stringstream ss;
    ss << hex_str;
    // string (hex shape) -> int (hex shape)
    ss >> std::hex >> n;

    if (crc16 == n)
        return 1;
    else return 0;
   
}

void gpsdecoding (unsigned char* str_data_arr, int str_len, geometry_msgs::QuaternionStamped & msg_Quaternion, geometry_msgs::TwistStamped & msg_Twist, sensor_msgs::NavSatFix & msg_Nav )
{
    //check integrity by CRC
    if ( checkcrc( str_data_arr, str_len ) == 1 )
    {
 
        //data type 06
        if ((int)str_data_arr[2] == 7)
        {
            msg_Quaternion.quaternion.x = ieee754decoding_32(str_data_arr, 13) ;
            msg_Quaternion.quaternion.y = ieee754decoding_32(str_data_arr, 17) ;
            msg_Quaternion.quaternion.z = ieee754decoding_32(str_data_arr, 21) ;
            msg_Quaternion.quaternion.w = ieee754decoding_32(str_data_arr, 25) ;

            
        }

      
       
        //data type 08
        if ((int)str_data_arr[2] == 8)
        {
            msg_Twist.twist.linear.x = ieee754decoding_32(str_data_arr, 13);
            msg_Twist.twist.linear.y = ieee754decoding_32(str_data_arr, 17);
            msg_Twist.twist.linear.z = ieee754decoding_32(str_data_arr, 21);
            msg_Nav.latitude = ieee754decoding_64(str_data_arr, 41);
            msg_Nav.longitude  = ieee754decoding_64(str_data_arr, 49);
            msg_Nav.altitude = ieee754decoding_64(str_data_arr, 57);
        }
    } 
}




int main (int argc, char** argv){
    ros::init(argc, argv, "gps_data_pub");
    ros::NodeHandle nh;
    ros::Publisher gps_Quaternion_pub = nh.advertise<geometry_msgs::QuaternionStamped>("Quaternion", 100);
    ros::Publisher gps_Twist_pub = nh.advertise<geometry_msgs::TwistStamped>("Twist",100);
    ros::Publisher gps_Nav_pub = nh.advertise<sensor_msgs::NavSatFix>("Nav",100);

    geometry_msgs::QuaternionStamped msg_Quaternion;
    geometry_msgs::TwistStamped msg_Twist;
    sensor_msgs::NavSatFix msg_Nav;
    msg_Quaternion.quaternion.x =0;
    msg_Quaternion.quaternion.y =0;
    msg_Quaternion.quaternion.w =0;
    msg_Quaternion.quaternion.z =0;
    msg_Twist.twist.linear.x =0;
    msg_Twist.twist.linear.y =0;
    msg_Twist.twist.linear.z =0;
    msg_Twist.twist.angular.x =0;
    msg_Twist.twist.angular.y =0;
    msg_Twist.twist.angular.z =0;
    msg_Nav.latitude =0;
    msg_Nav.longitude =0;
    msg_Nav.altitude =0;
    serial::Serial ser;

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen())
        ROS_INFO_STREAM("Serial Port initialized");
    else
        return -1;


    ros::Rate loop_rate(500);
    std::string str_data;
    unsigned char str_data_arr[100];
    int str_len = 0;

    while(ros::ok()){
       
        ros::spinOnce();
        if(ser.available()){
            str_data = ser.readline(200, "3");
            str_len = str_data.length();
            memcpy(str_data_arr, str_data.c_str(), str_len+1);
            str_data_arr[str_len+1] = 0x00;
            gpsdecoding(str_data_arr, str_len,msg_Quaternion, msg_Twist, msg_Nav);
            gps_Quaternion_pub.publish(msg_Quaternion);
            gps_Twist_pub.publish(msg_Twist);
            gps_Nav_pub.publish(msg_Nav);
        }
        loop_rate.sleep();

    }
}

