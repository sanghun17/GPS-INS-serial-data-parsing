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

/*
// below define is for encoding gpsdata (e.g int -> 32-hex data)
#define BYTE_TO_BIN(b)   (( b & 0x80 ) ) |\
            (( b & 0x40 ) ) |\
            (( b & 0x20 ) ) |\
            (( b & 0x10 ) ) |\
            (( b & 0x08 ) ) |\
            (( b & 0x04 ) ) |\
            (( b & 0x02 ) ) |\
            ( b & 0x01 )

#define MANTISSA_TO_BIN(b)  (( b & 0x400000 ) ) |\
             (( b & 0x200000 ) ) strToBinary(sss);|\
             (( b &  0x40000 ) ) |\
             (( b &  0x20000 ) ) |\
             (( b &  0x10000 ) ) |\
             (( b &  0x8000 ) ) |\
             (( b &  0x4000 ) ) |\
             (( b &  0x2000 ) ) |\
             (( b &  0x1000 ) ) |\
             (( b &  0x800 ) ) |\
             (( b &  0x400 ) ) |\
             (( b &  0x200 ) ) |\
             (( b &  0x100 ) ) |\
             (( b &  0x80 ) ) |\
             (( b &  0x40 ) ) |\
             (( b &  0x20 ) ) |\
             (( b &  0x10 ) ) |\
             (( b &  0x08 ) ) |\
             (( b &  0x04 ) ) |\
             (( b &  0x02 ) ) |\
              ( b & 0x01 )
*/

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
    std::cout << "q: "<< output_64 << std::endl;
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

void gpsdecoding (unsigned char* str_data_arr, float& roll, float& pitch, float& yaw, float& vel_n, float& vel_e, float& vel_d, double& latitude, double& longitude, double& altitude )
{
    //data type 06
    if ((int)str_data_arr[2] == 6)
    {
        roll = ieee754decoding_32(str_data_arr, 13) * 180 / 3.141592;
        pitch = ieee754decoding_32(str_data_arr, 17) * 180 / 3.141592;
        yaw = ieee754decoding_32(str_data_arr, 21) * 180 / 3.141592;
       
    }

    //data type 08
    if ((int)str_data_arr[2] == 8)
    {
        vel_n = ieee754decoding_32(str_data_arr, 13);
        vel_e = ieee754decoding_32(str_data_arr, 17);
        vel_d = ieee754decoding_32(str_data_arr, 21);
        latitude = ieee754decoding_64(str_data_arr, 41);
        longitude = ieee754decoding_64(str_data_arr, 49);
        altitude = ieee754decoding_64(str_data_arr, 57);
    }
}

uint16_t checkcrc(unsigned char *pBuffer, uint16_t bufferSize)
{
    const uint8_t *pArray = (unsigned uint8_t*)pBuffer;
    uint16_t poly = 0x8408;
    uint16_t crc16 = 0;
    uint8_t carry;
    uint8_t i;
    uint16_t j;
    for (j=0; j<bufferSize; j++)
    {
        crc16 = crc16 ^ pArray[j];
        for(i=0; i<8;i++)
        {
            carry = crc16 & 1;
            crc16 = crc16/2;
            if(carry)
            {
                crc16 = crc16 ^ poly;
            }
        }
    }

    unsigned char input[2] ={pBuffer[bufferSize-2],pBuffer[bufferSize-3]};
    char crc[5];
    //char[ string shape ] -> char[ hex shape ]
    for (int j=0; j<2; j++)
    {
        sprintf((char*)(crc + 2*j),"%02X", input[j]);
    }
    //insert NULL at the end of the output string
    crc[5] = '\0';

    uint16_t n;
    // char [ hex shape ] -> string (hex shape) 
    std::string hex_str(crc);
    std::stringstream ss;
    ss << hex_str;
    // string (hex shape) -> int (hex shape)
    ss >> std::hex >> n;
    std::cout << crc16 << std::endl;
    if (crc16 == n)
        return 1;
    else return 0;
   
}
serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}




int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    std_msgs::String result;
    int str_len = 0;
    char c_arr[100];
    std::string str_data;
    unsigned char str_data_arr[100];
    float roll, pitch, yaw, vel_n, vel_e, vel_d;
    double latitude, longitude, altitude;
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

    ros::Rate loop_rate(1000);
    while(ros::ok()){
        ros::spinOnce();
        if(ser.available()){
            str_data = ser.readline(200, "3");
            str_len = str_data.length();
            memcpy(str_data_arr, str_data.c_str(), str_len+1);
            str_data_arr[str_len+1] = 0x00;
            gpsdecoding(str_data_arr, roll, pitch, yaw, vel_n, vel_e, vel_d, latitude, longitude, altitude);
            std::cout << "length   : " << str_len << std::endl;
            std::cout << "Roll     : " << roll << std::endl;
            std::cout << "Pitch    : " << pitch << std::endl;
            std::cout << "Yaw      : " << yaw << std::endl;
            std::cout << "Vel_N    : " << vel_n << std::endl;
            std::cout << "Vel_E    : " << vel_e << std::endl;
            std::cout << "Vel_D    : " << vel_d << std::endl;
            std::cout << "latitude : " << latitude << std::endl;
            std::cout << "longitude: " << longitude << std::endl;
            std::cout << "altitude : " << altitude << std::endl;
            std::cout << checkcrc(str_data_arr,str_len) << std::endl;
            std::cout << str_len << std::endl;
            read_pub.publish(result);
        }
        loop_rate.sleep();

    }
}

