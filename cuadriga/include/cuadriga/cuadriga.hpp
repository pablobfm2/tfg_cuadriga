#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <string>
#include <GeographicLib/UTMUPS.hpp>
#include <cmath>
#include <sensor_msgs/msg/joy.hpp>
#include "cuadriga_interfaces/msg/gpx_path.hpp"

class Cuadriga : public rclcpp::Node
{ 
    public:
        Cuadriga();
        ~Cuadriga();
        
        //FUNCIONES Y PROCEDIMIENTOS
        //============================================
        void prueba();
        void process_GPS_data(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
        void process_query(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
        std::array<float,2> LLR2XYR(float Latitud, float Longitud, float Altitud); //Funcion que pasa de coordenadas GPS a coordenadas x,y,z
        char ASCII2Char(const int ascii_data);
        int Hex2Dec(const std::array<char,2> hex_value);
        std::array<std::vector<u_char>,2> EnviaVelocidad(int velocidad, int rueda);
        std::array<float,2> computeLookaheadPoint(float x, float y, const std::vector<std::array<float,2>>& path, float Ld);
        void PurePursuit(std::vector<std::array<float,2>> trayectoria);
        void FollowTheCarrot();
        void process_orientation_data(const geometry_msgs::msg::Vector3::SharedPtr msg);
        void process_joystick_data (const sensor_msgs::msg::Joy::SharedPtr msg);
        void process_trayectoria (const cuadriga_interfaces::msg::GPXPath::SharedPtr msg);
        void process_trayectoria_topic(const std_msgs::msg::Float32MultiArray::SharedPtr msg);


        //PUBLICADORES
        //=============================================
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub1_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub2_;

        //SUBCRIPTORES
        //=============================================
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub1_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub2_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub3_;
        rclcpp::Subscription<cuadriga_interfaces::msg::GPXPath>::SharedPtr sub4_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub5_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub6_;
        //VARIABLES
        //=============================================
        float Latitud;
        float Longitud;
        float Altitud;
        float Latitud0;
        float Longitud0;
        float Altitud0;
        std::array<float,2> posicion;
        std::array<float,2> origen;
        bool flag_origen;
        std::vector<u_char> cmd_vel_hex; //por ahora no la uso

        std::vector<std::array<float,2>> trayectoria;
        float v_lineal;
        float w;
        float vmin;
        float vmax;

    private:
    std_msgs::msg::UInt8MultiArray cmd;
    std_msgs::msg::Float32MultiArray posicion_grafica;
    std::vector<u_int8_t> palabra_actual;
    std::vector<std::vector<u_int8_t>> palabras;
    float Voltaje_bat;
    float Voltaje_int;
    std::vector<u_int8_t> data;
    bool flag;
    bool flag_rafaga_activa;
    int contador_rafaga;
    int temporizador;
    std::array <char,2> Caracter;
    float Voltaje_bateria;
    float Voltaje_controladora;

    float angulo_objetivo;
    float orientacion_actual;

};


