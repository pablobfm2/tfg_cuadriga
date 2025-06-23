#include "cuadriga/cuadriga.hpp"

using std::placeholders::_1;


Cuadriga::Cuadriga() : Node("cuadriga")
{
    float Latitud=0;
    float Longitud=0;
    float Altitud=0;
    float Latitud0=0;
    float Longitud0=0;
    float Altitud0=0;
    bool flag_origen=false;
    std::vector<u_int8_t> data = {0};
    bool flag = 0;
    bool flag_rafaga_activa =false;
    int contador_rafaga = 0;
    int temporizador = 0;
    std::array <char,2> Caracter = {0,0};
    float Voltaje_bateria = 0;
    float Voltaje_controladora = 0;
    this -> v_lineal = 2;
    float w = 0;
    this -> vmin = 0.4;
    this -> vmax = 1;
    std::vector<u_int8_t> palabra_actual = {0};
    std::vector<std::vector<uint8_t>> palabras = {{0},{0}};
    std::array<char,2> cmd_vel_hex = {0,0};
    this -> trayectoria = {};
    //auto qos = rclcpp::QoS(rclcpp::KeepLast(32)).best_effort();
    rclcpp::QoS qos_profile(rclcpp::KeepLast(150));
    //qos_profile.reliable();
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    //qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    //qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    pub1_ = this -> create_publisher<std_msgs::msg::UInt8MultiArray>("/serial_write",10);
    pub2_ = this -> create_publisher<std_msgs::msg::Float32MultiArray>("/posicion_grafica",10);

    sub1_ = this -> create_subscription<std_msgs::msg::UInt8MultiArray>("/serial_read",qos_profile,std::bind(&Cuadriga::process_query,this,_1));
    sub2_ = this -> create_subscription<sensor_msgs::msg::NavSatFix>("cuadriga/fixposition/navsatfix",1, std::bind(&Cuadriga::process_GPS_data,this,_1));
    sub3_ = this -> create_subscription<geometry_msgs::msg::Vector3>("cuadriga/fixposition/ypr",1, std::bind(&Cuadriga::process_orientation_data,this,_1));
    sub4_ = this -> create_subscription<cuadriga_interfaces::msg::GPXPath>("/trayectoria_gpx",10,std::bind(&Cuadriga::process_trayectoria,this,_1));
    sub5_ = this -> create_subscription<sensor_msgs::msg::Joy>("/ARGJ801/joy",1, std::bind(&Cuadriga::process_joystick_data,this,_1));
    sub6_ = this -> create_subscription<std_msgs::msg::Float32MultiArray>("/rambler",1,std::bind(&Cuadriga::process_mode_convoy,this,_1));

    this -> declare_parameter<bool>("flag_origen",false);
    this -> declare_parameter<bool>("enable_navegacion",false);
    this -> declare_parameter<float>("v_lineal",0.3);
}

Cuadriga::~Cuadriga()
{
    RCLCPP_INFO(this->get_logger(), "Se acabo");
}

std::array<std::vector<u_char>,2> Cuadriga::EnviaVelocidad(int v_pwm, int w_pwm)
{
    std::vector<u_char> cmd_vel_hex_A = {'!'}; //simbolo para abrir comando velocidad
    std::vector<u_char> cmd_vel_hex_B = {'!'}; //simbolo para abrir comando velocidad
   
    if(w_pwm<0) //Determino sentido de marcha
    {
        cmd_vel_hex_A.push_back('a');
    }else{
        cmd_vel_hex_A.push_back('A');
   
    }
    if(v_pwm<0) //Determino sentido de marcha
    {
        cmd_vel_hex_B.push_back('b');
    }else{
        cmd_vel_hex_B.push_back('B');
    }
   
 
    // Usamos un vector temporal para almacenar los caracteres hexadecimales
    std::vector<u_char> hexCharsA; // Si funciona lo nuevo esto no hace falta
    std::vector<u_char> hexCharsB;
    v_pwm = std::abs(v_pwm);
    w_pwm = std::abs(w_pwm);
 
    //--------------------LO NUEVO-----------------------
    char bufferA[3];
    snprintf(bufferA, sizeof(bufferA), "%02X", w_pwm);  // 2 dígitos siempre
    cmd_vel_hex_A.push_back(bufferA[0]);
    cmd_vel_hex_A.push_back(bufferA[1]);
 
    char bufferB[3];
    snprintf(bufferB, sizeof(bufferB), "%02X", v_pwm);  // 2 dígitos siempre
    cmd_vel_hex_B.push_back(bufferB[0]);
    cmd_vel_hex_B.push_back(bufferB[1]);
    //---------------------------------------------------
 
    /*while(v_pwm>0) //paso la velocidad lineal a hexadecimal
    {
        int resto = v_pwm%16;
        if(resto<10)
        {
            hexCharsA.push_back('0' + resto);
        }else{
            hexCharsA.push_back('A'+(resto-10));
        }
        v_pwm /= 16;
    }
 
    // Usamos un vector temporal para almacenar los caracteres hexadecimales
   
    while(w_pwm>0) //paso la velocidad angular a hexadecimal
    {
        int resto = w_pwm%16;
        if(resto<10)
        {
            hexCharsB.push_back('0' + resto);
        }else{
            hexCharsB.push_back('A'+(resto-10));
        }
        w_pwm /= 16;
    }
 
    // Ahora invertimos el vector hexChars para que los dígitos estén en el orden correcto
    for (int i = hexCharsA.size() - 1; i >= 0; --i) {
        cmd_vel_hex_A.push_back(hexCharsA[i]);
    }
 
    for (int i = hexCharsB.size() - 1; i >= 0; --i) {
        cmd_vel_hex_B.push_back(hexCharsB[i]);
    }*/
 
    cmd_vel_hex_A.push_back(13); //añado el retorno de carro para cerrar el comando
    cmd_vel_hex_B.push_back(13); //añado el retorno de carro para cerrar el comando
 
    std::array<std::vector<u_char>,2> Salida;
    Salida[0] = cmd_vel_hex_A;
    Salida[1] = cmd_vel_hex_B;
    return Salida;
}

void Cuadriga::prueba()
{
    temporizador = temporizador + 1;
    RCLCPP_INFO(this->get_logger(), "El temporizador es de %i segundos", temporizador);
    if(temporizador>3 && temporizador<6)
    {
        cmd.data = {'?','E',13};
    }else{
        cmd.data = {0};
    }
    
    //cmd.data = {'!','A','3','2',13};
    pub1_-> publish(cmd);
    /*if(temporizador>5 && temporizador<20)
    {
        //RCLCPP_INFO(this->get_logger(), "Estoy mandando 90 de velocidad \n\n");
        //cmd.data = {'!','A','3','2',13};
        //cmd.data = {'?','E',13};

        comando = EnviaVelocidad(v_lineal,w);
        cmd.data = comando[0];
        pub1_-> publish(cmd);
        out="";
        for (auto byte : comando[0]) {
            out += std::to_string(static_cast<unsigned int>(byte)) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Estoy mandando %s de velocidad lineal \n",out.c_str());

        out="";
        cmd.data = comando[1];
        pub1_-> publish(cmd);
        for (auto byte : comando[1]) {
            out += std::to_string(static_cast<unsigned int>(byte)) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Estoy mandando %s de velocidad angular \n",out.c_str());
        
        
        //cmd.data = EnviaVelocidad(60,false,2);
        //pub1_-> publish(cmd);
        out="";
        for (auto byte : comando[0]) {
            out += std::to_string(static_cast<unsigned int>(byte)) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Estoy mandando %s de velocidad lineal \n",out.c_str());

        out="";
        cmd.data = comando[1];
        pub1_-> publish(cmd);
        for (auto byte : comando[1]) {
            out += std::to_string(static_cast<unsigned int>(byte)) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Estoy mandando %s de velocidad angular \n",out.c_str());
        
        
        //cmd.data = EnviaVelocidad(60,false,2);
        //pub1_-> publish(cmd);
        pub1_-> publish(cmd);
        out="";
        for (auto byte : comando[0]) {
            out += std::to_string(static_cast<unsigned int>(byte)) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Estoy mandando %s de velocidad lineal \n",out.c_str());

        cmd.data = comando[1];
        pub1_-> publish(cmd);
        out = "";
        for (auto byte : comando[1]) {
            out += std::to_string(static_cast<unsigned int>(byte)) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Estoy mandando %s de velocidad angular \n",out.c_str());
    }*/
    /*if (flag)
    {
        if(data[0]!=87)
        {
        RCLCPP_INFO(this->get_logger(), "Dentro de if en prueba(). El mensaje es: %i", data[0]); 
        }
    }*/
   
}

char Cuadriga::ASCII2Char(const int ascii_data)
{
    if(ascii_data >= 32 && ascii_data <=126) //Comprueba que el caracter sea imprimible
    {
        return static_cast<char>(ascii_data); //Devuelve un unico caracter
    }
    else
    {
        return '\0';
    }
}

int Cuadriga::Hex2Dec(const std::array<char,2> hex_value)
{
    int decimal_value = 0;
    std::string hex_string;
    for (int i = 0;i<hex_value.size();i++) {
        //RCLCPP_INFO(this->get_logger(), "Char: '%c' (ASCII: %d)", hex_value[i], static_cast<int>(hex_value[i]));  // Mostrar cada carácter y su valor ASCII
        hex_string.push_back(hex_value[i]);  // Agregar el carácter a la cadena
    }

    //RCLCPP_INFO(this->get_logger(), "Cadena Hexadecimal construida: %s", hex_string.c_str());

    std::stringstream ss;
    ss << std::hex << hex_string;  // Convertimos la cadena hexadecimal
    ss >> decimal_value;  // Leemos el valor decimal
    //RCLCPP_INFO(this->get_logger(), "Cadena decimal traducida: %i", decimal_value);

    return decimal_value;
}

std::array<float,2> Cuadriga::LLR2XYR(float Latitud, float Longitud, float Altitud)
{
    float L0 = M_PI*Latitud0/180;
    float T0 = M_PI*Longitud0/180;
    float L = M_PI*Latitud/180;
    float T = M_PI*Longitud/180;
    float h = Altitud;
    float a = 6378137;
    float b = 6356752.3142;
    float rx = (pow(a,2)/(sqrt(pow(a*cos(L0),2) + pow(b*sin(L0),2))) + h)*cos(L0);
    float ry = pow(b,2)/(sqrt(pow(a*cos(L0),2) + pow(b*sin(L0),2))) + h;

    float X = -1*(T0-T)*rx;
    float Y = (L - L0)*ry;
    return {X,Y};
    /*double x_utm, y_utm;
    float x_rel, y_rel;
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(static_cast<double>(Latitud), static_cast<double>(Longitud), zone, northp, x_utm, y_utm);
    x_rel = static_cast<float>(x_utm);
    y_rel = static_cast<float>(y_utm);
    RCLCPP_INFO(this->get_logger(), "ΔX: %.4f m | ΔY: %.4f m", x_rel, y_rel);

    return {x_rel, y_rel};*/
}

void Cuadriga::process_trayectoria (const cuadriga_interfaces::msg::GPXPath::SharedPtr msg)
{
    flag_origen = this->get_parameter("flag_origen").as_bool();
    if(flag_origen == true)
    {
        trayectoria.clear(); //limpia trayectoria anterior por si acaso
        for (const auto& punto : msg->waypoints)
        {
            // LLH → XY
            std::array<float, 2> punto_xy;
            punto_xy = LLR2XYR(punto.latitude, punto.longitude, punto.altitude);
            //punto_xy[0] = punto_xy[0] - origen[0];
            //punto_xy[1] = punto_xy[1] - origen[1];

            trayectoria.push_back(punto_xy);
            RCLCPP_INFO(this->get_logger(), "Punto LLH (%.6f, %.6f) → XY (%.2f, %.2f)",
                        punto.latitude, punto.longitude,
                        punto_xy[0], punto_xy[1]);
        }
    }
    RCLCPP_INFO(this->get_logger(), " Trayectoria cargada con %zu puntos.", trayectoria.size());

}

void Cuadriga::process_trayectoria_topic(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // Crear el vector donde guardaremos los arrays de dos elementos
    std::vector<std::array<float, 2>> coordenadas;

    // Asegurarnos de que el tamaño del mensaje sea múltiplo de 2
    if (msg->data.size() % 2 != 0)
    {
        RCLCPP_WARN(this->get_logger(), "El tamaño del array de datos no es múltiplo de 2.");
        return;
    }

    // Cargar los datos en el vector
    for (size_t i = 0; i < msg->data.size(); i += 2)
    {
        std::array<float, 2> punto = {msg->data[i], msg->data[i + 1]};
        coordenadas.push_back(punto);
    }

    // Opcional: imprimir el contenido de las coordenadas cargadas
    for (const auto& punto : coordenadas)
    {
        RCLCPP_INFO(this->get_logger(), "Punto cargado: (%.2f, %.2f)", punto[0], punto[1]);
    }
}

void Cuadriga::FollowTheCarrot()
{
    float look_ahead_distance = 0.5;
    float distancia;
    if (trayectoria.empty()) return;
 
    // Posición actual
    float x = posicion[0];
    float y = posicion[1];
   
    // Buscar el punto de seguimiento (look-ahead)
    std::array punto_objetivo_global = trayectoria.back(); // por defecto el último
   
    while(!trayectoria.empty())
    {
        float dx = trayectoria[0][0] - x;
        float dy = trayectoria[0][1] - y;
        distancia = std::sqrt(dx * dx + dy * dy);
       
        if(distancia < look_ahead_distance)
        {
            trayectoria.erase(trayectoria.begin());
        }
        else
        {
            punto_objetivo_global = trayectoria[0];
            break;
        }
    }
    if (trayectoria.empty())
    {
        v_lineal = 0;
        flag_origen = false; //Se reinicia para cuando se inicie otra trayectoria sin haber parado el nodo Cuadriga
        return;
    }else{

        //Calculo objetivo en coordenadas locales al robot
        std::array<float,2> punto_objetivo_local;
        punto_objetivo_local[0] = std::cos(orientacion_actual)*(punto_objetivo_global[0]-x) + std::sin(orientacion_actual)*(punto_objetivo_global[1] - y);
        punto_objetivo_local[1] = std::cos(orientacion_actual)*(punto_objetivo_global[1]-y) - std::sin(orientacion_actual)*(punto_objetivo_global[0] - x);
    
        RCLCPP_INFO(this->get_logger(), "Mi punto objetivo local es: (%f, %f)",punto_objetivo_local[0],punto_objetivo_local[1]);
        RCLCPP_INFO(this->get_logger(), "Mi punto objetivo global es: (%f, %f)",punto_objetivo_global[0],punto_objetivo_global[1]);

        // Cálculo del ángulo objetivo (relativo a la posición actual)
        angulo_objetivo = std::atan2(punto_objetivo_local[1], punto_objetivo_local[0]); // Radianes
        //angulo_objetivo = fmod(angulo_objetivo + 2*M_PI,2*M_PI); //Normalizar angulo entre [0, 2pi]
    
        //Cálculo de la velocidad lineal en función de la distancia al waypoint
    
        float k = 1; //Ganancia que le da peso a la distancia en el calculo
        //v_lineal = vmin +(vmax - vmin)*std::tanh(k*distancia);
        v_lineal = 0.4;
        //float alpha_error = angulo_objetivo - orientacion_actual;
        float alpha_error = angulo_objetivo;
        if(alpha_error > M_PI) alpha_error -= 2*M_PI;
        if(alpha_error<  -M_PI) alpha_error += 2*M_PI;

        //Cálculo de la velocidad angular en funcion de la lineal
        float L = 0.5;
        w = 0.6*alpha_error;
        RCLCPP_INFO(this->get_logger(), "Orientacion objetivo : (%f)",alpha_error);
        RCLCPP_INFO(this->get_logger(), "Velocidad lineal y angular : (%f, %f)",v_lineal,w);
    }
}

std::array<float,2> computeLookaheadPoint(
    float x, float y,
    const std::vector<std::array<float,2>>& path,
    float Ld)
{
    int n = path.size();
    if (n < 2) return path.back();

    // 1) Precalc arco acumulado
    std::vector<float> arcLen(n);
    arcLen[0] = 0.0f;
    for (int i = 0; i+1 < n; ++i) {
        float dx = path[i+1][0] - path[i][0];
        float dy = path[i+1][1] - path[i][1];
        arcLen[i+1] = arcLen[i] + std::hypot(dx, dy);
    }

    // 2) Proyección más cercana (línea infinita)
    float bestD2 = std::numeric_limits<float>::infinity();
    int   bestSeg = 0;
    float bestT   = 0.0f;
    for (int i = 0; i+1 < n; ++i) {
        auto& P = path[i];
        auto& Q = path[i+1];
        float vx = Q[0] - P[0], vy = Q[1] - P[1];
        float wx = x      - P[0], wy = y      - P[1];
        float denom = vx*vx + vy*vy;
        if (denom < 1e-6f) continue;

        // t sin clamp
        float t = (wx*vx + wy*vy) / denom;
        // punto proyectado
        float px = P[0] + t*vx;
        float py = P[1] + t*vy;
        float d2 = (px - x)*(px - x) + (py - y)*(py - y);
        if (d2 < bestD2) {
            bestD2 = d2;
            bestSeg = i;
            bestT   = t;
        }
    }

    // 3) Longitud de arco al punto proyectado
    auto& A = path[bestSeg];
    auto& B = path[bestSeg+1];
    float segLen = std::hypot(B[0]-A[0], B[1]-A[1]);
    float s0     = arcLen[bestSeg] + bestT * segLen;

    // 4) Longitud objetivo
    float sTarget = s0 + Ld;
    // clamp entre 0 y final
    if (sTarget < 0)            sTarget = 0;
    else if (sTarget > arcLen.back()) sTarget = arcLen.back();

    // 5) Busco el segmento donde cae sTarget
    int j = 0;
    while (j+1 < n && arcLen[j+1] < sTarget) ++j;

    float remain = sTarget - arcLen[j];
    // t* dentro del segmento j
    auto& C = path[j];
    auto& D = path[j+1];
    float vx2 = D[0]-C[0], vy2 = D[1]-C[1];
    float segLen2 = std::hypot(vx2, vy2);
    float tStar = (segLen2 < 1e-6f ? 0.0f : remain/segLen2);

    // 6) Look-ahead point interpolado
    return {
        C[0] + tStar * vx2,
        C[1] + tStar * vy2
    };
}


void Cuadriga::PurePursuit(std::vector<std::array<float,2>> trayectoria)
{
	const float Ld = 2.0f;               
    auto [xo, yo] = computeLookaheadPoint(
    posicion[0], posicion[1],
        path, Ld
    );
    float theta = orientacion_actual;
    float x= posicion[0];
    float y = posicion[1];
    float m_perp = -1 / tan(theta);
    float b_perp = y - m_perp * x;
    
    // Punto medio entre el robot y el objetivo
    float x_mid = (x + xo) / 2;
    float y_mid = (y + yo) / 2;

    // Cálculo de la pendiente de la recta que conecta el robot con el punto objetivo
    float slope_segment = (yo - y) / (xo - x);
    float m_mediatriz, b_mediatriz;

    // Evitar división por cero para segmentos verticales
    if (std::isinf(slope_segment)) {
        m_mediatriz = 0;
        b_mediatriz = y_mid;
    } else if (slope_segment == 0) {
        m_mediatriz = INFINITY;
        b_mediatriz = x_mid;
    } else {
        m_mediatriz = -1 / slope_segment;
        b_mediatriz = y_mid - m_mediatriz * x_mid;
    }

    // Resolver el sistema de ecuaciones para encontrar el centro de la circunferencia (xc, yc)
    float xc, yc;
    if (std::isinf(m_mediatriz)) {
        // Caso mediatriz vertical
        xc = x_mid;
        yc = m_perp * xc + b_perp;
    } else if (m_perp == m_mediatriz) {
        // Rectas paralelas, no hay solución
        return 0;
    } else {
        // Resolver el sistema general de ecuaciones
        xc = (b_mediatriz - b_perp) / (m_perp - m_mediatriz);
        yc = m_perp * xc + b_perp;
    }

    // Radio de la circunferencia
    float R = std::sqrt((xc - x) * (xc - x) + (yc - y) * (yc - y));
    
    // Determinar la dirección (izquierda o derecha)
    float signo = std::signbit(std::cos(theta) * (yo - y) - std::sin(theta) * (xo - x)) ? -1 : 1;

    // Velocidad angular para seguir la circunferencia
    float w = signo * v / R;
    return w;
}



void Cuadriga::process_query(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    flag = 1;
    data = msg -> data;
    //RCLCPP_INFO(this->get_logger(), "Mensaje: %d",data[0]);
    if(data[0]!= 87 && contador_rafaga <12) //Si no es una 'W', es un mensaje. if(data[0]!= 87)
    {
        if(data[0]==13) //Si es un 13, se ha terminado un comando nuevo
        {
            if(!palabra_actual.empty()) //Compruebo que la palabra del comando no este vacia
            {
                palabras.push_back(palabra_actual); //Meto la palabra en un vector que las almacena, para luego procesarlas
                palabra_actual.clear();
            }
        }else{
                palabra_actual.push_back(data[0]); //Si no es un 13, es un numero del comando, asi que lo meto junto
        }
        //Esto resulta en la siguiente estructura:
        //PALABRAS = [ [63,69],         //Aqui es donde catalogo en el switch con el 63='?', etc
        //             [65,70],
        //             [54,60]
        //           ]
        contador_rafaga = contador_rafaga + 1;
    }
    else{
        if(contador_rafaga > 11) //Mayor que cuatro para filtrar ruido que me llegan que no son rafagas de mensajes, un 0 ocasional, etc.
        {
            int separador_queries_comandos;
            int clasificador_queries;
            if(palabras[0][0]==0)
            {
                clasificador_queries = palabras[0][2];
                separador_queries_comandos = palabras[0][1];
            }else
            {
                clasificador_queries = palabras[0][1];
                separador_queries_comandos = palabras[0][0];
            }
            switch(separador_queries_comandos)
            {
                case 63:
                    bool flag_bateria = false;
                    RCLCPP_INFO(this->get_logger(), "He procesado un mensaje entero, y es un query:");
                    for (size_t i = 0; i < palabras.size(); ++i) {
                        std::string row_str = "[ ";
                        for (size_t j = 0; j < palabras[i].size(); ++j) {
                            row_str += std::to_string(palabras[i][j]);
                            if (j < palabras[i].size() - 1) {
                                row_str += ", ";
                            }
                        }
                        row_str += " ]";
                        RCLCPP_INFO(this->get_logger(), "Fila %zu: %s", i, row_str.c_str());
                    }

                    for(size_t i = 1; i<palabras.size();i++)
                    {
                        for(size_t j = 0; j<palabras[i].size();j++)
                        {
                            //RCLCPP_INFO(this->get_logger(), "El problema esta en ASCII2CHAR");
                            Caracter[j] = ASCII2Char(palabras[i][j]);
                            //RCLCPP_INFO(this->get_logger(), "Caracter: %c",Caracter[j]);
                        }
                        switch(clasificador_queries)
                        {
                            case 69:
                
                            if(!flag_bateria)
                                {
                                    RCLCPP_INFO(this->get_logger(), "El query es de voltaje de bateria y de controladora:");
                                    flag_bateria = true;
                                    //RCLCPP_INFO(this->get_logger(), "Contenido del vector: %c,%c",Caracter[0],Caracter[1]);
                                    //RCLCPP_INFO(this->get_logger(), "Tamaño del vector: %zu", Caracter.size());
                                    Voltaje_bateria = Hex2Dec(Caracter)*55.0/256;
                                    RCLCPP_INFO(this->get_logger(), "Voltaje de bateria: %f",Voltaje_bateria);
                                }else
                                {
                                    flag_bateria = false;
                                    Voltaje_controladora = Hex2Dec(Caracter)*28.5/256;
                                    RCLCPP_INFO(this->get_logger(), "Voltaje de controladora: %f",Voltaje_controladora);
                                }
                            //Aqui deben de estar los otros tipos de consultas: Consumo motores, y no se cual más
                        }
                    }
                break;
                //Aqui debe de estar un case para los echo de los comandos, separador_queries_comandos = 33
                //Por alguna razon el default me da error.
                default:
                    for (size_t i = 0; i < palabras.size(); ++i) {
                        std::string row_str = "[ ";
                        for (size_t j = 0; j < palabras[i].size(); ++j) {
                            row_str += std::to_string(palabras[i][j]);
                            if (j < palabras[i].size() - 1) {
                                row_str += ", ";
                            }
                        }
                        row_str += " ]";
                        RCLCPP_INFO(this->get_logger(), "Fila %zu: %s", i, row_str.c_str());
                    }
                    RCLCPP_INFO(this->get_logger(), "Hola6");
                break;
            }
        }
        palabras.clear();
        contador_rafaga = 0;
    }
    
    /*for(int i=0; i<segmentos.size(); i++)
    {
        for(int j=0; j<segmentos[i].size(); j++)
        {
            //Caracter = ASCII2Char(segmentos[i][j]);
            //Respuesta_HEX[i].push_back(Caracter); //Cojo los caracteres que pertenecen al mismo comando y los junto en una palabra. Esa palabra es el comando en hexadecimal que me ha mandado la controladora
        }
    }*/
    //RCLCPP_INFO(this->get_logger(), "He pasado los for anidados. El tamaño de Respuesta_HEX es: %i", tamaño);

    //Voltaje_bat=Hex2Dec(Respuesta_HEX[1])*55/256; //Formula de Manual_Usuario ax_1500. Voltaje bateria
    //Voltaje_int=Hex2Dec(Respuesta_HEX[2])*28.5/256; //Voltaje 12 V interno microcontroladora ax_1500

     
    //RCLCPP_INFO(this->get_logger(), "El voltaje del microcontrolador es: %f \n\n", Voltaje_int);
    
}

void Cuadriga::process_orientation_data(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    orientacion_actual = msg->x;
    orientacion_actual = fmod(orientacion_actual+2*M_PI,2*M_PI);
}

void Cuadriga::process_GPS_data(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    flag_origen = this -> get_parameter("flag_origen").as_bool();
    bool enable_navegacion = this -> get_parameter("enable_navegacion").as_bool();
    std::array<std::vector<u_char>,2> comando;
    std::string out="";
    Latitud = msg -> latitude;
    Longitud = msg -> longitude;
    Altitud = msg -> altitude;
    

    if(flag_origen==false)
    {
        Latitud0 = Latitud;
        Longitud0 = Longitud;
        Altitud0 = Altitud;
        //origen = LLR2XYR(Latitud, Longitud, Altitud);
        flag_origen=true;
        this ->set_parameter(rclcpp::Parameter("flag_origen",true));
        RCLCPP_INFO(this->get_logger(), "Mi coordenada geografica de origen es: (%f, %f)",Latitud0,Longitud0);
    }
    if(!trayectoria.empty() && enable_navegacion)
    {
        posicion = LLR2XYR(Latitud, Longitud, Altitud);
        //posicion[0] = posicion[0] - origen[0];
        //posicion[1] = posicion[1] - origen[1];
        RCLCPP_INFO(this->get_logger(), "Mi posicion es: (%f, %f)\n",posicion[0],posicion[1]);
        posicion_grafica.data = std::vector<float>(posicion.begin(),posicion.end());
        pub2_-> publish(posicion_grafica);


        FollowTheCarrot();
        float L = 0.5f;

        //normalizo entre 0 y 127 para la controladora
        
        float v_maxima = 1.2f;
        float v_norm = std::clamp(v_lineal / v_maxima, 0.0f, 1.0f);
        int v_pwm = static_cast<int>(v_norm * 127);

        RCLCPP_INFO(this->get_logger(), "v_lineal: %.2f | v_pwm: %d", v_lineal, v_pwm);

        
        float w_max = 6.0f; // máximo valor que consideras normal
        float w_sign;

        if(w<0)//Es al reves de lo intuitivo. A giros a la izquierda le corresponde una w positiva
        {
            w_sign = 1;
        }else{
            w_sign = -1;
        }
        float w_abs=std::abs(w);
        float w_norm = w_abs / w_max;
        int w_pwm = static_cast<int>(w_norm*w_sign*127);
        RCLCPP_INFO(this->get_logger(), "w: %.2f | w_pwm: %d", w, w_pwm);
        
        //Aqui tengo que llamar a las funciones de escritura y mandar v y w por channel 1 y channel 2.
        comando = EnviaVelocidad(v_pwm,w_pwm);
        cmd.data = comando[0];
        pub1_-> publish(cmd);
            out="";
            for (auto byte : comando[0]) {
                out += std::to_string(static_cast<unsigned int>(byte)) + " ";
            }
            RCLCPP_INFO(this->get_logger(), "Estoy mandando %s de velocidad lineal \n",out.c_str());

        cmd.data = comando[1];
        pub1_-> publish(cmd);
            out = "";
            for (auto byte : comando[1]) {
                out += std::to_string(static_cast<unsigned int>(byte)) + " ";
            }
            RCLCPP_INFO(this->get_logger(), "Estoy mandando %s de velocidad angular \n",out.c_str());
    }
}

void Cuadriga::process_joystick_data(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    bool enable_navegacion = this -> get_parameter("enable_navegacion").as_bool();
    if (!enable_navegacion)
    {
    std::string out="";
    std::vector<float> Ejes = msg -> axes;
    float v_pwm = Ejes[1]*127;
    float w_pwm = -Ejes[0]*127;
    std::array<std::vector<u_char>,2> comando= EnviaVelocidad(v_pwm, w_pwm);
    cmd.data = comando[1];
    pub1_-> publish(cmd);

    out="";
        for (auto byte : comando[0]) {
            out += std::to_string(static_cast<unsigned int>(byte)) + " ";
        }
        //RCLCPP_INFO(this->get_logger(), "Estoy mandando %s de velocidad lineal \n",out.c_str());

    cmd.data = comando[0];
    pub1_-> publish(cmd);

    out = "";
        for (auto byte : comando[1]) {
            out += std::to_string(static_cast<unsigned int>(byte)) + " ";
        }
        //RCLCPP_INFO(this->get_logger(), "Estoy mandando %s de velocidad angular \n",out.c_str());
    }
}

int main(int argc, char **argv){

    rclcpp::init(argc,argv);

    auto node = std::make_shared<Cuadriga>();

    rclcpp::Rate rate(1);

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        //node -> prueba();
        RCLCPP_INFO(node ->get_logger(), "................");
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
