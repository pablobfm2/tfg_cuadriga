from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, LogInfo


def generate_launch_description():
    
    # Nodo cuadriga (sin parámetros)
        Cuadriga = Node(
            package='cuadriga',
            executable='cuadriga',
            name='cuadriga',
            namespace='Cuadriga',
            output='screen'
        )
    #Nodo FixPosition
        FixPosition = Node(
            package='fixposition_driver_ros2',
            executable='fixposition_driver_ros2_exec',
            name='fixposition_driver_ros2',
            output='screen',
            respawn=True,
            respawn_delay='5',
            parameters=[{
                "fp_output.formats": ["ODOMETRY", "LLH", "RAWIMU", "CORRIMU", "TF"],
                "fp_output.type": "tcp",
                "fp_output.port": "21000",
                "fp_output.ip": "192.168.2.113",# change to VRTK2's IP address in the network
                "fp_output.rate": 200,
                "fp_output.reconnect_delay": 5.0, # wait time in [s] until retry connection
                "customer_input.speed_topic": "/fixposition/speed"  
                }]
            )

    # Nodo serial_bridge
        serial_bridge = Node(
            package='serial_driver',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{"device_name": "/dev/ttyUSB0",
                        "baud_rate": 9600,
                        "flow_control": "none",
                        "parity": "even",
                        "stop_bits": "1"
                        }]
        )

        log_before_configure = LogInfo(msg="Configurando serial_bridge...")

        # Comando para configurar el puerto serie
        serial_config = ExecuteProcess(
            cmd=['sudo', 'stty', '-F', '/dev/ttyUSB0', '9600', 'cs7', 'parenb','-parodd','-cstopb' ],
            output='screen'
        )

        # Comando para configurar el serial_bridge
        set_configure = ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/serial_bridge', 'configure'],
            output='screen'
        )

        log_before_activate = LogInfo(msg="Activando serial_bridge...")

        # Comando para activar el serial_bridge
        set_activate = ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/serial_bridge', 'activate'],
            output='screen'
        )

        wait_before_configure = TimerAction(
                period = 2.0,
                actions=[log_before_configure, set_configure]
        )

        wait_before_activate = TimerAction(
                period = 4.0,
               actions=[log_before_activate, set_activate]
        )

        wait_before_launch = TimerAction(
                period = 5.0,
               actions=[Cuadriga, serial_config, FixPosition]
        )
        
        #wait_before_config = TimerAction(
         #       period = 6.0,
          #     actions=[serial_config]
        #)
        return LaunchDescription([
            serial_bridge,
            wait_before_configure,
            wait_before_activate,
            wait_before_launch
            #wait_before_config
        ])