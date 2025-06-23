#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cuadriga_interfaces.msg import GPXPath
from sensor_msgs.msg import NavSatFix
import xml.etree.ElementTree as ET
import os

class GPXLoaderNode(Node):
    def __init__(self):
        super().__init__('gpx_loader')

        self.pub_ = self.create_publisher(GPXPath, '/trayectoria_gpx', 10)

        # Ruta al archivo GPX (ajústala a tu caso)
        self.gpx_file_path = os.path.expanduser("~/tfgcuadriga/src/gpx_loader/waypoints.gpx")

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.sent = False

    def timer_callback(self):
        if self.sent:
            return

        msg = self.leer_gpx(self.gpx_file_path)
        if msg is not None:
            self.pub_.publish(msg)
            self.get_logger().info(f" Publicados {len(msg.waypoints)} puntos desde GPX")
        else:
            self.get_logger().error(" No se pudo cargar el archivo GPX.")
        self.sent = True

    def leer_gpx(self, file_path):
        if not os.path.exists(file_path):
            self.get_logger().error(f"Archivo GPX no encontrado: {file_path}")
            return None

        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            ns = {'default': 'http://www.topografix.com/GPX/1/1'}  # GPX default namespace

            msg = GPXPath()

            # Buscar todos los puntos dentro de <trkpt>
            for trkpt in root.findall('.//default:trkpt', ns):
                lat = float(trkpt.attrib['lat'])
                lon = float(trkpt.attrib['lon'])
                ele_elem = trkpt.find('default:ele', ns)
                alt = float(ele_elem.text) if ele_elem is not None else 0.0

                punto = NavSatFix()
                punto.latitude = lat
                punto.longitude = lon
                punto.altitude = alt
                msg.waypoints.append(punto)

            return msg
        except Exception as e:
            self.get_logger().error(f"Error al leer GPX: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = GPXLoaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
