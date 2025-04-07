import socket
import struct
import math
import serial
import pyudev
from time import time
import wrapper_pb2 as wr


def find_stm32_port():
    context = pyudev.Context()
    for device in context.list_devices(subsystem='tty'):
        if 'ID_VENDOR_ID' in device and 'ID_MODEL_ID' in device:
            vendor_id = device.get('ID_VENDOR_ID')
            model_id = device.get('ID_MODEL_ID')
            if vendor_id == '0483' and model_id == '5740':
                return device.device_node
    raise RuntimeError("STM32 Virtual COM Port não encontrado!")


def setup_serial():
    port = find_stm32_port()
    ser = serial.Serial(port, 115200, timeout=1)
    print(f"✅ Serial conectado em {ser.port}")
    return ser


class FiraClient:

    def __init__(self, vision_ip="224.5.23.2", vision_port=10015):
        """
        Init FiraClient object to receive SSL-Vision packets and control STM32.
        """
        self.vision_ip = vision_ip
        self.vision_port = vision_port

        self.vision_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.vision_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        self.vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.vision_sock.setsockopt(
            socket.IPPROTO_IP,
            socket.IP_ADD_MEMBERSHIP,
            struct.pack("=4sl", socket.inet_aton(self.vision_ip), socket.INADDR_ANY)
        )
        self.vision_sock.bind((self.vision_ip, self.vision_port))

        self.serial = setup_serial()
        self.robot_id = 0  

    def angle_to_ball(self, robot_x, robot_y, ball_x, ball_y):
        dx = ball_x - robot_x
        dy = ball_y - robot_y
        return math.atan2(dy, dx)

    def send_motor_commands(self, vl, vr):
        try:
            data = struct.pack('iff', self.robot_id, vl, vr)
            self.serial.write(data)
            print(f"📤 Enviado para STM32 → VL={vl:.2f}, VR={vr:.2f}")
        except Exception as e:
            print(f"❌ Erro ao enviar comandos: {e}")

    def gotoball(self, robot, ball, angle_threshold=0.1):
        """
        Computa e envia comandos para o robô diferencial ir até a bola.
        """
        angle_to_ball = self.angle_to_ball(robot.x, robot.y, ball.x, ball.y)
        angle_diff = angle_to_ball - robot.orientation
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        Kp_ang = 0.3
        Kp_lin = 0.2
        max_speed = 1.0

        distance = math.hypot(ball.x - robot.x, ball.y - robot.y)

        if abs(angle_diff) > angle_threshold:
            v_linear = 0.0
        else:
            v_linear = Kp_lin * distance
            v_linear = min(v_linear, max_speed)

        v_angular = Kp_ang * angle_diff
        v_angular = max(min(v_angular, 1.0), -1.0)

        vl = v_linear - v_angular
        vr = v_linear + v_angular

        self.send_motor_commands(vl, vr)

        print(f"  → Angle to Ball: {math.degrees(angle_to_ball):.2f}°")
        print(f"  → Angle Diff   : {math.degrees(angle_diff):.2f}°")
        print(f"  → Comandos     : VL={vl:.2f}, VR={vr:.2f}")

    def receive_frame(self) -> None:
        """Recebe pacote do SSL-Vision e executa controle GoToBall."""
        data = None
        while True:
            try:
                data, _ = self.vision_sock.recvfrom(8192)
                break
            except Exception as e:
                print(f"[Socket Error] {e}")
                continue

        if data:
            try:
                wrapper = wr.SSL_WrapperPacket()
                wrapper.ParseFromString(data)

                ball = None

                # 🎯 Detections
                if wrapper.HasField("detection"):
                    detection = wrapper.detection

                    # 🟠 Bola
                    if detection.balls:
                        ball = detection.balls[0]
                        print(f"Ball Position: x={ball.x:.2f}, y={ball.y:.2f}")

                    # 🔵 Robôs azuis
                    for robot in detection.robots_blue:
                        print(f"Blue Robot ID {robot.robot_id}: x={robot.x:.2f}, y={robot.y:.2f}, orientation={math.degrees(robot.orientation):.2f}°")
                        if ball and robot.robot_id == 7:
                            self.gotoball(robot, ball)

                # 📐 Geometria do campo
                if wrapper.HasField("geometry") and wrapper.geometry.HasField("field"):
                    field = wrapper.geometry.field
                    print("[FIELD DIMENSIONS]")
                    print(f"  Length       : {field.field_length}")
                    print(f"  Width        : {field.field_width}")
                    print(f"  Goal Width   : {field.goal_width}")
                    print(f"  Goal Depth   : {field.goal_depth}")
                    print(f"  Boundary     : {field.boundary_width}")
                    print(f"  Penalty Area : {field.penalty_area_width} x {field.penalty_area_depth}")

            except Exception as e:
                print(f"[Protobuf Error] {e}")


# 🟢 Loop principal
if __name__ == "__main__":
    a = FiraClient()
    while True:
        a.receive_frame()
