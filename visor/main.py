# Importações de bibliotecas necessárias para o funcionamento do código
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
import threading
import pygame
import cv2
import base64
import numpy as np
from PIL import Image
import io
import queue
import time
import collections

pygame.init()

screen = pygame.display.set_mode((1920, 1080))
pygame.display.set_caption("Teleoperação do Robô")

arrow_up = pygame.image.load('arrow_up.png').convert_alpha()
arrow_down = pygame.image.load('arrow_down.png').convert_alpha()
arrow_left = pygame.image.load('arrow_left.png').convert_alpha()
arrow_right = pygame.image.load('arrow_right.png').convert_alpha()

button_rects = {
    "kill": pygame.Rect(10, 10, 100, 50),
    "Frente": pygame.Rect(910, 10, 100, 50),
    "Tras": pygame.Rect(910, 70, 100, 50),
    "left": pygame.Rect(800, 40, 100, 50),
    "right": pygame.Rect(1020, 40, 100, 50),
}

arrow_up = pygame.transform.scale(arrow_up, (button_rects["Frente"].width, button_rects["Frente"].height))
arrow_down = pygame.transform.scale(arrow_down, (button_rects["Tras"].width, button_rects["Tras"].height))
arrow_left = pygame.transform.scale(arrow_left, (button_rects["left"].width, button_rects["left"].height))
arrow_right = pygame.transform.scale(arrow_right, (button_rects["right"].width, button_rects["right"].height))

def draw_buttons():
    for label, rect in button_rects.items():
        pygame.draw.rect(screen, (200, 200, 200), rect, border_radius=10)
    screen.blit(arrow_up, (button_rects["Frente"].topleft))
    screen.blit(arrow_down, (button_rects["Tras"].topleft))
    screen.blit(arrow_left, (button_rects["left"].topleft))
    screen.blit(arrow_right, (button_rects["right"].topleft))
    kill_text = font.render("Kill Switch", True, (0, 0, 0))
    screen.blit(kill_text, (button_rects["kill"].centerx - kill_text.get_width() // 2, button_rects["kill"].centery - kill_text.get_height() // 2))

ui_queue = queue.Queue(maxsize=10)

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.emergency_client = self.create_client(Empty, 'emergency_stop') 
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.killed = False
        self.safety_distance = 0.35
        self.front_clear = True
        self.back_clear = True

    def lidar_callback(self, msg):
        num_ranges = len(msg.ranges)
        sector_size = num_ranges // 12

        front_left_indices = range(num_ranges - sector_size, num_ranges)
        front_right_indices = range(0, sector_size)
        back_indices = range(5 * sector_size, 7 * sector_size)

        front_ranges = [msg.ranges[i] for i in front_left_indices if 0.01 < msg.ranges[i] < 100.0] + \
                       [msg.ranges[i] for i in front_right_indices if 0.01 < msg.ranges[i] < 100.0]
        back_ranges = [msg.ranges[i] for i in back_indices if 0.01 < msg.ranges[i] < 100.0]

        self.front_clear = not any(r < self.safety_distance for r in front_ranges)
        self.back_clear = not any(r < self.safety_distance for r in back_ranges)

        if not self.front_clear and self.linear_speed > 0:
            self.stop_robot()
        elif not self.back_clear and self.linear_speed < 0:
            self.stop_robot()


    def move_robot(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher.publish(msg)
        print(f"Movendo: velocidade linear={self.linear_speed} m/s, velocidade angular={self.angular_speed} rad/s")

    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        print("Parando o robô.")

    
    def increase_linear_speed(self):
        if self.front_clear:
            self.linear_speed = 0.1
            self.move_robot()
        else:
            self.stop_robot()

    
    def decrease_linear_speed(self):
        if self.back_clear:
            self.linear_speed = -0.1
            self.move_robot()
        else:
            self.stop_robot()

    
    def increase_angular_speed(self):
        self.angular_speed = 0.4
        self.move_robot()

    def decrease_angular_speed(self):
        self.angular_speed = -0.4
        self.move_robot()

    def send_emergency_stop(self):
        if self.emergency_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            future = self.emergency_client.call_async(req)
            future.add_done_callback(self.emergency_stop_callback)
        else:
            print('Serviço de emergência não disponível, DESLIGUE O ROBÔ')


    def emergency_stop_callback(self, future):
        try:
            future.result()
            print('Sinal de parada de emergência enviado com sucesso, processo do robô terminado.')
        except Exception as e:
            print(f'Falha ao chamar o serviço de parada de emergência: {e}, RETIRE A BATERIA DO ROBÔ!!!')

    def kill_switch(self):
        print("Parada de emergência forçada do processo.")
        self.stop_robot()
        self.send_emergency_stop()
        rclpy.shutdown()

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription

    
    def listener_callback(self, msg):
        timestamp, jpg_as_text = msg.data.split('|', 1)
        timestamp = float(timestamp)
        current_time = time.time()
        latency = current_time - timestamp

        jpg_original = base64.b64decode(jpg_as_text)
        jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
        img = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)
        
        if img is not None:
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(img_rgb)
            img_bytes = io.BytesIO()
            pil_image.save(img_bytes, format="JPEG")
            img_bytes.seek(0)
            if not ui_queue.full():
                ui_queue.put((img_bytes, latency))
        else:
            self.get_logger().error('Não foi possível decodificar a imagem')

def init_ros_nodes():
    rclpy.init()
    robot_controller = RobotController()
    listener = Listener()

    executor_thread = threading.Thread(target=spin_nodes, args=(robot_controller, listener), daemon=True)
    executor_thread.start()

    return robot_controller, listener

def spin_nodes(robot_controller, listener):
    while rclpy.ok():
        rclpy.spin_once(robot_controller, timeout_sec=0.01)
        rclpy.spin_once(listener, timeout_sec=0.01)

robot_controller, listener = init_ros_nodes()

def main():
    running = True
    clock = pygame.time.Clock()

    global font
    font = pygame.font.Font(None, 36)
    
    key_mapping = {
        pygame.K_w: robot_controller.increase_linear_speed,
        pygame.K_s: robot_controller.decrease_linear_speed,
        pygame.K_a: robot_controller.increase_angular_speed,
        pygame.K_d: robot_controller.decrease_angular_speed,
        pygame.K_SPACE: robot_controller.stop_robot,
        pygame.K_q: lambda: (pygame.quit(), rclpy.shutdown()),
        pygame.K_b: robot_controller.kill_switch
    }
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                action = key_mapping.get(event.key)
                if action:
                    action()
            elif event.type == pygame.KEYUP:
                if event.key in key_mapping:
                    robot_controller.stop_robot()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = event.pos
                if button_rects["kill"].collidepoint(mouse_pos):
                    robot_controller.kill_switch()
                    running = False
                elif button_rects["Frente"].collidepoint(mouse_pos):
                    robot_controller.increase_linear_speed()
                elif button_rects["Tras"].collidepoint(mouse_pos):
                    robot_controller.decrease_linear_speed()
                elif button_rects["left"].collidepoint(mouse_pos):
                    robot_controller.increase_angular_speed()
                elif button_rects["right"].collidepoint(mouse_pos):
                    robot_controller.decrease_angular_speed()

    
        latency_values = collections.deque(maxlen=10) 

        if not ui_queue.empty():
            img_bytes, latency = ui_queue.get()
            img_np = np.array(Image.open(img_bytes))
            img_np = np.rot90(img_np, 1)
            img_np = cv2.resize(img_np, (1080, 1920))
            img_surface = pygame.surfarray.make_surface(img_np)
            screen.blit(img_surface, (0, 0))

            latency_ms = latency * 1000
            latency_values.append(latency_ms)
            

            avg_latency = sum(latency_values) / len(latency_values)
            latency_text = font.render(f"Latência: {avg_latency:.2f} ms", True, (255, 255, 255))
            screen.blit(latency_text, (1600, 10))  
        
        draw_buttons()

        speed_text = font.render(f"Velocidade Linear: {robot_controller.linear_speed} m/s, Angular: {robot_controller.angular_speed} rad/s", True, (255, 255, 255))
        screen.blit(speed_text, (10, 100))

        pygame.display.flip()
        
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
