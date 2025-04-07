import numpy as np
import pygame
import struct
import serial
import time
import pyudev
import threading
import queue


def find_stm32_port():
    context = pyudev.Context()
    for device in context.list_devices(subsystem='tty'):
        if 'ID_VENDOR_ID' in device and 'ID_MODEL_ID' in device:
            vendor_id = device.get('ID_VENDOR_ID')
            model_id = device.get('ID_MODEL_ID')

            if vendor_id == '0483' and model_id == '5740':
                return device.device_node

    raise RuntimeError("STM32 Virtual COM Port não encontrado!")


class KeyboardControl:
    def __init__(self):
        self.robot_id = 0
        self.MAX_SPEED = 1.0
        self.vl = 0.0
        self.vr = 0.0
        self.last_x_key_state = False
        self.running = True

        self.ser = serial.Serial(find_stm32_port(), 115200, timeout=0.1)
        print(f"✅ Conectado ao STM32 na porta {self.ser.port}")

        self.feedback_queue = queue.Queue()
        self.feedback_lines = []
        self.transmission_message = ""
        self.last_link_quality = 0.0

        self.reader_thread = threading.Thread(target=self.read_feedback_loop, daemon=True)
        self.reader_thread.start()

    def send_data(self):
        try:
            data = struct.pack('iff', self.robot_id, self.vl, self.vr)
            self.ser.write(data)
            self.transmission_message = "data sent successfully"
        except Exception as e:
            self.transmission_message = f"data transmission failed: {e}"

    def read_feedback_loop(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line and "data sent successfully" not in line and "data transmission failed" not in line:
                        self.feedback_queue.put(line)
            except Exception as e:
                self.feedback_queue.put(f"Erro leitura: {e}")

    def process_input(self, keys):
        y = 0
        x = 0

        if keys[pygame.K_w]:
            y += 1
        if keys[pygame.K_s]:
            y -= 1
        if keys[pygame.K_d]:
            x += 1
        if keys[pygame.K_a]:
            x -= 1

        y = float(y)
        x = float(x)

        self.vl = (y + x) * self.MAX_SPEED
        self.vr = (y - x) * self.MAX_SPEED

        x_key_state = keys[pygame.K_x]
        if x_key_state and not self.last_x_key_state:
            self.robot_id = (self.robot_id + 1) % 4
            print(f"🚀 Robô alterado para ID: {self.robot_id}")

        self.last_x_key_state = x_key_state

        self.send_data()

    def draw_signal_bars(self, screen, quality_percent):
        """Desenha barras de sinal estilo Wi-Fi baseado na qualidade."""
        bars = int((quality_percent / 100) * 5)

        base_x = 550
        base_y = 10
        bar_width = 6
        spacing = 4

        for i in range(5):
            height = (i + 1) * 8
            x = base_x + i * (bar_width + spacing)
            y = base_y + (40 - height)

            color = (0, 255, 0) if i < bars else (60, 60, 60)
            pygame.draw.rect(screen, color, (x, y, bar_width, height))

    def draw_feedback(self, screen, font):
        while not self.feedback_queue.empty():
            line = self.feedback_queue.get()
            sanitized_line = line.replace('\x00', '').strip()

            # ⛔️ Ignorar linhas redundantes
            if "data sent successfully" in sanitized_line.lower() or "data transmission failed" in sanitized_line.lower():
                continue

            if sanitized_line:
                self.feedback_lines.append(sanitized_line)
                if len(self.feedback_lines) > 5:
                    self.feedback_lines.pop(0)

        # Renderizar feedback
        for i, line in enumerate(reversed(self.feedback_lines)):
            try:
                text = font.render(line, True, (0, 255, 0))
                screen.blit(text, (10, 140 + i * 20))

                if "Link:" in line:
                    try:
                        link_str = line.split("Link:")[1].strip().replace("%", "")
                        self.last_link_quality = float(link_str)
                    except ValueError:
                        pass

            except Exception as e:
                print(f"⚠️ Erro ao renderizar texto: {line} -> {e}")

        # Desenhar as barrinhas
        self.draw_signal_bars(screen, self.last_link_quality)


    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((600, 300))
        pygame.display.set_caption("Controle de Robô por Teclado")

        font = pygame.font.SysFont("consolas", 18)

        try:
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return

                screen.fill((0, 0, 0))

                keys = pygame.key.get_pressed()
                self.process_input(keys)

                # Mostrar informações de controle
                status = f"ID: {self.robot_id} | VL: {self.vl:.2f} | VR: {self.vr:.2f}"
                screen.blit(font.render(status, True, (255, 255, 255)), (10, 10))

                # Mensagem de transmissão
                color = (0, 255, 0) if "successfully" in self.transmission_message else (255, 0, 0)
                screen.blit(font.render(self.transmission_message, True, color), (10, 70))

                # Instruções
                screen.blit(font.render("WASD para mover | X para trocar ID", True, (200, 200, 200)), (10, 40))

                # Feedback
                screen.blit(font.render("Mensagens do VSSS:", True, (255, 255, 0)), (10, 110))
                self.draw_feedback(screen, font)

                pygame.display.flip()

        except KeyboardInterrupt:
            print("\n🛑 Finalizando...")
        finally:
            self.running = False
            self.reader_thread.join(timeout=1)
            pygame.quit()
            self.ser.close()


if __name__ == "__main__":
    controller = KeyboardControl()
    controller.run()
