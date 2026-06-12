#!/usr/bin/env python3
"""
Ripple Laptop Controller - macOS version

Sends control commands from Xbox controller to Pi over TCP.
Displays SRT video stream from Pi.
Uses pygame for cross-platform controller support.

Run with --no-pi to test controller input without a Pi connection.
"""

import socket
import time
import sys
import logging
import threading
import subprocess
import protocol

try:
    import pygame
except ImportError:
    print("ERROR: pygame required for Xbox controller support")
    print("Install: pip install pygame")
    sys.exit(1)

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)

PI_IP = "192.168.100.10"
TCP_PORT = 5000
SRT_PORT = 9000

FREQUENCY_MIN = 1.5
FREQUENCY_MAX = 3.0
AMPLITUDE_MIN = 10.0
AMPLITUDE_MAX = 40.0
OFFSET_MIN = -40.0
OFFSET_MAX = 0.0
WAVELENGTH_MIN = 0.5
WAVELENGTH_MAX = 2.0

# Xbox controller axis/button mappings (pygame)
AXIS_RT = 5       # Right trigger
AXIS_LT = 4       # Left trigger
AXIS_LEFT_Y = 1   # Left stick Y
AXIS_LEFT_X = 0   # Left stick X
BTN_A = 0
BTN_B = 1
BTN_Y = 3

TRIGGER_DEADZONE = 0.05
STICK_DEADZONE = 0.1


class LaptopController:
    def __init__(self, no_pi=False):
        self.no_pi = no_pi
        self.sock = None
        self.joystick = None

        self.frequency = 1.5
        self.amplitude = AMPLITUDE_MIN
        self.offset = -25.0
        self.wavelength = 1.0
        self.mirror_mode = True
        self.emergency_stop = True
        self.vertical_mode = False
        self.vertical_amplitude = 0.0

        self.video_process = None
        self.connected = False

        self.btn_a_prev = False
        self.btn_b_prev = False
        self.btn_y_prev = False

    def connect_controller(self):
        pygame.init()
        pygame.joystick.init()

        while True:
            if pygame.joystick.get_count() == 0:
                logging.warning("Xbox controller not found. Plug in controller and press Enter...")
                input()
                pygame.joystick.quit()
                pygame.joystick.init()
                continue

            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            # Seed prev state to avoid spurious first-frame edge
            pygame.event.pump()
            self.btn_a_prev = bool(self.joystick.get_button(BTN_A))
            self.btn_b_prev = bool(self.joystick.get_button(BTN_B))
            self.btn_y_prev = bool(self.joystick.get_button(BTN_Y))
            logging.info(f"Xbox controller connected: {self.joystick.get_name()}")
            return

    def connect_tcp(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((PI_IP, TCP_PORT))
                self.sock.setblocking(False)
                logging.info(f"Connected to Pi at {PI_IP}:{TCP_PORT}")
                self.connected = True
                return
            except Exception as e:
                logging.warning(f"Failed to connect to Pi: {e}. Retrying in 2s...")
                time.sleep(2)

    def send_command(self, cmd_id, value):
        if self.no_pi:
            logging.info(f"[DRY RUN] {protocol.command_to_string(cmd_id)} {value:.3f}")
            return True

        if not self.sock or not self.connected:
            return False

        try:
            message = protocol.encode_command(cmd_id, value)
            self.sock.send(message)

            import select
            ready = select.select([self.sock], [], [], 0.05)
            if ready[0]:
                ack_data = self.sock.recv(protocol.ACK_SIZE)
                result = protocol.decode_ack(ack_data)
                if result:
                    return result[1] == protocol.STATUS_OK
            return True

        except Exception as e:
            logging.error(f"Send error: {e}")
            self.connected = False
            return False

    def map_value(self, value, in_min, in_max, out_min, out_max):
        normalized = max(0.0, min(1.0, (value - in_min) / (in_max - in_min)))
        return out_min + normalized * (out_max - out_min)

    def apply_deadzone(self, value):
        if abs(value) < STICK_DEADZONE:
            return 0.0
        return value

    def start_video_player(self):
        srt_url = f"srt://{PI_IP}:{SRT_PORT}"

        try:
            cmd = [
                "ffplay", "-fflags", "nobuffer", "-flags", "low_delay",
                "-framedrop", "-strict", "experimental",
                "-window_title", "Ripple ROV", srt_url,
            ]
            self.video_process = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            logging.info(f"Video player started: {srt_url}")
            return True
        except FileNotFoundError:
            logging.warning("ffplay not found, trying VLC...")

        try:
            cmd = ["vlc", "--network-caching=100", "--no-video-title-show", srt_url]
            self.video_process = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            logging.info(f"VLC started: {srt_url}")
            return True
        except FileNotFoundError:
            logging.error("No SRT player found (ffplay or VLC required)")
            logging.error("Install: brew install ffmpeg  (or brew install --cask vlc)")
            return False

    def process_controller_state(self):
        pygame.event.pump()

        # Right trigger - frequency
        rt_value = (self.joystick.get_axis(AXIS_RT) + 1.0) / 2.0
        if rt_value < TRIGGER_DEADZONE:
            if self.frequency != FREQUENCY_MIN:
                self.frequency = FREQUENCY_MIN
                self.send_command(protocol.CMD_FREQ, self.frequency)
        else:
            new_freq = self.map_value(rt_value, TRIGGER_DEADZONE, 1.0, FREQUENCY_MIN, FREQUENCY_MAX)
            if abs(new_freq - self.frequency) > 0.05:
                self.frequency = new_freq
                self.send_command(protocol.CMD_FREQ, self.frequency)
                logging.info(f"Frequency: {self.frequency:.2f} Hz")

        # Left trigger - amplitude
        lt_value = (self.joystick.get_axis(AXIS_LT) + 1.0) / 2.0
        if lt_value < TRIGGER_DEADZONE:
            if self.amplitude != AMPLITUDE_MIN:
                self.amplitude = AMPLITUDE_MIN
                self.send_command(protocol.CMD_AMP, self.amplitude)
        else:
            new_amp = self.map_value(lt_value, TRIGGER_DEADZONE, 1.0, AMPLITUDE_MIN, AMPLITUDE_MAX)
            if abs(new_amp - self.amplitude) > 1.0:
                self.amplitude = new_amp
                self.send_command(protocol.CMD_AMP, self.amplitude)
                logging.info(f"Amplitude: {self.amplitude:.1f} deg")

        # Left stick Y - vertical amplitude (vertical mode) or body offset (forward mode)
        left_y = -self.joystick.get_axis(AXIS_LEFT_Y)  # invert Y
        left_y = self.apply_deadzone(left_y)
        if self.vertical_mode:
            new_vamp = self.map_value(left_y, -1, 1, -AMPLITUDE_MAX, AMPLITUDE_MAX)
            if abs(new_vamp - self.vertical_amplitude) > 1.0:
                self.vertical_amplitude = new_vamp
                self.send_command(protocol.CMD_VAMP, self.vertical_amplitude)
                logging.info(f"Vertical amplitude: {self.vertical_amplitude:.1f} deg")
        else:
            if abs(left_y) > 0:
                new_offset = self.map_value(left_y, -1, 1, OFFSET_MIN, OFFSET_MAX)
                if abs(new_offset - self.offset) > 1.0:
                    self.offset = new_offset
                    self.send_command(protocol.CMD_OFFSET, self.offset)
                    logging.info(f"Offset: {self.offset:.1f} deg")

        # Left stick X - wavelength
        left_x = self.joystick.get_axis(AXIS_LEFT_X)
        left_x = self.apply_deadzone(left_x)
        if abs(left_x) > 0:
            new_wave = self.map_value(left_x, -1, 1, WAVELENGTH_MIN, WAVELENGTH_MAX)
            if abs(new_wave - self.wavelength) > 0.05:
                self.wavelength = new_wave
                self.send_command(protocol.CMD_WAVE, self.wavelength)
                logging.info(f"Wavelength: {self.wavelength:.2f}")

        # A button - emergency stop
        btn_a = self.joystick.get_button(BTN_A)
        if btn_a and not self.btn_a_prev:
            self.emergency_stop = not self.emergency_stop
            self.send_command(protocol.CMD_ESTOP, 1.0 if self.emergency_stop else 0.0)
            if self.emergency_stop:
                logging.warning("EMERGENCY STOP ACTIVATED")
            else:
                logging.info("Resuming operation")
        self.btn_a_prev = btn_a

        # B button - mirror mode
        btn_b = self.joystick.get_button(BTN_B)
        if btn_b and not self.btn_b_prev:
            self.mirror_mode = not self.mirror_mode
            self.send_command(protocol.CMD_MIRROR, 1.0 if self.mirror_mode else 0.0)
            logging.info(f"Mirror mode: {'ON' if self.mirror_mode else 'OFF'}")
        self.btn_b_prev = btn_b

        # Y button - vertical gait mode
        btn_y = self.joystick.get_button(BTN_Y)
        if btn_y and not self.btn_y_prev:
            self.vertical_mode = not self.vertical_mode
            self.send_command(protocol.CMD_VMODE, 1.0 if self.vertical_mode else 0.0)
            if not self.vertical_mode:
                self.vertical_amplitude = 0.0
                self.send_command(protocol.CMD_VAMP, 0.0)
            logging.info(f"Vertical mode: {'ON' if self.vertical_mode else 'OFF'}")
        self.btn_y_prev = btn_y

    def run(self):
        logging.info("Ripple laptop controller starting...")

        self.connect_controller()

        if self.no_pi:
            logging.info("Running in dry-run mode (no Pi connection)")
        else:
            self.connect_tcp()
            video_thread = threading.Thread(target=self.start_video_player, daemon=True)
            video_thread.start()

        time.sleep(1)
        logging.info("Controller ready - press A to start robot")
        logging.info("Press Ctrl+C to exit")

        clock = pygame.time.Clock()
        last_heartbeat = time.time()

        try:
            while True:
                self.process_controller_state()

                if time.time() - last_heartbeat > 1.0:
                    self.send_command(protocol.CMD_FREQ, self.frequency)
                    last_heartbeat = time.time()

                if not self.no_pi and not self.connected:
                    logging.warning("Lost connection to Pi. Reconnecting...")
                    self.connect_tcp()

                clock.tick(60)

        except KeyboardInterrupt:
            logging.info("Shutting down...")

        if not self.emergency_stop:
            self.send_command(protocol.CMD_ESTOP, 1.0)

        if self.video_process:
            self.video_process.terminate()

        if self.sock:
            self.sock.close()

        pygame.quit()


if __name__ == "__main__":
    no_pi = "--no-pi" in sys.argv
    controller = LaptopController(no_pi=no_pi)
    controller.run()