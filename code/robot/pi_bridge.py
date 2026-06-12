#!/usr/bin/env python3
"""
Ripple Pi Bridge - TCP control server + Xbox controller fallback

Runs on Raspberry Pi, receives TCP commands from laptop and forwards to Arduino.
Xbox controller provides backup control when TCP connection is down.
"""

import serial
import time
import sys
import logging
import socket
import select
from evdev import InputDevice, ecodes
import protocol

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('/tmp/ripple_bridge.log')
    ]
)

ARDUINO_BAUDRATE = 115200
ARDUINO_PORT = '/dev/ttyACM0'

TCP_PORT = 5000
TCP_TIMEOUT = 2.0

FREQUENCY_MIN = 1.5
FREQUENCY_MAX = 3.0
AMPLITUDE_MIN = 10.0
AMPLITUDE_MAX = 40.0
OFFSET_MIN = -40.0
OFFSET_MAX = 0.0
WAVELENGTH_MIN = 0.5
WAVELENGTH_MAX = 2.0

# Xbox controller event codes
ABS_RZ = 5
ABS_Z = 2
ABS_Y = 1
ABS_X = 0
BTN_SOUTH = 304
BTN_EAST = 305

TRIGGER_MAX = 1023
TRIGGER_DEADZONE = 50
STICK_MAX = 32768
STICK_DEADZONE = 0.1


class RippleBridge:
    def __init__(self):
        self.ser = None
        self.gamepad = None
        self.tcp_socket = None
        self.tcp_client = None

        self.frequency = 1.5
        self.amplitude = AMPLITUDE_MIN
        self.offset = -25.0
        self.wavelength = 1.0
        self.mirror_mode = True
        self.emergency_stop = True
        self.vertical_mode = False
        self.vertical_amplitude = 0.0

        self.last_tcp_message = time.time()
        self.tcp_active = False

    def find_controller(self):
        import os
        for path in os.listdir('/dev/input/'):
            if path.startswith('event'):
                try:
                    device = InputDevice(f'/dev/input/{path}')
                    if 'xbox' in device.name.lower() or 'controller' in device.name.lower():
                        return device
                except:
                    continue
        return None

    def connect_controller(self):
        self.gamepad = self.find_controller()
        if self.gamepad:
            logging.info(f"Xbox controller connected: {self.gamepad.name}")
        else:
            logging.warning("Xbox controller not found - TCP only mode")

    def connect_serial(self):
        while True:
            try:
                self.ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUDRATE, timeout=1)
                time.sleep(2)
                logging.info(f"Connected to Arduino on {ARDUINO_PORT}")
                return
            except serial.SerialException as e:
                logging.warning(f"Failed to connect to Arduino: {e}. Retrying in 2s...")
                time.sleep(2)

    def setup_tcp_server(self):
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp_socket.bind(('0.0.0.0', TCP_PORT))
        self.tcp_socket.listen(1)
        self.tcp_socket.setblocking(False)
        logging.info(f"TCP server listening on port {TCP_PORT}")

    def send_command(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(f"{cmd}\n".encode())
                self.ser.flush()
                return True
            except serial.SerialException as e:
                logging.error(f"Serial write error: {e}")
                self.ser = None
                return False
        return False

    def map_value(self, value, in_min, in_max, out_min, out_max):
        normalized = max(0.0, min(1.0, (value - in_min) / (in_max - in_min)))
        return out_min + normalized * (out_max - out_min)

    def apply_deadzone(self, value, max_value):
        normalized = abs(value) / max_value
        if normalized < STICK_DEADZONE:
            return 0.0
        return value / max_value

    def update_parameters(self):
        if self.emergency_stop:
            self.send_command("STOP")
            return
        self.send_command(f"FREQ {self.frequency:.2f}")
        self.send_command(f"AMP {self.amplitude:.1f}")
        self.send_command(f"OFFSET {self.offset:.1f}")
        self.send_command(f"WAVE {self.wavelength:.2f}")
        self.send_command(f"MIRROR {1 if self.mirror_mode else 0}")
        self.send_command(f"VMODE {1 if self.vertical_mode else 0}")

    def process_tcp_command(self, cmd_id, value):
        try:
            if cmd_id == protocol.CMD_FREQ:
                self.frequency = max(FREQUENCY_MIN, min(FREQUENCY_MAX, value))
                if not self.emergency_stop:
                    self.send_command(f"FREQ {self.frequency:.2f}")
                logging.info(f"TCP: Frequency {self.frequency:.2f} Hz")

            elif cmd_id == protocol.CMD_AMP:
                self.amplitude = max(AMPLITUDE_MIN, min(AMPLITUDE_MAX, value))
                if not self.emergency_stop:
                    self.send_command(f"AMP {self.amplitude:.1f}")
                logging.info(f"TCP: Amplitude {self.amplitude:.1f} deg")

            elif cmd_id == protocol.CMD_OFFSET:
                self.offset = max(OFFSET_MIN, min(OFFSET_MAX, value))
                if not self.emergency_stop:
                    self.send_command(f"OFFSET {self.offset:.1f}")
                logging.info(f"TCP: Offset {self.offset:.1f} deg")

            elif cmd_id == protocol.CMD_WAVE:
                self.wavelength = max(WAVELENGTH_MIN, min(WAVELENGTH_MAX, value))
                if not self.emergency_stop:
                    self.send_command(f"WAVE {self.wavelength:.2f}")
                logging.info(f"TCP: Wavelength {self.wavelength:.2f}")

            elif cmd_id == protocol.CMD_MIRROR:
                self.mirror_mode = (int(value) != 0)
                if not self.emergency_stop:
                    self.send_command(f"MIRROR {1 if self.mirror_mode else 0}")
                logging.info(f"TCP: Mirror {'ON' if self.mirror_mode else 'OFF'}")

            elif cmd_id == protocol.CMD_ESTOP:
                self.emergency_stop = (int(value) != 0)
                if self.emergency_stop:
                    logging.warning("TCP: EMERGENCY STOP")
                    self.send_command("STOP")
                else:
                    logging.info("TCP: Resume")
                    self.send_command("START")
                    self.update_parameters()

            elif cmd_id == protocol.CMD_VMODE:
                self.vertical_mode = (int(value) != 0)
                if not self.emergency_stop:
                    self.send_command(f"VMODE {1 if self.vertical_mode else 0}")
                logging.info(f"TCP: Vertical mode {'ON' if self.vertical_mode else 'OFF'}")

            elif cmd_id == protocol.CMD_VAMP:
                self.vertical_amplitude = max(-40.0, min(40.0, value))
                if not self.emergency_stop:
                    self.send_command(f"VAMP {self.vertical_amplitude:.1f}")
                logging.info(f"TCP: Vertical amplitude {self.vertical_amplitude:.1f} deg")

            else:
                return protocol.STATUS_ERROR

            self.last_tcp_message = time.time()
            self.tcp_active = True
            return protocol.STATUS_OK

        except Exception as e:
            logging.error(f"Error processing TCP command: {e}")
            return protocol.STATUS_ERROR

    def process_xbox_event(self, event):
        if self.tcp_active:
            return

        if event.type != ecodes.EV_ABS and event.type != ecodes.EV_KEY:
            return

        if event.code == ABS_RZ:
            if event.value < TRIGGER_DEADZONE:
                self.frequency = 1.5
                self.send_command(f"FREQ {self.frequency:.2f}")
                return
            self.frequency = self.map_value(event.value, TRIGGER_DEADZONE, TRIGGER_MAX, FREQUENCY_MIN, FREQUENCY_MAX)
            self.send_command(f"FREQ {self.frequency:.2f}")

        elif event.code == ABS_Z:
            if event.value < TRIGGER_DEADZONE:
                self.amplitude = AMPLITUDE_MIN
                self.send_command(f"AMP {self.amplitude:.1f}")
                return
            self.amplitude = self.map_value(event.value, TRIGGER_DEADZONE, TRIGGER_MAX, AMPLITUDE_MIN, AMPLITUDE_MAX)
            self.send_command(f"AMP {self.amplitude:.1f}")

        elif event.code == ABS_Y:
            normalized = self.apply_deadzone(event.value, STICK_MAX)
            if abs(normalized) > 0:
                self.offset = self.map_value(-normalized, -1, 1, OFFSET_MIN, OFFSET_MAX)
                self.send_command(f"OFFSET {self.offset:.1f}")

        elif event.code == ABS_X:
            normalized = self.apply_deadzone(event.value, STICK_MAX)
            if abs(normalized) > 0:
                self.wavelength = self.map_value(normalized, -1, 1, WAVELENGTH_MIN, WAVELENGTH_MAX)
                self.send_command(f"WAVE {self.wavelength:.2f}")

        elif event.code == BTN_SOUTH and event.value == 1:
            self.emergency_stop = not self.emergency_stop
            if self.emergency_stop:
                logging.warning("Xbox: EMERGENCY STOP")
                self.send_command("STOP")
            else:
                logging.info("Xbox: Resume")
                self.send_command("START")
                self.update_parameters()

        elif event.code == BTN_EAST and event.value == 1:
            self.mirror_mode = not self.mirror_mode
            logging.info(f"Xbox: Mirror {'ON' if self.mirror_mode else 'OFF'}")
            self.send_command(f"MIRROR {1 if self.mirror_mode else 0}")

    def check_tcp_watchdog(self):
        if self.tcp_active:
            if time.time() - self.last_tcp_message > TCP_TIMEOUT:
                logging.warning("TCP watchdog timeout - entering emergency stop")
                self.tcp_active = False
                self.emergency_stop = True
                self.send_command("STOP")

    def run(self):
        logging.info("Ripple bridge starting...")

        self.connect_controller()
        self.connect_serial()
        self.setup_tcp_server()

        time.sleep(1)
        self.send_command("STOP")
        logging.info("Bridge ready - waiting for START command")

        read_list = [self.tcp_socket]
        if self.gamepad:
            read_list.append(self.gamepad.fd)

        self.last_tcp_message = time.time()

        while True:
            try:
                self.check_tcp_watchdog()

                readable, _, _ = select.select(read_list, [], [], 0.1)

                for fd in readable:
                    if fd == self.tcp_socket:
                        if self.tcp_client is None:
                            self.tcp_client, addr = self.tcp_socket.accept()
                            self.tcp_client.setblocking(False)
                            logging.info(f"TCP client connected: {addr}")
                            read_list.append(self.tcp_client)

                    elif fd == self.tcp_client:
                        try:
                            data = self.tcp_client.recv(protocol.MESSAGE_SIZE)
                            if not data:
                                logging.info("TCP client disconnected")
                                read_list.remove(self.tcp_client)
                                self.tcp_client.close()
                                self.tcp_client = None
                                self.tcp_active = False
                                continue

                            result = protocol.decode_command(data)
                            if result:
                                cmd_id, value = result
                                status = self.process_tcp_command(cmd_id, value)
                                ack = protocol.encode_ack(cmd_id, status)
                                self.tcp_client.send(ack)
                            else:
                                logging.warning("Invalid TCP message received")

                        except BlockingIOError:
                            pass
                        except Exception as e:
                            logging.error(f"TCP error: {e}")
                            if self.tcp_client in read_list:
                                read_list.remove(self.tcp_client)
                            self.tcp_client.close()
                            self.tcp_client = None
                            self.tcp_active = False

                    elif self.gamepad and fd == self.gamepad.fd:
                        for event in self.gamepad.read():
                            self.process_xbox_event(event)

            except KeyboardInterrupt:
                logging.info("Shutting down...")
                if self.ser:
                    self.send_command("STOP")
                    self.ser.close()
                if self.tcp_client:
                    self.tcp_client.close()
                if self.tcp_socket:
                    self.tcp_socket.close()
                sys.exit(0)

            except Exception as e:
                logging.error(f"Unexpected error: {e}")
                if not self.ser or not self.ser.is_open:
                    logging.info("Reconnecting to Arduino...")
                    self.connect_serial()
                time.sleep(0.1)


if __name__ == "__main__":
    bridge = RippleBridge()
    bridge.run()
