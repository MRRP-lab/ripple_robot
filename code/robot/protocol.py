"""
Binary control protocol for Ripple ROV

8-byte message format:
[MAGIC:1][CMD:1][VALUE:4][CHECKSUM:2]

Commands:
0x01 = FREQ
0x02 = AMP  
0x03 = OFFSET
0x04 = WAVE
0x05 = MIRROR
0x06 = ESTOP

Acknowledgment (3 bytes):
[ACK:1][CMD:1][STATUS:1]
"""

import struct

CMD_FREQ = 0x01
CMD_AMP = 0x02
CMD_OFFSET = 0x03
CMD_WAVE = 0x04
CMD_MIRROR = 0x05
CMD_ESTOP = 0x06
CMD_VMODE = 0x07  # vertical gait mode toggle (1.0 = on, 0.0 = off)
CMD_VAMP = 0x08   # vertical gait amplitude, signed degrees (-max..+max), sign = direction

ACK_BYTE = 0xFF
STATUS_OK = 0x00
STATUS_ERROR = 0x01

MESSAGE_SIZE = 8
ACK_SIZE = 3

def calculate_checksum(data):
    """16-bit checksum"""
    return sum(data) & 0xFFFF

def encode_command(cmd_id, value):
    """Encode command into 8-byte binary message"""
    value_bytes = struct.pack('<f', float(value))
    
    payload = struct.pack('BB', 0xAA, cmd_id) + value_bytes
    checksum = calculate_checksum(payload)
    
    message = payload + struct.pack('<H', checksum)
    return message

def decode_command(data):
    """Decode 8-byte binary message, returns (cmd_id, value) or None"""
    if len(data) != MESSAGE_SIZE:
        return None
    
    magic, cmd_id = struct.unpack('BB', data[0:2])
    if magic != 0xAA:
        return None
    
    value = struct.unpack('<f', data[2:6])[0]
    received_checksum = struct.unpack('<H', data[6:8])[0]
    
    expected_checksum = calculate_checksum(data[0:6])
    if received_checksum != expected_checksum:
        return None
    
    return (cmd_id, value)

def encode_ack(cmd_id, status=STATUS_OK):
    """Encode acknowledgment"""
    return struct.pack('BBB', ACK_BYTE, cmd_id, status)

def decode_ack(data):
    """Decode acknowledgment, returns (cmd_id, status) or None"""
    if len(data) != ACK_SIZE:
        return None
    
    ack, cmd_id, status = struct.unpack('BBB', data)
    if ack != ACK_BYTE:
        return None
    
    return (cmd_id, status)

def command_to_string(cmd_id):
    """Convert command ID to string name"""
    cmds = {
        CMD_FREQ: "FREQ",
        CMD_AMP: "AMP",
        CMD_OFFSET: "OFFSET",
        CMD_WAVE: "WAVE",
        CMD_MIRROR: "MIRROR",
        CMD_ESTOP: "ESTOP",
        CMD_VMODE: "VMODE",
        CMD_VAMP: "VAMP",
    }
    return cmds.get(cmd_id, f"UNKNOWN_{cmd_id:02x}")
