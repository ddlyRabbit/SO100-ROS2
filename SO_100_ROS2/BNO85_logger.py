from sre_parse import HEXDIGITS

import serial
import time
import struct
from dataclasses import dataclass

@dataclass
class RotationVector:
    """Rotation vector data structure"""
    yaw: float
    pitch: float
    roll: float
    x_accel: float
    y_accel: float
    z_accel: float

class IMU_BNO:
    def __init__(self, port):
        self.port = port
        self.serial =  serial.Serial(self.port, 115200, timeout=0.002)
        self.serial.flush()
        self.last_read_ms = time.time() * 1000

    #   RVC Protocol
        self.PACKET_START_ID = bytearray([0xAA, 0xAA])
        self.PACKET_END_ID = bytearray([0x00, 0x00, 0x00])
        self.PACKET_SIZE = 19
        self.packet_buffer = bytearray()

    def parse_rvc_packet(self, packet: bytes) -> RotationVector:
        """Parse 19-byte RVC binary packet"""
        packet_counter = packet[2]

        # Parse little-endian int16 values (scale by 100)
        yaw = struct.unpack('<h', packet[3:5])[0] / 100.0
        pitch = struct.unpack('<h', packet[5:7])[0] / 100.0
        roll = struct.unpack('<h', packet[7:9])[0] / 100.0
        x_accel = struct.unpack('<h', packet[9:11])[0] * 9.8 / 1000.0
        y_accel = struct.unpack('<h', packet[11:13])[0] * 9.8 / 1000.0
        z_accel = struct.unpack('<h', packet[13:15])[0] * 9.8 / 1000.0

        return RotationVector(
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            x_accel=x_accel,
            y_accel=y_accel,
            z_accel=z_accel,
        )

    def print_rotation_vector_minimal(self, rv: RotationVector, sample_count: int = 0):
        """
        Minimal single-line display that updates in place
        """
        print(f"\r#{sample_count:05d} Y:{rv.yaw:+6.1f}° P:{rv.pitch:+6.1f}° R:{rv.roll:+6.1f}° |{rv.x_accel:+5.2f},{rv.y_accel:+5.2f},{rv.z_accel:+5.2f}|m/s²",
            end="", flush=True)

    def read(self):
        read_bytes = self.serial.read(50)
        self.packet_buffer.extend(read_bytes)
        delta = time.time() - self.last_read_ms
        self.last_read_ms = time.time()
        # if len(self.packet_buffer) > 0:
        #     print(self.packet_buffer.hex(), delta)
        msg_buffer = bytearray()

        if len(self.packet_buffer) >= self.PACKET_SIZE:
            last_start_index = self.packet_buffer.rfind(self.PACKET_START_ID)
            last_end_index = self.packet_buffer.rfind(self.PACKET_END_ID,last_start_index)
            # print(last_start_index, last_end_index)
            if last_start_index != -1 and \
                    last_end_index != -1 \
                    and last_end_index - last_start_index == 15 \
                    and len(self.packet_buffer) - 1 >= last_end_index+3:

                msg_buffer = self.packet_buffer[last_start_index:last_end_index+3]
                self.packet_buffer.clear()
                return self.parse_rvc_packet(msg_buffer)

            first_start_index = self.packet_buffer.find(self.PACKET_START_ID)
            first_end_index = self.packet_buffer.find(self.PACKET_END_ID, first_start_index)
            if first_start_index != -1 \
                and first_end_index != -1 \
                and first_end_index - first_start_index == 15 \
                and len(self.packet_buffer) - 1 >= first_end_index+3:

                msg_buffer = self.packet_buffer[first_start_index:first_end_index+3]
                self.packet_buffer.clear()
                return self.parse_rvc_packet(msg_buffer)

        return None

        # self.serial.reset_input_buffer()


if __name__ == "__main__":

    imu = IMU_BNO('/dev/tty.usbserial-1301')
    while True:
        rv = imu.read()
        if rv is not None:
            imu.print_rotation_vector_minimal(rv,2)