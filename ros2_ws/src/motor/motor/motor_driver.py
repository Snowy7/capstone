import time
import struct
from smbus2 import SMBus, i2c_msg

class MotorDriver:
    REG_MOTOR_FIXED_SPEED = 0x33  # 4 x int8: pulses per 10ms (closed-loop)
    REG_ENCODER_TOTAL = 0x3C      # 4 x int32 little-endian: cumulative counts
    REG_MOTOR_TYPE = 0x14         # optional: motor type select
    REG_ENCODER_POLARITY = 0x15   # 0 or 1; keep 0 unless you must flip

    def __init__(self, bus=1, addr=0x34, motor_type=None, encoder_polarity=0):
        self.bus_num = bus
        self.addr = addr
        self.bus = SMBus(bus)
        
        # Retry settings
        self.RETRIES = 5
        self.RETRY_DELAY = 0.005  # 5ms

        # Initialization sequence with retries
        if motor_type is not None:
            self._write_bytes(self.REG_MOTOR_TYPE, [int(motor_type) & 0xFF])
            time.sleep(0.01)
        
        if encoder_polarity in (0, 1):
            self._write_bytes(self.REG_ENCODER_POLARITY, [int(encoder_polarity) & 0x01])
            time.sleep(0.01)

    def close(self):
        try:
            self.bus.close()
        except Exception:
            pass

    def _write_bytes(self, reg, data_bytes):
        """
        Writes bytes to I2C with robust retry logic to handle noise.
        Returns True on success, False on failure.
        """
        data = [b & 0xFF for b in data_bytes]
        
        for attempt in range(self.RETRIES):
            try:
                self.bus.write_i2c_block_data(self.addr, reg, data)
                return True # Success, exit immediately
            except OSError:
                # I2C failed (noise/wiring). Wait briefly and retry.
                time.sleep(self.RETRY_DELAY)
        
        # If we get here, all attempts failed.
        print(f"[MotorDriver] Error: Failed to write to 0x{self.addr:02X} after {self.RETRIES} attempts.")
        return False

    def set_motor_speed(self, speeds):
        """
        speeds: list of 4 ints in range [-100..100].
        Order: [FL, FR, RL, RR]
        Returns True if command was sent successfully, False otherwise.
        """
        if len(speeds) != 4:
            return False # Ignore invalid input size

        clipped = [max(-100, min(100, int(s))) for s in speeds]
        
        # Convert signed int8 -> bytes 0..255
        packed = [(s + 256) % 256 for s in clipped]
        
        # Use the robust write method
        return self._write_bytes(self.REG_MOTOR_FIXED_SPEED, packed)

    def reset_encoders(self):
        self._write_bytes(self.REG_ENCODER_TOTAL, [0] * 16)

    def get_encoders(self):
        """
        Returns [FL, FR, RL, RR] as signed int32 totals.
        Returns None if read fails after retries.
        """
        data = None
        
        # Try Reading with Retries
        for attempt in range(self.RETRIES):
            try:
                # Attempt 1: Direct I2C RDWR (Faster/Standard for some boards)
                write = i2c_msg.write(self.addr, bytes([self.REG_ENCODER_TOTAL]))
                read = i2c_msg.read(self.addr, 16)
                self.bus.i2c_rdwr(write, read)
                data = bytes(list(read))
                break # Success
            except Exception:
                # Fallback: Block Read
                try:
                    data = bytes(self.bus.read_i2c_block_data(self.addr, self.REG_ENCODER_TOTAL, 16))
                    break # Success
                except Exception:
                    time.sleep(self.RETRY_DELAY)
        
        if data is None or len(data) != 16:
            return None

        try:
            # Little-endian 4 x int32
            return list(struct.unpack("<iiii", data))
        except struct.error:
            return None