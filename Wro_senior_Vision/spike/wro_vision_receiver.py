from pybricks.iodevices import UARTDevice
from pybricks.parameters import Port
from pybricks.tools import wait


class GridPattern:
    UNKNOWN = 0
    YELLOW = 1
    BLUE = 2
    GREEN = 3
    WHITE = 4

    def __init__(self, valid, confidence, colors, confidences, sequence):
        self.valid = valid
        self.confidence = confidence
        self.colors = tuple(colors)
        self.confidences = tuple(confidences)
        self.sequence = sequence

    def color(self, cell):
        if cell < 0 or cell >= 12:
            raise ValueError("cell must be between 0 and 11")
        return self.colors[cell]

    def confidence_at(self, cell):
        if cell < 0 or cell >= 12:
            raise ValueError("cell must be between 0 and 11")
        return self.confidences[cell]


class GridPatternReceiver:
    MAGIC = bytes((0xC3, 0x3C))
    VERSION = 1
    TYPE_PATTERN = 1
    TYPE_ACK = 2
    TYPE_COMMAND = 3
    COMMAND_FLASH = 1
    PATTERN_LENGTH = 26

    def __init__(
        self,
        port=Port.C,
        baudrate=115200,
        timeout=20,
    ):
        self.uart = UARTDevice(port, baudrate=baudrate, timeout=timeout, power_pin=1)
        self._buffer = bytearray()
        self._pattern = None
        self.crc_errors = 0
        self.format_errors = 0
        self._next_command_sequence = 1

    def poll(self):
        data = self.uart.read_all()
        if data:
            self._buffer.extend(data)

        received = False
        while True:
            packet = self._extract_packet()
            if packet is None:
                break
            packet_type, sequence, payload = packet
            if packet_type == self.TYPE_PATTERN:
                if len(payload) == self.PATTERN_LENGTH:
                    self._pattern = GridPattern(
                        payload[0] != 0,
                        payload[1],
                        payload[2:14],
                        payload[14:26],
                        sequence,
                    )
                    self._send_ack(sequence)
                    received = True
                else:
                    self.format_errors += 1
        return received

    def set_flash(self, brightness):
        """Set flash brightness from 0 (off) to 255 (maximum)."""
        if isinstance(brightness, bool):
            brightness = 255 if brightness else 0
        if not isinstance(brightness, int) or brightness < 0 or brightness > 255:
            raise ValueError("brightness must be between 0 and 255")

        sequence = self._next_command_sequence
        self._next_command_sequence = (sequence + 1) & 0xFFFF
        payload = bytes((self.COMMAND_FLASH, brightness))
        packet = bytearray(self.MAGIC)
        packet.extend((self.VERSION, self.TYPE_COMMAND, len(payload)))
        packet.extend((sequence & 0xFF, (sequence >> 8) & 0xFF))
        packet.extend(payload)
        packet.extend(self._crc_bytes(packet[2:]))
        self.uart.write(packet)

    def pattern(self):
        return self._pattern

    def wait_for_pattern(self, timeout_ms=5000):
        elapsed = 0
        while elapsed < timeout_ms:
            if self.poll():
                return self._pattern
            wait(10)
            elapsed += 10
        return None

    def _extract_packet(self):
        header_length = 7
        crc_length = 2

        start = self._buffer.find(self.MAGIC)
        if start < 0:
            if len(self._buffer) > 1:
                self._buffer = bytearray(self._buffer[-1:])
            return None
        if start > 0:
            self._buffer = bytearray(self._buffer[start:])
        if len(self._buffer) < header_length:
            return None

        version = self._buffer[2]
        packet_type = self._buffer[3]
        payload_length = self._buffer[4]
        sequence = self._buffer[5] | (self._buffer[6] << 8)
        total_length = header_length + payload_length + crc_length
        if payload_length > 64:
            self._buffer = bytearray(self._buffer[2:])
            self.format_errors += 1
            return None
        if len(self._buffer) < total_length:
            return None

        packet = self._buffer[:total_length]
        payload = packet[header_length : header_length + payload_length]
        received_crc = packet[-2] | (packet[-1] << 8)
        calculated_crc = self._crc16(packet[2:-2])
        self._buffer = bytearray(self._buffer[total_length:])

        if version != self.VERSION:
            self.format_errors += 1
            return None
        if received_crc != calculated_crc:
            self.crc_errors += 1
            return None
        return packet_type, sequence, payload

    def _send_ack(self, sequence):
        payload = bytes((sequence & 0xFF, (sequence >> 8) & 0xFF))
        packet = bytearray(self.MAGIC)
        packet.extend((self.VERSION, self.TYPE_ACK, len(payload)))
        packet.extend(payload)
        packet.extend(self._crc_bytes(packet[2:]))
        self.uart.write(packet)

    @staticmethod
    def _crc16(data):
        crc = 0xFFFF
        for value in data:
            crc ^= value << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @classmethod
    def _crc_bytes(cls, data):
        crc = cls._crc16(data)
        return bytes((crc & 0xFF, (crc >> 8) & 0xFF))
