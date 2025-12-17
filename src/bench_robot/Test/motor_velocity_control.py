#!/usr/bin/env python3
from pymodbus.client import ModbusSerialClient
import time


DEVICE_ID = 1
PORT = "/dev/ttyUSB0"          # or /dev/serial/by-id/...
BAUD = 115200


def to_u16_signed(val: int) -> int:
    """Convert signed int16 (-32768..32767) to unsigned 0..65535 for Modbus."""
    return val & 0xFFFF


def write_single(client: ModbusSerialClient, addr: int, value: int):
    res = client.write_register(addr, value, device_id=DEVICE_ID)  # FC 0x06
    if res.isError():
        raise RuntimeError(f"Write single failed @0x{addr:04X}: {res}")
    return res


def write_two_speeds(client: ModbusSerialClient, left_rpm: int, right_rpm: int):
    # 0x2088, count=2, bytecount=4, data = left, right (signed int16)
    values = [to_u16_signed(left_rpm), to_u16_signed(right_rpm)]
    res = client.write_registers(0x2088, values, device_id=DEVICE_ID)  # FC 0x10
    if res.isError():
        raise RuntimeError(f"Write speeds failed @0x2088: {res}")
    return res


def main():
    client = ModbusSerialClient(
        port=PORT,
        baudrate=BAUD,
        parity="N",
        stopbits=1,
        bytesize=8,
        timeout=0.5,
    )

    print("Connecting:", client.connect())
    if not client.connected:
        raise SystemExit("❌ Could not open serial port")

    try:
        # 1) Set Profile Velocity Mode: 01 06 20 0D 00 03 ...
        print("Set profile velocity mode (0x200D = 0x0003)")
        write_single(client, 0x200D, 0x0003)
        time.sleep(0.05)

        # 2) Motor enable: 01 06 20 0E 00 08 ...
        print("Motor enable (0x200E = 0x0008)")
        write_single(client, 0x200E, 0x0008)
        time.sleep(0.05)

        # 3) Example: target speed +100 RPM both wheels: 01 10 20 88 ... 00 64 00 64 ...
        print("Speed: left=+100 rpm, right=+100 rpm")
        write_two_speeds(client, 25, 25)
        time.sleep(5.0)

        # 4) Example: target speed -100 RPM both wheels: ... FF 9C FF 9C ...
        print("Speed: left=-100 rpm, right=-100 rpm")
        write_two_speeds(client, -25, -25)
        time.sleep(5.0)

        # 5) Example from your table: left=-10 rpm, right=+100 rpm  => FF F6, 00 64
        print("Speed: left=-10 rpm, right=+100 rpm")
        write_two_speeds(client, 25, 5)
        time.sleep(5.0)

        # 6) Stop
        print("Stop: left=0 rpm, right=0 rpm")
        write_two_speeds(client, 0, 0)

        print("✅ Done")

    finally:
        client.close()


if __name__ == "__main__":
    main()
