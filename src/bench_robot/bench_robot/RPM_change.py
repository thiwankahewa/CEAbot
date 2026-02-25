# ZLAC8015D: Set RPM resolution to 10, SAVE to EEPROM, then VERIFY after power-cycle
# Requires: pip install pymodbus

import time
from pymodbus.client import ModbusSerialClient

PORT = "/dev/ttyUSB0"
BAUD = 115200
DEVICE_ID = 1

RPM_RES_REGISTER = 0x2022
RPM_RES_VALUE = 10

SAVE_REGISTER = 0x2010   # 1 = Save all RW params to EEPROM (per manual)
SAVE_COMMAND = 0x0001


def connect_client() -> ModbusSerialClient:
    client = ModbusSerialClient(port=PORT, baudrate=BAUD)
    if not client.connect():
        raise RuntimeError("❌ Failed to connect to ZLAC8015D")
    return client


def write_u16(client: ModbusSerialClient, addr: int, value: int):
    res = client.write_register(addr, value, device_id=DEVICE_ID)
    if res is None or res.isError():
        raise RuntimeError(f"❌ Write failed @0x{addr:04X}: {res}")


def read_u16(client: ModbusSerialClient, addr: int) -> int:
    rr = client.read_holding_registers(address=addr, count=1, device_id=DEVICE_ID)
    if rr is None or rr.isError():
        raise RuntimeError(f"❌ Read failed @0x{addr:04X}: {rr}")
    return rr.registers[0] & 0xFFFF


def main():
    # 1) Set + Save
    client = connect_client()
    try:
        print("Connected to ZLAC8015D")

        write_u16(client, RPM_RES_REGISTER, RPM_RES_VALUE)
        print("✅ RPM resolution set to 10 (0x2022 = 10)")
        time.sleep(0.05)

        write_u16(client, SAVE_REGISTER, SAVE_COMMAND)
        print("✅ Saved parameters to EEPROM (0x2010 = 1)")
        time.sleep(0.2)

    finally:
        client.close()

    # 2) Power cycle + verify
    print("\n⚠ IMPORTANT: Power cycle the ZLAC8015D now (OFF -> wait 3–5s -> ON).")
    input("After power is back ON, press ENTER to verify...")

    client = connect_client()
    try:
        val = read_u16(client, RPM_RES_REGISTER)
        if val == RPM_RES_VALUE:
            print(f"✅ VERIFIED: 0x2022 = {val} (RPM resolution is active)")
        else:
            print(f"❌ NOT APPLIED: 0x2022 = {val} (expected {RPM_RES_VALUE})")
            print("   - Ensure you issued the save (0x2010=1) successfully")
            print("   - Ensure you fully power-cycled the driver")
    finally:
        client.close()


if __name__ == "__main__":
    main()