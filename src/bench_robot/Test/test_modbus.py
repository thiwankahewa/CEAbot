from pymodbus.client import ModbusSerialClient

client = ModbusSerialClient(
    port='/dev/ttyUSB0',
    baudrate=115200,
)

print("Connecting:", client.connect())

res = client.read_holding_registers(
    0x20AF,
    count=1,
    device_id=1
)

if res.isError():
    print("❌ Modbus error:", res)
else:
    print("✅ Raw register value:", res.registers[0])