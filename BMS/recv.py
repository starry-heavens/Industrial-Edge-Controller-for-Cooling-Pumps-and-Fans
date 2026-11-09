import can

# 接收端同样需要 fd=True
bus = can.Bus(interface='socketcan', channel='can0', fd=True)
print("等待接收 CAN-FD 报文...")
for msg in bus:
    print(f"收到: ID={hex(msg.arbitration_id)}, 数据={msg.data.hex()}, FD={msg.is_fd}")
