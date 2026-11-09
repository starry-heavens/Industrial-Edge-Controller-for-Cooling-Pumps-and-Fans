import can
import time

print("正在连接 CANable 硬件...")



# 使用 socketcan 接口，硬件由系统内核管理
try:
    bus = can.Bus(interface='socketcan', channel='can0', bitrate=500000)
except Exception as e:
    print(f"socketcan 接口初始化失败: {e}")
    # 可以在这里调用 modprobe 或给出提示
    exit(1)
# 2. 模拟一条包含 16 字节数据的 CAN-FD 报文
# 假设协议约定：ID 0x100 代表电压数据
msg = can.Message(
    arbitration_id=0x100,
    is_extended_id=False,
    is_fd=True,             # 标记为 CAN-FD 帧
    bitrate_switch=True,    # 开启数据段变速 (BRS)
    data=[
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10
    ]
)

print("=======================================")
print("开始向 CAN 总线发送 BMS 模拟报文...")
print("ID: 0x100 | 周期: 100ms | 数据长度: 16 字节")
print("按 Ctrl+C 停止发送")
print("=======================================")

try:
    # 3. 周期发送 (period 单位为秒，0.1 即 100ms)
    task = bus.send_periodic(msg, period=0.1)
    
    # 让程序一直运行，直到用户按下 Ctrl+C
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\n检测到 Ctrl+C，正在停止发送...")
except Exception as e:
    print(f"\n发送过程中出现错误: {e}")
finally:
    # 4. 清理现场，停止发送并关闭总线
    try:
        task.stop()
    except:
        pass
    bus.shutdown()
    print("已停止发送，硬件已安全断开。")
