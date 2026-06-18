if __name__ == "__main__":
    # Windows 示例：'COM3'，请根据设备管理器中的实际端口号修改
    servo = BusServo(port='COM9', baudrate=1000000, verbose=True)

    try:
        print("\n--- TEST 1: PING ID=10 ---")
        err = servo.ping(10)   # 使用 ID=10
        if err == 0:
            print("Ping Success!")
        else:
            print(f"Ping Failed or Error Code: {err}")

        # 可选：读取传感器数据
        print("\n--- TEST 2: READ SENSOR DATA ---")
        sensor_data = servo.read_sensor_data(10)
        if sensor_data:
            print("Sensor Data:", sensor_data)
        else:
            print("Failed to read sensor data.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        servo.close()