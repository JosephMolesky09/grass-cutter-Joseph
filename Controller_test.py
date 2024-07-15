import time
import serial
import signal
import sys

motor_ser = None
angle_ser = None

def stop_motors():
    if motor_ser and motor_ser.is_open:
        motor_ser.write("0,0\n".encode())

def signal_handler(sig, frame):
    print("Stopping motors...")
    stop_motors()
    sys.exit(0)

def move_forward(motor_ser, target_distance_cm, wheel_diameter_cm):
    wheel_circumference = 3.14 * wheel_diameter_cm
    rotations_needed = target_distance_cm / wheel_circumference
    encoder_ticks_needed = rotations_needed * encoder_ticks_per_revolution
    
    motor_ser.write("150,150\n".encode())  # Adjust speed as needed

    total_ticks = 0
    while total_ticks < encoder_ticks_needed:
        line = angle_ser.readline().decode('utf-8').strip()
        if line:
            try:
                _, encoder1, encoder2 = line.split(',')
                encoder1 = int(encoder1)
                encoder2 = int(encoder2)
                total_ticks = (encoder1 + encoder2) / 2
            except ValueError:
                print(f"Received malformed line: {line}")
    
    motor_ser.write("0,0\n".encode())

def turn_180_degrees(motor_ser, angle_ser):
    initial_yaw = get_yaw(angle_ser)
    target_yaw = (initial_yaw + 180) % 360
    
    motor_ser.write("150,-150\n".encode())  # Adjust speed as needed

    while True:
        current_yaw = get_yaw(angle_ser)
        if abs(current_yaw - target_yaw) < 5:  # Adjust tolerance as needed
            break

    motor_ser.write("0,0\n".encode())

def get_yaw(angle_ser):
    while True:
        line = angle_ser.readline().decode('utf-8').strip()
        if line:
            try:
                yaw, _, _ = line.split(',')
                return float(yaw)
            except ValueError:
                print(f"Received malformed line: {line}")

def main():
    global motor_ser, angle_ser
    motor_ser = serial.Serial('COM7', 115200, timeout=1)  # Motor control serial port
    angle_ser = serial.Serial('COM8', 115200, timeout=1)  # Sensor data serial port

    signal.signal(signal.SIGINT, signal_handler)  # Handle Ctrl+C

    try:
        move_forward(motor_ser, 10, 14)  # Move forward 10 cm
        turn_180_degrees(motor_ser, angle_ser)  # Make a U-turn
    except serial.SerialException as e:
        print(f"Error with serial communication: {e}")
    finally:
        stop_motors()  # Ensure motors stop when exiting
        if motor_ser and motor_ser.is_open:
            motor_ser.close()
        if angle_ser and angle_ser.is_open:
            angle_ser.close()

if __name__ == "__main__":
    main()
