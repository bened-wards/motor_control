import serial

def get_velocity_str(v, w):
    return f"v{'+' if v >= 0 else '-'}{v:.3f}w{'+' if w >= 0 else '-'}{w:.3f}\n"

def write_velocity(ser, v, w):
    ser.write(get_velocity_str(v,w)).encode('utf-8')

def write_stop(ser):
    write_velocity(ser, 0.0, 0.0)

def decode_velocity_msg(msg):
    """Expected format: v+0.000w-0.000"""
    v = float(msg[1:7])
    w = float(msg[8:])
    return v, w

def decode_state_msg(msg):
    """Expected format: x+0.000y+0.000t-0.000"""
    x = float(msg[1:7])
    y = float(msg[8:14])
    theta = float(msg[15:])
    return x, y, theta

if __name__ == "__main__":
    # Set up the serial connection
    ser = serial.Serial('/dev/ttyS0', 115200)  # Adjust the port and baud rate as needed

    while True:
        if ser.in_waiting > 0:
            print("Got message")
            msg = ser.readline().decode('utf-8').strip()
            print("Received msg:", msg)
            if msg[0] == 'v':
                v, w = decode_velocity_msg(msg)
                print(f"Decoded velocity: v={v}, w={w}")
            elif msg[0] == 'x':
                x, y, theta = decode_state_msg(msg)
                print(f"Decoded state: x={x}, y={y}, theta={theta}")

            write_velocity(ser, 1.500, 2.500)
