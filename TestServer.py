import socket
import struct
import numpy as np
import cv2



def start_server(robot_socket, host='0.0.0.0', port=12345):
    robot_socket
    origin_x = 0
    origin_y = 0
    # Create a UDP socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Bind the socket to a specific address and port
    server_socket.bind((host, port))
    print(f"Server listening on {host}:{port}")

    canvas_width = 1000
    canvas_height = 1000
    canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

    # Draw a circle
    cv2.circle(canvas, (canvas_width//2, canvas_height//2), 400, (0, 255, 0), 2)

    while True:
        try:
            # Receive data from the client
            data, client_address = server_socket.recvfrom(1024)
            
            float_count = len(data) // 4
            float_array = struct.unpack(f'{float_count}f', data)
            # print(f"Received from {client_address}: {float_array}")

            # Clear previous robot position
            temp_canvas = canvas.copy()

            # Calculate robot position (assuming first two floats are x and y)
            robot_x = int(float_array[0] + canvas_width/2) - origin_x
            robot_y = int(float_array[1] + canvas_height/2) - origin_y
            print(f"Robot position: {robot_x}, {robot_y}")
            # Draw robot position (larger red dot)
            cv2.circle(temp_canvas, (robot_x, robot_y), 10, (0, 0, 255), -1)

            cv2.imshow('Data', temp_canvas)
            key = cv2.waitKey(1)
            if key == ord('q'):
                cv2.destroyAllWindows()
                break
            if key == ord('c'):
                origin_x = int(float_array[0])
                origin_y = int(float_array[1])


        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
        
    robot_address = "192.168.2.115"
    robot_port = 12345
    robot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start_server(robot_socket)
