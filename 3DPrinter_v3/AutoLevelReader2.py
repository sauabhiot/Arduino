import serial
import time
import math
import sys

z_height = 0
sample_x_points = 2
sample_y_points = 2
length = 0
breadth = 0

ARDUINO_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200 
arduino_serial = None
zero_zero_z_offset = 0.0
height_map = {}
#gcode_file_path = "/home/saurabhsamant/Arduino/3DPrinter_v3/gcd_files/depth_test_rect.gcd"
gcode_file_path = "/home/saurabhsamant/Downloads/Top_layer2.gcode"
#gcode_file_path = "/home/saurabhsamant/Downloads/Gerber_TopLayer.GTL.gcode"
X = 0.00
Y = 0.00
Z = 0.00
prev_X = 0.00
prev_Y = 0.00
redo = True

def send_command(command):
    if not command.endswith('\n'):
        command += '\n'
    arduino_serial.write(command.encode('utf-8'))

def read_gcode_file():
    global prev_X
    global prev_Y
    try:
        with open(gcode_file_path, 'r') as file:
            for line in file:
                #print(line)
                gcode = line.strip()
                if ';' in gcode:
                    gcode = gcode.split(';')[0].strip()
                tokens = gcode.split()
                if 'X' in gcode:
                    data = {'G_code': None, 'X': None, 'Y': None, 'Z': None, 'E': None, 'F': None, 'R': None}
                    for token in tokens:
                        if token.startswith('G'):
                            data['G_code'] = token
                        elif token.startswith('X'):
                            data['X'] = float(token[1:]) 
                        elif token.startswith('Y'):
                            data['Y'] = float(token[1:]) 
                        elif token.startswith('E'):
                            data['E'] = float(token[1:]) 
                        elif token.startswith('F'):
                            data['F'] = float(token[1:])
                        elif token.startswith('R'):
                            data['R'] = float(token[1:])    
                            
                    adjusted_z = get_z(data['X'], data['Y'])
                    
                    if(data['G_code'] == 'G0' or data['G_code'] == 'G00'):
                        adjusted_z = adjusted_z + 3
                    z_token = f"Z{adjusted_z:.10f} P0" 
                    amended_tokens = []
                    for token in tokens:
                        if token.startswith('Y'):
                            amended_tokens.append(z_token)
                        amended_tokens.append(token)

                    if(data['G_code'] == 'G2' or data['G_code'] == 'G3' or data['G_code'] == 'G02' or data['G_code'] == 'G03'):
                        IJ = get_IJ_from_R(data['X'], data['Y'], data['R'],data['G_code'])
                        amended_tokens.append(IJ)
                    prev_X = data['X']
                    prev_Y = data['Y']
                    gcode = " ".join(amended_tokens)
                    
                    if(data['G_code'] == 'G0' or data['G_code'] == 'G00'):
                        send_command("G91")
                        while(read_response() != 'ok'): continue
                        send_command("G0 Z3 P1 F300")
                        while(read_response() != 'ok'): continue
                        send_command("G90")
                        while(read_response() != 'ok'): continue
                    print(gcode)
                    send_command(gcode)
                    
                    while(read_response() != 'ok'): continue
                    
                    if(data['G_code'] == 'G0' or data['G_code'] == 'G00'):
                        send_command("G91")
                        while(read_response() != 'ok'): continue
                        send_command("G0 Z-3 P1 F300")
                        while(read_response() != 'ok'): continue
                        send_command("G90")
                        while(read_response() != 'ok'): continue
                
    except FileNotFoundError:
        print("Unable to open a file")
    except Exception:
        print("Unknow error occured")


def get_IJ_from_R(current_X, current_Y, radius, code):
    mx = float(current_X + prev_X)/2.00
    my = float(current_Y + prev_Y)/2.00
    d = math.sqrt(pow((current_X - prev_X),2) + pow((current_X - prev_X),2))
    h = (radius * radius) - pow((d/2.00),2)
    if(code =='G2' or code == 'G02'):
        Xc = mx + h * (prev_Y - current_Y)/d
        Yc = my + h * (current_X - prev_X)/d
    if(code =='G3' or code == 'G03'):
        Xc = mx - h * (prev_Y - current_Y)/d
        Yc = my - h * (current_X - prev_X)/d
    
    I = Xc - prev_X
    J = Yc - prev_Y
    
    IJ = f"I{I:.10f} J{J:.10f}"
    return IJ
    
    
def get_z(act_x,act_y):
    bottom_left_x = 0;
    bottom_left_y = 0;
    bottom_left_z = height_map[0][0];

    bottom_right_x = length;
    bottom_right_y = 0;
    bottom_right_z = height_map[1][0];

    top_left_x = 0;
    top_left_y = breadth;
    top_left_z = height_map[0][1];

    top_right_x = length;
    top_right_y = breadth;
    top_right_z = height_map[1][1];
    
    bottom_edge_height_diff = (bottom_right_z - bottom_left_z)
    bottom_edge_angle = math.asin(bottom_edge_height_diff/length)
    bottom_edge_z_at_x = bottom_left_z + act_x * bottom_edge_angle

    top_edge_height_diff = (top_right_z - top_left_z)
    top_edge_angle = math.asin(top_edge_height_diff/length)
    top_edge_z_at_x = top_left_z + act_x * top_edge_angle
    
    left_edge_height_diff = (top_left_z - bottom_left_z) 
    left_edge_angle = math.asin(left_edge_height_diff/breadth)
    left_edge_z_at_y = bottom_left_z + act_y * left_edge_angle 
    
    right_edge_height_diff = (top_right_z - bottom_right_z) 
    right_edge_angle = math.asin(right_edge_height_diff/breadth)
    right_edge_z_at_y = bottom_right_z + act_y * right_edge_angle
    
    height = bottom_edge_z_at_x + (act_y - 0)/(breadth) * (top_edge_z_at_x - bottom_edge_z_at_x) 
    height2 = left_edge_z_at_y + (act_x - 0)/(length) * (right_edge_z_at_y - left_edge_z_at_y)

    print(height)
    print(height2)
    
    
    return height

def read_response():
    while True:
        response = arduino_serial.readline().decode('utf-8').strip()
        if(response.startswith('ok') or response == 'ok'):
            return response

def test_read_probe_data(x_coord, y_coord):
    if x_coord == 0.0 and y_coord == 0.0:
         #return 0.06000000000001968
         return 0   
    if x_coord == 80.0 and y_coord == 0.0:
        #return -1.029999999999981
        return -1
    if x_coord == 0.0 and y_coord == 80.0:
        #return -0.029999999999980327
        return 0
    if x_coord == 80.0 and y_coord == 80.0:
        #return -0.8799999999999809
        return -1
        
def read_probe_data(x_coord, y_coord):
    global z_height
    z_height_val = 0
    while True:
        z_height-=0.1
        gcode = f"G0 X{x_coord} Y{y_coord} Z{z_height} P1"
        #print(gcode)
        send_command(gcode)
        time.sleep(0.5)
        resp = read_response()
        if(resp == 'ok'):
            continue
        if(resp.startswith('ok Touched') and z_height < 1):
            arduino_serial.reset_input_buffer() 
            z_height_val = z_height #float(resp.split()[2])
            z_height = 0
            break
    return (z_height_val)        

def position_for_sampling(x_coord, y_coord):
    send_command("G90")
    while(read_response() != 'ok'): continue
    send_command(f"G0 Z0 P1 F10")
    while(read_response() != 'ok'): continue
    gcode = f"G0 X{x_coord} Y{y_coord} P1 F300"        
    send_command(gcode)
    print(gcode)
    while(read_response() != 'ok'): continue
    return True 
    
    
def execute_sampling():
    global zero_zero_z_offset 
    for j in range(sample_y_points):
        for i in range(sample_x_points):
            x_coord = i * length
            y_coord = j * breadth
            #if(i != 0): continue 
            position_for_sampling(x_coord, y_coord)
            z_value = read_probe_data(x_coord, y_coord)
            #z_value = test_read_probe_data(x_coord, y_coord)
            if(i == 0 and j == 0):
                zero_zero_z_offset = (-1 * z_value) 
            z_value = z_value + zero_zero_z_offset
            print(z_value)
            height_map[i][j] = z_value

def prepare_sampling_points():
    global length
    global breadth
    global height_map
    length, breadth = map(float, input("Enter length and breadth separated by spaces: ").split())
    height_map = [[0.0 for _ in range(sample_y_points)] for _ in range(sample_x_points)]

def setup_serial_communication():
    global arduino_serial
    try:
        arduino_serial = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=None)
        time.sleep(2) 
        print(f"Serial connection established on {ARDUINO_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        exit()
        
        
if __name__ == "__main__":
    prepare_sampling_points()
    setup_serial_communication()
    execute_sampling()
    send_command("G0 Z2 P1 F300")
    while(read_response() != 'ok'): continue
    
    send_command("G28")
    while(read_response() != 'ok'): continue
    
    input("Press Enter to continue to the next step...")
    arduino_serial.reset_input_buffer() 
    
    while(redo):
        read_gcode_file()
        resp = input("Do you want to Redo? then Enter Y, else any other key")
        if(resp!='Y'): break
    arduino_serial.close()