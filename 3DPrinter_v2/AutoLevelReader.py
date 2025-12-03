import serial
import time
import math
ARDUINO_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200 
arduino_serial = None
z_height = 3
sample_x_points = 0
sample_y_points = 0
pitch = 0
length = 0
breadth = 0
height_map = {}
gcode_file_path = "/home/saurabhsamant/Arduino/3DPrinter_v2/sd_filed/auto_level_rect.gcd"
X = 0.00
Y = 0.00
Z = 0.00

def setup_serial_communication():
    global arduino_serial
    try:
        arduino_serial = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=None)
        time.sleep(2) 
        print(f"Serial connection established on {ARDUINO_PORT} at {BAUD_RATE} baud.")
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        exit()

def send_command(command):
    if not command.endswith('\n'):
        command += '\n'
    arduino_serial.write(command.encode('utf-8'))
    
def read_probe_data(x_coord, y_coord):
    global z_height
    z_height_val = 0
    while True:
        z_height-=0.01
        gcode = f"G0 X{x_coord} Y{y_coord} Z{z_height}"
        send_command(gcode)
        time.sleep(0.1)
        resp = read_response()
        #print(gcode)
        if(resp == 'ok'):
            continue
        if(resp == 'ok Touched'):
            arduino_serial.reset_input_buffer() 
            z_height_val = z_height
            z_height = 3
            break
    return z_height_val
    
def position_for_sampling(x_coord, y_coord):
    gcode = f"G0 X{x_coord} Y{y_coord} Z{z_height} P1"        
    send_command(gcode)
    print(gcode)
    while(read_response() != 'ok'):
        continue
    return True    
        
def prepare_sampling_points():
    global pitch
    global length
    global breadth
    global sample_x_points
    global sample_y_points
    global height_map
    length, breadth, pitch = map(float, input("Enter length, breadth, and pitch separated by spaces: ").split())
    sample_x_points = int(length//pitch + 1)
    sample_y_points = int(breadth//pitch + 1)
    height_map = [[0.0 for _ in range(sample_y_points)] for _ in range(sample_x_points)]
    
def execute_sampling():
    for j in range(sample_y_points):
        for i in range(sample_x_points):
            x_coord = i * pitch
            y_coord = j * pitch
            position_for_sampling(x_coord,y_coord)
            #z_value = 1 if ((i==1 and j==0) or (i==1 and j==1)) else 0 
            z_value = read_probe_data(x_coord, y_coord)
            height_map[i][j] = z_value
            print(z_value)
            
def read_response():
    while True:
        response = arduino_serial.readline().decode('utf-8').strip()
        if(response.startswith('ok') or response == 'ok'):
            return response

def gather_sampling_data():
    send_command("G28")
    while(read_response() != 'ok'): continue
    execute_sampling()
    
    

def read_gcode_file():
    try:
        with open(gcode_file_path, 'r') as file:
            for line in file:
                gcode = line.strip()
                #print(gcode)
                tokens = gcode.split()
                data = {'G_code': None, 'X': None, 'Y': None, 'Z': None, 'E': None, 'F': None}
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
               

                adjusted_z = get_z(data['X'], data['Y'])
                z_token = f"Z{adjusted_z:.10f} P0" 
                amended_tokens = []
                for token in tokens:
                    if token.startswith('Y'):
                        amended_tokens.append(z_token)
                    amended_tokens.append(token)
                gcode = " ".join(amended_tokens)
                print(gcode)
                #send_command(gcode)
                #while(read_response() != 'ok'): continue
                
    except FileNotFoundError:
        print("Unable to open a file")
    except Exception:
        print("Unknow error occured")
        
        
def get_intermediate_z(square_corner_x1, square_corner_x2, square_corner_y1, square_corner_y2, square_corner_z1, square_corner_z2, actual_x, actual_y,is_horizontal_edge):
        x_diff = (square_corner_x1 - square_corner_x2)
        y_diff = (square_corner_y1 - square_corner_y2)
        adj = math.sqrt(pow(x_diff,2) + pow(y_diff,2))
        
        z_diff = (square_corner_z1 - square_corner_z2)
        diff = x_diff if is_horizontal_edge else y_diff
        hyp = math.sqrt(pow(diff,2) + pow(z_diff,2))
        theta = math.acos(adj/hyp)
        
        x_diff_actual = (square_corner_x1 - actual_x) if is_horizontal_edge else 0
        y_diff_actual = 0 if is_horizontal_edge  else (square_corner_y1 - actual_y)
        adj_actual = math.sqrt((x_diff_actual * x_diff_actual) + (y_diff_actual * y_diff_actual))
        
        z_diff_actual = adj_actual * math.tan(theta)       

        return z_diff_actual
        
def bilinear_interpolation(i,j,sample_x, sample_y, actual_x, actual_y):
    top_left_x = sample_x
    top_left_y = sample_y + pitch
    top_right_x = sample_x + pitch
    top_right_y = sample_y + pitch
    bottom_left_x = sample_x
    bottom_left_y = sample_y
    bottom_right_x =  sample_x + pitch
    bottom_right_y = sample_y
    left_clear = (actual_x >= bottom_left_x and actual_x >= top_left_x)
    right_clear = (actual_x < bottom_right_x and actual_x < top_right_x) and (bottom_right_x <= length and top_right_x <= length)
    top_clear = (actual_y < top_left_y and actual_y < top_right_y) and (top_left_y <= breadth and top_right_y <= breadth)
    bottom_clear = (actual_y >= bottom_left_y and actual_y >= bottom_right_y)
    if(left_clear and right_clear and top_clear and bottom_clear):
        bottom_left_z = height_map[i][j]
        bottom_right_z = height_map[i+1][j]
        top_left_z = height_map[i][j+1]
        top_right_z = height_map[i+1][j+1]

        ##### TOP LINE BEGIN ###
        top_z = min(top_left_z, top_right_z) + get_intermediate_z(top_left_x, top_right_x, top_left_y, top_right_y, top_left_z, top_right_z, actual_x, actual_y, True)
        bottom_z = min(bottom_left_z, bottom_right_z) + get_intermediate_z(bottom_left_x, bottom_right_x, bottom_left_y, bottom_right_y, bottom_left_z, bottom_right_z, actual_x, actual_y, True)
        
        left_z = min(top_left_z, bottom_left_z) + get_intermediate_z(top_left_x, bottom_left_x, top_left_y, bottom_left_y, top_left_z, bottom_left_z, actual_x, actual_y, False)
        right_z = min(top_right_z,bottom_right_z) + get_intermediate_z(top_right_x, bottom_right_x, top_right_y, bottom_right_y, top_right_z, bottom_right_z, actual_x, actual_y, False)

        
        z_x = min(left_z, right_z) + get_intermediate_z(bottom_left_x, bottom_right_x, actual_y, actual_y, left_z, right_z, actual_x, actual_y, True)
        z_y = min(top_z, bottom_z) + get_intermediate_z(actual_x, actual_x, top_left_y, bottom_left_y, top_z, bottom_z, actual_x, actual_y, False)
        


        return z_x

       
        
def get_z(target_x, target_y):
    for j in range(sample_y_points):
        for i in range(sample_x_points):
            x_coord = i * pitch
            y_coord = j * pitch
            return bilinear_interpolation(i,j,x_coord,y_coord,target_x,target_y)
            


        
if __name__ == "__main__":
    prepare_sampling_points()
    setup_serial_communication()
    gather_sampling_data()
    input("Press Enter to continue to the next step...")
    read_gcode_file()
    arduino_serial.close()
        