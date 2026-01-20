import serial
import time
import math
import sys
ARDUINO_PORT = '/dev/ttyACM0' 
BAUD_RATE = 115200 
arduino_serial = None
z_height = 0
sample_x_points = 0
sample_y_points = 0
pitch = 0
length = 0
breadth = 0
height_map = {}
gcode_file_path = "/home/saurabhsamant/Arduino/3DPrinter_v3/gcd_files/depth_test_rect.gcd"
X = 0.00
Y = 0.00
Z = 0.00
prev_X = 0.00
prev_Y = 0.00
zero_zero_z_offset = 0.0

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

def test_read_probe_data(x_coord, y_coord):
    if x_coord == 0.0 and y_coord == 0.0:
         return 0.06000000000001968
         #return 0   
    if x_coord == 80.0 and y_coord == 0.0:
        return -1.029999999999981
        #return -2
    if x_coord == 0.0 and y_coord == 80.0:
        return -0.029999999999980327
        #return 0
    if x_coord == 80.0 and y_coord == 80.0:
        return -0.8799999999999809
        #return -2
    
    
def read_probe_data(x_coord, y_coord):
    global z_height
    z_height_val = 0
    while True:
        z_height-=0.01
        gcode = f"G0 X{x_coord} Y{y_coord} Z{z_height} P1"
        send_command(gcode)
        time.sleep(0.1)
        resp = read_response()
        #print(gcode)
        if(resp == 'ok'):
            continue
        if(resp.startswith('ok Touched') and z_height < 1):
            arduino_serial.reset_input_buffer() 
            z_height_val = float(resp.split()[2])
            #print(str(z_height_val)+"---"+str(z_height))
            z_height = 0
            break
    return (z_height_val)
    
def position_for_sampling(x_coord, y_coord):
    send_command(f"G0 Z{z_height} P1 F300")
    while(read_response() != 'ok'): continue
    gcode = f"G0 X{x_coord} Y{y_coord} F300"        
    send_command(gcode)
    print(gcode)
    while(read_response() != 'ok'): continue
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
    global zero_zero_z_offset 
    for j in range(sample_y_points):
        for i in range(sample_x_points):
            x_coord = i * pitch
            y_coord = j * pitch
            #position_for_sampling(x_coord,y_coord)
            #z_value = 1 if ((i==1 and j==0) or (i==1 and j==1)) else 0 
            #z_value = read_probe_data(x_coord, y_coord)
            z_value = test_read_probe_data(x_coord, y_coord)
            if(i == 0 and j == 0):
                zero_zero_z_offset = (-1 * z_value) 
            z_value = z_value + zero_zero_z_offset
            print(z_value)
            #z_value = test_read_probe_data(x_coord, y_coord)
            height_map[i][j] = z_value

            
def read_response():
    while True:
        response = arduino_serial.readline().decode('utf-8').strip()
        if(response.startswith('ok') or response == 'ok'):
            return response


    

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
                    print(gcode)
                    send_command(gcode)
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
    
    #print(current_x)
    #print(current_y)
    #print(radius)
    IJ = f"I{I:.10f} J{J:.10f}"
    return IJ
    #print(I)
    #print(J)
        
def get_intermediate_z2(square_corner_x1, square_corner_x2, square_corner_y1, square_corner_y2, square_corner_z1, square_corner_z2, actual_x, actual_y,is_horizontal_edge):
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
        
def get_intermediate_z(square_corner_x1, square_corner_x2, square_corner_y1, square_corner_y2, square_corner_z1, square_corner_z2, actual_x, actual_y,is_horizontal_edge):
    x_diff = (square_corner_x1 - square_corner_x2)
    y_diff = (square_corner_y1 - square_corner_y2)
    
    adj = math.sqrt(pow(x_diff,2) + pow(y_diff,2))
    
    z_diff = (square_corner_z2 - square_corner_z1)
    


    
    
    x_diff_actual = (square_corner_x1 - actual_x) if is_horizontal_edge else 0
    y_diff_actual = 0 if is_horizontal_edge  else (square_corner_y1 - actual_y)
    
    
    adj_actual = math.sqrt((x_diff_actual * x_diff_actual) + (y_diff_actual * y_diff_actual))
    print(z_diff)
    #print(adj_actual)
    #print(adj)
    z_diff_actual = (adj_actual * z_diff)/adj      
    #print(z_diff_actual)
    return z_diff_actual
        
def bilinear_interpolation2(i,j,sample_x, sample_y, actual_x, actual_y):
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
        top_z_term = min(top_left_z,top_right_z, key=abs) #top_left_z if min(abs(top_left_z), abs(top_right_z)) else top_right_z
        
        top_z = top_z_term + get_intermediate_z(top_left_x, top_right_x, top_left_y, top_right_y, top_left_z, top_right_z, actual_x, actual_y, True)
        

       
        bottom_z_term = min(bottom_left_z,bottom_right_z,key=abs) #bottom_left_z if min(abs(bottom_left_z), abs(bottom_right_z)) else bottom_right_z
        bottom_z = bottom_z_term + get_intermediate_z(bottom_left_x, bottom_right_x, bottom_left_y, bottom_right_y, bottom_left_z, bottom_right_z, actual_x, actual_y, True)
        
        left_z_term = min(top_left_z,bottom_left_z,key=abs) #top_left_z if min(abs(top_left_z), abs(bottom_left_z)) else bottom_left_z
        left_z = left_z_term + get_intermediate_z(top_left_x, bottom_left_x, bottom_left_y, top_left_y, bottom_left_z, top_left_z, actual_x, actual_y, False)
        
        
        right_z_term = min(top_right_z,bottom_right_z,key=abs) #top_right_z if min(abs(top_right_z), abs(bottom_right_z)) else bottom_right_z
        right_z = right_z_term + get_intermediate_z(top_right_x, bottom_right_x, bottom_right_y ,top_right_y, bottom_right_z, top_right_z, actual_x, actual_y, False)

        
        z_x_term = min(left_z,right_z,key=abs) #left_z if min(abs(left_z), abs(right_z)) else right_z
        z_x = z_x_term + get_intermediate_z(bottom_left_x, bottom_right_x, actual_y, actual_y, left_z, right_z, actual_x, actual_y, True)
        
        z_y_term = min(top_z,bottom_z,key=abs) #top_z if min(abs(top_z), abs(bottom_z)) else bottom_z
        z_y = z_y_term + get_intermediate_z(actual_x, actual_x, top_left_y, bottom_left_y, top_z, bottom_z, actual_x, actual_y, False)
        
        #print(z_x_term)
        #print(min(abs(left_z), abs(right_z)))
        #print(right_z)

        #print(z_x)
        #print(z_y)
        #print(top_z)
        
        #print(top_left_x)
        #print(bottom_left_x)
        #print(top_left_y)
        #print(bottom_left_y)
        #print(top_left_z)
        #print(bottom_left_z)
        #print(actual_x)
        #print(actual_y)
        
        #print(get_intermediate_z(top_left_x, bottom_left_x, top_left_y, bottom_left_y, top_left_z, bottom_left_z, actual_x, actual_y, False))
        return z_x

def bilinear_interpolation(i,j,sample_x, sample_y, actual_x, actual_y):
    bottom_left_x = sample_x
    bottom_left_y = sample_y
    top_left_x = sample_x
    top_left_y = sample_y + pitch
    top_right_x = sample_x + pitch
    top_right_y = sample_y + pitch
    bottom_right_x =  sample_x + pitch
    bottom_right_y = sample_y
    print(bottom_left_x)
    print(bottom_left_y)
    print(top_left_x)
    print(top_left_y)
    print(top_right_x)
    print(top_right_y)
    print(bottom_right_x)
    print(bottom_right_y)
    print("------")
        
def get_z(target_x, target_y):
    for j in range(sample_y_points):
        for i in range(sample_x_points):
            x_coord = i * pitch
            y_coord = j * pitch
            val = bilinear_interpolation(i,j,x_coord,y_coord,target_x,target_y)
            if val is not None:
                return val  
            else: continue
    return 0;            
       

        
if __name__ == "__main__":
    prepare_sampling_points()
    setup_serial_communication()
    execute_sampling()
    send_command("G0 Z2 P1 F300")
    while(read_response() != 'ok'): continue
    
    #send_command("G28")
    #while(read_response() != 'ok'): continue
    
    input("Press Enter to continue to the next step...")
    arduino_serial.reset_input_buffer() 
    read_gcode_file()
    arduino_serial.close()
        