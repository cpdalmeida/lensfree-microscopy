# PARSE ERROR on line 148: IndentationError: unindent does not match any outer indentation level (<unknown>, line 148)
# ANNEX D – PYTHON CODE TO CONTROL DEVICES FOR MULTISPECTRAL METHOD

"""
*** Python code to control the multispectral lens-free microscope ***
    Control of the camera (Arducam MT9J001) and RGB LEDs
"""

# ------------- Libraries  --------------------- #
import sys
import time
import os
import threading
from sys import exit

import cv2
import json
from ImageConvert import convert_image
import ArducamSDK

import serial

from datetime import date

global cfg,handle,running,Width,Heigth,saved_flag,color_mode,save_raw, rtn_val, led_flag, z, folder, exposure, exposure_time, ser, z_new

# ---------- CONTROL PARAMETERS --------------- #
z = 0                 ## Auxiliary variable to control the image acquisition
z_new = 0             ## Auxiliary variable to control saving images
led_flag = False      ## False if LED is off and can be used
running = True        ## True if the program is running
saved_flag = False    ## True to save the current image
flag_capture_error = False
cfg = {}
handle = {}
prp_exp = (100/31)    ## Proportion between exposure values in code and time
intensity = 1

# ---------- ADJUSTABLE PARAMETERS --------------- #
save_raw = False         ## True to save file .raw of images
n_pictures = 3             ## Number of images
exposure = [320, 840, 110]       ## exposure values for PLASTIC DIFUSOR
exposure_time = [i *prp_exp*(1/1000) for i in exposure] ## exposure time values in seconds

# Current address
addr = os.getcwd()

# ------------- Functions --------------------- #
# Set the configurations of the camera board

def configBoard(fileNodes):
    global handle
    for i in range(0,len(fileNodes)):
        fileNode = fileNodes[i]
        buffs = []
        command = fileNode[0]
        value = fileNode[1]
        index = fileNode[2]
        buffsize = fileNode[3]
        for j in range(0,len(fileNode[4])):
            buffs.append(int(fileNode[4][j],16))
        ArducamSDK.Py_ArduCam_setboardConfig(handle,int(command,16),int(value,16),int(index,16),int(buffsize,16),buffs)

pass

# Write on configuration camera registers
def writeSensorRegs(fileNodes):
    global handle
    for i in range(0,len(fileNodes)):
        fileNode = fileNodes[i]
        if fileNode[0] == "DELAY":
            time.sleep(float(fileNode[1])/1000)
            continue
        regAddr = int(fileNode[0],16)
        val = int(fileNode[1],16)

        if regAddr != 12306: ## Add the desired exposure value
            ArducamSDK.Py_ArduCam_writeSensorReg(handle,regAddr,val)
        else:
            ArducamSDK.Py_ArduCam_writeSensorReg(handle,regAddr,exposure[2])

pass

# Read the JSON file and set the parameters
def camera_initFromFile(fialeName):
    global cfg,handle,Width,Height,color_mode,save_raw
    #load config file
    config = json.load(open(fialeName,"r"))

    camera_parameter = config["camera_parameter"]
    Width = int(camera_parameter["SIZE"][0])
    Height = int(camera_parameter["SIZE"][1])

    BitWidth = camera_parameter["BIT_WIDTH"]
    ByteLength = 1
    if BitWidth > 8 and BitWidth <= 16:
        ByteLength = 2
        save_raw = True
    FmtMode = int(camera_parameter["FORMAT"][0])
    color_mode = (int)(camera_parameter["FORMAT"][1])
    #print("color mode",color_mode)

    I2CMode = camera_parameter["I2C_MODE"]
    I2cAddr = int(camera_parameter["I2C_ADDR"],16)
    TransLvl = int(camera_parameter["TRANS_LVL"])
    cfg = {"u32CameraType":0x4D091031,
        "u32Width":Width,"u32Height":Height,
        "usbType":0,
        "u8PixelBytes":ByteLength,
        "u16Vid":0,
        "u32Size":0,
        "u8PixelBits":BitWidth,
        "u32I2cAddr":I2cAddr,
        "emI2cMode":I2CMode,
        "emImageFmtMode":FmtMode,
        "u32TransLvl":TransLvl }

    # ArducamSDK.
    #ret,handle,rtn_cfg = ArducamSDK.Py_ArduCam_open(cfg,0)
    ret,handle,rtn_cfg = ArducamSDK.Py_ArduCam_autoopen(cfg)
    if ret == 0:

        #ArducamSDK.Py_ArduCam_writeReg_8_8(handle,0x46,3,0x00)
        usb_version = rtn_cfg['usbType']
        #print("USB VERSION:",usb_version)
        #config board param
        configBoard(config["board_parameter"])

        if usb_version == ArducamSDK.USB_1 or usb_version == ArducamSDK.USB_2:
            configBoard(config["board_parameter_dev2"])
        if usb_version == ArducamSDK.USB_3:
            configBoard(config["board_parameter_dev3_inf3"])
        if usb_version == ArducamSDK.USB_3_2:
            configBoard(config["board_parameter_dev3_inf2"])

        writeSensorRegs(config["register_parameter"])

        if usb_version == ArducamSDK.USB_3:
            writeSensorRegs(config["register_parameter_dev3_inf3"])
        if usb_version == ArducamSDK.USB_3_2:
            writeSensorRegs(config["register_parameter_dev3_inf2"])

        rtn_val,datas = ArducamSDK.Py_ArduCam_readUserData(handle,0x400-16, 16)
        '''print("Serial: %d%d%d%d-%d%d%d%d-%d%d%d%d"%(datas[0],datas[1],datas[2],datas[3],datas[4],datas[5],datas[6],datas[7],datas[8],datas[9],datas[10],datas[11]))
        '''
        return True
    else:
                    print("open fail,rtn_val = ",hex(ret))
    return False

pass

def error_capture():

    global running_capture, running_acquisition, running_read, error_capture_flag
    running_capture = False
    running_acquisition = False
    running_read = False
    #led_flag = False

    if lt.is_alive():
                        lt.join()

    #Turn off the camera
    #close_camera()
    close_components()
    time.sleep(5)
    #Turn on the camera
    #init_camera()
    init_components()
    #led_flag = True

    if not lt.is_alive():
                        lt.start()
    if lt.is_alive():
                        lt.join()

    running_capture=True

# Perform image capture and add FIFO
def captureImage_thread():
    global handle,running, running_capture, flag_capture_error, running

    running_capture = True

    rtn_val = ArducamSDK.Py_ArduCam_beginCaptureImage(handle)
    if rtn_val != 0:
        print("Error beginning capture, rtn_val = ",rtn_val)
        running = False
        return
    else:
        print("Capture began, rtn_val = ",rtn_val)

    while running_capture:
        #print "capture"
        rtn_val = ArducamSDK.Py_ArduCam_captureImage(handle)
        if rtn_val > 255:
            flag_capture_error = True
            print("Error capture image, rtn_val = ",rtn_val)
            if rtn_val == ArducamSDK.USB_CAMERA_USB_TASK_ERROR:
                break
        else:
            flag_capture_error = False

    running_capture = False
    ArducamSDK.Py_ArduCam_endCaptureImage(handle)

# Get the image from FIFO, perform the conversion, and show the image
def readImage_thread():

    global handle,running,Width,Height,saved_flag,cfg,color_mode,save_raw, folder, addr, intensity, z, running_read, z_new
    global COLOR_BayerGB2BGR,COLOR_BayerRG2BGR,COLOR_BayerGR2BGR,COLOR_BayerBG2BGR
    data = {}

    cv2.namedWindow("Lens Free",1)

    running_read = True

    while running_read:

        if ArducamSDK.Py_ArduCam_availableImage(handle) > 0:
            rtn_val,data,rtn_cfg = ArducamSDK.Py_ArduCam_readImage(handle)
            datasize = rtn_cfg['u32Size']

            if rtn_val != 0:
                print(">>>>> Read data fail!")
                continue

            if datasize == 0:
                continue

            image = convert_image(data,rtn_cfg,color_mode)

            today = date.today()
            # Month abbreviation, day and year
            current_date = today.strftime("%b-%d-%Y")
            timestr = time.strftime("%Y%m%d-%H%M%S")
            ## save image in the folder
            z_str = '%d'%z_new
            z_str = z_str.zfill(4)
            if saved_flag:
                if not os.path.exists(addr+"\\images_multispectral\\"+current_date+"_"+folder+"_int%.2f"%intensity):
                    os.makedirs(addr+"\\images_multispectral\\"+current_date+"_"+folder+"_int%.2f"%intensity)
                #path =addr+"\\images_multispectral\\"+current_date+"_"+folder+"_int%.2f"%intensity+"\\image%d"%z_new+"-"+timestr+".jpg"
                path = addr+"\\images_multispectral\\"+current_date+"_"+folder+"_int%.2f"%intensity+"\\image"+z_str+"-"+timestr+".jpg"
                if(cv2.imwrite(path, image)):
                    print("Image saved successfully!!\n")
                    z_new+=1

                else:
                    print(">>>>> ERROR to save image.\n")

                saved_flag = False

            image = cv2.resize(image,(640,480),interpolation = cv2.INTER_LINEAR)

            cv2.imshow("Lens Free",image)
            cv2.waitKey(5)
            ArducamSDK.Py_ArduCam_del(handle)

        else:
            time.sleep(0.001);

# Check the USB port that connects the LEDs to the computer
def check_usb_port():

    active_ports = []
    for number in ['COM%s' % (i + 1) for i in range(32)]:
        try:
            check_obj = serial.Serial(number)
            active_ports.append((number))
            check_obj.close()

        except serial.SerialException:
            pass
    return active_ports

# Perform the LEDs changing and the image acquisition control
def LED_thread():

    global saved_flag, running_acquisition, z, led_flag, ser, n_pictures, flag_capture_error

    running_acquisition = True
    while running_acquisition:
        if led_flag == True:

            if flag_capture_error:
                break

            if(z<(n_pictures)):

                if not saved_flag:
                    if(z<n_pictures):
                        # Turn on only one LED color
                        if z==0:
                            ArducamSDK.Py_ArduCam_writeSensorReg(handle,12306,int(exposure[0]))
                            #ArducamSDK.Py_ArduCam_writeSensorReg(handle,12343,int(3))
                            ser.write(b'R')
                            time.sleep(2)
                            saved_flag = True
                            print('Image ', z+1,' - LED: Red')
                            time.sleep(5)
                        if z==1:
                            ArducamSDK.Py_ArduCam_writeSensorReg(handle,12306,int(exposure[1]))
                            ser.write(b'G')
                            time.sleep(2)
                            saved_flag = True
                            print('Image ', z+1,' - LED: Green')
                            time.sleep(5)
                        if z==2:
                            ArducamSDK.Py_ArduCam_writeSensorReg(handle,12306,int(exposure[2]))
                            ser.write(b'B')
                            time.sleep(2)
                            saved_flag = True
                            print('Image ', z+1,' - LED: Blue')
                            time.sleep(5)
                        ser.write(b'X')
                        saved_flag = False
                        z+=1
            else:
                led_flag = False # Turn off the LED
                z=0
                running_acquisition = False

def init_components():

    global config_file_name, ci, rt, lt, ser, flag_erro, flag_capture_error

    # Start the LEDs
    print("Starting components: \n")
    print('--- LED RGB --- \n')
    try:
        # Start the LED
        port=check_usb_port()
        ser = serial.Serial(port[0], 9600, timeout=1)
        print('> Status: OK!')

    except:
        print('>>>>> Status: Error!')
        print(' LED not found')
        #print('Please, check if the LED is plugged in or if it is connected to the computer.\n')
        flag_erro = True

    # Start the camera
    print('\n--- Camera ---\n')
    ser.write(b'L')     # Connect the camera to the computer
    time.sleep(2)
    if camera_initFromFile(config_file_name):
        ArducamSDK.Py_ArduCam_setMode(handle, ArducamSDK.CONTINUOUS_MODE)
        print('> Status: Ok!\n')
    else:
        int('>>>>> Status: Error!')
        int(" Please, check if the camera driver has been installed correctly or if the camera is connected.\n")
        flag_erro = True

    if (flag_erro):
        exit()

    # Create the threads
    ci = threading.Thread(target = captureImage_thread)
    rt = threading.Thread(target = readImage_thread)
    lt = threading.Thread(target = LED_thread)

    # Start the capture and read threads
    ci.start()
    rt.start()

def close_components():

    global running_capture, running_acquisition, running_read

    running_capture = False
    running_acquisition = False
    running_read = False

    global ci, rt, lt, ser
    # Close the threads
    ci.join()
    rt.join()

    cv2.destroyAllWindows()
    cv2.waitKey(1)

    print("Closing components:\n")

    rtn_val = ArducamSDK.Py_ArduCam_close(handle)
    if rtn_val == 0:
        print('> Camera closed successfully!\n')
        ser.write(b'D')     # Disconnect the camera to the computer
        time.sleep(2)
    else:
        print('>>>>> ERROR to close camera.\n')

    try:
        ser.close()
        print('> LEDs closed successfully!\n')
    except:
        print(">>>>> ERROR to close LED!")

def menu():

    print("\n======= LENS FREE MICROSCOPY =======\n")
    print("Command options:\n")
    print("  I - Change intensity values")
    print("  P - Preview (with blue LED)")
    print("  S - Start multispectral acquisition")
    print("  Z - Start a sequence of multiwavelength acquisition with defined interval")
    print("  C - Close program\n")

def parameters():

    global exposure_time, intensity

    print('\n. Number of images: %s' %n_pictures)
    print('. LED: RGB')
    print('. Intensity proportion: %.2f' %intensity)
    print('. Exposure time for each LED is: %.2f s (Red), %.2f s (Green) and %.2f s (Blue)' %(exposure_time[0], exposure_time[1], exposure_time[2]))
    print('------------------------------------------------\n')

def countdown(t):

    while t:
        mins, secs = divmod(t, 60)
        timer = '> {:02d}:{:02d}'.format(mins, secs)
        print(timer, end="\r")
        time.sleep(1)
        t -= 1
    print('End of the acquisition process.\n')
    print('------------------------------------------------\n')

# ------------ Main function -------------------- #
def main():

    global running, led_flag, folder, exposure, exposure_time, ser, intensity, saved_flag, config_file_name, flag_erro, n_pictures, flag_capture_error, time_preview

    flag_erro = False

    #print("\n======= LENS FREE MICROSCOPY =======\n")

    ## Adress of JSON
    config_file_name = addr+"\\Json\\MT9J001_10MP.json"

    if not os.path.exists(config_file_name):
        print("File JSON not found.")
        exit()

    #init_components()

    while running:

        if not led_flag:
            time.sleep(1)
            menu()

            input_kb = str(sys.stdin.readline()).strip("\n")

            if input_kb == 'i' or input_kb == 'I': ## Time exposure adjust for the camera
                print('Given the defaut exposure time for each LED is %.2f s (Red), %.2f s (Green) and %.2f s (Blue).'%(exposure_time[0], exposure_time[1], exposure_time[2]))
                intensity = float(input("Insert the intensity proportion value: "))
                exposure_time = [i * intensity for i in exposure_time]
                exposure = [round(i *1000/prp_exp) for i in exposure_time]
                ArducamSDK.Py_ArduCam_writeSensorReg(handle,12306,int(exposure[2]))

        elif input_kb == 'p' or input_kb == 'P': ## Preview image - turn on blue LED
            init_components()

            time.sleep(2)
            if not flag_capture_error:
                time_preview = int(input("Insert time (s) for preview:"))
                ser.write(b'B')
                time.sleep(time_preview)
                ser.write(b'X')
            #if lt.is_alive():
            #    lt.join()

            close_components()

        elif input_kb == 'c' or input_kb == 'C': ## Close the program
            running = False

        elif input_kb == 'z' or input_kb == 'Z': ## Multiple acquisitions
            folder = str(input("Insert folder name:"))
            num_acquisitions = int(input("Insert the number of acquisitions:"))
            time_interval = float(input("Insert the time interval (min) between acquisitions:"))
            n=0
            while n< num_acquisitions:

                if n==0:
                    init_components()
                if not n==0:
                    init_components()

                    print('------------------------------------------------')
                    print("[Iteration %d] Acquisition started with the following parameters:" %(n+1))
                    parameters()
                    time.sleep(2)

                    start_acquisition = time.time()
                    led_flag = True

                    if flag_capture_error:
                        print(">>>>> FLAG_CAPTURE_ERROR")
                        n = n - 1

                    if not lt.is_alive():
                        lt.start()
                    if lt.is_alive():
                        lt.join()

                    #close_components()

                    if not n==(num_acquisitions - 1):
                        close_components()

                        if flag_capture_error:
                            pass

                        else:
                            end_acquisition = time.time()
                            time_aquisition = end_acquisition - start_acquisition
                            time_interval_seg = time_interval*60 - time_aquisition
                            print('Please, wait for more %.1f s' %time_interval_seg)
                            countdown(int(time_interval_seg))
                    n=n+1        
                close_components()

        elif (input_kb == 's' or input_kb == "S"): # Single acquisition
            init_components()
            time.sleep(2)
            
            if not flag_capture_error:
                folder = str(input("Insert folder name:"))
                print('------------------------------------------------')
                print("Acquisition started with the following parameters:")
                parameters()
                time.sleep(2)
                led_flag = True
                
                if not lt.is_alive():
                    lt.start()
                if lt.is_alive():
                    lt.join()
            close_components()

            '''if lt.is_alive():
                lt.join()'''

    #close_components()

if __name__ == '__main__':
    sys.exit(main())

