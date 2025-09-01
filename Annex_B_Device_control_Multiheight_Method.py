
# ANNEX B – PYTHON CODE TO CONTROL DEVICES FOR MULTIHEIGHT METHOD

"""
*** Python code to control the multiheight lens-free microscope ***
    Control of the camera (Arducam MT9J001) and actuator
"""

# ------------- Libraries  --------------------- #
import sys
import time
import os
import threading

import pylibftdi
import pyAPT

import cv2
import numpy as np
import signal
import json
from ImageConvert import *
import ArducamSDK

global cfg,handle,running,Width,Heigth,saved_flag,color_mode,save_raw,home_position,serial,delta_z, rtn_val, con, translation, z, folder, exposure

# ---------- CONTROL PARAMETERS --------------- #

z = 0                 ## Auxiliary variable to control the image acquisition
translation = True    ## True if Translation is off and can be used
running     = True    ## True if the program is running
saved_flag  = True    ## True to save the current image
cfg = {}
handle = {}

# ---------- ADJUSTABLE PARAMETERS --------------- #

save_raw      = False        ## If the user wants the images to be registered with .raw format
serial        = '83854962'    ## Actuator serial number
home_position = 7.3500        ## Initial position of the camera
n_pictures    = 6        ## Number of images to be registered
delta_z       = 0.05        ## Moving step for the camera [mm]
exposure      = 2        ## Initial value for exposure values

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
        ArducamSDK.Py_ArduCam_setboardConfig(handle, int(command,16), int(value,16), int(index,16), int(buffsize,16), buffs)


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

        if regAddr != 12306: ## Add the desired exposition value
            ArducamSDK.Py_ArduCam_writeSensorReg(handle,regAddr,val)
        else:
            ArducamSDK.Py_ArduCam_writeSensorReg(handle,regAddr,exposure)

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

# Perform image capture and add FIFO)
def captureImage_thread():
    
    global handle,running
    rtn_val = ArducamSDK.Py_ArduCam_beginCaptureImage(handle)
    if rtn_val != 0:
        print("Error beginning capture, rtn_val = ",hex(rtn_val))
        running = False
        return
    #else:
        #print("Capture began, rtn_val = ",rtn_val)

    while running:
        #print "capture"
        rtn_val = ArducamSDK.Py_ArduCam_captureImage(handle)
        if rtn_val > 255:
            print("Error capture image, rtn_val = ",hex(rtn_val))
            if rtn_val == ArducamSDK.USB_CAMERA_USB_TASK_ERROR:
                break
        time.sleep(0.005)

    running = False
    ArducamSDK.Py_ArduCam_endCaptureImage(handle)

# Get the image from FIFO, perform the conversion, and show the image
def readImage_thread():
    global handle,running,Width,Height,saved_flag,cfg,color_mode,save_raw, folder, addr, exposure, con, z
    global COLOR_BayerGB2BGR,COLOR_BayerRG2BGR,COLOR_BayerGR2BGR,COLOR_BayerBG2BGR
    time0 = time.time()
    time1 = time.time()
    data = {}

    cv2.namedWindow("Lens Free",1)

    while running:

        display_time = time.time()
        if ArducamSDK.Py_ArduCam_availableImage(handle) > 0:
            rtn_val,data,rtn_cfg = ArducamSDK.Py_ArduCam_readImage(handle)
            datasize = rtn_cfg['u32Size']

            if rtn_val != 0:
                print("read data fail!")
                continue

            if datasize == 0:
                continue

            image = convert_image(data,rtn_cfg,color_mode)

            time1 = time.time()
            if time1 - time0 >= 1:
                #print("%s %d %s\n"%("fps:",count,"/s"))
                count = 0
                time0 = time1

            ## save the image in the folder
            if not saved_flag:
                if not os.path.exists(addr+"\\images\\"+folder+"_exp"+str(exposure)):
                    os.makedirs(addr+"\\images\\"+folder+"_exp"+str(exposure))

                if(cv2.imwrite(addr+"\\images\\"+folder+"_exp"+str(exposure)+"\\image%d.jpg"%z,image)):
                    print("The image has been saved successfully!\n")
                else:
                    print("Error saving image!\n")

                if save_raw:
                    with open(addr+"\\images\\"+folder+"_exp"+str(exposure)+"\\image%d.raw"%z,'wb') as f:

                        f.write(data)

                saved_flag = True

            image = cv2.resize(image,(640,480),interpolation = cv2.INTER_LINEAR)

            cv2.imshow("Lens Free",image)
            cv2.waitKey(10)
            ArducamSDK.Py_ArduCam_del(handle)
            #print("------------------------display time:",(time.time()- display_time))
    
        else:
            time.sleep(0.001);



# Performs the translation and controls the image acquisition
def translation_thread():

    global saved_flag, delta_z, running, z, translation, home_position,con

    while running:
        if translation == False:

            if(z<(n_pictures)):
                saved_flag = False

                if (saved_flag):
                    if(z<n_pictures-1):
                        con.move(delta_z)
                        time.sleep(3)
                        print('Step %d - Position: %.2f'%(z+1,con.position()))

                    #time.sleep(3)
                    #saved_flag = False
                    z+=1
            else:
                con.goto(home_position)
                translation = True
                print("End of the acquisition process.\n")
                z=0

def menu():

    print("\n======= LENS FREE MICROSCOPY =======\n")
    print("Command options:\n")
    print("  I - Change intensity values")
    print("  S - Start multiheight acquisition")
    print("  C - Close program\n")

def parameters():

    print('\n. Number of images: %s' %n_pictures)
    print('. Exposure time: %s' %exposure)
    print('. Delta_z: %.2f\n' %delta_z)

# ------------ Main function -------------------- #
def main():

    global running, translation, con, folder, exposure

    flag_erro = False

    print("\n======= LENS FREE MICROSCOPY =======\n")

    # Adress of JSON
    config_file_name = addr+"\\Json\\MT9J001_10MP.json"

    if not os.path.exists(config_file_name):
        print("File JSON not found.")
        exit()

    # Starts the actuator
    print("Starting components. Please, wait ... \n")
    print('--- Actuator ---')
    try:
        con = pyAPT.MTS50(serial_number = serial)

        print('\nStatus: OK!')
        con.goto(home_position)

        print('. Initial position: %.2f' %(con.position()))

    except:
        print('>>>>> Status: Error')
        print(' Actuator %s not found'%serial)
        print(' Please, check if the actuator is plugged in or if it is connected to the computer.\n')
        flag_erro = True

    # Starts the camera
    print('\n--- Camera ---\n')
    if camera_initFromFile(config_file_name):
        ArducamSDK.Py_ArduCam_setMode(handle, ArducamSDK.CONTINUOUS_MODE)
        print('> Status: Ok!\n')
    else:
        print('>>>>> Status: Error')
        print(" Please, check if the camera driver has been installed correctly or if the camera is connected.\n")
        flag_erro = True

    if (flag_erro):
        exit()


    # Create the threads
    ci = threading.Thread(target = captureImage_thread)
    rt = threading.Thread(target = readImage_thread)
    tt = threading.Thread(target = translation_thread)

    # Start the treads associate with the capture and read the images
    ci.start()
    rt.start()

    while running:

        if translation:
            time.sleep(1)
            menu()

            input_kb = str(sys.stdin.readline()).strip("\n")

            if input_kb == 'i' or input_kb == 'I': ## Exposure values changing

                print("Enter the new exposure value:")
                exposure = sys.stdin.readline().strip("\n")

                ArducamSDK.Py_ArduCam_writeSensorReg(handle,12306,int(exposure))
                
            elif input_kb == 'c' or input_kb == 'C':
                running = False

            elif input_kb == 's' or input_kb == 'S': ## Starts the image acquisition
                folder = input("Insert folder name:")
                print("\nAcquisition started with the following parameters:")
                parameters()
                time.sleep(2)
                print('. Step %d - Position: %.2f' %(z,con.position()))
                translation = False

                if not tt.is_alive():
                    tt.start()
    # Create the threads
    ci = threading.Thread(target=captureImage_thread)
    rt = threading.Thread(target=readImage_thread)
    tt = threading.Thread(target=translation_thread)

    # Start the threads associated with capture and image reading
    ci.start()
    rt.start()

    while running:
        if translation:
            time.sleep(1)
            menu()
            input_kb = str(sys.stdin.readline()).strip("\n")

            if input_kb == 'i' or input_kb == 'I':  # Exposure values changing
                print("Enter the new exposure value:")
                exposure = sys.stdin.readline().strip("\n")
                ArducamSDK.Py_ArduCam_writeSensorReg(handle, 12306, int(exposure))

            elif input_kb == 'c' or input_kb == 'C':
                running = False

            elif input_kb == 's' or input_kb == 'S':  # Starts the image acquisition
                folder = input("Insert folder name:")
                print("\nAcquisition started with the following parameters:")
                parameters()
                time.sleep(2)
                print('. Step %d - Position: %.2f' % (z, con.position()))
                translation = False

                if not tt.is_alive():
                    tt.start()


    ci.join()
    rt.join()

    if tt.is_alive():
        tt.join()

    print("Closing components. Please, wait ...\n")

    rtn_val = ArducamSDK.Py_ArduCam_close(handle)

    con.home()

    if(con.position() == 0):
        print('> The actuator has been closed successfully!')
    else:
        print('>>>>> ERROR to close the actuator.')

    if rtn_val == 0:
        print('> The camera has been closed successfully!\n')
    else:
        print('>>>>> ERROR to close camera.\n')

if __name__ == '__main__':
    sys.exit(main())

