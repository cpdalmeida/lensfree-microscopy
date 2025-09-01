# ANNEX A – PHASE RECOVERY PYTHON CODE FOR MULTIHEIGHT METHOD

'''
*** Multiwavelenth code - alingment and phase reconstruction ***
	~ for intire images (all Field of View) ~
'''


# ------------- Libraries  --------------------- #
import cv2                           # Library for image processing
import numpy as np                   # Library for math
import os                            # Library for orgnizing archives
from matplotlib import pyplot as plt # Library for display imgs
import time                          # Library for time counting

# ------------- Functions --------------------- #
# Function to rescale an image
def rescale(img, scale_factor):
    width = img.shape[1] 
    ImgScale = scale_factor/width 
    newX, newY = img.shape[1]*ImgScale, img.shape[0]*ImgScale
    image = cv2.resize(img,(int(newX),int(newY)))
    Height,Width = image.shape
    return image, ImgScale

# Global variables that are used on click_and_crop function
refPt = [] # vector to save the selected reference coordinates 
cropping = False

# click_and_crop shows an image and saves the reference coordinates of the user-drawn rectangle
def click_and_crop(event, x, y, flags, param):
    global refPt, cropping
    
    # event is an user action 
    if event == cv2.EVENT_LBUTTONDOWN: # reference initial coordinate set on the user click with 
        refPt = [(x,y)]                # left mouse button
        cropping = True

    elif event == cv2.EVENT_LBUTTONUP:  # reference final coordinate set on the user click with 
        refPt.append((x,y))             # left mouse button
        cropping = False
        # draw a retangle with the reference coordinates
        cv2.rectangle(image, refPt[0], refPt[1], (0, 255, 0), 2)
        cv2.imshow("image", image)

# load_images opens the selected folder and return a list with all images that it has inside
def load_images(folder):
    I = []
    
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename), 0)
        if img is not None:
            I.append(img)
    return I

# crop images with selected reference coordinates
def crop(image): 
    clone = image.copy()
    return clone[int(refPt[0][1]/Scale):int(refPt[1][1]/Scale),int(refPt[0][0]/Scale):int(refPt[1][0]/Scale)]#função que corta nos pontos de referência

def Interactive_propagation(val):
    global img_crop, z_value, lambda_value
    lambda_value = cv2.getTrackbarPos('Lambda', window_name)
    z_value = cv2.getTrackbarPos('10z', window_name)
    z_value = z_value/10

    Uimg = U(img_crop,z_value*1000,lambda_value*0.001)
    
    img_show = rescale_intensity(np.abs(Uimg))
    cv2.imshow(window_name, img_show)
    
def alignment(im1,im2,im2_fov):
    sz = im2_fov.shape
    # Define the motion model
    warp_mode = cv2.MOTION_TRANSLATION

    # Define 2x3 or 3x3 matrices and initialize the matrix to identity
    if warp_mode == cv2.MOTION_HOMOGRAPHY :
        warp_matrix = np.eye(3, 3, dtype=np.float32)
    else :
        warp_matrix = np.eye(2, 3, dtype=np.float32)

    # Specify the number of iterations.
    number_of_iterations = 5000;

    # Specify the threshold of the increment
    # in the correlation coefficient between two iterations
    termination_eps = 1e-10;

    # Define termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, number_of_iterations,  termination_eps)
    
    # Run the ECC algorithm. The results are stored in warp_matrix.
    (cc, warp_matrix) = cv2.findTransformECC(im1,im2,warp_matrix, warp_mode, criteria, inputMask=None, gaussFiltSize=1)

    if warp_mode == cv2.MOTION_HOMOGRAPHY :
        im_aligned = cv2.warpPerspective (im2_fov, warp_matrix, (sz[1],sz[0]), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
    else:
        im_aligned = cv2.warpAffine(im2_fov, warp_matrix, (sz[1],sz[0]), flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)

    return im_aligned

def imgradient(img):
    grad_x = cv2.Sobel(img, cv2.CV_8UC1, 1,0)
    grad_y = cv2.Sobel(img, cv2.CV_8UC1, 0,1)
    abs_grad_x = cv2.convertScaleAbs(grad_x)
    abs_grad_y = cv2.convertScaleAbs(grad_y)
    grad = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
    return grad

def rescale_intensity(img):
    min_val,max_val=img.min(),img.max()
    img = 255.0*(img - min_val)/(max_val - min_val)
    img = img.astype(np.uint8)
    return img


# ------------- Parameters --------------------- #
# -------- units: micrometers ------------------
dx = 1.67; dy = dx          # pixel size
refractive_index = 1        # refractive index - medium between sample to sensor
num_iterations = 10         # number of iterations
deltaz = 50                 # distance between captured holograms     

# ------------- Loading images --------------------- #
# Replace the current file path with the path to the images on your computer
path = r"C:\Users\Camila\Downloads\leveduras_azul_metileno_exp900"
Original_imgs = load_images(path)


# --------- Selecting cropping area ---------------- #
img = Original_imgs[0]

# Rescaling image to display 
image, Scale = rescale(img, 1000)  
clone = image.copy()
cv2.namedWindow("image")
cv2.setMouseCallback("image",click_and_crop)

while True:
    cv2.imshow("image",image)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("r"):
        image = clone.copy()

    elif key == ord("c"):
        break

if len(refPt)==2:
    crop_img = clone[refPt[0][1]:refPt[1][1],refPt[0][0]:refPt[1][0]]
    newimg, imgScale = rescale(crop_img, 1000)


# --------- Cropping Original images ---------------- #
Cropped_imgs = []
for img in Original_imgs:
    Cropped_imgs.append(crop(img))


# -------- Propagation - Cropped images ------------- #

# image to be used as reference for functions of propagation
img_crop = Cropped_imgs[0] 
[Nx,Ny] = img_crop.shape

# Frequence domain matrix
f = np.linspace(0,0+Nx, Nx,endpoint=False)
fxv = np.fft.ifftshift((f-np.floor(Nx/2+1))*(1/(Nx*dx)))
f = np.linspace(0,0+Ny, Ny,endpoint=False)
fyv = np.fft.ifftshift((f-np.floor(Ny/2+1))*(1/(Ny*dx)))
fx, fy = np.meshgrid(fxv, fyv, sparse=False, indexing='ij')

# Transfer function
def TF(z,Lambda):
    tf = np.exp((1j*refractive_index*2*np.pi*z*np.sqrt((1/Lambda)**2-fx**2-fy**2)))
    return tf

def U(Ui,z,Lambda):
    Uift = np.fft.fft2(Ui)
    u = np.fft.ifft2(np.multiply(Uift,TF(z,Lambda)))
    return u

# ---------------- Autofocus ------------------------ #
def autofocus(img, z2_min, z2_max, n_z2, Lambda):
    hU = np.sqrt(img)
    deltaz= (z2_max - z2_min)/n_z2
    z2_vector = np.arange(z2_min,z2_max,deltaz)
    tot_img = z2_vector.size
    U0 = []
    TC_list = []
    for i in range(tot_img):
        U0.append(np.abs(U(hU, z2_vector[i], Lambda)))
        grad = imgradient(U0[i])
        mean = np.mean(grad)
        std = grad.std()
        TC = np.sqrt(std/mean)
        TC_list.append(TC)
    
    TC_max = max(TC_list) 
    a = 0
    for i in range(len(TC_list)):
        if TC_list[i] == TC_max:
            a = i
    return z2_vector[a]

# ------ trackbar to define z2_vector and lambda --------- #
window_name = 'Propagation Controls for Amplitude Image'
# Create window
cv2.namedWindow(window_name)
# Create Trackbar to choose variable values
cv2.createTrackbar('Lambda', window_name, 455, 700, Interactive_propagation)  
cv2.createTrackbar('10z', window_name , 8, 28, Interactive_propagation)  
# Call the function to initialize
Interactive_propagation(0)
# Wait until user finishes program
cv2.waitKey()

start = time.time() ########################################

# Redifine parameters 
Lambda = lambda_value/1000         # selected lambda in micrometers
z_value = z_value*1000             # selected z2 in micrometers
dz2 = 0.01                         # step bettween z2 values
n_z2 = 50                          # number of divisions to be consider between z2_min and z2_max
z2_min = z_value - (n_z2//2)*dz2   # mininum distance to be used as input of autofocus function
z2_max = z_value + (n_z2//2)*dz2   # maximum distance to be used as input of autofocus function


z2 = []
for a in range(len(Cropped_imgs)):
    z2.append(autofocus(Cropped_imgs[a], z2_min, z2_max, n_z2, Lambda))
    
OnFocus_imgs = []
for a in range(len(Cropped_imgs)):
    OnFocus_imgs.append(np.uint8(np.absolute(U(Cropped_imgs[a],z2[a],Lambda))))

# -------- Alignment - On Focus images ------------- #

Aligned_imgs = []
Aligned_imgs.append(Original_imgs[0])
for i in range(len(OnFocus_imgs)):
    if i != 0:
        Aligned_imgs.append(alignment(OnFocus_imgs[0],OnFocus_imgs[i],Original_imgs[i]))
print("Alignment done \n")


# ****** Ploting On Focus (Cropped) images **********
fig = plt.figure(figsize=(10, 7))

fig.add_subplot(1, 3, 1)            # (rows, columns, position)
plt.imshow(img_crop,cmap='gray')
plt.axis('off')
plt.title("Hologram")

fig.add_subplot(1, 3, 2)            # (rows, columns, position)
plt.imshow(np.absolute(U(Cropped_imgs[0],z2[0], Lambda)),cmap='gray')
plt.axis('off')
plt.title("Amplitude - On focus")

fig.add_subplot(1, 3, 3)            # (rows, columns, position)
plt.imshow(np.angle(U(Cropped_imgs[0],z2[0], Lambda)),cmap='gray')
plt.axis('off')
plt.title("Phase - On focus")

plt.show()


# -------- Multiheight Phase Recorvering ------------ # 
# --------- Cropping and Aligned holograms ------------- # 
I = []   
for i in range(len(Aligned_imgs)):
    I.append(Aligned_imgs[i])
    
# -------- Propagation - Cropped images ------------- #

# image to be used as reference for functions of propagation
img = Original_imgs[0]
[Nx,Ny] = img.shape

# Frequence domain matrix
f = np.linspace(0,0+Nx, Nx,endpoint=False)
fxv = np.fft.ifftshift((f-np.floor(Nx/2+1))*(1/(Nx*dx)))
f = np.linspace(0,0+Ny, Ny,endpoint=False)
fyv = np.fft.ifftshift((f-np.floor(Ny/2+1))*(1/(Ny*dx)))
fx, fy = np.meshgrid(fxv, fyv, sparse=False, indexing='ij')

hU = []
U0 = []

for a in range(len(I)):
    hU.append(np.sqrt(I[a]))
    U0.append(U(hU[a],z2[a], Lambda))


print("Starting multiheight imaging processing")
print("................................................")

for i in range(num_iterations):
    print("iteration: ", i+1)
    
    for a in range(0,len(I)-1):
        U0[a+1] = U(U0[a], -deltaz, Lambda)
        U0[a+1] = np.multiply(hU[a+1],np.exp(1j*np.angle(U0[a+1])))

    for a in range(len(I)-1,0,-1):
        U0[a-1] = U(U0[a], deltaz, Lambda)
        U0[a-1] = np.multiply(hU[a-1],np.exp(1j*np.angle(U0[a-1])))


Hologram_img = Original_imgs[0]
Amplitude_img = np.abs(U(U0[0],z2[0], Lambda))
Phase_img = np.angle(U(U0[0],z2[0], Lambda))

# ****** Ploting Final Cropped images **********
fig = plt.figure(figsize=(10, 7))

fig.add_subplot(1, 3, 1)            # (rows, columns, position)
plt.imshow(Hologram_img,cmap='gray')
plt.axis('off')
plt.title("Hologram") 

fig.add_subplot(1, 3, 2)            # (rows, columns, position)
plt.imshow(Amplitude_img,cmap='gray')
plt.axis('off')
plt.title("Amplitude")

fig.add_subplot(1, 3, 3)            # (rows, columns, position)
plt.imshow(Phase_img,cmap='gray')
plt.axis('off')
plt.title("Phase")

plt.show()


# Running time
end = time.time()   ###################################
print("\n""Running time: ", round((end-start)//60),"min e ", round((end-start)%60), "s" )

cv2.imwrite(path+'\\Amplitude_final.png',rescale_intensity(Amplitude_img))
cv2.imwrite(path+'\\Phase_final.png',rescale_intensity(Phase_img))
cv2.imwrite(path+'\\OnFocus.png',rescale_intensity(np.absolute(U(Original_imgs[a],z2[a],Lambda))))