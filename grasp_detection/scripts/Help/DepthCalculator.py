import cv2
import matplotlib.pyplot as plt
from scripts.Help.data_preparation.Calibration import CameraCalibrator
import time
import numpy as np

class DepthCalculator():
    def __init__(self):
        # RGB images
        self.l_img = None
        self.r_img = None

        # Image undistorters
        self.l_undist = CameraCalibrator("config/l_calib_params.yaml")
        self.r_undist = CameraCalibrator("config/r_calib_params.yaml") # It only works depending on the directory from which you launch it

        # Grayscale images
        self.gl_img = None
        self.gr_img = None

        # Disparity image
        self.disparity_img = None

        # Depth image
        self.depth_img = None

        # Disparity parameters
        self.nDisp = 32
        self.wSize = 11
        self.lmbda = 75e3
        self.sigma = 0.5

        # Create a camera video object
        camera_index = 2
        self.cap = cv2.VideoCapture(camera_index)

        # FHD resolution
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        self.cap.set(3, 1920)
        self.cap.set(4, 1080)
        self.cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

        # self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 64)
        # self.cap.set(cv2.CAP_PROP_CONTRAST, 0)

    def nothing(self, x):
        pass

    def get_images(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()
        # Display the resulting frame
        if ret:
            h = frame.shape[0]
            w = frame.shape[1]//2

            l, r = frame[:, :w,:], frame[:, w:,:]

            self.l_img = cv2.resize(l, (w//2, h//2))
            self.r_img = cv2.resize(r, (w//2, h//2))

            self.l_img = self.l_undist.undistort_img(self.l_img)
            self.r_img = self.r_undist.undistort_img(self.r_img)

            self.r_img = cv2.resize(self.r_img, (self.l_img.shape[1], self.l_img.shape[0]))

            # print(l.shape)
            # print(r.shape)

            # print(self.l_img.shape)
            # print(self.r_img.shape)

            self.gl_img = cv2.cvtColor(self.l_img,cv2.COLOR_BGR2GRAY)
            self.gr_img = cv2.cvtColor(self.r_img,cv2.COLOR_BGR2GRAY)
            return True
        
        else:
            print("Frame not captured")
            return False

    def param_gui(self):
        cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('disp',600,600)
        
        cv2.createTrackbar('nDisp','disp',10,20,self.nothing) #10
        cv2.createTrackbar('wSize','disp',4,20,self.nothing) #7 14 19 
        cv2.createTrackbar('lmbda','disp',15,20,self.nothing) #13
        cv2.createTrackbar('sigma','disp',10,20,self.nothing) #13

        while(self.get_images()):
            self.nDisp = cv2.getTrackbarPos('nDisp','disp')*16 + 16
            self.wSize = cv2.getTrackbarPos('wSize','disp')*2 + 3
            self.lmbda = cv2.getTrackbarPos('lmbda','disp')*10000 + 5000
            self.sigma = cv2.getTrackbarPos('sigma','disp')*0.1 - 0.4

            self.compute_disparity_img()

            cv2.imshow("disp",self.disparity_img)

            # Close window using esc key
            if cv2.waitKey(1) == 27:
                break
        
    def take_picture(self):
        while(True):
            self.get_images()

            self.compute_disparity_img()

            # self.compute_depth_image()

            # self.get_midpoint_depth()
            img = self.l_img
            scale = 1

            h, w = img.shape[:2]
            h *= scale
            w *= scale

            img = cv2.resize(img, (w, h))

            center = (w//2, h//2)
            radius = 5
            color = (0, 255, 0)
            thickness = 2
            cv2.circle(img, center, radius, color, thickness)
            cv2.imshow('left', img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()

        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()



    def compute_depth_image(self):
        f = 2.8 # mm
        B = 120 # mm
        pixel_size = 3e-3 # mm 

        self.depth_img = (f*B) / (pixel_size*self.disparity_img)
    
    def get_midpoint_depth(self):
        img = self.disparity_img
        h, w = img.shape
        w//=2
        h//=2
        
        center = (w, h)   
        radius = 5
        color = (0, 255, 0)
        thickness = 2

        print("Midpoint depth: ", self.depth_img[w, h], " mm, at position: ", center, ".")
        cv2.circle(img, center, radius, color, thickness)

        cv2.imshow('d', img)

    def store_disp_imgs(self, compute_params):
        if compute_params:
            self.wSize = 1
            for i in range(6):
                self.nDisp = 16
                self.wSize = self.wSize + 2
                for j in range(6):
                    name = "nDisp:"+str(self.nDisp)+"__wSize:"+str(self.wSize)+".jpg"
                    path = "/home/oscar/Escritorio/Master Thesis/SampleImages/ComputeP/"
                    
                    self.compute_disparity_img()
                    cv2.imwrite(path+name, self.disparity_img)
                    print("i: ", i, "nDisp: ", self.nDisp, "j: ", j, "wSize:", self.wSize)
                    self.nDisp = self.nDisp + 16
        else:
            self.lmbda = 0
            for i in range(6):
                self.sigma = 0.5
                self.lmbda = self.lmbda + 15000
                for j in range(6):
                    name = "sigma:"+str(self.sigma)+"__wSize:"+str(self.lmbda)+".jpg"
                    path = "/home/oscar/Escritorio/Master Thesis/SampleImages/FilterP/"
                    
                    self.compute_disparity_img()
                    cv2.imwrite(path+name, self.disparity_img)
                    print("i: ", i, "sigma: ", self.sigma, "j: ", j, "lmbda:", self.lmbda)
                    self.sigma = self.sigma + 0.2
                    self.sigma = round(self.sigma, 1)          

    def plot_imgs(self):
        cv2.imshow('left', self.l_img)
        cv2.imshow('right', self.r_img)

        cv2.imshow('disparity', self.disparity_img)

        cv2.waitKey(0)
        cv2.destroyAllWindows()


# dc = DepthCalculator()
# dc.take_picture()

# dc.param_gui()
# dc.get_images()
# dc.compute_disparity_img()
# dc.plot_imgs()

# # print(dc.depth_img.shape)
# # print(dc.disparity_img.shape)
