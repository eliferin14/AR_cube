import numpy as np
import cv2
import glob

images = glob.glob('../ChArUco_images/*.jpg')
target_size = (960,720)

for image_filename in images:
    image = cv2.imread(image_filename)
    resized_img = cv2.resize(image, target_size)
    cv2.imwrite(image_filename, resized_img)