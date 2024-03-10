import numpy as np
import cv2
import glob

images = glob.glob('*.jpg')
target_size = (640,480)

for image_filename in images:
    image = cv2.imread(image_filename)
    resized_img = cv2.resize(image, target_size)
    cv2.imwrite(image_filename, resized_img)