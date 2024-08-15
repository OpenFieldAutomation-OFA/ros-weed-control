import cv2
import numpy as np

# Estimate the illumination pattern using a large Gaussian blur
kernel_size = 15  # Large kernel size for blurring to capture lighting pattern
# image = cv2.imread('/home/ofa/ros2_ws/src/ros-weed-control/ofa_weed_detection/images/mock_images/back/ir.png', cv2.IMREAD_GRAYSCALE)
image = cv2.imread('ir_new.png', cv2.IMREAD_UNCHANGED)
image = image.astype(np.float32)
min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(image)
print(f"Maximum value: {max_val}")
image *= 8
image = image.astype(np.uint16)
min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(image)
print(f"Maximum value: {max_val}")
# # image2 = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
# cv2.imwrite('ir_new.png', image)