import cv2
import numpy as np

# Load the grayscale image
image = cv2.imread('/home/ofa/ros2_ws/src/ros-weed-control/runs/20240814_115106/back_0/images/ir.png', cv2.IMREAD_GRAYSCALE)
# image = cv2.imread('/home/ofa/ros2_ws/src/ros-weed-control/runs/20240814_125043/back_2/images/ir.png', cv2.IMREAD_GRAYSCALE)
# image = cv2.imread('/home/ofa/ros2_ws/src/ros-weed-control/runs/20240814_144821/back_0/images/ir.png', cv2.IMREAD_GRAYSCALE)
# image = cv2.imread('/home/ofa/ros2_ws/src/ros-weed-control/ofa_weed_detection/images/mock_images/back/ir.png', cv2.IMREAD_GRAYSCALE)

min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(image)
print(f"Maximum value: {max_val}")

# Estimate the illumination pattern using a large Gaussian blur
kernel_size = 501  # Large kernel size for blurring to capture lighting pattern
# illumination_pattern = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
illumination_pattern = cv2.imread('illumination3.png', cv2.IMREAD_GRAYSCALE)
# illumination_pattern = cv2.divide(illumination_pattern, 4)

# min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(image)
# print(f"Extremum illumination_invert value: {min_val, min_loc}")

# print(illumination_pattern[1788, 3838])

# Option 1: Subtract the illumination pattern (Additive model)
corrected_image_subtract = cv2.subtract(image, illumination_pattern)
corrected_image_subtract = cv2.normalize(corrected_image_subtract, None, 0, 255, cv2.NORM_MINMAX)

# Option 2: Divide by the illumination pattern (Multiplicative model)
image_float = image.astype(np.float32)  # Convert image to float for division
illumination_pattern_float = illumination_pattern.astype(np.float32)  # Convert illumination pattern to float
corrected_image_divide = image_float / illumination_pattern_float * 150
ret, corrected_image_divide = cv2.threshold(corrected_image_divide, 255, 255, cv2.THRESH_TRUNC)
# corrected_image_divide = cv2.normalize(corrected_image_divide, None, 0, 255, cv2.NORM_MINMAX)
corrected_image_divide = corrected_image_divide.astype(np.uint8)  # Convert back to 8-bit
ret, thresh_image = cv2.threshold(corrected_image_divide, 70, 255, cv2.THRESH_BINARY)

min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(corrected_image_divide)
print(f"Maximum value: {max_val}")

# Display the results
cv2.imwrite('original.png', image)
# cv2.imwrite('illumination.png', illumination_pattern)
cv2.imwrite('subtract.png', corrected_image_subtract)
cv2.imwrite('divide.png', corrected_image_divide)
cv2.imwrite('thresh.png', thresh_image)
# cv2.imshow('Original Image', image)
# cv2.imshow('Illumination Pattern', illumination_pattern)
# cv2.imshow('Corrected Image - Subtract', corrected_image_subtract)
# cv2.imshow('Corrected Image - Divide', corrected_image_divide)

cv2.waitKey(0)  # Wait for a key press to close the windows
cv2.destroyAllWindows()
