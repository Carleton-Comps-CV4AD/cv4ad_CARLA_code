import cv2
import numpy as np


#from google gemnni


def apply_curve(image, curve):
   """Applies a curve adjustment to an image.


   Args:
       image: The input image as a NumPy array.
       curve: A lookup table (LUT) representing the curve.


   Returns:
       The enhanced image as a NumPy array.
   """
   return cv2.LUT(image, curve)


def create_curve(points):
   """Creates a curve lookup table (LUT) from control points.


   Args:
       points: A list of (x, y) tuples representing control points,
               where x and y are in the range [0, 255].


   Returns:
       A NumPy array of shape (256,) representing the curve LUT.
   """
   curve_x = [p[0] for p in points]
   curve_y = [p[1] for p in points]
   curve = np.interp(np.arange(256), curve_x, curve_y).astype(np.uint8)
   return curve


# # Load the image
# image = cv2.imread('700.png')


# # Define control points for the curve
# # Example: S-curve for contrast enhancement
# points = [(0, 0), (7, 34), (35, 64), (80, 66), (110, 145), (142, 135), (175, 173), (180, 200), (194, 175), (202, 153), (216, 253), (255, 255)]


# # Create the curve LUT
# curve = create_curve(points)


# # Apply the curve to the image
# enhanced_image = apply_curve(image, curve)


# # Display or save the enhanced image
# cv2.imwrite('700dw.png', enhanced_image)
