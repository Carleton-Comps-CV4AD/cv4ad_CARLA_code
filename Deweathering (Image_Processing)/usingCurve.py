import cv2
import numpy as np


#adapted from google gemini


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
