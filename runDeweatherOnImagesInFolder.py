import os
from usingCurve import create_curve
from usingCurve import apply_curve
import cv2
import os

def process_images_in_directory(directory):
    points = [(0, 0), (7, 34), (35, 64), (80, 66), (110, 145), (142, 135), (175, 173), (180, 200), (194, 175), (202, 153), (216, 253), (255, 255)]
    curve = create_curve(points)
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.lower().endswith('.png'):
                file_path = os.path.join(root, file)
                image = cv2.imread(file_path)
                enhanced_image = apply_curve(image, curve)
                output_file_path = os.path.join(root, f'{file}')
                cv2.imwrite(output_file_path, enhanced_image)
                

# Example usage
directory = '/Backup/val_clearNightDeweather/val/rgb'

process_images_in_directory(directory)
print('Done')