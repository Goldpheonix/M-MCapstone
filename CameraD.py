import math

def calculate_real_distance(d_pixels, focal_length, sensor_width, image_width, object_distance):
    # Calculate the scale (real world units per pixel)
    scale = sensor_width / image_width
    
    # Calculate the real-world distance between the two points
    d_real = d_pixels * scale
    
    # You could also use object distance (D) to adjust the measurement, if necessary.
    # For example, if you're using triangulation, you could factor in the object distance.
    
    return d_real

# Example parameters
d_pixels = 200          # Pixel distance between two points in the image
focal_length = 50       # Focal length in mm
sensor_width = 36       # Sensor width in mm (full-frame example)
image_width = 4000      # Image width in pixels
object_distance = 1000  # Real-world distance to the object (in mm or meters)

# Calculate the real-world distance between the points
real_distance = calculate_real_distance(d_pixels, focal_length, sensor_width, image_width, object_distance)

print(f"The real-world distance between the two points is: {real_distance} mm")
