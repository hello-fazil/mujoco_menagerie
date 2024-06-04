import math

def calculate_vfov(hfov_degrees, image_width, image_height):
    # Convert HFOV from degrees to radians
    hfov_radians = math.radians(hfov_degrees)
    
    # Calculate the aspect ratio
    aspect_ratio = image_height / image_width
    
    # Calculate VFOV in radians
    vfov_radians = 2 * math.atan(aspect_ratio * math.tan(hfov_radians / 2))
    
    # Convert VFOV back to degrees
    vfov_degrees = math.degrees(vfov_radians)
    
    return vfov_degrees

# Example usage
hfov = 90  # horizontal field of view in degrees
width = 640  # image width in pixels
height = 480  # image height in pixels
vfov = calculate_vfov(hfov, width, height)
print(f"Vertical Field of View: {vfov} degrees")