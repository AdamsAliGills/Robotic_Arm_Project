import numpy as np

# Camera Calibration Parameters
FX, FY = 699.42, 683.60
CX, CY = 354.77, 230.91
CENTER_U, CENTER_V = 284, 235
CAM_TO_PLANE_MM = 160.0

# Robot Frame Offsets
TRAY_CENTER_X_MM = 140.0
TRAY_CENTER_Y_MM = 0.0

# Z HEIGHT CALCULATION
# Robot base is elevated 55mm.
# Adjust based on the surface height of M&Ms
Z_TARGET_MM = -160.0 

CAMERA_TO_PROBE = {
    "x": -115.0,     # mm
    "y": 15.0,       # mm (probe is 45mm to the right of the camera)
    "z": 47.0        # mm
}

def pixel_to_world(u, v, z_world=Z_TARGET_MM):
    """
    Convert pixel coordinates to world coordinates
    
    Args:
        u (int): Pixel x-coordinate
        v (int): Pixel y-coordinate
        z_world (float, optional): Z height in mm. Defaults to Z_TARGET_MM.
    
    Returns:
        tuple: (x_world, y_world, z_world) in mm
    """
    # Pixel offset from center
    du = u - CENTER_U
    dv = v - CENTER_V
    
    # Convert pixel offsets to camera plane distances
    dx_cam = du * (CAM_TO_PLANE_MM / FX)
    dy_cam = dv * (CAM_TO_PLANE_MM / FY)

    # Transform to world coordinates
    x_world = TRAY_CENTER_X_MM + dy_cam 
    y_world = TRAY_CENTER_Y_MM + dx_cam
    z_world = z_world

    # Apply camera → probe offset
    x_world += CAMERA_TO_PROBE["x"]
    y_world += CAMERA_TO_PROBE["y"]
    z_world += CAMERA_TO_PROBE["z"]

    return x_world, y_world, z_world