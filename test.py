import cv2

# Load the image containing the ArUco marker
image = cv2.imread("models/aruco_box/materials/textures/marker_tiles_square.png")

# Map readable names to their OpenCV enum values
aruco_dicts = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

found = False
for dict_name, dict_type in aruco_dicts.items():
    # Get the dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)
    
    # Get default detector parameters
    parameters = cv2.aruco.DetectorParameters()
    
    # Create the detector object (New in OpenCV 4.7+)
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    # Detect markers using the detector object
    corners, ids, rejected = detector.detectMarkers(image)
    
    # If an ID is found, this is the correct dictionary
    if ids is not None and len(ids) > 0:
        print(f"Found match! Dictionary name: {dict_name}")
        print(f"Detected Marker ID(s): {ids.flatten()}")
        found = True
        break

if not found:
    print("Could not match the marker to any standard dictionary.")
