import cv2
import numpy as np

def detect_gate(image):
    """
    Detects a red gate in the image.
    Returns: (center_x, center_y, area) or None if no gate found
    
    Logic:
    - Convert image to HSV color space (better for color detection)
    - Create a mask for red color
    - Find contours (outlines) of red regions
    - Find the largest contour (that's our gate)
    - Calculate its center
    """

    # Convert BGR to HSV
    # HSV is better than RGB for color detection
    # because it separates color from brightness
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Red color has two ranges in HSV
    # (red wraps around 0/180 in HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for both red ranges
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    # Combine both masks
    mask = cv2.bitwise_or(mask1, mask2)

    # Remove noise using morphological operations
    # This cleans up small dots that aren't the gate
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours (outlines of red regions)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None  # No gate found

    # Get the largest contour (most likely the gate)
    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    # Ignore very small detections (noise)
    if area < 500:
        return None

    # Calculate center of the gate
    M = cv2.moments(largest)
    if M['m00'] == 0:
        return None

    center_x = int(M['m10'] / M['m00'])
    center_y = int(M['m01'] / M['m00'])

    return (center_x, center_y, area)

def draw_detection(image, detection):
    """
    Draws detection result on image for visualization
    Shows where the drone thinks the gate is
    """
    if detection is None:
        cv2.putText(image, 'No gate detected', (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return image

    cx, cy, area = detection

    # Draw circle at gate center
    cv2.circle(image, (cx, cy), 10, (0, 255, 0), -1)

    # Draw crosshair
    h, w = image.shape[:2]
    cv2.line(image, (w//2, 0), (w//2, h), (255, 0, 0), 1)  # vertical center line
    cv2.line(image, (0, h//2), (w, h//2), (255, 0, 0), 1)  # horizontal center line

    # Show gate info
    cv2.putText(image, f'Gate at ({cx},{cy})', (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(image, f'Area: {int(area)}', (10, 60),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return image

def test_with_image(image_path):
    """Test gate detection with a saved image"""
    image = cv2.imread(image_path)
    if image is None:
        print(f'Could not load image: {image_path}')
        return

    detection = detect_gate(image)
    print(f'Detection result: {detection}')

    result = draw_detection(image, detection)
    cv2.imshow('Gate Detection', result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def test_with_camera():
    """Test gate detection with live webcam"""
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detection = detect_gate(frame)
        result = draw_detection(frame, detection)

        cv2.imshow('Gate Detection', result)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    print('Testing gate detection with webcam...')
    print('Show something RED to the camera!')
    print('Press Q to quit')
    test_with_camera()
