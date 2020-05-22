# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team

    You can run this script to find the right inferior and superior value of 
    H, S, and V for the calibration pattern
"""

# =============================================================================
import pathlib
import glob
import cv2
import os

# =============================================================================
def nothing(x): pass

def hsv_tunner(img_src, hsv_i, hsv_s):
    """ You can run this script to find the right inferior and superior value of 
        H, S, and V for the calibration pattern
    Args:
        img_src: `cv2.math` image to with the calibration pattern
        hsv_i: `list` with inferior hsv values for color thresholding
        hsv_s: `list` with superior hsv values for color thresholding
    Returns:
    """    

    # Window gui params
    WIN_NAME="hsv_tunner"
    cv2.imshow(WIN_NAME, img_src)

    # create trackbars for color change
    cv2.createTrackbar('HI', WIN_NAME, hsv_i[0], 255, nothing)
    cv2.createTrackbar('HS', WIN_NAME, hsv_s[0], 255, nothing)
    cv2.createTrackbar('SI', WIN_NAME, hsv_i[1], 255, nothing)
    cv2.createTrackbar('SS', WIN_NAME, hsv_s[1], 255, nothing)
    cv2.createTrackbar('VI', WIN_NAME, hsv_i[2], 255, nothing)
    cv2.createTrackbar('VS', WIN_NAME, hsv_s[2], 255, nothing)

    while True: # Run local GUI
        img_scr_hsv = cv2.cvtColor(src=img_src, code=cv2.COLOR_BGR2HSV)
        img_scr_hsv = cv2.inRange(src=img_scr_hsv, 
            lowerb=(
                cv2.getTrackbarPos('HI', WIN_NAME), 
                cv2.getTrackbarPos('SI', WIN_NAME), 
                cv2.getTrackbarPos('VI', WIN_NAME)), 
            upperb=(
                cv2.getTrackbarPos('HS', WIN_NAME), 
                cv2.getTrackbarPos('SS', WIN_NAME), 
                cv2.getTrackbarPos('VS', WIN_NAME)))
        cv2.imshow(WIN_NAME, img_scr_hsv)
        key = cv2.waitKey(100)
        if key==113 or key==81: # (Q) If press q then quit
            exit()

def main():

    # get current vision pattern variables
    HI = os.getenv("VISION_CALIBRATION_PATTERN_HI", 0)
    SI = os.getenv("VISION_CALIBRATION_PATTERN_SI", 0)   
    VI = os.getenv("VISION_CALIBRATION_PATTERN_VI", 0)   
    HS = os.getenv("VISION_CALIBRATION_PATTERN_HS", 255)
    SS = os.getenv("VISION_CALIBRATION_PATTERN_SS", 255)   
    VS = os.getenv("VISION_CALIBRATION_PATTERN_VS", 255)   
    hsv_i = (HI, SI, VI); hsv_s = (HS, SS, VS)

    os.chdir(pathlib.Path(__file__).parent.absolute())
    for file_ in glob.glob("*.jpg"):
        img_src=cv2.imread(file_)
        hsv_tunner(img_src, hsv_i=hsv_i, hsv_s=hsv_s)

# =============================================================================
if __name__ == '__main__':
    main()

# =============================================================================