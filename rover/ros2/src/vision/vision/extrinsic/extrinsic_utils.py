# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import numpy as np
import yaml
import os

from vision.utils.vision_utils import printlog

# =============================================================================
def read_extrinsic_params(CONF_PATH, FILE_NAME):
    """ 
        Loads extrinsic camera parameters from file  
    Args:
        file_path: `string` absolute path to yaml file
    Returns:
        file_path: `dict` extrinsic camera configuration
            dictionary
    """

    abs_path = os.path.join(CONF_PATH, FILE_NAME)
    if os.path.isfile(abs_path):
        with open(abs_path, 'r') as stream:
            data_loaded = yaml.safe_load(stream)
    else:
        return None

    return data_loaded

def find_projection(img_src, mtx, dist, PATTERN_THRESH_TOP, PATTERN_THRESH_BOTTOM, 
    HSVI, HSVS, PATTERN_FILTER_KERNEL=1, PATTERN_ITERATION_TRIES=20):
    """ Calculates a projection of a surface on floor from an images with a
        lines pattern 
    Args:
        img_src: `cv2.mat` original image with line patten
        mtx: `numpy.narray` camera's distortion matrix
        dist: `numpy.narray` camera's distortion vector
        PATTERN_THRESH_TOP: `int` top limit of surface projection on original image
        PATTERN_THRESH_BOTTOM: `int` bottom limit of surface projection on original image
        HSVI: `dictionary` HSV color space inferior threshold for color filtering
        HSVS: `dictionary` HSV color space superior threshold for color filtering
        PATTERN_FILTER_KERNEL: `int` kernel for filter
        PATTERN_ITERATION_TRIES: `int` number of iterations for hsv filtering
    Returns:
        extrinsic_calibration: `dictionary`or`None`
            vp: `tuple`  vanishing point coordinate in original image space
            p1: `tuple` point 1 of quadrangle vertices's Coordinates in source image
            p2: `tuple` point 2 of quadrangle vertices's Coordinates in source image
            p3: `tuple` point 3 of quadrangle vertices's Coordinates in source image
            p4: `tuple` point 4 of quadrangle vertices's Coordinates in source image
            Left_Line: `tuple` left line of surface projection space
            Right_Line: `tuple` right line of surface projection space
            img_result" `c2.mat` Original image without distortion with all 
                                        information drawn
            Hy: `int` Y axis coordinate of horizontal line in projection pattern image
    """

    IMG_SIZE = (img_src.shape[1], img_src.shape[0])
    LOCAL_WIN_NAME="extrinsic_calibration"

    # Apply Undistort to image in path
    img_scr_undistort = cv2.undistort(
        src=img_src, cameraMatrix=mtx, distCoeffs=dist)

    # Apply Gaussian filter to image
    img_scr_blur = cv2.GaussianBlur(
        src=img_scr_undistort, 
        ksize=(PATTERN_FILTER_KERNEL*2+1, 
                PATTERN_FILTER_KERNEL*2+1), 
        sigmaX=0)

    # -------------------------------------------------------------------------
    # Find if the pattern in the image in good enough
    solid_pattern = False # Solid pattern found variable
    iteration = 0 # Iteration for pattern search
    printlog(msg="Looking for pattern")
    while not solid_pattern and iteration < PATTERN_ITERATION_TRIES:

        pattern_sides = 0 # Number of sides detected in pattern
        # Color segmentation 
        if iteration <= int(PATTERN_ITERATION_TRIES*0.5): 
            HI_T = HSVI["H"]              ; HS_T = HSVS["H"]
            SI_T = HSVI["S"]              ; SS_T = HSVS["S"]
            VI_T = HSVI["V"]-10*iteration ; VS_T = HSVS["V"]
        else:
            HI_T = HSVI["H"]                  ; HS_T = HSVS["H"]
            SI_T = HSVI["S"]-5*(iteration-10) ; SS_T = HSVS["S"]
            VI_T = VI_T                       ; VS_T = HSVS["V"]

        img_scr_hsv = cv2.cvtColor(src=img_scr_blur, code=cv2.COLOR_BGR2HSV)
        img_scr_hsv = cv2.inRange(src=img_scr_hsv, lowerb=(HI_T, SI_T, VI_T), upperb=(HS_T, SS_T, VS_T))
        img_scr_hsv = cv2.dilate(src=img_scr_hsv, kernel=np.ones((3, 3), np.uint8), iterations=1)

        # Contours detection and classification
        # https://docs.opencv.org/3.3.1/dd/d49/tutorial_py_contour_features.html
        _, contours, _ = cv2.findContours(image=img_scr_hsv, 
            mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        
        img_scr_hsv_gui = cv2.cvtColor(src=img_scr_hsv, code=cv2.COLOR_GRAY2BGR) # Image to debug and test
        img_scr_hsv = np.zeros((IMG_SIZE[1], IMG_SIZE[0], 1), np.uint8) # Mask to draw valid  pattern contours 

        # Check first if a solid pattern was detected
        cnt = []; cnt_idx = 0
        for cnt_idx, cnt in enumerate(contours):
            if LOCAL_CALIBRATION: 
                cv2.destroyAllWindows()
                cv2.imshow(LOCAL_WIN_NAME, img_scr_hsv_gui)
                cv2.waitKey(1000)

            # Calculate contour features
            cnt_area = cv2.contourArea(contour=cnt)
            cnt_perimeter = cv2.arcLength(curve=cnt, closed=True)
            cnt_M = cv2.moments(cnt)
            _, _, cnt_w, cnt_h = cv2.boundingRect(cnt)

            if cnt_M['m00'] != 0: # Get contour center point if moment found
                cnt_cx = int(cnt_M['m10'] / cnt_M['m00'])
                cnt_cy = int(cnt_M['m01'] / cnt_M['m00'])
            else:
                cv2.drawContours(image=img_scr_hsv_gui, contours=[cnt], 
                   contourIdx=-1, color=(0, 0, 255), thickness=-1)
                continue

            # Evaluate contour
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = float(cnt_area) / hull_area

            # GUI: Print contours properties
            cv2.putText(img_scr_hsv_gui, str(cnt_idx), (cnt_cx, cnt_cy), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
            
            # Discriminate contour by features
            if cnt_area < 120 or cnt_h < 40 or cnt_cy < int(IMG_SIZE[1] / 2):
                cv2.drawContours(image=img_scr_hsv_gui, contours=[cnt], 
                    contourIdx=-1, color=(0, 0, 255), thickness=-1)
                continue

            # Check contour's solidity for a complete pattern
            if ((solidity > 0.12 and solidity < 0.25)):
                solid_pattern = True
                contours = [cnt]
                printlog(msg="Complete partern found", msg_type="OKGREEN")
                cv2.drawContours(image=img_scr_hsv, contours=[cnt], contourIdx=-1, 
                    color=(255, 255, 255), thickness=-1)
                cv2.drawContours(image=img_scr_hsv_gui, contours=[cnt], contourIdx=-1, 
                    color=(0, 255, 0), thickness=-1)
                if LOCAL_CALIBRATION: 
                    cv2.imshow(LOCAL_WIN_NAME, img_scr_hsv_gui)
                    cv2.waitKey(1000)
                break
            elif (
                (solidity > 0.7 and solidity < 0.9 and cnt_area > 1600 and cnt_perimeter > 300) or
                (solidity > 0.24 and cnt_area > 800 and cnt_perimeter > 200)):  
                pattern_sides += 1
                cv2.drawContours(image=img_scr_hsv_gui, contours=[cnt], contourIdx=-1, 
                    color=(0, 255, 0), thickness=-1)
                cv2.drawContours(image=img_scr_hsv, contours=[cnt], contourIdx=-1, 
                    color=(255, 255, 255), thickness=-1)
                printlog(msg="One side of partern found", msg_type="WARN")
                if pattern_sides == 2:
                    solid_pattern = True
                    printlog(msg="Fragmented partern found", msg_type="WARN")
                    break
                continue
            else:
                cv2.drawContours(image=img_scr_hsv_gui, contours=[cnt], 
                    contourIdx=-1, color=(0, 0, 255), thickness=-1)
        
        if not solid_pattern:
            printlog(msg="No pattern found in iteration {}".format(iteration), msg_type="ERROR")
        iteration += 1# Increment iteration

    if not solid_pattern:
        printlog(msg="No solid pattern found", msg_type="WARN")
        if len(cnt):
            cv2.drawContours(image=img_scr_undistort, contours=[cnt], 
                contourIdx=-1, color=(255, 255, 0), thickness=-1)
            cv2.putText(img_scr_undistort, str(round(solidity, 2)), (cnt_cx, cnt_cy), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1, cv2.LINE_AA)

    # -------------------------------------------------------------------------
    # https://docs.opencv.org/3.4.2/d9/d61/tutorial_py_morphological_ops.html
    kernel = 3 # Kernel for dilatation process
    img_scr_hsv = cv2.dilate(src=img_scr_hsv, 
        kernel=np.ones((kernel, kernel), np.uint8), iterations = 1)

    # Canny and hough for lines detection
    # https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_imgproc/py_houghl
    # ines/py_houghlines.html
    img_scr_edges = cv2.Canny(image=img_scr_hsv, threshold1=0, threshold2=255) 

    # Find lines with Hough transform in edges results
    lines = cv2.HoughLinesP(image=img_scr_edges, rho=0.2, theta=np.pi/180., 
        threshold=10, lines=200, minLineLength=8)
    Lhs = np.zeros((2,2), dtype= np.float32)
    Rhs = np.zeros((2,1), dtype= np.float32)
    vp = None

    # Find a pre - vanishing point with detected lines
    if lines is not None:
        for line in lines:
            try:
                for x1, y1, x2, y2 in line:
                    normal = np.array([[-(y2-y1)], [x2-x1]], dtype=np.float32)
                    normal /= np.linalg.norm(normal)
                    point = np.array([[x1],[y1]], dtype=np.float32)
                    outer = np.matmul(normal, normal.T)
                    Lhs += outer
                    Rhs += np.matmul(outer, point)
                    cv2.line(img=img_scr_undistort, pt1=(x1, y1), 
                        pt2=(x2, y2), color=(255, 0, 255), thickness=2)
            except:
                pass
    else:
        print("[ERROR][EXTRINSIC_CALIBRATION]: No lines found in hough detection")
    if LOCAL_CALIBRATION: 
        cv2.imshow(LOCAL_WIN_NAME, img_scr_undistort)
        cv2.waitKey(1000)

    try:
        vp = np.matmul(np.linalg.inv(Lhs), Rhs)
    except Exception as e:
        singular = (Lhs.shape[0] == Lhs.shape[1] and np.linalg.matrix_rank(Lhs) == Lhs.shape[0])
        if not singular:
            print("[CRITICAL_ERROR][EXTRINSIC_CALIBRATION]: singular matrix in lines projections")
        else:
            print("[CRITICAL_ERROR][EXTRINSIC_CALIBRATION]: {}".format(e))
        if LOCAL_CALIBRATION: 
            cv2.waitKey(1000)
            cv2.destroyWindow(LOCAL_WIN_NAME)
        return None

    # -------------------------------------------------------------------------
    print("[INFO][EXTRINSIC_CALIBRATION]: Calculating projection")

    # Calculate the floor reflected area points from aperture angle and 
    # thresholds from vanish point 
    Hy= []          # Y position for horizontal line
    Left_Lines =[]  # Left lines list
    Right_Lines =[] # Right lines list

    for line in lines: # Find angles in detected lines
        for x1, y1, x2, y2 in line:
            angle = math.atan2(y2 - y1, x2 - x1)*(180.0 / math.pi)
            # Do not consider any line up to vanishing_point + PATTERN_THRESH_TOP
            if y2 > vp[1][0]+PATTERN_THRESH_TOP or y1 > vp[1][0] + PATTERN_THRESH_TOP:
                if int(angle) < -10 and int(angle) > -80:  # For left lines
                    Left_Lines.append(line)
                    cv2.line(img_scr_undistort, (int(x1), int(y1)), 
                        (int(x2), int(y2)), (102, 255, 51), 2)
                elif int(angle) > 10: # For right lines
                    Right_Lines.append(line)
                    cv2.line(img_scr_undistort, (int(x1), int(y1)), 
                        (int(x2), int(y2)), (77, 195, 255),  2)
                elif int(angle) < 10 and int(angle) > -10: # For horizontal lines
                    Hy.append(y1); Hy.append(y2)
                else: # No lines considered
                    cv2.line(img_scr_undistort, (x1, y1), (x2, y2), (0, 0, 0), 3)
                cv2.putText(img_scr_undistort, str(int(angle)), (int((x1+x2)*0.5), 
                    int((y1 + y2)*0.5)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 
                    1, cv2.LINE_AA)

    if len(Hy): # Assign horizontal line Y Cord
        Hy = max(Hy)
        cv2.line(img_scr_undistort, (0, int(Hy)), (IMG_SIZE[0], int(Hy)), 
            (255, 179, 255), 2)
    else: Hy = 0

    TLI = [] # Top Left Intersection  X Cord
    LLI = [] # Low Left Intersection  X Cord
    TRI = [] # Top Right Intersection X Cord
    LRI = [] # Low Right Intersection X Cord

    # Compute and append low and superior intersections with limits for left lines
    for line in Left_Lines:
        for x1,y1,x2,y2 in line:
            TLI.append(line_intersection(((x1, y1),(x2, y2)), 
                                         ((0, int(IMG_SIZE[1] * 0.5)),
                                         (IMG_SIZE[0], int(IMG_SIZE[1] * 0.5))))[0])
            LLI.append(line_intersection(((x1, y1),(x2, y2)), 
                                         ((0, IMG_SIZE[1]),
                                         (IMG_SIZE[0], IMG_SIZE[1])))[0])

    # Compute and append low and superior intersections with limits for right lines
    for line in Right_Lines:
        for x1,y1,x2,y2 in line:
            TRI.append(line_intersection(((x1, y1),(x2, y2)), 
                                         ((0,int(IMG_SIZE[1] * 0.5)),
                                         (IMG_SIZE[0], int(IMG_SIZE[1] * 0.5))))[0] )
            LRI.append(line_intersection(((x1, y1),(x2, y2)), 
                                         ((0,IMG_SIZE[1]),
                                         (IMG_SIZE[0], IMG_SIZE[1])))[0] )

    if LOCAL_CALIBRATION:
        for point in LLI:
            cv2.line(img=img_scr_undistort, pt1=(point, IMG_SIZE[1]), 
                pt2=(point, IMG_SIZE[1] - 10), color=[102, 255, 51], 
                thickness=1)
        for point in LRI:
            cv2.line(img=img_scr_undistort, pt1=(point, IMG_SIZE[1]), 
                pt2=(point, IMG_SIZE[1] - 10), color=[77, 195, 255], 
                thickness=1)
        for point in TLI:
            cv2.line(img=img_scr_undistort, pt1=(point, int(IMG_SIZE[1] * 0.5) - 5), 
                pt2=(point,int(IMG_SIZE[1] * 0.5) + 10), color=[102, 255, 51], 
                thickness=1)
        for point in TRI:
            cv2.line(img=img_scr_undistort, pt1=(point, int(IMG_SIZE[1] * 0.5) - 5), 
                pt2=(point,int(IMG_SIZE[1] * 0.5) + 10), color=[77, 195, 255], 
                thickness=1)
        cv2.imshow(LOCAL_WIN_NAME, img_scr_undistort)
        cv2.waitKey(1000)

    if not len(TLI) or not len(LLI) or not len(TRI) or not len(LRI):
        print("[CRITICAL_ERROR][EXTRINSIC_CALIBRATION]: calibration has failed for lines intersections")
        if LOCAL_CALIBRATION: 
            cv2.waitKey(1000)
            cv2.destroyWindow(LOCAL_WIN_NAME)
        return None
    else: print("[INFO][EXTRINSIC_CALIBRATION]: Both calibration pattern's sides found")

    # Compute the average of superior and inferior intersections for left and 
    # right lines
    Av_TLI =int(np.mean(TLI)) # Average of Top Left Intersection points X Cord
    Av_TRI =int(np.mean(TRI)) # Average of Top Right Intersection points X Cord
    Av_LLI =int(np.mean(LLI)) # Average of Inferior Left Intersection points X Cord
    Av_LRI =int(np.mean(LRI)) # Average of Inferior Right Intersection points X Cord
    
    # Declaration of process variables
    new_LLI = []; new_LRI = []; new_TRI = []; new_TLI = []

    # Limits to delete any left or right intersection out of boundaries
    LI_Thres = 50; TI_Thres = 10

    for LLI_aux in LLI: # Delete any left and inferior intersection out of boundaries
        if LLI_aux < (Av_LLI + LI_Thres) and  LLI_aux > (Av_LLI - LI_Thres):
            new_LLI.append(LLI_aux)
    for LRI_aux in LRI: # Delete any right and inferior intersection out of boundaries
        if LRI_aux < (Av_LRI + LI_Thres) and  LRI_aux > (Av_LRI - LI_Thres):
            new_LRI.append(LRI_aux)
    for TRI_aux in TRI: # Delete any right and superior intersection out of boundaries
        if TRI_aux < (Av_TRI + TI_Thres) and  TRI_aux > (Av_TRI - TI_Thres):
            new_TRI.append(TRI_aux)
    for TLI_aux in TLI: # Delete any left and superior intersection out of boundaries
        if TLI_aux < (Av_TLI + TI_Thres) and  TLI_aux > (Av_TLI - TI_Thres):
            new_TLI.append(TLI_aux)

    if not len(new_LRI) or not len(new_LLI) or not len(new_TRI) or not len(new_TLI):
        print("[CRITICAL_ERROR][EXTRINSIC_CALIBRATION]: No intersections found in left or right line")  

    if not len(new_LLI): new_LLI = LLI
    if not len(new_LRI): new_LRI = LRI
    if not len(new_TRI): new_TRI = TRI
    if not len(new_TLI): new_TLI = TLI

    if LOCAL_CALIBRATION: # Draw lines which averages were calculated
        if len(new_LRI) and len(new_LLI) and len(new_TRI) and len(new_TLI):
            cv2.line(img_scr_undistort, (max(new_LRI), IMG_SIZE[1]), 
                (min(new_LRI), IMG_SIZE[1]), [77, 195, 255], 2)
            cv2.line(img_scr_undistort, (max(new_LLI), IMG_SIZE[1]), 
                (min(new_LLI), IMG_SIZE[1]), [102, 255, 51], 2)
            cv2.line(img_scr_undistort, (max(new_TRI), int(IMG_SIZE[1]*0.5) + 10), 
                (min(new_TRI), int(IMG_SIZE[1]*0.5) + 10), [77, 195, 255], 1)
            cv2.line(img_scr_undistort, (max(new_TLI), int(IMG_SIZE[1]*0.5) + 10), 
                (min(new_TLI), int(IMG_SIZE[1]*0.5) + 10), [102, 255, 51], 1)
            cv2.imshow(LOCAL_WIN_NAME, img_scr_undistort)
            cv2.waitKey(1000)

    # Get superior and inferior intersection points in X cord
    TLI =int((max(new_TLI) + min(new_TLI)) * 0.5)
    TRI =int((max(new_TRI) + min(new_TRI)) * 0.5)
    LRI =int((max(new_LRI) + min(new_LRI)) * 0.5)
    LLI =int((max(new_LLI) + min(new_LLI)) * 0.5)

    # Re-calculate vanishing point and assing vanishing point
    new_vp = line_intersection(
        line1=((TLI, int(IMG_SIZE[1] * 0.5)), (LLI, IMG_SIZE[1])), 
        line2=((TRI, int(IMG_SIZE[1] * 0.5)), (LRI, IMG_SIZE[1])))
    vp[0][0] = new_vp[0]; vp[1][0] = new_vp[1]

    # Y cord point for horizons
    top = vp[1] + PATTERN_THRESH_TOP
    bottom = IMG_SIZE[1] - PATTERN_THRESH_BOTTOM
    top_line = ((0, top[0]), (IMG_SIZE[0], top[0]))

    # Calculate angle for left and right line
    Right_angle = round(math.atan2( int(IMG_SIZE[1] * 0.5),  TRI - LRI) * (180.0 / math.pi) - 180, 2) 
    Left_angle  = round(math.atan2( int(IMG_SIZE[1] * 0.5),  TLI - LLI) * (180.0 / math.pi) - 180, 2) 

    # Draw detected left and right 
    LLI_2 = line_intersection(((TLI, int(IMG_SIZE[1] * 0.5)),(LLI, IMG_SIZE[1])), 
                              ((0, top[0]),(IMG_SIZE[0], top[0])))
    LRI_2 = line_intersection(((TRI, int(IMG_SIZE[1] * 0.5)),(LRI, IMG_SIZE[1])), 
                              ((0, top[0]),(IMG_SIZE[0], top[0])))
    
    Left_Line  = ((LLI, IMG_SIZE[1] ), LLI_2)
    Right_Line = ((LRI, IMG_SIZE[1] ), LRI_2)
    if LOCAL_CALIBRATION:
        cv2.line(img_scr_undistort, Left_Line[0] , Left_Line[1], [255, 255, 255], 2)
        cv2.line(img_scr_undistort, Right_Line[0], Right_Line[1], [255, 255, 255], 2)

    # Define point in geometric figure on floor
    Bottom_L = int(vp[0][0] + (vp[1][0] - bottom) / math.tan(Left_angle*(math.pi / 180.)))
    Bottom_R = int(vp[0][0] + (vp[1][0] - bottom) / math.tan(Right_angle*(math.pi / 180.)))
    
    # Expand view area
    Expand_Factor = 1.3
    Bottom_L = int(Bottom_L - vp[0][0] * Expand_Factor)
    Bottom_R = int(Bottom_R + abs(IMG_SIZE[0] - vp[0][0]) * Expand_Factor)
    
    Top_R = int(line_intersection(top_line, ((vp[0][0], vp[1][0]),(Bottom_R,bottom)))[0])
    Top_L = int(line_intersection(top_line, ((vp[0][0], vp[1][0]),(Bottom_L,bottom)))[0])

    # Points of projected area
    p1 = (Top_L, int(round(top[0])) )
    p2 = (Top_R, int(round(top[0])) )
    p3 = (Bottom_R, bottom )
    p4 = (Bottom_L, bottom )
    
    # Show result projection in the original image 
    if LOCAL_CALIBRATION:
        img_scr_undistort = draw_surface_projection(
            img = img_scr_undistort, vanishing_point = vp, bottom = bottom, 
            top = top, Bottom_R = Bottom_R, Bottom_L = Bottom_L, 
            Top_R = Top_R, Top_L = Top_L, IMG_SIZE = IMG_SIZE)

        dotline(src = img_scr_undistort, p1 = (0, Hy), p2 = (IMG_SIZE[0], Hy), 
            color = (255, 255, 255), thickness = 1, Dl = 5)
        cv2.imshow(LOCAL_WIN_NAME, img_scr_undistort)
        cv2.waitKey(1000)

    # Print results
    print("[INFO][EXTRINSIC_CALIBRATION]: VP:({}, {}), P1:{}, P2:{}, P3:{}, P4:{}".format(
        int(vp[0][0]), int(vp[1][0]), p1, p2, p3, p4))  

    extrinsic_calibration={"vp":vp, "p1":p1, "p2":p2, "p3":p3, "p4":p4, "Hy":Hy,
        "Left_Line":Left_Line, "Right_Line":Right_Line, "img_result":img_scr_undistort}

    # Return results
    if LOCAL_CALIBRATION: 
        cv2.waitKey(1000)
        cv2.destroyWindow(LOCAL_WIN_NAME)
    return extrinsic_calibration

# =============================================================================
if __name__ == '__main__':
    
    CONF_PATH = os.path.abspath(__file__ + "/../../../../../../configs")
    VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
    VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))
    CAM_LABEL = "C"
    FILE_NAME = "Extrinsic_{}_{}_{}.yaml".format(
        VIDEO_WIDTH, VIDEO_HEIGHT, CAM_LABEL)

    extrinsic_params = read_extrinsic_params(
        CONF_PATH=CONF_PATH, 
        FILE_NAME=FILE_NAME)
    
    print(extrinsic_params)

# =============================================================================