# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
import numpy as np
import datetime
import yaml
import math
import cv2
import os

from vision.utils.vision_utils import line_intersection
from vision.utils.vision_utils import discrete_contour
from vision.utils.vision_utils import matrix_from_flat
from vision.utils.vision_utils import overlay_image
from vision.utils.vision_utils import printlog
from vision.utils.vision_utils import dotline

cv2v_base = cv2.__version__.split(".")[0]

# =============================================================================
class Extrinsic():

    def __init__(self, cam_labels, mtx=None, dist=None):

        self._CONF_PATH = str(os.getenv(key="CONF_PATH", 
            default=os.path.dirname(os.path.abspath(__file__))))
        self._VIDEO_WIDTH = int(os.getenv(key="VIDEO_WIDTH", default=640))
        self._VIDEO_HEIGHT = int(os.getenv(key="VIDEO_HEIGHT", default=360))

        self._CALIBRATION_DAYS_OUT = int(os.getenv(
            key="VISION_CAL_DAYS_OUT", default=10))
        self._CALIBRATION_DAYS_REMOVE = int(os.getenv(
            key="VISION_CAL_DAYS_REMOVE", default=20))

        # enable print logs
        self.flush = True

        # Intrinsic parameters
        self.mtx = mtx
        self.dist = dist

        # Extrinsic parameters
        self.cams = {cam_key: self.load(
            mtx=mtx, dist=dist, 
            FILE_NAME="Extrinsic_{}_{}_{}.yaml".format(self._VIDEO_WIDTH, 
                self._VIDEO_HEIGHT, cam_key)) for cam_key in cam_labels}

        # Disable print logs
        printlog(msg=str(self), msg_type="INFO", flush=self.flush)
        self.flush = False

    def __str__(self):
        str_ = "\n"
        for cam_label, params in self.cams.items():
            str_ += "\nCAM:{}\n".format(cam_label)
            if params is not None:
                for param_name, param_value in params.items():
                    value = ": {}".format(param_value) if type(param_value
                        ) in [float, int, tuple, list] else ""                        
                    str_ += "\t{}: {}{}\n".format(param_name, 
                        type(param_value), value)
            else:
                str_ += "\t{}\n".format(params)
        str_ += "\n"
        return str_

    def load(self, FILE_NAME, mtx=None, dist=None):
        """ 
            Loads extrinsic camera parameters from file  
        Args:
            FILE_NAME: `string` extrinsic calibration yaml file name
            mtx: 'numpy.darray' matrix distortion coeficients
            dist: 'numpy.darray' vector distortion coeficients
        Returns:
        """

        CONF_PATH = self._CONF_PATH

        abs_path = os.path.join(CONF_PATH, FILE_NAME)
        if os.path.isfile(abs_path):

            try: 
                # -----------------------------------------------------------------
                # Read base data from file
                with open(abs_path, 'r') as stream:
                    extrinsic_cal = yaml.safe_load(stream)

                # -----------------------------------------------------------------
                # Get extra parameters
                mtime = datetime.datetime.fromtimestamp(os.path.getmtime(abs_path))
                ctime = datetime.datetime.fromtimestamp(os.path.getctime(abs_path))
                atime = datetime.datetime.fromtimestamp(os.path.getatime(abs_path))
                extrinsic_cal["mtime_days"] = (atime-mtime).days # Last modification date in days
                extrinsic_cal["ctime_days"] = (atime-ctime).days # Last creation date in days
                extrinsic_cal["out_date"]={"state":False, "days":0}
                
                if (extrinsic_cal["mtime_days"] > self._CALIBRATION_DAYS_OUT
                    or extrinsic_cal["ctime_days"] > self._CALIBRATION_DAYS_OUT):

                    extrinsic_cal["out_date"]["state"]=True
                    extrinsic_cal["out_date"]["days"]= extrinsic_cal["mtime_days"] if (
                        extrinsic_cal["mtime_days"] > extrinsic_cal["ctime_days"]) else extrinsic_cal["ctime_days"] 

                    # Remove calibration file if it's too old and report log
                    if extrinsic_cal["out_date"]["days"] > self._CALIBRATION_DAYS_REMOVE:
                        os.remove(abs_path)
                        printlog(msg="{} camera calibration has been removed, "
                            "please re-calibrate cameras now".format(
                            FILE_NAME), msg_type="WARN", flush=self.flush)
                    else: # Warning if calibration file is being to old    
                        printlog(msg="{} camera calibration will be removed in {}"
                            " days".format(FILE_NAME, self._CALIBRATION_DAYS_REMOVE - extrinsic_cal["out_date"]["days"]), 
                            msg_type="WARN", flush=self.flush)

                # -----------------------------------------------------------------
                # Calculate extra parameters
                SurfacesPoints=(
                    extrinsic_cal["p1"], 
                    extrinsic_cal["p2"], 
                    extrinsic_cal["p3"], 
                    extrinsic_cal["p4"])

                extrinsic_cal["M_inv"] = np.linalg.inv(extrinsic_cal["M"])
                extrinsic_cal["M"] = np.linalg.inv(extrinsic_cal["M_inv"])

                extrinsic_cal["waypoint_area"] = get_contour_dist(
                    cnt=[(0                            , extrinsic_cal["p1"][1]), 
                        (extrinsic_cal["image_size"][0], extrinsic_cal["p2"][1]), 
                        (extrinsic_cal["image_size"][0], extrinsic_cal["image_size"][1]), 
                        (0                             , extrinsic_cal["image_size"][1])], 
                    mtx=mtx, dist=dist)

                extrinsic_cal["undistord_cnt"] = get_contour_dist(
                    cnt=[(0                            , 0), 
                        (extrinsic_cal["image_size"][0], 0), 
                        (extrinsic_cal["image_size"][0], extrinsic_cal["image_size"][1]), 
                        (0                             , extrinsic_cal["image_size"][1])], 
                    mtx=mtx, dist=dist)

                extrinsic_cal["view_coord_m"] = FindBotViewCord( 
                    M=extrinsic_cal["M"],
                    SurfacesPoints=SurfacesPoints, 
                    image_size=extrinsic_cal["image_size"])
                extrinsic_cal["max_distance"] = \
                    extrinsic_cal["unwarped_size"][1]/extrinsic_cal["ppmy"]

                cam_disp_angle, cam_view_angle, cam_aper_angle = \
                    find_angle_displacement(
                        SurfacesPoints=SurfacesPoints,
                        image_size=extrinsic_cal["image_size"], 
                        M=extrinsic_cal["M"], 
                        ppmx=extrinsic_cal["ppmx"], 
                        ppmy=extrinsic_cal["ppmy"])
                extrinsic_cal["cam_disp_angle"] = round(cam_disp_angle, 2)
                extrinsic_cal["cam_view_angle"] = round(cam_view_angle, 2)
                extrinsic_cal["cam_aper_angle"] = round(cam_aper_angle, 2)

                printlog(msg="{} extrinsic parameters loaded".format(
                    FILE_NAME), msg_type="OKGREEN", flush=self.flush)

                # -----------------------------------------------------------------
                return extrinsic_cal

            except Exception as e:
                printlog(msg="error while reading {}, {}".format(
                    abs_path, e), msg_type="ERROR", flush=self.flush)
                return None
        else:
            printlog(msg="No extrinsic file {}".format(
                FILE_NAME), msg_type="WARN", flush=self.flush)

        return None

# =============================================================================
def find_projection(img_src, mtx, dist, PATTERN_THRESH_TOP, PATTERN_THRESH_BOTTOM, 
    HSVI, HSVS, PATTERN_FILTER_KERNEL=1, PATTERN_ITERATION_TRIES=20, 
    LOCAL_CALIBRATION=False):
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

        img_scr_hsv = cv2.cvtColor(
            src=img_scr_blur, 
            code=int(cv2.COLOR_BGR2HSV))
        img_scr_hsv = cv2.inRange(
            src=img_scr_hsv, 
            lowerb=(HI_T, SI_T, VI_T), 
            upperb=(HS_T, SS_T, VS_T))
        img_scr_hsv = cv2.dilate(
            src=img_scr_hsv, 
            kernel=np.ones((3, 3), np.uint8), 
            iterations=1)

        # Contours detection and classification
        # https://docs.opencv.org/3.3.1/dd/d49/tutorial_py_contour_features.html
        if cv2v_base == "4":
            contours, hierarchy = cv2.findContours(image=img_scr_hsv, 
                mode=int(cv2.RETR_TREE), method=int(cv2.CHAIN_APPROX_SIMPLE))
        else:
            _, contours, _ = cv2.findContours(image=img_scr_hsv, 
                mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

        img_scr_hsv_gui = cv2.cvtColor(src=img_scr_hsv, code=int(cv2.COLOR_GRAY2BGR)) # Image to debug and test
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
                printlog(msg="Complete partern found", msg_type="INFO")
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
            printlog(msg="No pattern found in iteration {}".format(iteration), 
                msg_type="WARN")
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
        printlog(msg="No lines found in hough detection", msg_type="ERROR")
    if LOCAL_CALIBRATION: 
        cv2.imshow(LOCAL_WIN_NAME, img_scr_undistort)
        cv2.waitKey(1000)

    try:
        vp = np.matmul(np.linalg.inv(Lhs), Rhs)
    except Exception as e:
        singular = (Lhs.shape[0] == Lhs.shape[1] and np.linalg.matrix_rank(Lhs) == Lhs.shape[0])
        if not singular:
            printlog(msg="singular matrix in lines projections", msg_type="ERROR")
        else:
            printlog(msg="{}".format(e), msg_type="ERROR")
        if LOCAL_CALIBRATION: 
            cv2.waitKey(1000)
            cv2.destroyWindow(LOCAL_WIN_NAME)
        return None

    # -------------------------------------------------------------------------
    printlog(msg="Calculating projection")

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
        printlog(msg="calibration has failed for lines intersections", msg_type="ERROR")
        if LOCAL_CALIBRATION: 
            cv2.waitKey(1000)
            cv2.destroyWindow(LOCAL_WIN_NAME)
        return None
    else: 
        printlog(msg="Both calibration pattern's sides found", msg_type="INFO")

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
        printlog(msg="No intersections found in left or right line", msg_type="ERROR")

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
    printlog(msg="VP:({}, {}), P1:{}, P2:{}, P3:{}, P4:{}".format(
        int(vp[0][0]), int(vp[1][0]), p1, p2, p3, p4))  

    extrinsic_calibration={"vp":vp, "p1":p1, "p2":p2, "p3":p3, "p4":p4, "Hy":Hy,
        "Left_Line":Left_Line, "Right_Line":Right_Line, "img_result":img_scr_undistort}

    # Return results
    if LOCAL_CALIBRATION: 
        cv2.waitKey(1000)
        cv2.destroyWindow(LOCAL_WIN_NAME)

    return extrinsic_calibration

def draw_surface_projection(img, vanishing_point, bottom, top, Bottom_R, 
    Bottom_L, Top_R, Top_L, IMG_SIZE):
    """ Draws a projection of a surface in the floor from a set of images with 
        line pattern 
    Args:
        img: `cv2.math` image to be analyzed
        vanishing_point: `tuple` coordinate point of the vanishing point
        bottom: `int` bottom line projection 
        top: `int` top line projection
        Bottom_R: `int` bottom Right Polygon vertices's
        Bottom_L: `int` bottom left Polygon vertices's
        Top_R: `int` Top Right Polygon vertices's 
        Top_L: `int` Top left Polygon vertices's
        IMG_SIZE: `tuple` original video streaming size
    Returns:
        img_lines: `cv2.math` The function circle draws a simple or filled circle 
            with a given center and radius.
    """

    # Take a copy of image input variable
    img_lines= img.copy() 

    # Draw the upper and lower boundaries in image (horizons)
    cv2.line(img_lines, (0, bottom), (IMG_SIZE[0], bottom), [0, 0, 255], 2)
    cv2.line(img_lines, (0, top)   , (IMG_SIZE[0], top)   , [0, 0, 255], 2)

    # Draw the geometric projection
    cv2.line(img_lines, (Top_L, top)      , (Top_R, top)      , [255, 0, 0], 2)
    cv2.line(img_lines, (Top_R, top)      , (Bottom_R,bottom ), [255, 0, 0], 2)
    cv2.line(img_lines, (Bottom_R,bottom ), (Bottom_L,bottom ), [255, 0, 0], 2)
    cv2.line(img_lines, (Bottom_L,bottom ), (Top_L, top)      , [255, 0, 0], 2)

    # Draw center lines
    cv2.line(img_lines, (0, int(round(img_lines.shape[0]/2))), (img_lines.shape[1], int(round(img_lines.shape[0]/2))), [0, 255, 255], 1)
    cv2.line(img_lines, (int(round(img_lines.shape[1]/2)), 0), (int(round(img_lines.shape[1]/2)), img_lines.shape[0]), [0, 255, 255], 1)

    # Draw corners and points of interest of geometric projection
    img_lines = cv2.circle(img_lines, (vanishing_point[0][0], vanishing_point[1][0]) , 5, (0, 255, 0), -1)
    img_lines = cv2.circle(img_lines, (Bottom_L, bottom)    , 5, (255, 255, 0), -1)
    img_lines = cv2.circle(img_lines, (Bottom_R, bottom)     , 5, (255, 255, 0), -1)
    img_lines = cv2.circle(img_lines, (Top_L   ,top)        , 5, (255, 255, 0), -1)
    img_lines = cv2.circle(img_lines, (Top_R   ,top)        , 5, (255, 255, 0), -1)

    return img_lines

def get_rot_matrix(p1, p2 , p3 , p4, UNWARPED_SIZE):
    """     
        Calculates rotation matrix from points
    Args:
        p1: `tuple` First point of Coordinates of quadrangle vertices in the source image
        p2: `tuple` Second point of Coordinates of quadrangle vertices in the source image
        p3: `tuple` Third point of Coordinates of quadrangle vertices in the source image
        p4: `tuple` Fourth point of Coordinates of quadrangle vertices in the source image
        UNWARPED_SIZE: `tuple` surface projection space size
    Returns:
        M: `numpy.darray` The matrix for a perspective transform
    """

    # Calculate rotation matrix from surface from original source image to projected four points surfaces
    src_points = np.array([p1, p2, p3, p4], dtype=np.float32) 
    dst_points = np.array([
        [0, 0],                                 # p1
        [UNWARPED_SIZE[0], 0],                  # p2
        [UNWARPED_SIZE[0], UNWARPED_SIZE[1]],   # p3
        [0, UNWARPED_SIZE[1]]],                 # p4
        dtype=np.float32)
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    
    return M

def pixel_relation(img_src, M, mtx, dist, p1, p2 , p3 , p4, Left_Line, 
    Right_Line, Hy, Dx_lines, Dy_lines, UNWARPED_SIZE, HSVI, HSVS, 
    PATTERN_ITERATION_TRIES=20, LOCAL_CALIBRATION=False):
    """  Calculates the pixel relation in 'X' and 'Y' axis from surface 
         projection space and Rotation Matrix and others
    Args:
        img_src: `cv2.mat` original image source
        M: `cv2.math` rotation matrix from original image to surface
                      projection space
        mtx: `numpy.narray` camera's distortion matrix
        dist: `numpy.narray` camera's distortion vector
        p1: `tuple` point 1 of quadrangle vertices's Coordinates in source image
        p2: `tuple` point 2 of quadrangle vertices's Coordinates in source image
        p3: `tuple` point 3 of quadrangle vertices's Coordinates in source image
        p4: `tuple` point 4 of quadrangle vertices's Coordinates in source image
        Left_Line: `tuple` left line of surface projection space
        Right_Line: `tuple` right line of surface projection space
        Hy: `int` Y axis cordinate of horizontal line in projection pattern image
        Dx_lines: `float` Distances between vertical lines in pattern
        Dy_lines: `float` Distance to horizontal line in the pattern
        UNWARPED_SIZE: `tuple` Surface projection space size
        HSVI: `dictionary` HSV color space inferior threshold for color filtering
        HSVS: `dictionary` HSV color space superior threshold for color filtering
        PATTERN_ITERATION_TRIES: `int` number of iterations for hsv filtering
    Returns:
        extrinsic_relation: `dictionary`or`None`
            ppmx: `float` pixel per meter relation in X axis [pix/m]
            ppmy: `float` pixel per meter relation in Y axis [pix/m]
            dead_view_distance: `float` robots dead view distance
            img_result: `cv2.mat` image of surface projection space with information
    """

    printlog(msg="Calculating pixel relation parameters")
    IMG_SIZE = (img_src.shape[1], img_src.shape[0])
    LOCAL_WIN_NAME="extrinsic_calibration"

    # -------------------------------------------------------------------------
    # Get intersection of geometric projection lines 
    inters_L1 = line_intersection(((0, 0), (0, IMG_SIZE[1])), 
                                 ((p1[0], p1[1]), (p4[0], p4[1])))

    inters_L2 = line_intersection(((0, 0), (0, IMG_SIZE[1])), 
                                  ((p4[0], p4[1]), (p3[0], p3[1])))

    inters_R1 = line_intersection(((IMG_SIZE[0], 0) ,
                                   (IMG_SIZE[0], IMG_SIZE[1])), 
                                  ((p2[0],p2[1]),(p3[0],p3[1])))

    inters_R2 = line_intersection(((IMG_SIZE[0], 0),
                                   (IMG_SIZE[0], IMG_SIZE[1])), 
                                  ((p4[0], p4[1]), (p3[0], p3[1])))

    # Get projection of intersection of geometric projection lines
    inters_L1 = get_projection_point_dst(coords_src=(inters_L1[0], inters_L1[1], 1), M=M)
    inters_L2 = get_projection_point_dst(coords_src=(inters_L2[0], inters_L2[1], 1), M=M)
    inters_R1 = get_projection_point_dst(coords_src=(inters_R1[0], inters_R1[1], 1), M=M)
    inters_R2 = get_projection_point_dst(coords_src=(inters_R2[0], inters_R2[1], 1), M=M)

    # Calculate camera position with intersection between previous lines
    cam_view_coord = line_intersection((inters_L1, inters_L2), 
                                       (inters_R1, inters_R2))
    cam_view_pix = abs(cam_view_coord[1]-UNWARPED_SIZE[1])

    # -------------------------------------------------------------------------
    # X axis pixels relation [pixels/m]
    Left_Line_proj_P1 = get_projection_point_dst(coords_src=(Left_Line[0][0], Left_Line[0][1], 1), M=M)
    Left_Line_proj_P2 = get_projection_point_dst(coords_src=(Left_Line[1][0], Left_Line[1][1], 1), M=M)
    Left_Line_proj = (Left_Line_proj_P1, Left_Line_proj_P2)

    Right_Line_proj_P1 = get_projection_point_dst(coords_src=(Right_Line[0][0], Right_Line[0][1], 1), M=M)
    Right_Line_proj_P2 = get_projection_point_dst(coords_src=(Right_Line[1][0], Right_Line[1][1], 1), M=M)
    Right_Line_proj = (Right_Line_proj_P1, Right_Line_proj_P2)

    ppmx = abs(Right_Line_proj[0][0] - Left_Line_proj[0][0])/Dx_lines
    if not ppmx: 
        printlog(msg="No 'X' axis pixel relation found", msg_type="ERROR")
        if LOCAL_CALIBRATION: 
            cv2.waitKey(1000)
            cv2.destroyWindow(LOCAL_WIN_NAME)
        return None

    # -------------------------------------------------------------------------
    # Y axis pixels relation [pixels/m]

    # Get perspective transformation
    img_proj = cv2.warpPerspective(src=cv2.undistort(
        src=img_src, cameraMatrix=mtx, distCoeffs=dist), 
        M=M, dsize=(UNWARPED_SIZE[0], UNWARPED_SIZE[1]))
    if LOCAL_CALIBRATION: cv2.imshow(LOCAL_WIN_NAME, img_proj); cv2.waitKey(500)

    # Apply Gaussian filter to image
    filt_kernel_size = 3
    img_proj = cv2.GaussianBlur(src=img_proj, 
        ksize=(filt_kernel_size, filt_kernel_size), sigmaX=0)
    if LOCAL_CALIBRATION: cv2.imshow(LOCAL_WIN_NAME, img_proj); cv2.waitKey(500)

    # Delete limits of no interest
    img_proj_hsv = cv2.cvtColor(src=img_proj, code=cv2.COLOR_BGR2HSV) # Change color space to HSV
    img_proj_hsv[0: UNWARPED_SIZE[1], 0:Left_Line_proj_P1[0] - 5, :] = 0
    img_proj_hsv[0: UNWARPED_SIZE[1], Right_Line_proj_P2[0] + 5:UNWARPED_SIZE[0], :] = 0
    img_proj_hsv[0: int(UNWARPED_SIZE[1]*0.5), 0:UNWARPED_SIZE[0], :] = 0
    if LOCAL_CALIBRATION: cv2.imshow(LOCAL_WIN_NAME, img_proj_hsv); cv2.waitKey(500)

    # Calculated horizons
    H1 = Hy; H2 = 0; H3 = 0
    if not Hy:
        printlog(msg="Hy=0, No pattern's horizontal component found in projection calculation", 
            msg_type="WARN")

    iterations = 0; lines = None; ppmy=0
    while not ppmy and iterations<PATTERN_ITERATION_TRIES and (not H2 or not H3):

        # Apply color segmentation
        if iterations <= int(PATTERN_ITERATION_TRIES*0.5):
            img_proj_thresh = cv2.inRange(src=img_proj_hsv, 
                lowerb=(HSVI["H"], HSVI["S"] - 2*iterations, HSVI["V"]- 15*iterations), 
                upperb=(HSVS["H"], HSVS["S"], HSVS["V"]))
        else:
            img_proj_thresh = cv2.inRange(src=img_proj_hsv, 
                lowerb=(HSVI["H"], HSVI["S"] - 5*iterations, 0), 
                upperb=(HSVS["H"], HSVS["S"], HSVS["V"]))
        
        # Apply erosion to delete small particles
        kernel = np.ones((3, 3),np.uint8)
        img_proj_thresh = cv2.erode(src=img_proj_thresh, 
            kernel=kernel, iterations = 1)

        # Apply canny to find boundaries
        img_scr_edges = cv2.Canny(img_proj_thresh, 0, 255)  # Get canny
        
        # ---------------------------------------------------------------------
        # Find Y Axis relation with canny and hough lines
        lines = cv2.HoughLinesP(image=img_scr_edges, rho=0.2, theta=np.pi/180., 
            threshold=2, lines=50, minLineLength=5)

        Hy_ave = []
        if not isinstance(lines, type(None)):
            for line in lines:
                for x1, y1, x2, y2 in line:
                    angle = math.atan2(y2 - y1, x2 - x1) * (180.0 / math.pi)
                        # For horizontal lines
                    if int(angle) < 60 and int(angle) > -60:
                        # cv2.line(img_proj, (x1, y1), (x2, y2), (255, 179, 255), 2)
                        Hy_ave.append(max(y1, y2))
                        if LOCAL_CALIBRATION:
                            cv2.line(img_proj, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    elif LOCAL_CALIBRATION:
                        cv2.line(img_proj, (x1, y1), (x2, y2), (0, 0, 255), 2)

        cv2.putText(img_proj, "x", (cam_view_coord[0], Hy), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

        Hy_print = 20
        if Hy > 0:
            cv2.putText(img_proj, "Hy:{}[pix]".format(Hy), (10, Hy_print), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA)
            Hy_print += 18

        if len(Hy_ave):
            new_Hy = int(max(Hy_ave))
            cv2.putText(img_proj, "x", (cam_view_coord[0] + 10 , new_Hy), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2, cv2.LINE_AA)
            H2 = new_Hy
            if new_Hy > 0:
                cv2.putText(img_proj, "Hy:{}[pix]".format(new_Hy), (10, Hy_print), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1, cv2.LINE_AA)
                Hy_print += 18
            if new_Hy > Hy:
                Hy = new_Hy

        # ---------------------------------------------------------------------
        # Find Y Axis relation with white pixels in binary mask
        xc = cam_view_coord[0]
        idx = UNWARPED_SIZE[1]
        while idx >= 0:
            idx -= 1
            if img_proj_thresh[idx - 1, xc] != 0:
                break

        if LOCAL_CALIBRATION:
            cv2.putText(img_proj, "x", (cam_view_coord[0] -10 , idx), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2, cv2.LINE_AA)
            if idx > 0:
                cv2.putText(img_proj, "Hy:{}[pix]".format(idx), (10, Hy_print), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1, cv2.LINE_AA)

        H3 = idx if idx>0 else 0
        if idx > Hy: Hy = idx

        if Hy > int(UNWARPED_SIZE[1]*0.1) and Hy < int(UNWARPED_SIZE[1]*0.9) and ( H2 or H3):
            if Hy == H1 and (Hy > H2 or Hy > H3):
                Hy = max(H2, H3)
                if abs(H2 - H3) < 20:
                    Hy = int(np.mean(np.array([H2, H3])))
            # Calculate y axis pixel relation
            ppmy = (UNWARPED_SIZE[1] + cam_view_pix - Hy) / Dy_lines
            if LOCAL_CALIBRATION: cv2.imshow(LOCAL_WIN_NAME, img_proj); cv2.waitKey(500)
            break

        iterations += 1
        printlog(msg="\n\tIteration {} - Warning: No relation found / H1 = {} / H2 = {} / H3 = {} ".format(
            iterations, H1, H2, H3), msg_type="INFO")
        if not H2: printlog(msg="\t\tH2 = 0 - No pattern's horizontal component found for hough lines", msg_type="INFO")
        if not H3: printlog(msg="\t\tH3 = 0 - No pattern's horizontal component found by pixels", msg_type="INFO")
        if LOCAL_CALIBRATION: cv2.imshow(LOCAL_WIN_NAME, img_proj); cv2.waitKey(500)

    if not ppmy:
        printlog(msg="Horizontal line in pattern no found, check light conditions and try again", msg_type="ERROR")
        if LOCAL_CALIBRATION: 
            cv2.waitKey(1000)
            cv2.destroyWindow(LOCAL_WIN_NAME)
        return None

    # -------------------------------------------------------------------------
    # Calculation of dead view distance
    if ppmy: dead_view_distance = cam_view_pix/ppmy # bots No view area in meters [m]

    # Print some information of pixel relations
    printlog(msg="ppx:{}[pix/m], ppy:{}[pix/m], dead_view:{}[m], Hy:{}".format(
        round(ppmx, 2), round(ppmy, 2), round(dead_view_distance, 2), int(Hy)), msg_type="INFO")

    extrinsic_relation = {"ppmx": ppmx, "ppmy": ppmy, 
        "dead_view_distance":dead_view_distance, "img_result": img_proj}

    if LOCAL_CALIBRATION: 
        cv2.waitKey(1000)
        cv2.destroyWindow(LOCAL_WIN_NAME)
    return extrinsic_relation

def create_ruler_mask(flag_img, extrinsic_params, fontScale=0.7):
    """ creates mask of rulers for original and surface projection spaces to 
        overlay later 
    Args:
        flag_img: `cv2.math` flag image to overlay 4m point
        extrinsic_params: `dictionary` with extrinsic calibration parameters
            vp: `tuple`  vanishing point coordinate in original image space
            p1: `tuple` point 1 of quadrangle vertices's Coordinates in source image
            p2: `tuple` point 2 of quadrangle vertices's Coordinates in source image
            p3: `tuple` point 3 of quadrangle vertices's Coordinates in source image
            p4: `tuple` point 4 of quadrangle vertices's Coordinates in source image
            unwarped_size: `tuple` Surface projection space size
            image_size: `tuple` original image/video size
            ppmx: `float` pixel per meter relation in X axis [pix/m]
            ppmy: `float` pixel per meter relation in Y axis [pix/m]
            dead_view: `float` robots dead view distance
            M: `numpy.darray` The matrix for a perspective transform
    Returns:
        Ruler_Mask: `cv2.math` Ruler mask for real world image
        Inv_Ruler_Mask: `cv2.math` Ruler mask for real world surface projection
    """

    # Function variables
    dead_distance = extrinsic_params["dead_view"]
    UNWARPED_SIZE = tuple(extrinsic_params["unwarped_size"])
    ORIGINAL_SIZE = tuple(extrinsic_params["image_size"])
    ppmy = extrinsic_params["ppmy"]
    p1 = tuple(extrinsic_params["p1"])
    p2 = tuple(extrinsic_params["p2"])
    p3 = tuple(extrinsic_params["p3"])
    p4 = tuple(extrinsic_params["p4"])
    M = extrinsic_params["M"]
    
    # Create a mask with extra size to draw ruler
    Ruler_Mask = np.zeros((UNWARPED_SIZE[1]+int(round(dead_distance*ppmy)), 
        UNWARPED_SIZE[0], 4), np.uint8)

    # Find robot's view cord M, SurfacesPoints
    bot_cord = FindBotViewCord(image_size=ORIGINAL_SIZE, M=M, 
        SurfacesPoints=(p1, p2, p3, p4))

    # Draw center line according to X cord in ruler mask
    cv2.line(img=Ruler_Mask, pt1=(bot_cord[0],0), pt2=(bot_cord[0], Ruler_Mask.shape[0]), 
        color=(0,255,0,255), thickness=1)

    # Max distance in rule given for pixel relation in Y axis
    if ppmy: Max_distance = math.trunc(Ruler_Mask.shape[0]/ppmy)
    else: return None, None
    
    # Draw tens in ruler mask
    for i in range(0, Max_distance+1):
        y_pos = int(round(i*ppmy))
        cv2.line(img=Ruler_Mask, 
            pt1=(bot_cord[0], Ruler_Mask.shape[0] - y_pos),
            pt2=(int(round(bot_cord[0]+Ruler_Mask.shape[1]*0.10)), Ruler_Mask.shape[0] - y_pos),
            color=(0,255,0,255), thickness=1)
        cv2.putText(Ruler_Mask, str(i*100),
            (int(round(bot_cord[0]+Ruler_Mask.shape[1]*0.12)),Ruler_Mask.shape[0] - int(round(y_pos+ppmy*0.02))),
            cv2.FONT_HERSHEY_SIMPLEX, fontScale*0.5, (0, 0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(Ruler_Mask,str(i*100),
            (int(round(bot_cord[0]+Ruler_Mask.shape[1]*0.12)),Ruler_Mask.shape[0] - int(round(y_pos+ppmy*0.02))),
            cv2.FONT_HERSHEY_SIMPLEX, fontScale*0.5, (255, 255, 255, 255), 1, cv2.LINE_AA)
    
    # Draw hundreds in ruler mask
    for i in range(0, Max_distance*11):
        y_pos = int(round(i*ppmy)/10)
        cv2.line(img=Ruler_Mask,
            pt1=(bot_cord[0], Ruler_Mask.shape[0] - y_pos),
            pt2=(int(round(bot_cord[0]+Ruler_Mask.shape[1]*0.10))-10 , Ruler_Mask.shape[0] - y_pos),
            color=(0,255,0,255), thickness=1)

    # Draw hundreds of labels in ruler mask
    for i in range(0, Max_distance+1):
        y_pos = int(round((i+0.5)*ppmy))
        cv2.line(img=Ruler_Mask,
            pt1=(int(round(bot_cord[0]-Ruler_Mask.shape[1]*0.04)) , Ruler_Mask.shape[0] - y_pos),
            pt2=(bot_cord[0], Ruler_Mask.shape[0] - y_pos),
            color=(0,255,0, 255), thickness=1)
        cv2.putText(Ruler_Mask,str(int(round((i+0.5)*100))),
            (int(round(bot_cord[0]-Ruler_Mask.shape[1]*0.15)) , int(round(Ruler_Mask.shape[0] - int(round(y_pos+ppmy*0.07))))),
            cv2.FONT_HERSHEY_SIMPLEX, fontScale*0.4, (0, 0, 0, 255),2,cv2.LINE_AA)
        cv2.putText(Ruler_Mask,str(int(round((i+0.5)*100))),
            (int(round(bot_cord[0]-Ruler_Mask.shape[1]*0.15)) , int(round(Ruler_Mask.shape[0] - int(round(y_pos+ppmy*0.07))))),
            cv2.FONT_HERSHEY_SIMPLEX, fontScale*0.4, (200, 200, 200, 255),1,cv2.LINE_AA)

    # Resize ruler mask
    Ruler_Mask =  Ruler_Mask[ :UNWARPED_SIZE[1], :] 
    
    # Temporal resized ruler mask for surface projection
    Temp_Ruler_Mask = cv2.resize(src=Ruler_Mask, 
        dsize=(int(round(UNWARPED_SIZE[0]*2)), int(round(UNWARPED_SIZE[1]*2))), 
        interpolation = cv2.INTER_AREA) 

    # Create ruler mask for projected surface
    Scale_Factor = 8
    pts1 = np.float32([ [0,0],
                        [Temp_Ruler_Mask.shape[1],0],
                        [Temp_Ruler_Mask.shape[1],Temp_Ruler_Mask.shape[0]],
                        [0,Temp_Ruler_Mask.shape[0]]])

    p1_aux =  [x * Scale_Factor for x in p1]
    p2_aux =  [x * Scale_Factor for x in p2]
    p3_aux =  [x * Scale_Factor for x in p3]
    p4_aux =  [x * Scale_Factor for x in p4]

    pts2 = np.float32([p1_aux, p2_aux, p3_aux, p4_aux])
    Temp_M = cv2.getPerspectiveTransform(src=pts1, dst=pts2)

    Inv_Ruler_Mask = cv2.warpPerspective(src=Temp_Ruler_Mask, M=Temp_M, 
        dsize=(ORIGINAL_SIZE[0]*Scale_Factor, ORIGINAL_SIZE[1]*Scale_Factor))
    Inv_Ruler_Mask = cv2.resize(src=Inv_Ruler_Mask, 
        dsize=ORIGINAL_SIZE, interpolation = cv2.INTER_AREA   ) 

    persp_proc = 1-4/10.

    y_pos = int(UNWARPED_SIZE[1]+int(round(dead_distance*ppmy)) - int(round(4*ppmy)))
    x_pos = int(round(bot_cord[0]))
    pos = get_projection_point_src(coords_dst=(x_pos, y_pos, 1), 
        INVM=np.linalg.inv(M))
    if flag_img is not None:
        flag_img  = cv2.resize(src=flag_img, 
            dsize=(int(flag_img.shape[1]*persp_proc), int(flag_img.shape[0]*persp_proc)), 
            interpolation = cv2.INTER_AREA) 
        Inv_Ruler_Mask = overlay_image(l_img=Inv_Ruler_Mask, s_img=flag_img, 
            pos=(pos[0], pos[1]-flag_img.shape[0]), transparency=1)

    return Ruler_Mask,  Inv_Ruler_Mask

def get_projection_point_dst(coords_src, M):
    """ 
        this function returns the cords of a point in "X" and "Y" in a surface 
        according to the Rotation Matrix surface and origin point in an original 
        source image
    Args:
        coords_src: `tuple` cords in the original image
        M: `numpy.narray` rotation matrix from geometric projection to original 
    Returns:
        _: return the projected point according with rotation matrix "M" and 
        the original point 'coords_src'
    """

    # ------------------------------------------------------------
    coords_dst=np.matmul(M, coords_src)
    coords_dst=coords_dst/coords_dst[2]

    # ------------------------------------------------------------
    # return results
    return (int(coords_dst[0]), int(coords_dst[1]))

def get_projection_point_src(coords_dst, INVM):
    """ 
        Get the cords of a point in 'X' and 'Y' from projected area to source 
        area in original image
    Args:
        coords_src: `tuple` cords in the original image
        INVM: `numpy.narray` inverse of rotation matrix from geometric projection to original 
    Returns:
        _: return the projected point according with rotation matrix 'M' and 
            the original point 'coords_dst'  
    """

    # ------------------------------------------------------------
    coords_src=np.matmul(INVM, coords_dst)
    coords_src=coords_src/coords_src[2]

    # ------------------------------------------------------------
    # return results
    return (int(coords_src[0]), int(coords_src[1]))

def FindBotViewCord(image_size, M, SurfacesPoints, mtx=None, dist=None):
    """ 
        Calculates the bot's view cord in surface projection        
    Args:
        image_size: `tuple` original image size
        M: `numpy.narray` rotation matrix from geometric projection to original image
        SurfacesPoints: `tuple` Coordinates of quadrangle vertices in the source image
    Returns:
        x_dst: `tuple` bot's view x cordinate
        y_dst: `tuple` bot's view y cordinate   
    """

    # Re - assign surface projection points
    p1 = SurfacesPoints[0]
    p2 = SurfacesPoints[1]
    p3 = SurfacesPoints[2]
    p4 = SurfacesPoints[3]

    # Declare surface projection limit lines
    top_Line = (p1, p2)
    bottom_line = (p3, p4)
    left_line = ((0, 0), (0, image_size[1]))
    right_line = ((image_size[0], 0), (image_size[0], image_size[1]))

    # Find intersections of surface projection lines
    LSInterc = line_intersection(line1=top_Line, line2=left_line)
    LIInterc = line_intersection(line1=bottom_line, line2=left_line)
    RSInterc = line_intersection(line1=top_Line, line2=right_line)
    RIInterc = line_intersection(line1=bottom_line, line2=right_line)

    # Project intersections in surface projection space
    LSInterc_dst = get_projection_point_dst((LSInterc[0], LSInterc[1], 1), M)
    LIInterc_dst = get_projection_point_dst((LIInterc[0], LIInterc[1], 1), M)
    RSInterc_dst = get_projection_point_dst((RSInterc[0], RSInterc[1], 1), M)
    RIInterc_dst = get_projection_point_dst((RIInterc[0], RIInterc[1], 1), M)

    # Find bot camera cords
    x_dst, y_dst = line_intersection((LSInterc_dst, LIInterc_dst), 
        (RSInterc_dst, RIInterc_dst))

    if mtx is not None and dist is not None:
        x_dst, y_dst = get_distor_point(
            pt=(x_dst, y_dst), mtx=mtx, dist=dist)

    return x_dst, y_dst

def get_contour_dist(cnt, mtx=None, dist=None):
    """ 
        Calculates surface projection contour in original 
        image with distortion         
    Args:
        cnt: `np.array` countour to be calculated in distortion space
        mtx: `numpy.narray` camera's distortion matrix
        dist: `numpy.narray` camera's distortion vector
    Returns:
        new_contours_dist: `np.array` surface projection contour in original 
            image with distortion        
    """
 
    new_contours = discrete_contour(contour=cnt, Dl=5)

    if mtx is None or dist is None: 
        return new_contours

    new_contours_dist = []
    for countour in new_contours:
        point = get_distor_point(countour, mtx, dist)
        new_contours_dist.append(point)
    
    return np.array(new_contours_dist)

def get_distor_point(pt, mtx, dist):
    """ 
        Get input point in distortion space
    Args:
        pt: `tuple` (x, y) coordinate to distord
        mtx: `numpy.narray` camera's distortion matrix
        dist: `numpy.narray` camera's distortion vector
    Returns:
        http://answers.opencv.org/question/148670/re-distorting-a-set-of-points-
        after-camera-calibration/   
    """

    test = np.zeros((1,1,2), dtype=np.float32)
    test[0,0,0]=pt[0]
    test[0,0,1]=pt[1]

    rtemp = ttemp = np.array([0, 0, 0], dtype='float32')

    # Normalize the points to be independent of the camera matrix using undistortPoints with no distortion matrix
    xy_normalized = cv2.undistortPoints(test, mtx, None)

    # Convert them to 3d points 
    ptsTemp = cv2.convertPointsToHomogeneous(xy_normalized)

    # Project them back to image space using the distortion matrix
    output = cv2.projectPoints(ptsTemp, rtemp, ttemp, mtx, dist, xy_normalized)

    x_undistor = output[0][0,0,0]
    y_undistor = output[0][0,0,1]

    return int(round(x_undistor)),int(round(y_undistor))

def find_angle_displacement(SurfacesPoints, image_size, M, ppmx, ppmy):
    """ 
        Calculates surface projection contour in original 
        image with distortion         
    Args:
        image_size: `tuple` original image size
        M: `numpy.narray` rotation matrix from geometric projection to original image
        SurfacesPoints: `tuple` Coordinates of quadrangle vertices in the source image
        ppmx: `int` pixels per meters in X axis in unwrapped projection
        ppmy: `int` pixels per meters in Y axis in unwrapped projection
    Returns:
        cam_disp_angle: `float` camera displacement angle 
        cam_view_angle: `float` camera view angle
        cam_aper_angle: `float` camera aperture angle
    """

    # Re - assign surface projection points
    p1 = SurfacesPoints[0]
    p2 = SurfacesPoints[1]
    p3 = SurfacesPoints[2]
    p4 = SurfacesPoints[3]

    # Declare surface projection limit lines
    top_Line = (p1, p2)
    bottom_line = (p3, p4)
    left_line = ((0, 0), (0, image_size[1]))
    right_line = ((image_size[0], 0), (image_size[0], image_size[1]))

    # Find intersections of surface projection lines
    LSInterc = line_intersection(top_Line, left_line)
    LIInterc = line_intersection(bottom_line, left_line)
    RSInterc = line_intersection(top_Line, right_line)
    RIInterc = line_intersection(bottom_line, right_line)

    # Project intersections in surface projection space
    LSInterc_dst = get_projection_point_dst((LSInterc[0], LSInterc[1], 1), M)
    LIInterc_dst = get_projection_point_dst((LIInterc[0], LIInterc[1], 1), M) 
    RSInterc_dst = get_projection_point_dst((RSInterc[0], RSInterc[1], 1), M)
    RIInterc_dst = get_projection_point_dst((RIInterc[0], RIInterc[1], 1), M)

    BotViewCord = line_intersection((LSInterc_dst, LIInterc_dst), 
                                    (RSInterc_dst, RIInterc_dst))

    dxr = (RIInterc_dst[0] - BotViewCord[0])/ppmx
    dyr = (BotViewCord[1] - RIInterc_dst[1])/ppmy
    angle_r = round(math.degrees(math.atan2(dyr, dxr)), 2)

    dxl = (LIInterc_dst[0] - BotViewCord[0])/ppmx
    dyl = (BotViewCord[1] - LIInterc_dst[1])/ppmy
    angle_l = round(math.degrees(math.atan2(dyl, dxl)), 2)

    cam_aper_angle = angle_l - angle_r
    angle_ll = 180 - angle_l
    cam_disp_angle = angle_r - angle_ll 

    cam_view_angle = cam_aper_angle/2 + angle_r
    
    return cam_disp_angle, cam_view_angle, cam_aper_angle

# =============================================================================