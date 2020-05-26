#!/usr/bin/env python3
# =============================================================================
"""
Code Information:
    Programmer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus Computer Vision &Ai Team

Sources:
https://docs.opencv.org/3.4/d4/d15/group__videoio__flags__base.html#gaeb8dd9c89c10a5c63c139bf7c4f5704d
https://unix.stackexchange.com/questions/10671/usb-performance-traffic-monitor
https://www.ximea.com/support/wiki/usb3/multiple_cameras_setup
"""

# =============================================================================
import numpy as np
import inspect
import math
import cv2
import os

# =============================================================================
# VISUAL - VISUAL - VISUAL - VISUAL - VISUAL - VISUAL - VISUAL - VISUAL - VISUA


class bcolors:
    LOG = {
        "WARN": ['\033[33m', "WARN"],
        "ERROR": ['\033[91m', "ERROR"],
        "OKGREEN": ['\033[32m', "INFO"],
        "INFO": ['\033[0m', "INFO"], # ['\033[94m', "INFO"], 
        "BOLD": ['\033[1m', "INFO"],
        "GRAY": ["\033[90m", "INFO"],
    }
    BOLD = "\033[1m"
    ENDC = "\033[0m"
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    GRAY = "\033[90m"
    UNDERLINE = "\033[4m"


def printlog(msg, msg_type="INFO", flush=True):
    """     
        Print log message tracking file and function caller 
        Args:
            msg: `string` message to print
            msg_type: `string` message type
            flush: `boolean` sure that any output is buffered and go to the 
                destination.
        Returns:
    """
    if not flush:
        return

    org = os.path.splitext(os.path.basename(inspect.stack()[1][1]))[0].upper()
    caller = inspect.stack()[1][3].upper()
    _str = "[{}][{}][{}]: {}".format(bcolors.LOG[msg_type][1], org, caller, msg)

    print(bcolors.LOG[msg_type][0] + _str + bcolors.ENDC, flush=True)


def print_text_list(
    img, tex_list, color=(0, 0, 255), orig=(10, 25), fontScale=0.7, y_jump=30
):
    """
        Print a text list on image in desending order
        Args:
            img: 'cv2.math' image to draw components
            tex_list: 'list' list with text to print/draw
            color: 'list' bgr opencv color of text
            orig: 'tuple' origin to start draw components
            fontScale: 'float' text font scale
            y_jump: 'int' jump or space between lines
        returns:
    """

    for idx, text in enumerate(tex_list):
        cv2.putText(
            img=img,
            text=text,
            org=(orig[0], int(orig[1] + y_jump * idx)),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=fontScale,
            color=(0, 0, 0),
            thickness=3,
            lineType=cv2.LINE_AA,
        )
        cv2.putText(
            img=img,
            text=text,
            org=(orig[0], int(orig[1] + y_jump * idx)),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=fontScale,
            color=color,
            thickness=1,
            lineType=cv2.LINE_AA,
        )


def dotline(src, p1, p2, color, thickness, Dl):
    """  draws a doted line on input image
    Args:
        src: `cv2.mat` source image
        p1: `tuple` line's first point 
        p2: `tuple` line's second point 
        color: `tuple` lines' color RGB [B, G, R] [int]
        thickness: `int` lines' thickness
        Dl: `int` distance in pixels between every point
    Returns:
        src: `cv2.mat` image with doted line drawn
    """
    # Get a number of intermediate points
    segments = discrete_contour((p1, p2), Dl)

    for segment in segments:  # Draw doted line
        cv2.circle(img=src, center=segment, radius=thickness, color=color, thickness=-1)

    return src


def overlay_image(l_img, s_img, pos, transparency):
    """ Overlay 's_img on' top of 'l_img' at the position specified by
        pos and blend using 'alpha_mask' and 'transparency'.
    Args:
        l_img: `cv2.mat` inferior image to overlay superior image
        s_img: `cv2.mat` superior image to overlay
        pos: `tuple`  position to overlay superior image [pix, pix]
        transparency: `float` transparency in overlayed image
    Returns:
        l_img: `cv2.mat` original image with s_img overlayed
    """

    # Get superior image dimensions
    _, _, s_img_channels = s_img.shape

    if s_img_channels == 3 and transparency != 1:
        s_img = cv2.cvtColor(s_img, cv2.COLOR_BGR2BGRA)
        s_img_channels = 4

    # Take 3rd channel of 'img_overlay' image to get shapes
    img_overlay = s_img[:, :, 0:4]

    # cords assignation to overlay image
    x, y = pos

    # Image ranges
    y1, y2 = max(0, y), min(l_img.shape[0], y + img_overlay.shape[0])
    x1, x2 = max(0, x), min(l_img.shape[1], x + img_overlay.shape[1])

    # Overlay ranges
    y1o, y2o = max(0, -y), min(img_overlay.shape[0], l_img.shape[0] - y)
    x1o, x2o = max(0, -x), min(img_overlay.shape[1], l_img.shape[1] - x)

    # Exit if nothing to do
    if y1 >= y2 or x1 >= x2 or y1o >= y2o or x1o >= x2o:
        return l_img

    if s_img_channels == 4:
        # Get alphas channel
        alpha_mask = (s_img[:, :, 3] / 255.0) * transparency
        alpha_s = alpha_mask[y1o:y2o, x1o:x2o]
        alpha_l = 1.0 - alpha_s

        # Do the overlay with alpha channel
        for c in range(0, l_img.shape[2]):
            l_img[y1:y2, x1:x2, c] = (
                alpha_s * img_overlay[y1o:y2o, x1o:x2o, c]
                + alpha_l * l_img[y1:y2, x1:x2, c]
            )

    elif s_img_channels < 4:
        # Do the overlay with no alpha channel
        if l_img.shape[2] == s_img.shape[2]:
            l_img[y1:y2, x1:x2] = s_img[y1o:y2o, x1o:x2o]
        else:
            printlog(
                msg="Error: to overlay images should have the same color channels",
                msg_type="ERROR",
            )
            return l_img

    # Return results
    return l_img


def insert_image(
    original_image,
    inserted_image,
    new_width,
    new_height,
    position="uc",
    line_width=2,
    border_color=(255, 255, 255),
    interpolation=cv2.INTER_NEAREST,
):
    """     
        inserts an image "inserted_image" over input image "original_image"
    Args:
        original_image: `cv2.math` background image
        inserted_image: `cv2.math` overlayed image
        new_width: `int` new width for overlayed image
        new_height: `int` new height for overlayed image
        position: `string` corner to overlay image
            upper left: ul
            upper right: ur
            lower left: ll
            lower right: lr
        line_width: `int` frame line width
        border_color: `list` [B,G,R] color to draw a frame in output image
        interpolation: `string` interpolation flag to resize images
    Returns:
    """

    # Overlay image to upper left side
    height, width, _ = original_image.shape
    if position == "ul":
        original_image[0:new_height:, 0:new_width, :] = cv2.resize(
            inserted_image, (new_width, new_height), interpolation=interpolation
        )
        cv2.rectangle(
            original_image, (0, 0), (new_width, new_height), border_color, line_width
        )

    # Overlay image to upper right side
    elif position == "ur":
        original_image[0:new_height:, -new_width:, :] = cv2.resize(
            inserted_image, (new_width, new_height), interpolation=interpolation
        )
        cv2.rectangle(
            original_image,
            (width - new_width, 0),
            (width, new_height),
            border_color,
            line_width,
        )

    # Overlay image to upper center side
    elif position == "uc":
        original_image[
            0:new_height:,
            int(width * 0.5 - new_width * 0.5) : int(width * 0.5 + new_width * 0.5),
            :,
        ] = cv2.resize(
            inserted_image, (new_width, new_height), interpolation=interpolation
        )
        cv2.rectangle(
            original_image,
            (int(width * 0.5 - new_width * 0.5), 0),
            (int(width * 0.5 + new_width * 0.5), new_height),
            border_color,
            line_width,
        )

    # Overlay image to lower left side
    elif position == "ll":
        original_image[-new_height:, 0:new_width, :] = cv2.resize(
            inserted_image, (new_width, new_height), interpolation=interpolation
        )
        cv2.rectangle(
            original_image,
            (0, height - new_height),
            (new_width, height),
            border_color,
            line_width,
        )

    # Overlay image to lower right side
    elif position == "lr":
        original_image[-new_height:, -new_width:, :] = cv2.resize(
            inserted_image, (new_width, new_height), interpolation=interpolation
        )
        cv2.rectangle(
            original_image,
            (width - new_width, height - new_height),
            (width, height),
            border_color,
            line_width,
        )


# =============================================================================
# MATH/GEOMETRY OPERATIONS - MATH/GEOMETRY OPERATIONS - MATH/GEOMETRY OPERATION


def flat_matrix_for_service(numpy_array):
    """ 
        Flat numpy matrix in vector
    Args:
        numpy_array: `np.array` matrix to flat
    Returns:
        _: `list` vector of matrix flatten
    """

    numpy_array = np.array(numpy_array)

    rows = len(numpy_array)
    cols = len(numpy_array[0])
    nelem = rows * cols
    elems = numpy_array.reshape(1, nelem)[0]

    return list(np.append([float(rows), float(cols)], elems))


def matrix_from_flat(list_vector):
    """ 
        reshape a list vector to matrix numpy array     
    Args:
        list_vector: `list` list to convert to numpy array (matrix)
    Returns:
        _: `numpy.array` matrix of list reshaped
    """

    rows = list_vector[0]
    cols = list_vector[1]

    return np.array(list_vector[2:]).reshape(rows, cols)


def line_intersection(line1, line2):
    """ 
        returns the instersection coordinate between two lines
    Args:
        line1: `tuple` line 1 to calculate intersection coordinate
        line2: `tuple` line 2 to calculate intersection coordinate
        M: `numpy.narray` rotation matrix from geometric projection to original 
    Returns:
        _: `tuple` intersection cord between line 1 and line 2
    """

    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise Exception("lines do not intersect")

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    return int(round(x)), int(round(y))


def discrete_contour(contour, Dl):
    """  Takes contour points to get a number of intermediate points
    Args:
        contour: `List` contour or list of points to get intermediate points
        Dl: `int` distance to get a point by segment
    Returns:
        new_contour: `List` new contour with intermediate points
    """

    # If contour has less of two points is not valid for operations
    if len(contour) < 2:
        printlog(msg="No valid segment", msg_type="ERROR")
        return contour

    # New contour variable
    new_contour = []

    # Iterate through all contour points
    for idx, cordinate in enumerate(contour):

        # Select next contour for operation
        if not idx == len(contour) - 1:
            next_cordinate = contour[idx + 1]
        else:
            next_cordinate = contour[0]

        # Calculate length of segment
        segment_lenth = math.sqrt(
            (next_cordinate[0] - cordinate[0]) ** 2
            + (next_cordinate[1] - cordinate[1]) ** 2
        )

        divitions = segment_lenth / Dl  # Number of new point for current segment
        dy = next_cordinate[1] - cordinate[1]  # Segment's height
        dx = next_cordinate[0] - cordinate[0]  # Segment's width

        if not divitions:
            ddy = 0  # Dy value to sum in Y axis
            ddx = 0  # Dx value to sum in X axis
        else:
            ddy = dy / divitions  # Dy value to sum in Y axis
            ddx = dx / divitions  # Dx value to sum in X axis

        # get new intermediate points in segments
        for idx in range(0, int(divitions)):
            new_contour.append(
                (int(cordinate[0] + (ddx * idx)), int(cordinate[1] + (ddy * idx)))
            )

    # Return new contour with intermediate points
    return new_contour


# =============================================================================
