# =============================================================================
"""
Code Information:
    Maintainer: Eng. John Alberto Betancourt G
	Mail: john@kiwicampus.com
	Kiwi Campus / Computer & Ai Vision Team
"""

# =============================================================================
from __future__ import division

import numpy as np

import json
import math
import os

import cv2
import requests
import PIL.ImageColor as ImageColor

from web_client.msg import Messages
from python_utils.utils import get_projection_point_src, get_distor_point

# =============================================================================
HTTP_SERVER_URL = os.getenv("HTTP_SERVER_URL","api.kiwicampus.com")

# =============================================================================
STANDARD_COLORS = [
    'AliceBlue', 'Chartreuse', 'Aqua', 'Aquamarine', 'Azure', 'Beige', 'Bisque',
    'BlanchedAlmond', 'BlueViolet', 'BurlyWood', 'CadetBlue', 'AntiqueWhite',
    'Chocolate', 'Coral', 'CornflowerBlue', 'Cornsilk', 'Crimson', 'Cyan',
    'DarkCyan', 'DarkGoldenRod', 'DarkGrey', 'DarkKhaki', 'DarkOrange',
    'DarkOrchid', 'DarkSalmon', 'DarkSeaGreen', 'DarkTurquoise', 'DarkViolet',
    'DeepPink', 'DeepSkyBlue', 'DodgerBlue', 'FireBrick', 'FloralWhite',
    'ForestGreen', 'Fuchsia', 'Gainsboro', 'GhostWhite', 'Gold', 'GoldenRod',
    'Salmon', 'Tan', 'HoneyDew', 'HotPink', 'IndianRed', 'Ivory', 'Khaki',
    'Lavender', 'LavenderBlush', 'LawnGreen', 'LemonChiffon', 'LightBlue',
    'LightCoral', 'LightCyan', 'LightGoldenRodYellow', 'LightGray', 'LightGrey',
    'LightGreen', 'LightPink', 'LightSalmon', 'LightSeaGreen', 'LightSkyBlue',
    'LightSlateGray', 'LightSlateGrey', 'LightSteelBlue', 'LightYellow', 'Lime',
    'LimeGreen', 'Linen', 'Magenta', 'MediumAquaMarine', 'MediumOrchid',
    'MediumPurple', 'MediumSeaGreen', 'MediumSlateBlue', 'MediumSpringGreen',
    'MediumTurquoise', 'MediumVioletRed', 'MintCream', 'MistyRose', 'Moccasin',
    'NavajoWhite', 'OldLace', 'Olive', 'OliveDrab', 'Orange', 'OrangeRed',
    'Orchid', 'PaleGoldenRod', 'PaleGreen', 'PaleTurquoise', 'PaleVioletRed',
    'PapayaWhip', 'PeachPuff', 'Peru', 'Pink', 'Plum', 'PowderBlue', 'Purple',
    'Red', 'RosyBrown', 'RoyalBlue', 'SaddleBrown', 'Green', 'SandyBrown',
    'SeaGreen', 'SeaShell', 'Sienna', 'Silver', 'SkyBlue', 'SlateBlue',
    'SlateGray', 'SlateGrey', 'Snow', 'SpringGreen', 'SteelBlue', 'GreenYellow',
    'Teal', 'Thistle', 'Tomato', 'Turquoise', 'Violet', 'Wheat', 'White',
    'WhiteSmoke', 'Yellow', 'YellowGreen'
]

# =============================================================================
# SKYNET DRAWING FUNCTIONS - SKYNET DRAWING FUNCTIONS - SKYNET DRAWING FUNCTION

def draw_bounding_box_on_image(image, ymin, xmin, ymax, xmax, color=(255,0,0),
    thickness=2, display_str_list=(), display_distance=None, meta=None,
    use_normalized_coordinates=True, font_scale=0.5, font_thickness=1):
    """     
        Draws predictions bounding boxes in input image
    Args:
        image: `cv2.math` image to draw bounding boxes
        ymin: `float` inferior y axis coordinate of bounding box
        xmin: `float` inferior x axis coordinate of bounding box
        ymax: `float` superior y axis coordinate of bounding box
        xmax: `float` superior x axis coordinate of bounding box
        color: `list` [B, G, R] color of bounding box
        thickness: `int` thickness of bounding box lines
        display_str_list: `boolean` Enable/Disable printings
        display_distance: `boolean` Enable/Disable distances drawings
        meta: `boolean` Enable/Disable meta data drawings
        use_normalized_coordinates: `boolean` Enable/Disable normalized coordinates
        font_scale: `float` font scale factor
        font_thickness: `int` font line thickness       
    Returns:
    """

    # Get video dimensions
    im_height, im_width, _ = image.shape

    if use_normalized_coordinates: # Use normalized coordinates
        (left, right, top, bottom) = (int(xmin*im_width), int(xmax*im_width),
                                      int(ymin*im_height), int(ymax*im_height))
    else: # Not using normalized coordinates
        (left, right, top, bottom) = (int(xmin), int(xmax), int(ymin), int(ymax))

    # Draw bounding box in image
    cv2.rectangle(image, (left, top), (right, bottom), color, thickness)

    if len(display_str_list):
        display_str_heights = [cv2.getTextSize(ds, cv2.FONT_HERSHEY_SIMPLEX, 
            font_scale, font_thickness)[0][1] for ds in display_str_list]
        total_display_str_height = (1 + 2 * 0.05) * sum(display_str_heights)
        text_bottom = top if top > total_display_str_height else int(bottom + total_display_str_height)            
        
        for display_str in display_str_list[::-1]:
            (text_width, text_height), _ = cv2.getTextSize(display_str, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
            margin = int(np.ceil(0.05 * text_height))
            cv2.rectangle(image, (left, text_bottom - text_height - 2 * margin),
                          (left + text_width, text_bottom), color, -1)
            cv2.putText(image, display_str,
                        (left + margin, text_bottom - text_height//2 + margin),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1)
            text_bottom -= text_height - 2 * margin

    if display_distance is not None:
 
        display_distance = "{:.2f}".format(display_distance)
 
        (text_width, text_height), _ = cv2.getTextSize(
            display_distance, cv2.FONT_HERSHEY_DUPLEX, 0.6, 2)

        margin = int(np.ceil(0.05 * text_height))

        cv2.rectangle(image, (right - text_width - 2*margin, bottom - text_height - 2*margin),
                      (right, bottom), color, -1)

        mid_x = int( (xmax + xmin) / 2)
        mid_y = int( (ymax + ymin) / 2)
        warn_indicator = (0,0,255) if (0.3 < mid_x < 0.7) else (0,0,0)

        cv2.putText(image, display_distance,
                    (right - text_width - margin, bottom - 2*margin),
                    cv2.FONT_HERSHEY_DUPLEX, 0.6, warn_indicator, 2)

    if meta is not None:

        state, score = meta["state"], "{:.2f}".format(meta["estimator"])
        display_str_meta = "{}: {}".format(state,score)

        (text_width, text_height), _ = cv2.getTextSize(display_str_meta, cv2.FONT_HERSHEY_DUPLEX, 0.6, 1)
        margin = int(np.ceil(0.05 * text_height))

        cv2.rectangle(image, (left, top),
                      (left + text_width + 2*margin, top + text_height + 2*margin), color, -1)

        cv2.putText(image, display_str_meta,
                    (left + margin, top + text_height + 2*margin),
                    cv2.FONT_HERSHEY_DUPLEX, 0.6, (0,0,0), 1)

def visualize_boxes_on_image(img, predictions, show_labels=True, show_distances=True,
    show_meta=True, use_normalized_coordinates=True):
    """     
        Draws predictions bounding boxes in input image
    Args:
        img: `type`  image to draw bounding boxes
        predictions: `list` predictions of object detection
        show_labels: `boolean` Enable/Disable labels drawings
        show_distances: `boolean` Enable/Disable distances drawings
        show_meta: `boolean` Enable/Disable meta data drawings
        use_normalized_coordinates: `boolean` Enable/Disable normalized coordinates
    Returns:
        img: `cv2.math` image with bounding boxes drawn
    """

    for prediction in predictions:
        # Get box coordinates
        ymin, xmin, ymax, xmax = prediction['box']

        # Get color to draw bounding box
        color = STANDARD_COLORS[prediction['id'] % 110]
        color = ImageColor.getrgb(color) #bgr to opencv

        # Get text to print in bounding box if show_labels
        display_str = [prediction['name']] if show_labels else []

        # Get distances to print in bounding box if show_distances 
        distance = prediction['distance'] if (show_distances and 'distance' in prediction) else None

        # Get meta data to print in bounding box if show_meta 
        meta = prediction['meta'] if (show_meta and 'meta' in prediction) else None

        # Draw bounding box for prediction
        draw_bounding_box_on_image(image=img, ymin=ymin, xmin=xmin, ymax=ymax, xmax=xmax,
            display_str_list = display_str, display_distance = distance, color = color,
            meta = meta, thickness = 2, use_normalized_coordinates = use_normalized_coordinates 
            )
    
    return img

def crop_image(img, up, down):
    """     
        Returns cropped image
    Args:
        img: `cv2.math` input image to crop
        up: `int` up side value to crop
        down: `int` down side value to crop
    Returns:
        img: `cv2.math` input cropped
    """

    h,_,_ = img.shape
    return img[int(up*h):h-int(down*h),:,:]

def visualize_vehicle_behavior(img, img_scale=None, steering=None, steering_label=None, 
    mask=None, angle_multiplier=3, line_width=3, line_length=100, mask_multiplier=1.0,
    interpolation=cv2.INTER_NEAREST):
    """     
        draws vehicle behavior in input image
    Args:
        img: `cv2.math` 
        img_scale: `cv2.math` description
        steering: `float` vehicles steering angle
        steering_label: `int` steering behavior label
        mask: `cv2.math` mask with activations of vehicle behavior
        angle_multiplier: `float` angle conversion constant
        line_width: `int` line thickness
        line_length: `float` line length of vehicle behavior
        mask_multiplier: `float` mask multiplier factor
        interpolation: `int` flag to interpolate image when is resized
    Returns:
        img: `cv2.math` image with vehicle behavior drawn
    """    

    if mask is not None:
        mask *= 255 * mask_multiplier
        if (img.shape != mask.shape):
            img = cv2.resize(img, (mask.shape[1], mask.shape[0]), interpolation=interpolation)
        img[..., 2] = mask[..., 0]

    if img_scale is not None:
        img = cv2.resize(img, (0,0), fx=img_scale, fy=img_scale, interpolation=interpolation)

    if steering_label is not None:
        line_color = (0, 255, 255)
        if steering_label == 0:
            line_color = (204, 0, 204) #pink
        draw_path(img, [steering_label * angle_multiplier], line_color = line_color, line_width = line_width, line_length = line_length)

    if steering is not None:
        draw_path(img, [steering * angle_multiplier], line_color = (0, 0, 255), line_width = line_width, line_length = line_length)

    return img

def draw_path(img, angles, angle_conversion_constante=0.3, line_length=2, 
    line_color=(0, 255, 0), line_width=1):
    """     
        Draws in center of image the path of robot
    Args:
        img: `cv2.math` input image to draw path (line)
        angles: `list` [rad] angles of lines to draw
        angle_conversion_constante: `float` angle conversion constant
        line_length: `float` line length
        line_color: `list` [R, G, B] line color
        line_width: `int` line thickness
    Returns:
    """    

    angles = np.asarray(angles)
    # negate it because negative angles means to the right
    cumsum = np.cumsum(-angles) * angle_conversion_constante 

    # Get image dimensions
    height, width, _ = img.shape

    # First point should be in the center of image
    point1 = (int(width/2), height)

    for angle in cumsum:
        point2 = calculate_next_point(point1, angle, line_length)
        cv2.line(img, point1, point2, line_color, line_width)
        point1 = point2

# =============================================================================
# DRAWING FUNCTIONS - DRAWING FUNCTIONS - DRAWING FUNCTIONS - DRAWING FUNCTIONS
def draw_line(image, x0, y0, line_length, angle, line_width=2, line_color=(255,255,255)):
    """     
        Draws a line in an input image
    Args:
        image: `cv2.math` image to draw a line
        x0: `int` initial point X axis coordinate of line
        y0: `int` initial point Y axis coordinate of line
        line_length: `float` line length 
        angle: `float` [rad] angle to calculate final point of line to draw
        line_width: `int` line thickness
        line_color: `list` [RGB] color of line
    Returns:
        point2: `tuple` final point coordinate of line to draw
    """    

    # Initial line point 
    point1 = (x0, y0)
    
    # Calculate final point to draw the line
    point2 = calculate_next_point(point1, angle*np.pi/180, line_length)

    # Draw the line
    cv2.line(image, point1, point2, line_color, line_width)

    return point2

def apply_virtual_pan(img_src, images_dict, pan, lateral_cutoff, switch_cams=False):
    """     
        Applies virtual pan to center image to concatenate with left or right image
    Args:
        img_src: `cv2.math` image to apply virtual pan (image concatenation)
        images_dict: `list` of cv2.math images to get image to concatenate
        central_image: `cv2.math` central image to concatenate images according with virtual pan
        pan: `float` virtual pan value
        lateral_cutoff: `int` lateral pixels to cut off image
    Returns:
        image_copy: `cv2.math` image with virtual pan applied
    """

    if switch_cams:
        pan *= -1

    # Get positions
    width = images_dict["C"].shape[1]
    left_position = int(width*abs(pan/100.))
    right_position = width - left_position
    
    if pan == 100:
        img_src = images_dict["LL"] if "LL" in images_dict else images_dict["C"]
    elif pan == -100:
        img_src = images_dict["RR"] if "RR" in images_dict else images_dict["C"]
    elif pan < 0: # Concatenate right image
        img_src[:, 0:right_position, :] = images_dict["C"][:, left_position:, :]
        if (lateral_cutoff >= right_position):
            img_src[:, right_position:lateral_cutoff, :] = images_dict["LL"][
                :, right_position:lateral_cutoff, :]
            img_src[:, lateral_cutoff:, :] = images_dict["LL"][
                :, lateral_cutoff:, :]
        else:
            img_src[:, right_position:, :] = images_dict["LL"][
                :, lateral_cutoff:left_position + lateral_cutoff, :]
    elif pan > 0: # Concatenate left image
        img_src[:, left_position:, :] = images_dict["C"][:, 0:right_position, :]
        if (lateral_cutoff >= right_position):
            img_src[:, width - lateral_cutoff:left_position, :] = images_dict["RR"][
                :, width - lateral_cutoff:left_position, :]
            img_src[:, 0:width-lateral_cutoff, :] = images_dict["RR"][
                :, 0:width-lateral_cutoff, :]
        else:
            img_src[:, 0:left_position, :] = images_dict["RR"][
                :, right_position-lateral_cutoff:width - lateral_cutoff, :]
    
    return img_src

def insert_image(original_image, inserted_image, new_width, new_height, position='ur', 
    line_width=2, border_color=(255,255,255), interpolation=cv2.INTER_NEAREST):
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
    if position == 'ul':
        original_image[0:new_height:, 0:new_width, :] = cv2.resize(
            inserted_image, (new_width, new_height), interpolation=interpolation)
        cv2.rectangle(
            original_image, (0, 0), (new_width, new_height), border_color, line_width)

    # Overlay image to upper right side
    elif position == 'ur':
        original_image[0:new_height:, -new_width:,:] = cv2.resize(
            inserted_image, (new_width, new_height), interpolation=interpolation)
        cv2.rectangle(
            original_image, (width - new_width, 0), (width, new_height), border_color, line_width)

    # Overlay image to lower left side
    elif position == 'll':
        original_image[-new_height:, 0:new_width, :] = cv2.resize(
            inserted_image, (new_width, new_height), interpolation=interpolation)
        cv2.rectangle(
            original_image, (0, height - new_height), (new_width, height), border_color, line_width)

    # Overlay image to lower right side
    elif position == 'lr':
        original_image[-new_height:, -new_width:, :] = cv2.resize(
            inserted_image, (new_width,new_height), interpolation=interpolation)
        cv2.rectangle(
            original_image, (width - new_width, height - new_height), (width, height), border_color, line_width)

def draw_pad_centers(image, x_coords, y_coords, M_inv, mtx, dist, 
    point_color=(0,255,0)):
    """     
        Draws pad center in input image
    Args:
        image: `cv2.math` image to draw chess board patterns
        x_coords: `list` x axis coordinates of chessboard patterns 
        y_coords: `list` y axis coordinates of chessboard patterns
        M: `numpy.darray` rotation matrix from original image to surface projection space
        mtx: `numpy.narray` camera's distortion matrix
        dist: `numpy.narray` camera's distortion vector
        point_color: `list` [R, G, B] color of pad center point
    Returns:
        image: `cv2.math` image with pads drawn
    """

    try:
        # Iterate over all coordinates to draw
        for i, (x_proj, y_proj) in enumerate(zip(x_coords, y_coords)):

            # Chose color to print pads center
            color = point_color if i == 0 else (0, 0, 255)
            
            # Convert coordinate from surface projection to original image space (undistorted)
            x,y = get_projection_point_src((int(x_proj), int(y_proj), 1), M_inv)

            # Convert coordinate from original image space undistorted to distorted
            x,y = get_distor_point((x, y), mtx, dist)

            # Check if point is inside image
            # TODO: (JOHN) Set with ORIGINAL_SIZE
            if 0 <= x < 10000 and 0 <= y < 10000:
                image = cv2.circle(image, (x, y), 6, (0, 0, 0), -1)
                image = cv2.circle(image, (x, y), 4, color, -1) 

    except Exception as e:
        print("[ERROR]: Drawing pad center, {}".format(str(e)))
            
    return image

def draw_chess_board(image, x_coords, y_coords, M, mtx, dist):
    """     
        Draws chessboard patterns features
    Args:
        image: `cv2.math` image to draw chess board patterns
        x_coords: `list` x axis coordinates of chessboard patterns 
        y_coords: `list` y axis coordinates of chessboard patterns
        M: `numpy.darray` rotation matrix from original image to surface projection space
        mtx: `numpy.narray` camera's distortion matrix
        dist: `numpy.narray` camera's distortion vector
    Returns:
        image: `cv2.math` image with chess board patterns drawn
    """

    # If theres any chess board coordinates to print
    if x_coords:

        # Assing coordinates 
        point2track = x_coords[0], y_coords[0]
        x1roi, y1roi, x2roi, y2roi = x_coords[1], y_coords[1], x_coords[2], y_coords[2]

        # Get important coordinates to print 
        point2track_distor = get_distor_point(point2track, mtx, dist)
        x1roi_distor, y1roi_distor = get_distor_point((x1roi, y1roi), mtx, dist)
        x2roi_distor, y2roi_distor = get_distor_point((x2roi, y2roi), mtx, dist)
        xc, yc = (x2roi_distor - x1roi_distor)//2 + x1roi_distor, (y2roi_distor - y1roi_distor)//2 + y1roi_distor

        # Draw chess board pattern features
        cv2.rectangle(image, (x1roi_distor, y1roi_distor), (x2roi_distor, y2roi_distor), (0, 255, 0), 3)
        cv2.line(image, (xc, yc), point2track_distor, (255,0,0), 2)
        cv2.circle(image, (xc, yc), 3, (0, 0 ,255), -1)
        cv2.circle(image, point2track_distor, 3, (0, 255 ,255), -1)
        
        # assume pattern is 2/3 width of the bot with wheels
        pattern_wd = int((x2roi_distor - x1roi_distor)*(4/3.0)) 

        # Draw line from center of pattern to robots ground base
        xpoint, ypoint = point2track_distor
        cv2.line(image, (xpoint-pattern_wd//2, ypoint), (xpoint+pattern_wd//2, ypoint), (255,0,0), 2)

    return image

def show_msg(image, msg, msg_type, left=5, top=5, color=(0, 255, 255)):
    """     
        Prints rendered mesasage in image 
    Args:
        image: `cv2.math` image to print text
        msg: `string` message text
        msg_type: `string` message type
            msg_type == Messages.INFO -----> Informative
            msg_type == Messages.WARNING --> Warning
            msg_type == Messages.ERROR ----> Error
    Returns:
    """

    # Select color depending on message type
    if msg_type == Messages.INFO:
        color = (255, 255, 255)
    elif msg_type == Messages.WARNING:
        color = (0, 255, 255)
    elif msg_type == Messages.ERROR:
        color = (0, 0, 255)

    # Print message in image  
    render_lines(image=image, msg=msg, left=left, top=top, 
        color=color, font_scale=0.8, font_thickness=2)

def render_lines(image, msg, left, top, color, font_scale=1.3, font_thickness=2):
    """     
        Prints and render lines of text in an image
    Args:
        image: `cv2.math` image to print text
        msg: `string` message text
        left: `int` x coordinate to put text
        top: `int` y top coordinate to put text
        color: `list` [B,G,R] text color
        text_height: `float` text height
        font_scale: `float` font size
        font_thickness: `int` font thickness
    Returns:
    """

    # Calculates the width and height of a text string.
    (text_width, text_height), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
    (width_, _), _ = cv2.getTextSize("-", cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)

    # get image dimensions
    im_height, im_width, _ = image.shape

    # Calculate number of lines to print in image
    number_lines = int(math.ceil(text_width/(im_width-3*left-width_)))

    # Calculate y axis pixels relation to print text
    pixel_relation = len(msg)/text_width

    # Print all text lines
    for line in range(number_lines):
        if line + 1 == number_lines: # last line or first
            msg_str = msg
        else:
            upper = int(pixel_relation*(im_width-3*left-width_)) # to avoid going out from screen
            msg_str = msg[:upper] + "-"
            msg = msg[upper::]
        # Print text in image
        render_text(image, msg_str, left, top + (text_height+2)*(line+1), color, text_height, font_scale, font_thickness)

def render_text(image, msg, left, top, color, text_height=0, font_scale=1.3, 
    font_thickness=2):
    """     
        Prints text in an image
    Args:
        image: `cv2.math` image to print text
        msg: `string` message text
        left: `int` x coordinate to put text
        top: `int` y top coordinate to put text
        color: `list` [B,G,R] text color
        text_height: `float` text height
        font_scale: `float` font size
        font_thickness: `int` font thickness
    Returns:
    """

    # Draw text background 
    cv2.putText(image, msg, (left, top + text_height),
        cv2.FONT_HERSHEY_DUPLEX, font_scale, (0,0,0), font_thickness+5)

    # Draw text frontground 
    cv2.putText(image, msg,(left, top + text_height),
        cv2.FONT_HERSHEY_DUPLEX, font_scale, color, font_thickness)

def dotline(src, p1, p2, color, thickness, Dl):
    """  draws a doted line on input image
    Args:
        src: cv2.mat source image
        p1: tuple line's first point 
        p2: tuple line's second point 
        color: tuple lines' color RGB [B, G, R] [int]
        thickness: int lines' thickness
        Dl: int distance in pixels between every point
    Returns:
        src: cv2.mat image with doted line drawn
    """

    # Get a number of intermediate points
    segments = discrete_contour((p1, p2), Dl)

    # Draw doted line 
    for segment in segments:
        cv2.circle(src, segment, thickness, color, -1) 

    # Return result
    return src

# =============================================================================
# UTIL FUNCTIONS - UTIL FUNCTIONS - UTIL FUNCTIONS - UTIL FUNCTIONS - UTIL FUNC
def identity_fn(x, **kwargs): return x

def discrete_contour(contour, Dl):
    """  Takes contour points to get a number of intermediate points
    Args:
        contour: List contour or list of points to get intermediate points
        Dl: int distance to get a point by segment
    Returns:
        new_contour: List new contour with intermediate points
    """

    # If contour has less of two points is not valid for operations
    if len(contour) < 2:
        print("Error: no valid segment")
        return contour

    # New contour variable
    new_contour = []

    # Iterate through all contour points
    for idx, cordinate in enumerate(contour):

        # Select next contour for operation
        if not idx == len(contour)-1:
            next_cordinate = contour[idx+1]
        else:
            next_cordinate = contour[0]

        # Calculate length of segment
        segment_lenth = math.sqrt((next_cordinate[0] - cordinate[0])**2 +\
                        (next_cordinate[1] - cordinate[1])**2)
        
        divitions = segment_lenth/Dl # Number of new point for current segment
        dy = next_cordinate[1] - cordinate[1]    # Segment's height
        dx = next_cordinate[0] - cordinate[0]    # Segment's width
        
        if not divitions:
            ddy = 0 # Dy value to sum in Y axis
            ddx = 0 # Dx value to sum in X axis
        else:
            ddy = dy/divitions  # Dy value to sum in Y axis
            ddx = dx/divitions  # Dx value to sum in X axis
        
        # get new intermediate points in segments
        for idx in range(0, int(divitions)):
            new_contour.append((int(cordinate[0] + (ddx*idx)), 
                                int(cordinate[1] + (ddy*idx))))    

    # Return new contour with intermediate points
    return new_contour

def calculate_next_point(point, theta, line_length):
    """     
        Calculate the coordinate given a reference or orin point and 
        angle theta and the line length
    Args:
        point: `tuple` [X, Y][pix, pix] Coordinates of reference point in image space
        theta: `float` [rad] angle of line
        line_length: `float` line length
    Returns:
        x1: `int` x axis coordinate
        y1: `int` y axis coordinate
    """
    
    x1 = int(point[0] - np.sin(theta) * line_length)
    y1 = int(point[1] - np.cos(theta) * line_length)

    return x1, y1

def predictions_warning_labels(detections, critical_dist=1.0, warning_dist=2.0):
    """ Return activation and specifications to display warning labels
    Args:
        detections: `list` of dictionaries with object detections 
        critical_dist: `float` critical distance to activate critical label
        warning_dist: `float` warning distance to activate warning label
    Returns:
        label: `string` warning label name
            warning_object_icon
            critical_object_icon
    """

    label = None
    for detection in detections["predictions"]:
        if str(detection["name"]) == "person":
            dist = math.sqrt(float(math.pow(detection["distance_components"][0], 2))+
                float(math.pow(detection["distance_components"][1], 2)))
            if dist <= critical_dist:
                return "critical_object_icon"
            elif dist <= warning_dist:
                label = "warning_object_icon"   
    return label

# =============================================================================
# MONO VISION FUNCTIONS - MONO VISION FUNCTIONS - MONO VISION FUNCTIONS - MONO 

def world_to_projections(x_list, y_list, ppy, ppx, bot_cord_proj):
    """     
        Convert all input coordinates from world [m] to projection [pix]
    Args:
        x_list: `list` x axis coordinates [m]
        y_list: `list` y axis coordinates [m]
        ppy: `float` 'y' axis pixels to meters relation
        ppx: `float` 'x' axis pixels to meters relation
        bot_cord_proj: `list` [pix, pix] robots view coordinates in 
            projection in an image reference
    Returns:
        x_coords: `list` new projection coordinates in 'x' axis [pix]
        y_coords: `list` new projection coordinates in 'y' axis [pix]
    """

    # initialize new coordinates list
    x_coords, y_coords = [], []

    # Convert all input coordinates from world [m] to projection [pix]
    for  i, (x, y) in enumerate(zip(x_list, y_list)):
        xi, yi = World2Projection((x, y), ppy, ppx, bot_cord_proj)
        x_coords.append(xi); y_coords.append(yi)

    return x_coords, y_coords

def World2Projection(cord_world, ppy, ppx, bot_cord_proj):
    """     
        Converts world coordinates to projection coordinates 
    Args:
        cord_world: `list` [m, m] projection coordinates in meters
        ppy: `float` 'y' axis pixels to meters relation
        ppx: `float` 'x' axis pixels to meters relation
        bot_cord_proj: `list` [pix, pix] robots view coordinates in 
            projection in an image reference
    Returns:
        coord_proj: `tuple` projection coordinates
    Note:
        for the robot, x is positive to the left
    """

    # Calculate projection coordinates from world coordinates
    x_proj = int(bot_cord_proj[0] - int(cord_world[0]*ppx)) 
    y_proj = int(bot_cord_proj[1] - int(cord_world[1]*ppy))

    coord_proj = (x_proj, y_proj)

    return coord_proj

# =============================================================================
# ZOOM FUNCTIONS - ZOOM FUNCTIONS - ZOOM FUNCTIONS - ZOOM FUNCTIONS - ZOOM FUNC

class Zoomer(object):

    def __init__(self):
        """     
            Zoom object to track move commands for zoom
        Args:
        Returns:
        """
        super(Zoomer, self).__init__()

        self.last_move = 0 # Last movement acction for zoomer axis
        self.counter = 0 # Counter for movement iterations
        self.last_move_long = False # Last movement to move zoom in some direction
        self.max_iterations = 10 # Number of iterations to move the zoom

def handle_zoom_command(val, zoom_obj, move_rel, roi, direction="x", thresh=70):
    """     
        calculates zooms roi according with x and y axis movements
    Args:
        val: `float` value of tilt or pan to move zoom [-100 to 100]
        zoom_obj: `Zoomer` variable of zoom object type
        move_rel: `float` value to move in the axis 'direction'
        roi: `list` roi list of roi normalized coordinates [%]xmin, [%]ymin, [%]xmax, [%]ymax
        direction: `string` axis direction to move zoom window
        thresh: `int` value to consider a movement in tilt or pan 
    Returns:
        roi: `list` roi list of roi normalized coordinates [%]xmin, [%]ymin, [%]xmax, [%]ymax
    """

    # because positive tilt is opposite to positive pan (teb logic?)
    sign = 1 if direction=="x" else -1 

    # Check if movement value is superior to threshold to do any zoom action
    if abs(val) > thresh:

        # Increment zoom counter if current move is equal to previous move
        if val == zoom_obj.last_move:
            zoom_obj.counter += 1

            # Update roi if counter is enough to update roi
            if zoom_obj.counter == 1 and not zoom_obj.last_move_long:
                zoom_obj.last_move_long = False
                roi = update_roi(roi=roi, val=sign*val, base_move=move_rel, direction=direction)

            # Update roi if counter is superior to max_iterations to move zoom window
            if zoom_obj.counter > zoom_obj.max_iterations:
                zoom_obj.counter = 0
                zoom_obj.last_move_long = True
                roi = update_roi(roi=roi, val=sign*val, base_move=move_rel, direction=direction)

        # Otherwise reset counter value and zoom state
        else:
            zoom_obj.counter = 0
            zoom_obj.last_move_long = False 

    zoom_obj.last_move = val
    
    return roi

def update_roi(roi, val, base_move, direction="x"):
    """     
        updates zooms roi according with x and y axis movements
    Args:
        roi: `list` roi list of roi normalized coordinates [%]xmin, [%]ymin, [%]xmax, [%]ymax
        val: `float` value of tilt or pan to move zoom [-100 to 100]
        base_move: `float` value to move in the axis 'direction'
        direction: `string` axis direction to move zoom window
    Returns:
        roi: `list` roi list of roi normalized coordinates [%]xmin, [%]ymin, [%]xmax, [%]ymax
    """

    # If direction in x axis
    if direction == "x":
        x_move = base_move*np.sign(-val)
        y_move = 0
    # Otherwise direction in y axis
    else:
        y_move = base_move*np.sign(-val)
        x_move = 0 

    # roi list of roi normalized coordinates [%]xmin, [%]ymin, [%]xmax, [%]ymax
    xmin, ymin, xmax, ymax = roi 
    xmin, xmax, ymin, ymax = xmin + x_move, xmax + x_move, ymin + y_move, ymax + y_move 

    # X axis superior conditional
    if xmax > 1.001:
        xmax = xmax - xmin
        xmin = 0.

    # X axis inferior conditional
    if xmin < -0.001:
        xmin = 1.-abs(xmax - xmin)
        xmax = 1.
        
    # Y axis superior conditional
    if ymax > 1.001:
        ymax = ymax - ymin
        ymin = 0.

    # Y axis inferior conditional
    if ymin < -0.001:
        ymin = 1.-abs(ymax - ymin)
        ymax = 1.
        
    return xmin, ymin, xmax, ymax

def get_zoom_image(hd_image, zoom_roi, hd_image_flipped = False):
    """     
    Cuts 'hd_image' image variable according with the region of interest ROI
    Args:
        hd_image: `cv2.math` input image to get region of interest ROI
        zoom_roi: `list` [float] relative coordinates to get roi
        hd_image_flipped: `boolean` Enable/Disable flip result image
    Returns:
        zoom_image: `cv2.math` roi of 'hd_image' (zoom image)
    Note:
        hd_image_flipped refers if the hd image is straight or flipped 180 
        degrees (upside down)
    """

    # Get input image dimensions
    hd_image_h, hd_image_w,_ = hd_image.shape

    # relative coordinates
    # print(zoom_roi)
    xmin, ymin, xmax, ymax =  zoom_roi 

    ymax = 1 if ymax > 1 else ymax if ymax > 0 else 0
    ymin = 1 if ymin > 1 else ymin if ymin > 0 else 0
    xmax = 1 if xmax > 1 else xmax if xmax > 0 else 0
    xmin = 1 if xmin > 1 else xmin if xmin > 0 else 0

    # transform to absolute pixels in hd image
    xmin = int(xmin*(hd_image_w-1))
    ymin = int(ymin*(hd_image_h-1))
    xmax = int(xmax*(hd_image_w-1))
    ymax = int(ymax*(hd_image_h-1))

    # If flip result zoom image
    if hd_image_flipped:
        # Transform coordinates because image is not more straight, it is flipped 180
        roi_w = xmax - xmin
        xmin = hd_image_w - xmin - roi_w # i.e. hd_image_w - xmax
        xmax = xmin + roi_w 

        roi_h = ymax - ymin
        ymin = hd_image_h - ymin - roi_h
        ymax = ymin + roi_h

        # define flip function to apply to zoom region
        return np.flip(np.flip(hd_image[ymin:ymax, xmin:xmax,:], 1), 0)

    # Get from original image region of interest
    return hd_image[ymin:ymax, xmin:xmax,:]

# =============================================================================