import numpy as np

from conformance import correlate
from geometric import geometric
from lanefilter import LaneFilter

SCALE_FACTOR = .5
_PATH_SIZE = 192.0
_PATH_Y = np.arange(192.0)

def get_binary(img, thresholds):
    """
    Applies transformations to process the given image
    """
    lane_filter = LaneFilter(thresholds)
    binary = lane_filter.apply(img)
    return binary.astype(np.uint8)

def warp_points(pt_s, warp_matrix):
    '''
    uses the warp_matrix to transform points
    '''
    # pt_s are the source points, nxm array.
    pt_d = np.dot(warp_matrix[:, :-1], pt_s.T) + warp_matrix[:, -1, None]

    # Divide by last dimension for representation in image space.
    return (pt_d[:-1, :] / pt_d[-1, :]).T

def get_pts(img, path, calibration):
    '''
    converts points given from the top-down view to points in the camera view
    '''
    uv_model_real = warp_points(np.column_stack((_PATH_Y, path)), calibration)
    uv_model = np.round(uv_model_real).astype(int)

    # keep points that are within frame
    uv_model_dots = uv_model[np.logical_and.reduce((np.all(  
        uv_model > 0, axis=1), uv_model[:, 0] < img.shape[1]/SCALE_FACTOR - 1, uv_model[:, 1] <
                                                    img.shape[0]/SCALE_FACTOR - 1))]
    return uv_model_dots

def get_transformed_fit(binary, fit, calibration, img):
    '''
    gets the lane line polynomials in the camera's view
    '''
    p = np.poly1d(fit)
    xpts = p(_PATH_Y)
    transformed_pts = (get_pts(binary, xpts, calibration) * SCALE_FACTOR).astype(int)
    transformed_fit = np.polyfit(transformed_pts[:,1], transformed_pts[:,0], 2) # :,1 is the y axis
    return transformed_fit 

def interlock(img, left_fit, right_fit, calibration, thresholds):
    '''
    runs the actual vision interlock; the parameters are the components that form the `certificate`
    :param img: image from the sensors
    :param lines: proposed lane lines
    :param thresholds: thresholds used for vision filters
    :param left_fit: polynomial for left lane line
    :param right_fit: polynomial for right lane line
    :return: the results of each vision interlock test
    '''
    binary = get_binary(img, thresholds)

    # perform geometric test
    geometric_result = geometric(left_fit, right_fit, _PATH_SIZE)

    left_transformed_fit = get_transformed_fit(binary, left_fit, calibration, img)
    right_transformed_fit = get_transformed_fit(binary, right_fit, calibration, img)

    # perform conformance test on left and right side of image
    half_width = img.shape[1]//2
    right_transformed_fit[2] -= half_width
    left_result = correlate(left_transformed_fit, binary[:, :half_width])
    right_result = correlate(right_transformed_fit, binary[:, half_width:])

    return geometric_result, left_result, right_result
