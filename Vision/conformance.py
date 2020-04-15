import numpy as np
from scipy import signal

def correlate(fit, img): 
    '''
    correlates a filter created from the shape of a polynomial with an image and determines 
    whether or not the polynomial matches the image closely enough
    :param fit: a polynomial to compare with the image
    :param img: the image being used for comparison
    '''
    # start from almost the bottom of the image to and go to almost the top of the image
    MAX_Y = int(img.shape[0] * (19.0/20))
    MIN_Y = int(img.shape[0] * (1.0/5))
    h = MAX_Y - MIN_Y - 1
    ypts = np.arange(MIN_Y, MAX_Y)

    p = np.poly1d(fit)
    xpts = p(ypts)
    min_x = np.amin(xpts)
    w = int(np.amax(xpts) - min_x)

    # filter values are heavier towards the bottom of the filter
    filter_values = np.arange(100, 255, 155.0/(h-1))

    padding = 10
    fill_val = -2
    curve_filter = np.full((h + padding, w + padding), fill_val)

    # creates filter of necessary size to encompass the shape of xpts and ypts
    for i in range(len(filter_values)):
        curve_filter[(ypts[i] + padding/2.0 - MIN_Y).astype(int), (xpts[i] - min_x + padding/2.0).astype(int)] = filter_values[i]

    blur = np.full([4, 4], 1.0 / 16)
    curve_filter = signal.convolve2d(curve_filter, blur, fillvalue=fill_val)

    # correlate the filter and image and find the location of max correlation
    grad = signal.correlate2d(curve_filter, img, mode='same', fillvalue=fill_val)
    result = np.unravel_index(grad.argmax(), grad.shape)

    # transforms points back to the image space
    image_y = result[0] - padding//2 + MIN_Y
    image_x = result[1] - padding//2 + min_x

    # difference between middle of image and a point on the curve at that height; necessary because
    # correlation is computed from middle of image
    offset = int(img.shape[1]/2 - p(img.shape[0]/2))

    actual_x = image_x - offset
    expected_x = p(image_y)

    return abs(actual_x - expected_x) < 17 and grad[result[0]][result[1]] > 600
