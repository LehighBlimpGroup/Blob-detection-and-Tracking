# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Color Binary Filter Example
#
# This script shows off the binary image filter. You may pass binary any
# number of thresholds to segment the image by.

import sensor
import time
import math
import random

sensor.reset()
sensor.set_framesize(sensor.HQVGA)
sensor.set_pixformat(sensor.RGB565)
sensor.skip_frames(time=2000)
clock = time.clock()

# Use the Tools -> Machine Vision -> Threshold Edtor to pick better thresholds.




def find_max(blobs):
    """ Find maximum blob in a list of blobs
        :input: a list of blobs
        :return: Blob with the maximum area,
                 None if an empty list is passed
    """
    max_blob = None
    max_area = 0
    for blob in blobs:
        if blob.area() > max_area:
            max_blob = blob
            max_area = blob.area()
    return max_blob



old_metric=0


lo,hi = -5, 10
red_threshold =[0, 80, -5, 15, None, None]  #[0, 100, -15, 15, 14, 78] (0, 100, 2, 18, -15, -6)
old_loc = (0,0)

# Color
cb = -10
MIN_CB = cb - 10
MAX_CB = cb + 15

nfc = 0 # not found counter
blackandwhite=True

CB_ALTERNATIVES = [MIN_CB, cb, MAX_CB]
ACTIONS=(-.5,-.3, .3,.5)
#ACTIONS=[0]


while True:

    clock.tick()
    img = sensor.snapshot()




    #print(img.get_statistics())

    # action

    action = random.choice(ACTIONS)

    cb+=action
    cb = min(MAX_CB, max(MIN_CB, cb))



    # B interval
    red_threshold[4] = int(cb) - hi
    red_threshold[5] = int(cb) + hi



    if blackandwhite:
        img.binary([red_threshold])
        threshold = (80, 100)
    else:
        threshold = red_threshold

    blobs = [blob for blob in img.find_blobs([threshold], pixels_threshold=100, area_threshold=300,merge=True)]

    total_hist = img.get_histogram(thresholds=[threshold],bins=2)
    ones = total_hist.bins()[1] * img.width()*img.height()

#    print(len(blobs))
#    for blob in img.find_blobs([threshold], pixels_threshold=50, area_threshold=100,merge=True):
    if blobs:
        blob = find_max(blobs)

        img.draw_edges(blob.min_corners(), color=(255, 0, 0))
        img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
        img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))


        #loc = (blob.cxf(), blob.cyf())
        #d2 = (loc[1]-old_loc[1])**2 + (loc[0]-old_loc[0])**2

        metric = 0*blob.area() + 100 * blob.density()

        hist = img.get_histogram(thresholds=[threshold],bins=2, roi=blob.rect())
        # Count the number of white pixels (ones) within the current cell
        ones_in_cell = hist.bins()[1] *blob.rect()[2] * blob.rect()[3]
        metric = ones_in_cell-ones

        if metric < old_metric:
            cb-=action

        old_metric = metric
        #old_loc = loc
        print(cb, metric, blob.area(), blob.density(), clock.fps())



        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        # Note - the blob rotation is unique to 0-180 only.
        img.draw_keypoints(
            [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
        )

        nfc=0
    else:
        nfc+=1
#        print("not detected", nfc)
        if nfc==10:
            cb=random.choice(CB_ALTERNATIVES)
            print("reset", cb)
            nfc=0
            old_metric=-1000
