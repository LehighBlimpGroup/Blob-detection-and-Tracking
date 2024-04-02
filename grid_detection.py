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








old_metric=0


lo,hi = -5, 10
red_threshold =[0, 100, 0, 20, None, None]  #[0, 100, -15, 15, 14, 78] (0, 100, 2, 18, -15, -6)
old_loc = (0,0)

# Color
cb = -30
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


    # action
    action = random.choice(ACTIONS)

    cb+=action
    cb = min(MAX_CB, max(MIN_CB, cb))



    # B interval
    red_threshold[4] = int(cb) - hi
    red_threshold[5] = int(cb) + hi



    if blackandwhite:
        binary_img = img.binary([red_threshold])
        # Convert the binary image to grayscale
        binary_img = binary_img.to_grayscale()

        threshold = (80, 100)
    else:
        threshold = red_threshold

    # Define the dimensions of the grid
    grid_rows = 3
    grid_cols = 3

    # Calculate the size of each cell
    cell_width = int(binary_img.width() / grid_cols)
    cell_height = int(binary_img.height() / grid_rows)


    # Draw grid
#    img.draw_line(cell_width, 0, cell_width, ht, color=(255, 255, 255), thickness=1)
#    img.draw_line(2*cell_width, 0, 2*cell_width, ht, color=(255, 255, 255), thickness=1)
#    img.draw_line(0, cell_height, wh, cell_height, color=(255, 255, 255), thickness=1)
#    img.draw_line(0, 2*cell_height, wh, 2*cell_height, color=(255, 255, 255), thickness=1)



    # Loop through each cell of the grid
    for row in range(grid_rows):
        for col in range(grid_cols):
            # Define the region of interest (ROI) for the current cell
            roi = (col * cell_width, row * cell_height, cell_width, cell_height)
            #print(roi)

            # Crop the ROI from the binary image
            #cropped_roi = binary_img.copy(roi=(roi[0], roi[1], roi[2], roi[3]))


            # Get the statistics of the cropped ROI
#            stats = cropped_roi.get_statistics()
            # Count the number of white pixels (ones) within the current cell
#            ones_in_cell = stats.mean()

            #hist = cropped_roi.get_histogram(bins=2)
            hist = binary_img.get_histogram(bins=2, roi=roi)

            # Count the number of white pixels (ones) within the current cell
            ones_in_cell = hist.bins()[1]*cell_width*cell_height

            #print(f"Number of ones (white pixels) in cell ({row},{col}):", ones_in_cell)

            # Draw the number of ones in the corner of the cell
            binary_img.draw_string(roi[0], roi[1], str(int(ones_in_cell)), color=(255))


            # Count the number of white pixels (ones) within the current cell
            # ones_in_cell = hist.get_percentile(255)
            #print(f"Number of ones (white pixels) in cell ({row},{col}):", ones_in_cell)


            # Draw the ROI on the image
            binary_img.draw_rectangle(roi, color=(int(ones_in_cell)), thickness=1)



    print("FPS=", clock.fps())
    sensor.snapshot().copy(img)

#    blobs = [blob for blob in img.find_blobs([threshold], pixels_threshold=100, area_threshold=400,merge=True)]

#    if blobs:
#        blob = find_max(blobs)



#        img.draw_edges(blob.min_corners(), color=(255, 0, 0))
#        img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
#        img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))



#        loc = (blob.cxf(), blob.cyf())
#        d2 = (loc[1]-old_loc[1])**2 + (loc[0]-old_loc[0])**2

#    metric = 0*blob.area() + 300 * blob.density() - 0*100*d2


#    if metric < old_metric:
#        cb-=action

#    old_metric = metric
#    old_loc = loc
#    print(cb, metric,d2, blob.area(), blob.density(), clock.fps())



#        img.draw_rectangle(blob.rect())
#        img.draw_cross(blob.cx(), blob.cy())
#        # Note - the blob rotation is unique to 0-180 only.
#        img.draw_keypoints(
#            [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
#        )

#        nfc=0
#    else:
#        nfc+=1
##        print("not detected", nfc)
#        if nfc==10:
#            cb=random.choice(CB_ALTERNATIVES)
#            print("reset", cb)
#            nfc=0
#            old_metric=-1000
