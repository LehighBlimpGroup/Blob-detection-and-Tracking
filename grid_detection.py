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
red_threshold =[40, 80, -5, 15, None, None]  #[0, 100, -15, 15, 14, 78] (0, 100, 2, 18, -15, -6)
old_loc = (0,0)

# Color
cb = -10
MIN_CB = cb - 10
MAX_CB = cb + 10

nfc = 0 # not found counter
blackandwhite=True

CB_ALTERNATIVES = [MIN_CB, cb, MAX_CB]
ACTIONS=(-.5,-.3, .3,.5)
#ACTIONS=[0]


img = sensor.snapshot()
# Define the dimensions of the grid
GRID_ROWS = 3
GRID_COLS = 3

# Calculate the size of each cell
CELL_WIDTH = int(img.width() / GRID_COLS)
CELL_HEIGHT = int(img.height() / GRID_ROWS)


while True:
    clock.tick()
    img = sensor.snapshot()


    # Apply random action
    #action = random.choice(ACTIONS)
    u1 = random.uniform(0, 1)
    u2 = random.uniform(0, 1)
    action = 2*math.sqrt(-2.0 * math.log(u1)) * math.cos(2 * math.pi * u2)

    cb+=action
    cb = min(MAX_CB, max(MIN_CB, cb))


    # B interval
    red_threshold[4] = int(cb) - hi
    red_threshold[5] = int(cb) + hi



    if blackandwhite:
        img = img.binary([red_threshold])
        # Convert the binary image to grayscale
        img = img.to_grayscale()
        threshold = (0,255)
    else:
        img = img.binary([red_threshold])
        threshold = red_threshold



    ones = []

    # Loop through each cell of the grid
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            # Define the region of interest (ROI) for the current cell
            roi = (col * CELL_WIDTH, row * CELL_HEIGHT, CELL_WIDTH, CELL_HEIGHT)
            #print(roi)

            # Crop the ROI from the binary image
            #cropped_roi = binary_img.copy(roi=(roi[0], roi[1], roi[2], roi[3]))


            # Get the statistics of the cropped ROI
#            stats = cropped_roi.get_statistics()
            # Count the number of white pixels (ones) within the current cell
#            ones_in_cell = stats.mean()

            #hist = cropped_roi.get_histogram(bins=2)

            hist = img.get_histogram(bins=2, roi=roi)
            #print(hist.bins())
            # Count the number of white pixels (ones) within the current cell
            ones_in_cell = hist.bins()[1] *CELL_WIDTH*CELL_HEIGHT
            ones.append(ones_in_cell)
            #print(f"Number of ones (white pixels) in cell ({row},{col}):", ones_in_cell)


            # Draw the number of ones in the corner of the cell
            img.draw_string(roi[0], roi[1], str(int(ones_in_cell)), color=(255))  #


            # Count the number of white pixels (ones) within the current cell
            # ones_in_cell = hist.get_percentile(255)
            #print(f"Number of ones (white pixels) in cell ({row},{col}):", ones_in_cell)


            # Draw the ROI on the image
            img.draw_rectangle(roi, color=(int(ones_in_cell * 255 )), thickness=1)





    totalones = max(sum(ones),1)
    nones=[i/totalones for i in ones]

    # Compute the mean of the sample vector
    mean = sum(nones) / len(ones)
    # Compute the sum of squared differences from the mean
    squared_diff_sum = sum((x - mean) ** 2 for x in nones)
    # Compute the variance
    variance = squared_diff_sum / len(ones)


    metric = variance

    if metric < old_metric:
        cb-=action


    old_metric = metric
    #old_loc = loc


    if sum(ones)>50:
        nfc=0
        print(cb, metric,action, sum(ones),  int(clock.fps()))
    else:
        nfc+=1
        if nfc==10:
            cb=random.choice(CB_ALTERNATIVES)
            print("reset", cb)
            nfc=0
            old_metric=-1000



    # Control action
    left_cells = sum([nones[i*GRID_COLS] for i in range(GRID_ROWS)])
    right_cells = sum([nones[i*GRID_COLS+(GRID_COLS-1)] for i in range(GRID_ROWS)])
    up_cells = sum([nones[i] for i in range(GRID_COLS)])
    down_cells = sum([nones[i+6] for i in range(GRID_COLS)])
    #print(left_cells, right_cells, up_cells, down_cells)
    ux = int((right_cells - left_cells) * img.width()) // 3
    uy = -int((up_cells - down_cells) * img.height()) // 3


    x0 = img.width() // 2
    y0 = img.height() // 2
    img.draw_arrow(x0, y0, x0+ux , y0+uy, color=(0, 255, 0), size=30, thickness=2)


