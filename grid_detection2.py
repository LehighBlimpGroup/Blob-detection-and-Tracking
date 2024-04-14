# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Color Binary Filter Example
#
# This script shows off the binary image filter. You may pass binary any
# number of thresholds to segment the image by.

import array
import sensor, image
import time
import math
import random
#from lib.Ibus import IBus
from pyb import UART
from pyb import LED

#sensor.reset()
#sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
#sensor.set_framesize(sensor.HQVGA)
#sensor.set_pixformat(sensor.RGB565)
#sensor.skip_frames(time=2000)

#def init_sensor_target(tracking_type:int=0, isColored:bool=True,
#                       framesize=sensor.HQVGA, windowsize=None) -> None:

# We do these whatever mode we are in
sensor.reset()
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
sensor.set_pixformat(sensor.RGB565)
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
sensor.set_framesize(sensor.HQVGA)

sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)

""" sensor setup
"""

sensor.__write_reg(0xfe, 0) # change to registers at page 0
sensor.__write_reg(0x80, 0b01111110)    # [7] reserved, [6] gamma enable, [5] CC enable,
                                        # [4] Edge enhancement enable
                                        # [3] Interpolation enable, [2] DN enable, [1] DD enable,
                                        # [0] Lens-shading correction enable - gives you uneven
                                        #                                      shade in the dark
                                        #                                      badly!!!!!
sensor.__write_reg(0x81, 0b01010100)    # [7] BLK dither mode, [6] low light Y stretch enable
                                        # [5] skin detection enable, [4] reserved, [3] new skin mode
                                        # [2] autogray enable, [1] reserved, [0] BFF test image mode
sensor.__write_reg(0x82, 0b00000100)    # [2] ABS enable, [1] AWB enable
#sensor.__write_reg(0x87, 0b00000001)    # [0] auto_edge_effect
sensor.__write_reg(0x9a, 0b00001111)    # [3] smooth Y, [2] smooth Chroma,
                                        # [1] neighbor average mode, [0] subsample extend opclk
sensor.skip_frames(time = 1000)
print("block enabling done")

# Edge enhancements
sensor.__write_reg(0xfe, 2)             # change to registers at page 2
sensor.__write_reg(0x90, 0b11101101)    # [7]edge1_mode, [6]HP3_mode, [5]edge2_mode, [4]Reserved,
                                        # [3]LP_intp_en, [2]LP_edge_en, [1]NA, [0] half_scale_mode_en
sensor.__write_reg(0x91, 0b11000000)    # [7]HP_mode1, [6]HP_mode2,
                                        # [5]only 2 direction - only two direction H and V, [4]NA
                                        # [3]only_defect_map, [2]map_dir, [1:0]reserved
sensor.__write_reg(0x96, 0b00001100)    # [3:2] edge leve
sensor.__write_reg(0x97, 0x88)          # [7:4] edge1 effect, [3:0] edge2 effect
sensor.__write_reg(0x9b, 0b00100010)    # [7:4] edge1 threshold, [3:0] edge2 threshold
sensor.skip_frames(time = 1000)
print("edge enhancement done")

# color correction -- this is very tricky: the color shifts on the color wheel it seems
sensor.__write_reg(0xfe, 2) # change to registers at page 2
# WARNING: uncomment the two lines to invert the color
#sensor.__write_reg(0xc1, 0x80)          # CC_CT1_11, feels like elements in a matrix
#sensor.__write_reg(0xc5, 0x80)          # CC_CT1_22 , feels like elements in a matrix
print("color correction setup done")

# ABS - anti-blur
sensor.__write_reg(0xfe, 1)             # change to registers at page 1
sensor.__write_reg(0x9a, 0b11110111)    # [7:4] add dynamic range, [2:0] abs adjust every frame
sensor.__write_reg(0x9d, 0xff)          # [7:0] Y stretch limit
sensor.skip_frames(time = 1000)
print("anti-blur setup done")


# color settings -- AWB
""" Ranting about the trickiness of the setup:
    Even the auto white balance is disabled, the AWB gains will persist to
    take effect. Although the correcponding registers are read-only in the
    document, they are actually manually writeable, and such writings are
    effective. On top of these messes, another set of registers that are
    R/W have the exact same effect on the RGB gains, but they are not
    controlled by the AWB.
"""
sensor.set_auto_exposure(False)
sensor.set_auto_whitebal(False) # no, the gain_rgb_db does not work

# reset RGB auto gains
sensor.__write_reg(0xb3, 64)    # reset R auto gain
sensor.__write_reg(0xb4, 64)    # reset G auto gain
sensor.__write_reg(0xb5, 64)    # reset B auto gain

sensor.__write_reg(0xfe, 0)     # change to registers at page 0
                                # manually set RGB gains to fix color/white balance
sensor.__write_reg(0xad, 64)    # R gain ratio
sensor.__write_reg(0xae, 62)    # G gain ratio
sensor.__write_reg(0xaf, 75)    # B gain ratio
sensor.set_auto_exposure(True)
sensor.skip_frames(time = 1000)
print("AWB Gain setup done.")


# color setup - saturation
sensor.__write_reg(0xfe, 2)     # change to registers at page 2
sensor.__write_reg(0xd0, 128)    # change global saturation,
sensor.__write_reg(0xd1, 48)    # Cb saturation
sensor.__write_reg(0xd2, 48)    # Cr saturation
sensor.__write_reg(0xd3, 40)    # contrast
sensor.__write_reg(0xd5, 0)     # luma offset





class LogOddFilter:
    def __init__(self, n, p_det=0.5, p_ndet=0.005, p_x=0.1):
        """

        p_x: Probability of having a balloon in a cell

        """

        self.init_belif = math.log(p_x/(1-p_x))
        self.l_det = math.log(p_det/(1-p_det))
        self.l_ndet = math.log(p_ndet / (1 - p_ndet))
        self.L = [0. for _ in range(n)]
        self.P = [0. for _ in range(n)]


    def update(self, measurements):
        for i, z in enumerate(measurements):
            li = self.l_det if z else self.l_ndet
            self.L[i]+= li -self.init_belif
        return self.L


    def probabilities(self):
        for i, l in enumerate(self.L):
            if l > 10:
                self.P[i] = 1.0
            elif l < -10:
                self.P[i] = 0.0
            else:
                self.P[i] = 1. / (1.+math.exp(-l))
        return self.P





def distance_point_to_segment(point, segment):
    x1, y1 = segment[0]
    x2, y2 = segment[1]
    x0, y0 = point

    dx = x2 - x1
    dy = y2 - y1

    # Compute the dot product of the vector from the first endpoint of the segment to the point
    # with the vector of the segment
    dot_product = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy)

    # Clamp the dot product to ensure the closest point lies within the line segment
    t = max(0, min(1, dot_product))

    # Calculate the coordinates of the closest point on the line segment
    closest_point = (x1 + t * dx, y1 + t * dy)

    # Calculate the distance between the given point and the closest point
    distance = math.sqrt((x0 - closest_point[0]) ** 2 + (y0 - closest_point[1]) ** 2)

    return distance


def checksum(arr, initial= 0):
    """ The last pair of byte is the checksum on iBus
    """
    sum = initial
    for a in arr:
        sum += a
    checksum = 0xFFFF - sum
    chA = checksum >> 8
    chB = checksum & 0xFF
    return chA, chB

def IBus_message(message_arr_to_send):
    msg = bytearray(32)
    msg[0] = 0x20
    msg[1] = 0x40
    for i in range(len(message_arr_to_send)):
        msg_byte_tuple = bytearray(message_arr_to_send[i].to_bytes(2, 'little'))
        msg[int(2*i + 2)] = msg_byte_tuple[0]
        msg[int(2*i + 3)] = msg_byte_tuple[1]

    # Perform the checksume
    chA, chB = checksum(msg[:-2], 0)
    msg[-1] = chA
    msg[-2] = chB
    return msg


def _normal_dist(mu=0, sigma=1):
    u1 = random.uniform(0, 1)
    u2 = random.uniform(0, 1)
    return mu + sigma*math.sqrt(-2.0 * math.log(u1)) * math.cos(2 * math.pi * u2)

class GridDetector:
    def __init__(self, num_rows, num_cols, img_width, img_height):
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.cell_width = int(img_width / num_cols)
        self.cell_height = int(img_height / num_rows)

    def count(self, img):
        metrics = []
        # Loop through each cell of the grid
        for row in range(self.num_rows):
            for col in range(self.num_cols):
                # Define the region of interest (ROI) for the current cell
                roi = (col * self.cell_width, row * self.cell_height, self.cell_width, self.cell_height)
                img_roi = img.copy(roi=roi)

#                hist = img_roi.get_histogram(bins=20)
#                a_bins = hist.a_bins()
#                b_bins = hist.b_bins()

#                mean1 = sum(a_bins)
                #print(mean1))

                #print(hist.get_statistics())
                # Copy the ROI from the original image
                stats = img_roi.get_statistics()
##                # Calculate the mean and variance of the ROI
                l_mean = stats.l_mean()
                a_mean = stats.a_mean()
                b_mean = stats.b_mean()
                a_std = stats.a_stdev()
                b_std = stats.b_stdev()


                ### metric ####
                # Distance to the line segment
#                mean_ref = (23,-21)
#                std_ref=(4, 5)
#                d_mean = math.sqrt((mean_ref[0]-a_mean)**2 + (mean_ref[1]-b_mean)**2)
#                d_std = math.sqrt((std_ref[0]-a_std)**2 + (std_ref[1]-b_std)**2)

                point = (a_mean, b_mean)
                mean_line_ref = ((8, -8), (25, -25))  # represetend as a line
                d_mean = distance_point_to_segment(point, mean_line_ref)


                MAX_DIST = 8
                STD_RANGE_A =(3,25)
                STD_RANGE_B =(3,25)

                # Clamp max distance
                d_mean = d_mean if d_mean<MAX_DIST else MAX_DIST

                # Compute metric
                metric = 1 - d_mean/MAX_DIST
#                metric *= metric  # square of the distance

                # Data is too spread
                if not STD_RANGE_A[0]<a_std<STD_RANGE_A[1] or not STD_RANGE_B[0]<b_std<STD_RANGE_B[1]:
                    metric = 0

                # Too much lightening
                if not 10<l_mean<60:
                    metric = 0.0

                metrics.append(metric)
                #print(metric, a_mean, b_mean, a_std, b_std)
                #variance = img.stdev(roi=roi)


                #print(mean) #"a_bins"
                # Count the number of white pixels (ones) within the current cell
                #ones_in_cell = hist.bins()[1] * self.cell_width * self.cell_height
                #ones.append(ones_in_cell)
                # print(f"Number of ones (white pixels) in cell ({row},{col}):", ones_in_cell)

                # Draw the number of ones in the corner of the cell
                img.draw_string(roi[0], roi[1],str(int(metric*10)) , color=(0,255,0))  #

                # Draw the ROI on the image
                img.draw_rectangle(roi, color=(int(metric*2*255)), thickness=1)
        return metrics

    def normalize(self, ones):
        totalones = max(sum(ones), 1)
        nones = [i / totalones for i in ones]
        return nones

    def statistics(self, nones):
        # Compute the mean of the sample vector
        mean = sum(nones) / len(nones)
        # Compute the sum of squared differences from the mean
        squared_diff_sum = sum((x - mean) ** 2 for x in nones)
        # Compute the variance
        variance = squared_diff_sum / len(nones)

        return mean,variance


    def count_column(self, nones, col):
        col_sum = sum([nones[col + i * self.num_cols] for i in range(self.num_rows)])
        return col_sum

    def count_row(self, nones, row):
        row_sum = sum([nones[i + self.num_cols * row] for i in range(self.num_cols)])
        return row_sum

    def action(self, metric):
        totalm= sum(metric) if sum(metric)>0 else 1
        metric = [m / totalm for m in metric]

        # Counting
        left_cells = sum([self.count_column(metric, i) for i in range(N_COLS//2) ])
        right_cells = sum([self.count_column(metric, N_COLS-i-1) for i in range(N_COLS//2) ])
        up_cells =  sum([self.count_row(metric, i) for i in range(N_ROWS//2) ])
        down_cells = sum([self.count_row(metric, N_ROWS-i-1) for i in range(N_ROWS//2) ])


#        maxim = 0
#        max_id = -1
#        # Find the index of the maximum value
#        for i in range(self.num_rows):
#            for j in range(self.num_cols):
#                m = metric[i+j*self.num_cols]
#                if m>maxim:
#                    m=maxim


        ux, uy = 0,0

        # Control action
        # print(left_cells, right_cells, up_cells, down_cells)
        ux = int((right_cells - left_cells) * img.width()) // N_COLS
        uy = -int((up_cells - down_cells) * img.height()) // N_ROWS

        return ux, uy


    def weighted_average(self, metrics):
        cell_row = self.num_rows/2.0 - 0.5
        cell_col = self.num_cols/2.0 - 0.5

        sum_score = sum(metrics)
        if sum_score > 0:
            led_blue.on()
            led_red.on()
            cell_row = 0.0
            cell_col = 0.0
            for row in range(self.num_rows):
                for col in range(self.num_cols):
                    index = row*self.num_cols + col
                    cell_row += (metrics[index] * row)
                    cell_col += (metrics[index] * col)

            cell_row /= sum_score
            cell_col /= sum_score
        else:
            led_blue.off()
            led_red.off()
        return cell_row + 0.5, cell_col + 0.5, sum_score


print("Start")

N_ROWS = 8
N_COLS = 12
FILTER = False

if __name__ == "__main__":
    # uart settings
    uart = UART("LP1", 115200, timeout_char=2000) # (TX, RX) = (P1, P0) = (PB14, PB15)

    # LED indicator
    led_red = LED(1)
    led_green = LED(2)
    led_blue = LED(3)

    clock = time.clock()
    img = sensor.snapshot()
    detector = GridDetector(N_ROWS, N_COLS, img.width(), img.height())
    filter = LogOddFilter(N_ROWS * N_COLS)

    flag = 0
    while True:
        clock.tick()

        img = sensor.snapshot()
        # Load the image from flash memory
#        img_from_flash = image.Image("bw.jpg")  # Load the image using its filename
        #mutable_img = img_from_flash.copy()

#        sensor.snapshot().copy(mutable_img)
#        mutable_img.get_histogram()

        metric_grid = detector.count(img)


        if FILTER:
            filter.update(metric_grid)
            metric_grid = filter.probabilities()
#        nones = detector.normalize(ones)
#        mean, variance = detector.statistics(nones)


#        metric = variance

#        if metric < old_metric:
#            cb -= action

#        old_metric = metric


#        if sum(ones) > 50:
#            nfc = 0
#            print(cb, metric, action, sum(ones), int(clock.fps()))
#        else:
#            nfc += 1
#            if nfc == 10:
#                cb = random.choice(CB_ALTERNATIVES)
#                print("reset", cb)
#                nfc = 0
#                old_metric = -1000

        total_score = sum(metric_grid)
        if total_score > 0:
            led_red.on()
            led_blue.on()
            if flag:
                flag = 3 - flag
            else:
                flag = 1
        else:
            led_red.off()
            led_blue.off()
            flag = 0

        ux, uy = detector.action(metric_grid)
        # average_cell_y, average_cell_x, total_score = detector.weighted_average(metric_grid)
        # x1 = int((average_cell_x)*(img.width()//N_COLS))
        # y1 = int((average_cell_y)*(img.height()//N_ROWS))

        x0 = img.width() // 2
        y0 = img.height() // 2
        x1 = x0 + ux
        y1 = y0 + uy
        img.draw_circle(x1, y1, int(total_score*5), color=(255, 0, 0), thickness=2)

        print("fps:\t", clock.fps())

        # Create message for ESP32
        x_roi,y_roi = x1, y1
        w_roi, h_roi = 10, 10
        x_value,y_value = x1, y1

        print("flag, x, y, score:\t", bin(flag), x_roi, y_roi, total_score)
        msg = IBus_message([flag | 0x40, x_roi, y_roi, w_roi, h_roi,
                            x_value, y_value, img.width()//N_COLS, img.height()//N_ROWS,
                            int(total_score*5)])


        # Receive message
        if uart.any():
            uart_input = uart.read()
            print(uart_input)

        # Send message
        uart.write(msg)
