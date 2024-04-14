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



R_GAIN, G_GAIN, B_GAIN = 64, 69, 91


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
sensor.__write_reg(0xad, R_GAIN)    # R gain ratio
sensor.__write_reg(0xae, G_GAIN)    # G gain ratio
sensor.__write_reg(0xaf, B_GAIN)    # B gain ratio
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
    def __init__(self, n, p_det=0.95, p_ndet=0.1, p_x=0.5):
        """

        p_x: Probability of having a balloon in a cell

        """

        self.init_belif = math.log(p_x/(1-p_x))
        self.l_det = math.log(p_det/(1-p_det))
        self.l_ndet = math.log(p_ndet / (1 - p_ndet))
        self.L = [0. for _ in range(n)]
        self.P = [0. for _ in range(n)]
        print("Initial belief=", self.init_belif, "L for detection=", self.l_det, "L for not detection", self.l_ndet )


    def update(self, measurements, l_max=10, l_min=-10):
        for i, z in enumerate(measurements):
            # Detected or not detected
            li = self.l_det if z else self.l_ndet
            # Belief for li
            self.L[i]+= li -self.init_belif
            # Cap
            self.L[i] = min(l_max, max(l_min, self.L[i]))


        return self.L


    def probabilities(self):
        for i, l in enumerate(self.L):
                self.P[i] = 1. / (1. + math.exp(-l))
        return self.P








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


class ColorDistance:
    def __init__(self, color_id, line_ref, max_dist, std_range):
        self.color_id = color_id
        self.line_ref = line_ref
        self.max_dist = max_dist
        self.std_range = std_range


    def distance(self, point, std):
        (l, a, b) = point
        d =  self.distance_point_to_segment((a,b), self.line_ref)

        # Clamp max distance
        d = d if d<self.max_dist else self.max_dist

        # Distnace is 1 if close to the line, and 0 if outside the line
        d = 1 - d / self.max_dist
#                metric *= metric  # square of the distance

        # Data is too spread
#                if not STD_RANGE_A[0]<a_std<STD_RANGE_A[1] or not STD_RANGE_B[0]<b_std<STD_RANGE_B[1]:
#                    metric = 0

        # Check if the standar deviation is not in range
#        for s in std:
#            if not self.std_range[0] <= s <= self.std_range[1]:
#                d = 0
#                break

#        # Too much lightening
        if not 10 < l < 60:  #fixme magic numbers
            d = 0.0

        return d



    def distance_point_to_segment(self, point, segment):
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

    def count(self, img, distancer):
        metrics = []
        # Loop through each cell of the grid
        for row in range(self.num_rows):
            for col in range(self.num_cols):
                # Define the region of interest (ROI) for the current cell
                roi = (col * self.cell_width, row * self.cell_height, self.cell_width, self.cell_height)
                img_roi = img.copy(roi=roi)

                # Calculate the mean and variance of the ROI
                s = img_roi.get_statistics()

                # metric is the distance
                d_mean = distancer.distance((s.l_mean(), s.a_mean(), s.b_mean()),
                                            (s.l_stdev(), s.a_stdev(), s.b_stdev()))

                if row==0:
                    print ((s.a_mean(), s.b_mean(), d_mean), end=', ')

                metrics.append(d_mean)

        return metrics

    def plot_metric(self, metrics):

        for row in range(self.num_rows):
            for col in range(self.num_cols):
                roi = (col * self.cell_width, row * self.cell_height, self.cell_width, self.cell_height)

                metric = metrics[row*self.num_cols + col]

#                if row!=5:
#                    continue

                # Draw the number of ones in the corner of the cell
#                img.draw_string(roi[0], roi[1],str(int(metric*10)) , color=(0,255,0))  #

                # Draw the ROI on the image
                img.draw_rectangle(roi, color=(int(metric*255),int(metric*255),int(metric*255)), thickness=1)

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
FILTER = True




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

    # Color distance
    cdist = ColorDistance("Purple", line_ref = ((4, -8), (18, -33)), max_dist=4, std_range=(5,30))

    while True:
        clock.tick()

        img = sensor.snapshot()

        # Detect
        metric_grid = detector.count(img, cdist)


        if FILTER:
            filter.update(metric_grid)
            metric_grid = filter.probabilities()

        detector.plot_metric(metric_grid)
        total_score = max(metric_grid)
        ux, uy = detector.action(metric_grid)

        x0 = img.width() // 2
        y0 = img.height() // 2
        x1 = x0 + ux
        y1 = y0 + uy
#        img.draw_circle(x1, y1, 1, color=(255, 0, 0), thickness=2)
#        img.draw_circle(x1, y1, int(total_score*img.height()/4), color=(255, 0, 0), thickness=2)

        print("fps:\t", clock.fps(), end='\t')

        # Create message for ESP32
        flag = 0b10000000
        x_roi,y_roi = x1, y1
        w_roi, h_roi = 10, 10
        x_value,y_value = x1, y1

        print("x, y =", x_roi, y_roi,"\t metric=", metric_grid[5*N_ROWS:6*N_ROWS])
        msg = IBus_message([flag, x_roi, y_roi, w_roi, h_roi,
                            x_value, y_value, img.width()//N_COLS, img.height()//N_ROWS,
                            int(total_score*10)])


        # Receive message
        if uart.any():
            uart_input = uart.read()
            print(uart_input)

        # Send message
        uart.write(msg)
