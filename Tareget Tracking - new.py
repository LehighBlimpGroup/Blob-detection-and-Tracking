from lib.Ibus import IBus
import time
import sensor
from lib.tracker import BLOBTracker, GoalTracker
from pyb import UART
from machine import I2C
from vl53l1x import VL53L1X
#from lib.Ibus import IBus


def blob_tracking(thresholds, clock, blob_type=1, isColored=True):
    """ The blob tracker initialization for balloons
        blob_type=1 for balloons
        blob_type=2 for goals
    """
    if blob_type == 1:
        blob_tracker = BLOBTracker(thresholds, clock)
    elif blob_type == 2:
        blob_tracker = GoalTracker(thresholds, clock, sensor_sleep_time=WAIT_TIME_US)
    else:
        raise ValueError("Invalid blob type!")
    return blob_tracker


def init_sensor_target(tracking_type:int=0, isColored:bool=True,
                       framesize=sensor.HQVGA, windowsize=None) -> None:
    """ Initialize sensors by updating the registers
        for the two different purposes
        @param       {int} tracking_type: 0 for balloons and 1 for goals
        @param       {bool} isColored: whether we would like to initialize the sensor
                                       in colored mode
        @return      {*} None
    """
    # We do these whatever mode we are in
    sensor.reset()
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False)
    sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
    sensor.__write_reg(0x80, 0b10111100) # enable gamma, CC, edge enhancer, interpolation, de-noise
    sensor.__write_reg(0x81, 0b01101100) # enable BLK dither mode, low light Y stretch, autogray enable
    sensor.__write_reg(0x82, 0b00000100) # enable anti blur, disable AWB
    sensor.set_framesize(framesize)
    if windowsize is not None:
        sensor.set_windowing(windowsize)

    # change sensor color type and exposure based on the tracking type
    if isColored and tracking_type == 0:
        # For balloon tracking
        sensor.set_pixformat(sensor.RGB565)
        sensor.__write_reg(0x03, 0b00000010) # high bits of exposure control
        sensor.__write_reg(0x04, 0b11100000) # low bits of exposure control
        sensor.__write_reg(0xb0, 0b11000000) # global gain
        # RGB gains
        sensor.__write_reg(0xa3, 0b01111100) # G gain odd
        sensor.__write_reg(0xa4, 0b01111100) # G gain even
        sensor.__write_reg(0xa5, 0b01111100) # R gain odd
        sensor.__write_reg(0xa6, 0b01111100) # R gain even
        sensor.__write_reg(0xa7, 0b11100000) # B gain odd
        sensor.__write_reg(0xa8, 0b11100000) # B gain even
        sensor.__write_reg(0xa9, 0b10000000) # G gain odd 2
        sensor.__write_reg(0xaa, 0b10000000) # G gain even 2
        sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
        # sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
                                               # strangely constrained by auto saturation
        sensor.__write_reg(0xd1, 0b01000000) # change Cb saturation
        sensor.__write_reg(0xd2, 0b01000000) # change Cr saturation
        sensor.__write_reg(0xd3, 0b01001000) # luma contrast
        # sensor.__write_reg(0xd5, 0b00000000) # luma offset
    elif isColored and tracking_type == 1:
        # For target tracking, colored
        sensor.set_pixformat(sensor.RGB565)
        sensor.__write_reg(0x03, 0b00000001) # high bits of exposure control
        sensor.__write_reg(0x04, 0b11000000) # low bits of exposure control
        sensor.__write_reg(0xb0, 0b11100000) # global gain
        # RGB gains
        sensor.__write_reg(0xa3, 0b01010000) # G gain odd
        sensor.__write_reg(0xa4, 0b01010000) # G gain even
        sensor.__write_reg(0xa5, 0b01101000) # R gain odd
        sensor.__write_reg(0xa6, 0b01101000) # R gain even
        sensor.__write_reg(0xa7, 0b01110000) # B gain odd
        sensor.__write_reg(0xa8, 0b01110000) # B gain even
        sensor.__write_reg(0xa9, 0b01101000) # G gain odd 2
        sensor.__write_reg(0xaa, 0b01101000) # G gain even 2
        sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
        # sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
                                               # strangely constrained by auto saturation
        sensor.__write_reg(0xd1, 0b01000000) # change Cb saturation
        sensor.__write_reg(0xd2, 0b01000000) # change Cr saturation
        sensor.__write_reg(0xd3, 0b01000000) # luma contrast
        # sensor.__write_reg(0xd5, 0b00000000) # luma offset
    elif tracking_type == 1:
        # For target tracking, BnW
        sensor.set_pixformat(sensor.GRAYSCALE)
        sensor.__write_reg(0x03, 0b00000000) # high bits of exposure control
        sensor.__write_reg(0x04, 0b10000000) # low bits of exposure control
        sensor.__write_reg(0xb0, 0b01000000) # global gain
        # RGB gains
        sensor.__write_reg(0xa3, 0b10000000) # G gain odd
        sensor.__write_reg(0xa4, 0b10000000) # G gain even
        sensor.__write_reg(0xa5, 0b01000000) # R gain odd
        sensor.__write_reg(0xa6, 0b01000000) # R gain even
        sensor.__write_reg(0xa7, 0b01000000) # B gain odd
        sensor.__write_reg(0xa8, 0b01000000) # B gain even
        sensor.__write_reg(0xa9, 0b10000000) # G gain odd 2
        sensor.__write_reg(0xaa, 0b10000000) # G gain even 2
        sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
        # sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
                                               # strangely constrained by auto saturation
        sensor.__write_reg(0xd1, 0b01000000) # change Cb saturation
        sensor.__write_reg(0xd2, 0b01000000) # change Cr saturation
        sensor.__write_reg(0xd3, 0b11100000) # luma contrast
        # sensor.__write_reg(0xd5, 0b00000000) # luma offset
    else:
        raise ValueError("Not a valid sensor-detection mode!")

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


def mode_initialization(input_mode, mode, isColored):
    """ Switching between blinking goal tracker and balloon tracker
    """
    if mode == input_mode:
        print("already in the mode")
        return None
    else:
        if input_mode == 0:
            # balloon tracking mode
            init_sensor_target(tracking_type=0, isColored=isColored)
            thresholds = GREEN + PURPLE
            tracker = blob_tracking(thresholds, clock, blob_type=1)
        elif input_mode == 1 and not isColored:
            init_sensor_target(tracking_type=1, isColored=isColored)
            # Find reference
            thresholds = GRAY
            tracker = blob_tracking(thresholds, clock, blob_type=2)
        elif input_mode == 1 and isColored:
            init_sensor_target(tracking_type=1, isColored=isColored)
            # Find reference
            thresholds = TARGET_COLOR
            tracker = blob_tracking(thresholds, clock, blob_type=2)

        return input_mode, tracker


if __name__ == "__main__":
    ### Macros
    GREEN = [(19, 43, -27, -11, -11, 13)]
    PURPLE = [(14, 42, 4, 21, -40, -20)]
    GRAY = [(0, 35)]
    TARGET_COLOR = [(36, 57, 0, 59, 2, 36), (35, 61, -23, 12, 8, 56)]
    THRESHOLD_UPDATE_RATE = 0.0
    WAIT_TIME_US = 50000
    ### End Macros

    clock = time.clock()
    ISCOLORED = True
    mode = 0

    # Initialize inter-board communication
    # ibus = IBus()
    # Sensor initialization
    tof = VL53L1X(I2C(2))

    mode, tracker = mode_initialization(mode, -1, ISCOLORED)
    # Initialize UART
    uart = UART("LP1", 115200, timeout_char=2000) # (TX, RX) = (P1, P0) = (PB14, PB15)

    while True:
        tracker.track()
        try: dis = tof.read()
        except: dis = 9999
        if tracker.tracked_blob.feature_vector:
            roi = tracker.roi.get_roi()
            feature_vec = tracker.tracked_blob.feature_vector
            x_roi = roi[0] + roi[2]//2
            y_roi = roi[1] + roi[3]//2
            w_roi = roi[2]
            h_roi = roi[3]

            x_value = int(feature_vec[0] + feature_vec[2]/2)
            y_value = int(feature_vec[1] + feature_vec[3]/2)
            w_value = int(feature_vec[2])
            h_value = int(feature_vec[3])
            msg = IBus_message([mode, x_roi, y_roi, w_roi, h_roi,
                                x_value, y_value, w_value, h_value, dis])
        else:
            msg = IBus_message([3, 0, 0, 0, 0,
                                0, 0, 0, 0, dis])

        uart.write(msg)
        if uart.any():
            uart_input = uart.read()
            if uart_input == 0x80:
                res = mode_initialization(0, mode, ISCOLORED)
                if res:
                    mode, tracker = res
            elif uart_input == 0x81:
                res = mode_initialization(1, mode, ISCOLORED)
                if res:
                    mode, tracker = res