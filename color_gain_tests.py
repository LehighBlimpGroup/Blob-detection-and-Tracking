# By: JiaweiXuAirlab - Wed Apr 3 2024

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
sensor.set_framesize(sensor.HQVGA)

sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
#sensor.set_auto_gain(True)
sensor.skip_frames(time = 2000)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)
sensor.skip_frames(time = 2000)

""" What are the auto values
"""
sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
high_exp = sensor.__read_reg(0x03)  # high bits of exposure control
low_exp = sensor.__read_reg(0x04)   # low bits of exposure control
print("high expo:\t\t", high_exp)
print("low expo:\t\t", low_exp)
print("global gain:\t\t", sensor.__read_reg(0xb0))   # global gain

# RGB gains
R_gain = sensor.__read_reg(0xb3)
G_gain = sensor.__read_reg(0xb4)
B_gain = sensor.__read_reg(0xb5)
pre_gain = sensor.__read_reg(0xb1)
pos_gain = sensor.__read_reg(0xb2)

print("R gain:\t\t", R_gain)    # R auto gain
print("G gain:\t\t", G_gain)    # G auto gain
print("B gain:\t\t", B_gain)    # B auto gain
print("pre-gain:\t\t", pre_gain)    # auto pre-gain, whatever that means
print("post-gain:\t\t", pos_gain)   # auto post-gain, whatever that means


sensor.__write_reg(0xfe, 0b00000010)    # change to registers at page 2
print("Global saturation:\t", sensor.__read_reg(0xd0))  # change global saturation,
                                                        # strangely constrained by auto saturation
print("Cb saturation:\t\t", sensor.__read_reg(0xd1))  # Cb saturation
print("Cr saturation:\t\t", sensor.__read_reg(0xd2))  # Cr saturation
print("luma contrast:\t\t", sensor.__read_reg(0xd3))  # luma contrast
print("luma offset:\t\t", sensor.__read_reg(0xd5))    # luma offset
print("showing off")
for i in range(50):
    img = sensor.snapshot()



""" Set the values - be aware, some of them are quite tricky
    as some registers do not give a **** to what we write
    and the related settings are actually controlled by some
    other registers
"""
print("setting up")
sensor.reset()
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)
sensor.set_pixformat(sensor.RGB565)
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
sensor.set_framesize(sensor.HQVGA)
sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
sensor.__write_reg(0xb0, 0b10000000) # global gain
sensor.__write_reg(0xad, (R_gain << 0)) # R gain ratio
sensor.__write_reg(0xae, (G_gain << 0)) # G gain ratio
sensor.__write_reg(0xaf, (B_gain << 0)) # B gain ratio
sensor.__write_reg(0x03, high_exp)  # high bits of exposure control
sensor.__write_reg(0x04, low_exp)   # low bits of exposure control


while(True):
    img = sensor.snapshot()
