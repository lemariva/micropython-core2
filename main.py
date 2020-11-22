"""
Copyright 2020 LeMaRiva|Tech (Mauro Riva) info@lemariva.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""
import gc
from machine import Pin, SoftI2C, SPI, I2S, mpu6886, axp192
import network
from neopixel import NeoPixel
from config import *

# copter = mpu6886(scl=Pin(mpu6886_config['scl']), 
#                sda=Pin(mpu6886_config['sda']))

axp = axp192(scl=Pin(mpu6886_config['scl']), 
                sda=Pin(mpu6886_config['sda']))

# i2c = SoftI2C(scl=Pin(mpu6886_config['scl']), 
#                 sda=Pin(mpu6886_config['sda']))

# address = 0x68
# register = 0x75
# buf = bytearray(2)
# i2c.readfrom_mem_into(address, register, buf)

# audio_out = I2S(I2S.NUM0, bck=Pin(device_config['bck']), 
#             ws=Pin(device_config['ws']), 
#             sdout=Pin(device_config['sdout']), 
#             standard=I2S.PHILIPS, mode=I2S.MASTER_TX,
#             dataformat=I2S.B16, channelformat=I2S.ONLY_RIGHT,
#             samplerate=SAMPLES_PER_SECOND, apllrate=0,
#             dmacount=6, dmalen=60)

# def warning_sound():
#     track = open('warning.wav','rb')
#     stop = False
#     track.seek(0)
#     while not stop:
#         audio_samples = bytearray(track.read(1024))
#         numwritten = 0
#         if len(audio_samples) == 0:
#             print("STOP")
#             stop = True
#         else:
#             # loop until samples can be written to DMA
#             while numwritten == 0:
#                 # return immediately when no DMA buffer is available (timeout=0)
#                 numwritten = audio_out.write(audio_samples, timeout=0)
#                 # await - allow other coros to run
#     print("done")
#     gc.collect()
