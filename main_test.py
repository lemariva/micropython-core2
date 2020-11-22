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
import lvgl as lv
import lvesp32

from machine import Pin, SPI, I2S, SPI, mpu6886, axp192, bm8563, ft6336u
from axpili9342 import ili9342
from m5stack import M5Stack

m5core = M5Stack()

display = ili9342(m5stack=m5core)
mpu = mpu6886()
clock = bm8563()
touch = ft6336u()

scr = lv.obj()
btn = lv.btn(scr)
btn.align(lv.scr_act(), lv.ALIGN.CENTER, 0, 0)
label = lv.label(btn)
label.set_text("Button")

# Load the screen
lv.scr_load(scr)

