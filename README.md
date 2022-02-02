# MicroPython for M5Stack CORE2

This repository extends LV-MicroPython for using it on the M5Stack Core2. It adds support for the:

* MPU6886: a 6-axis MotionTracking device that combines a 3-axis gyroscope and a 3-axis accelerometer
* ILI9342C: a-Si TFT LCD Single Chip Driver. 
* BM8563: a CMOS1 Real-Time Clock (RTC) and calendar optimized for low power consumption. 
* AXP192: a enhanced single Cell Li-Battery and Power System Management IC

## Examples
More examples can be found inside the `examples` folder.

|           |           |
|:---------:|:---------:|
|  Drag Me Example         |   Buttons Example     |
|   ![dragme example](docs/dragme_example.gif)         |  ![buttons example](docs/buttons_example.gif)         |

```python
import gc
import lvgl as lv

from axpili9342 import ili9341
from ft6x36c import ft6x36

display = ili9341()
touch = ft6x36()

def drag_event_handler(e):
    obj = e.get_target()
    indev = lv.indev_get_act()
    vect = lv.point_t()
    indev.get_vect(vect)
    x = obj.get_x() + vect.x
    y = obj.get_y() + vect.y
    obj.set_pos(x, y)

obj = lv.obj(lv.scr_act())
obj.set_size(150, 100)
obj.add_event_cb(drag_event_handler, lv.EVENT.PRESSING, None)

label = lv.label(obj)
label.set_text("Drag me")
label.center()

```

## Firmware
I've included a compiled MicroPython firmware (check the `firmware` folder). The firmware was compiled using following versions and hashes:

* esp-idf v4.4.x - [`b64925c56`](https://github.com/espressif/esp-idf/commit/b64925c5673206100eaf4337d064d0fe3507eaec)
* MicroPython v1.18-599-bf62dfc784-dirty - [`bf62dfc784`](https://github.com/lvgl/lv_micropython/commit/bf62dfc78497d47ced3b0931a270e553d4d2552b)


To flash it to the board, you need to type the following:
```sh
esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 1152000 write_flash -z 0x1000 lv_micropython_m5core2_bf62dfc784_esp32_idf4_4_x.bin
```
More information is available in this [tutorial](https://lemariva.com/blog/2022/01/micropython-upgraded-support-cameras-m5camera-esp32-cam-etc).

If you want to compile your driver from scratch follow the next section:

## DIY

Read this section if you want to include the camera support to MicroPython from scratch. To do that follow these steps:
  
- Note 1: if you get stuck, those pages are a good help:
  - https://github.com/micropython/micropython/tree/master/ports/esp32
  - https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#installation-step-by-step

- Note 2: Up to micropython version 1.14, the build tool for the esp32 port is Make. Starting with this [PR](https://github.com/micropython/micropython/pull/6892), it is CMAKE. You can find more discussion in this [micropython forum blog post](https://forum.micropython.org/viewtopic.php?f=18&t=9820)


1. Clone the MicroPython repository:
    ```
    git clone --recursive https://github.com/littlevgl/lv_micropython.git
    ```
    Note: The MicroPython repo changes a lot, I've done this using the version with the hash mentioned above.

    :warning: If you want to directly replace the original files with the provided in this repository, be sure that you've taken the same commit hash. MicroPython changes a lot, and you'll compiling issues if you ignore this warning.

2. Copy the files and folders inside the `boards` folder into `micropython/ports/esp32/boards` or use a symbolic link (`ln -s`).
3. Create a `m5core2` folder inside `~/esp/esp-idf/components` and copy the files and folders inside the `components` inside that folder.
4. Compile the firmware by typing following commands:
    ```
    cd micropython/ports/esp32
    . $HOME/esp/esp-idf/export.sh
    make USER_C_MODULES=../../../../micropython-core2/src/micropython.cmake BOARD=M5STACK_CORE2 all
    ```
    Note that the folder `micropython-core2` should be in the same folder level as the `micropython`. Otherwise, you'll need to change the path (`../../../../micropython-core2/src/`) to the `micropython.cmake` file.
5. Deploy the firmware into the ESP32 by typing:
    ```
    cd micropython/ports/esp32
    esptool.py --port /dev/ttyUSB0 erase_flash
    esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 1152000 write_flash -z 0x1000 build-M5STACK_CORE2/firmware.bin
    ```