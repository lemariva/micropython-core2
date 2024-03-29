import uos
from flashbdev import bdev
import lvgl as lv


def check_bootsec():
    buf = bytearray(bdev.ioctl(5, 0))  # 5 is SEC_SIZE
    bdev.readblocks(0, buf)
    empty = True
    for b in buf:
        if b != 0xFF:
            empty = False
            break
    if empty:
        return True
    fs_corrupted()


def fs_corrupted():
    import time

    while 1:
        print(
            """\
The filesystem appears to be corrupted. If you had important data there, you
may want to make a flash snapshot to try to recover it. Otherwise, perform
factory reprogramming of MicroPython firmware (completely erase flash, followed
by firmware programming).
"""
        )
        time.sleep(3)


def setup():
    check_bootsec()
    print("Performing initial setup")
    uos.VfsLfs2.mkfs(bdev)
    vfs = uos.VfsLfs2(bdev)
    uos.mount(vfs, "/")
    with open("boot.py", "w") as f:
        f.write(
            """\
# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()

# Init LVGL
import lvgl as lv
lv.init()

# Power Management
from m5stack import M5Stack
m5core2 = M5Stack()

# LCD screen
from axpili9342 import ili9341
display = ili9341(m5stack=m5core2)

# Touch sensor
from ft6x36 import ft6x36
touch = ft6x36(width=320, height=240)  
"""
        )

    with open("board.py", "w") as f:
        f.write(
            """\
name = "m5core2"
"""
        )

    with open("main.py", "w") as f:
        f.write(
            """\
import lvgl as lv

scr = lv.obj()
btn = lv.btn(scr)
btn.align(lv.ALIGN.CENTER, 0, 0)
label = lv.label(btn)
label.set_text("Button")
lv.scr_load(scr)
"""
        )

    return vfs
