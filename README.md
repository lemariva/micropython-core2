# MicroPython for M5Stack CORE2

More information coming soon.

```sh
sudo apt-get install build-essential libreadline-dev libffi-dev git pkg-config libsdl2-2.0-0 libsdl2-dev python3.8

git clone --recurse-submodules https://github.com/littlevgl/lv_micropython.git

# Follow the instructions to set up 
# toolchain and compiler: https://lemariva.com/blog/2020/03/tutorial-getting-started-micropython-v20

export BOARD=M5CORE2_BOARD

make -C mpy-cross
make -C ports/esp32 LV_CFLAGS="-DLV_COLOR_DEPTH=16 -DLV_COLOR_16_SWAP=1" PYTHON=python3 MICROPY_PY_BTREE=0

```