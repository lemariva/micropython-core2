SDKCONFIG += boards/sdkconfig.base
SDKCONFIG += boards/sdkconfig.spiram
#SDKCONFIG += boards/sdkconfig.display

FROZEN_MANIFEST ?= $(BOARD_DIR)/manifest.py

PART_SRC = $(BOARD_DIR)/partitions.csv
