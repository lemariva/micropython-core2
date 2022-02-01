set(srcs "i2cdev/i2c_drv.c"
         "i2cdev/i2cdev_esp32.c"
         "axp192/axp20x.c"
         "bm8563/bm8563.c"
         "ft6336u/ft6336u.c"
         "ili9341/ili9340.c"
         "ili9341/fontx.c"
         "ili9341/pngle.c"
         "ili9341/decode_image.c"
         "mpu6886/mpu6886.c"
         "mpu6886/debug_cf.c"
         "mpu6886/eprintf.c"
        )
set(includes "i2cdev/include"
              "axp192/include"
              "bm8563/include"
              "ft6336u/include"
              "ili9341/include"
              "mpu6886/include"
              "../spiffs/include")

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${includes}
                       PRIV_REQUIRES driver)

#spiffs_create_partition_image(storage font FLASH_IN_PROJECT)