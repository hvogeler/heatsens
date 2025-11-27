#! /bin/bash

python $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py \
    generate nvs_creds.csv nvs_creds.bin 0x6000

esptool.py -p /dev/cu.usbmodem1101 write_flash 0x9000 nvs_creds.bin