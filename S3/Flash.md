# ESP32S3
Flash command for ESP-IDF

esptool.py -p COM# -b 460800 --chip esp32s3 write_flash 0x0000 bootloader.bin 0x8000 partition-table.bin 0xd000 ota_data_initial.bin 0x10000 network_adapter.bin
