# nRF5_IoT_Breakout

Basic programs for nRF5 IoT breakout boards allowing use of MPU9250 motion sensor and BMP280 pressure sensor. Also a short program allowing rgb led to operate. Still to come is BLE demonstration program. All programs are intended to use the mbed compiler.

The nRF51822 Development Board is retired but this program will work on most nRF51-based devices.

![nRF52823 Development Board](https://d3s5r33r268y59.cloudfront.net/44691/products/thumbs/2016-05-21T06:41:32.433Z-nRF52DevBoard.top.3.jpg.855x570_q85_pad_rcrop.jpg)

These programs can be used for the [nRF52823 Development Board](https://www.tindie.com/products/onehorse/nrf52832-development-board/), you just have to change the rgb led pins to 22, 23 and 24 and change the I2C pins in mpu9250.h to pins 6 (SDA) and 7 (SCL). Pin 10 in the MPU9250 interrupt on the nR52 Dev Board. The UART can be on any two pins, but you will need to change from the code pins 9 and 10 because of the interrupt choice.
