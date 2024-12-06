# WorkingWithGPIO
CODAL build tools for the micro:bit with my own program, ready to be built and flashed.

ALL CODE ASIDE FROM THE CW1.cpp IN microbit-v2-samples/source IS WORK OF VARIOUS MICROBIT DEVELOPERS AND RESEARCHERS AND I TAKE NO CREDIT FOR THAT CODE.

1. sudo apt install gcc
2. sudo apt install git
3. sudo apt install cmake
4. sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi

THIS PROGRAM PROVIDES 4 FUNCTIONS:
1. 5HZ Binary counter from 0b00000 to 0b11111.
2. Binary display from 0b00000 to 0b11111 with button support for -1 and +1.
3. Voltage sampler to sample the voltage applied to edge connector Pin 0 (Serial prints and device LED display).
4. Breathe an RGB LED (Red, Edge Pin 0)(Green, Edge Pin 9)(Blue, Edge Pin 8).

Written November 2024 by Will Holbrook
