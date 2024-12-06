// Will Holbrook Dec 2024
// SCC369 CW3 All Tasks 1-3 fully completed!
// Lancaster University

#include "MicroBit.h"

//
// references to external objects
//
extern NRF52Serial serial;  // serial object for displaying type of sensor and debugging 
extern NRF52I2C    i2cInt;  // internal I2C object for talking to accelerometer
extern NRF52I2C    i2cExt;  // external I2C object for talking to OLED display


//###################################################################################
// Write your code for Subtask 1 below here (and leave these separators in your code!)
// put functions, global variables, consts, #DEFINEs etc. that you need for Subtask 1 here
//

uint8_t microBitDisplayFrameBuffer[5][5] = {0};     // The bitmap in memory for the LED pixels

// Interrupt Service Routine for the MicroBit display
static void microBitDisplayIsr() {
    // ISR code - Show the correct row of the microbit display
    // This ISR runs at 100Hz (i.e. 100fps), each time lighting up one row

    // Static variable to keep track of the current row
    static size_t current_row = 0;

    if (NRF_TIMER0->EVENTS_COMPARE[0]) {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;

        // Clear the current visible microbit LED config (turn off all rows)
        NRF_P0->OUTCLR = (1UL << MICROBIT_PIN_ROW1) |
                         (1UL << MICROBIT_PIN_ROW2) |
                         (1UL << MICROBIT_PIN_ROW3) |
                         (1UL << MICROBIT_PIN_ROW4) |
                         (1UL << MICROBIT_PIN_ROW5);
        NRF_P0->OUTSET = (1UL << MICROBIT_PIN_COL1) |
                         (1UL << MICROBIT_PIN_COL2) |
                         (1UL << MICROBIT_PIN_COL3) |
                         (1UL << MICROBIT_PIN_COL5);
        NRF_P1->OUTSET = (1UL << (MICROBIT_PIN_COL4) - 32);

        // Set the current row to HIGH (turn on the row)
        switch (current_row) {
            case 0:
                NRF_P0->OUTSET = (1UL << MICROBIT_PIN_ROW1);
                break;
            case 1:
                NRF_P0->OUTSET = (1UL << MICROBIT_PIN_ROW2);
                break;
            case 2:
                NRF_P0->OUTSET = (1UL << MICROBIT_PIN_ROW3);
                break;
            case 3:
                NRF_P0->OUTSET = (1UL << MICROBIT_PIN_ROW4);
                break;
            case 4:
                NRF_P0->OUTSET = (1UL << MICROBIT_PIN_ROW5);
                break;
        }

        // Set columns to LOW for the LEDs that should be on in the current row
        for (size_t j = 0; j < 5; j++) {
            if (microBitDisplayFrameBuffer[current_row][j]) {
                // Set the corresponding column to LOW to turn on the LED
                switch (j) {
                    case 0:
                        NRF_P0->OUTCLR = (1UL << MICROBIT_PIN_COL1);
                        break;
                    case 1:
                        NRF_P0->OUTCLR = (1UL << MICROBIT_PIN_COL2);
                        break;
                    case 2:
                        NRF_P0->OUTCLR = (1UL << MICROBIT_PIN_COL3);
                        break;
                    case 3:
                        NRF_P1->OUTCLR = (1UL << (MICROBIT_PIN_COL4 - 32));
                        break;
                    case 4:
                        NRF_P0->OUTCLR = (1UL << MICROBIT_PIN_COL5);
                        break;
                }
            }
        }

        // Increment the current row to move to the next row in the next ISR run
        current_row = (current_row + 1) % 5;
    }
}

// Clear the MicroBit display buffer
void clearMicroBitDisplay() {
    // Clear frame buffer
    memset(&microBitDisplayFrameBuffer, 0, sizeof(microBitDisplayFrameBuffer));
}

// Initialise the MicroBit's LED display
void initMicroBitDisplay() {
    serial.printf("Clearing display\n");

    serial.printf("Configuring GPIOs\n");
    // Configure the necessary GPIOs
    NRF_P0->DIRSET = (GPIO_DIR_PIN21_Output << MICROBIT_PIN_ROW1) |
                     (GPIO_DIR_PIN21_Output << MICROBIT_PIN_ROW2) |
                     (GPIO_DIR_PIN21_Output << MICROBIT_PIN_ROW3) |
                     (GPIO_DIR_PIN21_Output << MICROBIT_PIN_ROW4) |
                     (GPIO_DIR_PIN21_Output << MICROBIT_PIN_ROW5) |
                     (GPIO_DIR_PIN28_Output << MICROBIT_PIN_COL1) |
                     (GPIO_DIR_PIN11_Output << MICROBIT_PIN_COL2) |
                     (GPIO_DIR_PIN31_Output << MICROBIT_PIN_COL3) |
                     (GPIO_DIR_PIN30_Output << MICROBIT_PIN_COL5);
    NRF_P1->DIRSET = (GPIO_DIR_PIN5_Output << (MICROBIT_PIN_COL4 - 32));

    // Set up a periodic timer that results in an interrupt
    serial.printf("Setting up timer\n");
    // Disable timer first for configuration
    NRF_TIMER0->TASKS_STOP = 1;
    NRF_TIMER0->TASKS_CLEAR = 1;

    // Configure TIMER1
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;     // Set mode to Timer
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_32Bit; // Set 16-bit mode
    NRF_TIMER0->PRESCALER = 4; // Prescaler 4 => Timer frequency = 1 MHz

    // Set compare value for interrupt
    NRF_TIMER0->CC[0] = (200000 / 100);   // 1 Second = 1000000 so with 5 rows, to update, we are to make the display wait (200ms / fps) 
    NRF_TIMER0->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;

    // Enable compare event for CC[0]
    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;

    // Enable the IRQ in NVIC
    NVIC_SetVector(TIMER0_IRQn, (uint32_t)microBitDisplayIsr);
    NVIC_EnableIRQ(TIMER0_IRQn);

    // Start the timer
    NRF_TIMER0->TASKS_START = 1;

    // Call clear frame buffer function
    clearMicroBitDisplay();
}

// Set an individual pixel on the LED display
void setMicroBitPixel(uint8_t x, uint8_t y) {
    // Sets the given pixel on the micro:bit display. The top-left pixel is at (0, 0) and the top-right pixel is at (4, 0)
    microBitDisplayFrameBuffer[x][y] = 1;
}

// Clear an individual pixel on the LED display
void clearMicroBitPixel(uint8_t x, uint8_t y) {
    // Clear a given pixel on the display
    microBitDisplayFrameBuffer[x][y] = 0;
}


//###################################################################################
// Write your additional code for Subtask 2 below here (and leave these separators in your code!)
// put functions, global variables, consts, #DEFINEs etc. that you need for Subtask 2 here
//

#define i2cSCL MICROBIT_PIN_EXT_SCL     // i2c SCL Pin (MicroBit Edge Pin 19)
#define i2cSDA MICROBIT_PIN_EXT_SDA     // i2c SDA Pin (MicroBit Edge Pin 20)
#define i2cSSD1306 0x3C << 1            // Display i2c address, shifted left by 1 to fix a bug with the write() function

static uint8_t oledDisplayFrameBuffer[8][128] = {0xFF};     // The bitmap in memory for the OLED pixels

uint8_t screen[1025];           // Screen buffer, with an extra byte for the 0x40 control data byte

// Update the content on the OLED display with the buffer contents
static void updateOledDisplay() {
    // NOTE: screen[0] is declared and set in OLED init function as it never changes
    memcpy(&screen[1], oledDisplayFrameBuffer, sizeof(oledDisplayFrameBuffer));
    i2cExt.write(i2cSSD1306, screen, sizeof(screen), false);

    // Debug
    //serial.printf("Display updated\n");
}

// Clear the OLED display's buffer, and then update the display
void clearOledDisplay() {
    // Clear frame buffer
    memset(oledDisplayFrameBuffer, 0x00, sizeof(oledDisplayFrameBuffer));
    updateOledDisplay();
}

// Initialise the external OLED display
void initOledDisplay() {
    // Declare permanent variables
    screen[0] = 0x40;

    // Set the frequency of i2c to 400
    i2cExt.setFrequency(50000);

    // Initialising the SSD1306
    const uint8_t initCommands[] = {
        0xA8, 0x3F,     // SET MUX RATIO
        0xD3, 0x00,     // SET DISPLAY OFFSET
        0x40,           // SET DISPLAY START LINE
        0xA1,           // SET SEGMENT RE-MAP
        0xC8,           // SET COM OUTPUT SCAN DIRECTION
        0xDA, 0x12,     // SET COM PINS HARDWARE CONFIG
        0x81, 0x7F,     // SET CONTRAST CONTROL
        0xA4,           // DISABLE ENTIRE DISPLAY ON
        0xA6,           // SET NORMAL DISPLAY
        0xD5, 0x80,     // SET OSC FREQUENCY
        0x8D, 0x14,     // ENABLE CHARGE PUMP REGULATOR
        0xAF            // DISPLAY ON
    };
    // Add a 0x80 in the buffer before every required command
    uint8_t data[sizeof(initCommands) * 2];
    for (size_t i = 0, j = 0; i < sizeof(initCommands); ++i, j += 2) {
        data[j] = 0x80;
        data[j + 1] = initCommands[i];
    }
    // Send the buffer with the commands preceded by a command control byte
    i2cExt.write(i2cSSD1306, data, sizeof(data), false);
    serial.printf("Written config to OLED successfully!\n");

    // Sending addressing comands for position of pixel write to the SSD1306
    const uint8_t addressingCommands[] = {
        0x20, 0x10,        // Set Addressing Mode to Horizontal
        0x21, 0x00, 0x7F,  // Set Column Address range from 0 to 127
        0x22, 0x00, 0x07   // Set Page Address range from 0 to 7
    };
    // Again, add 0x80 before every command to the buffer
    uint8_t addressingData[sizeof(addressingCommands) * 2];
    for (size_t i = 0, j = 0; i < sizeof(addressingCommands); ++i, j += 2) {
        addressingData[j] = 0x80;
        addressingData[j + 1] = addressingCommands[i];
    }
    // Send the buffer with the addressing commands + control bytes
    i2cExt.write(i2cSSD1306, addressingData, sizeof(addressingData), false);
    serial.printf("Addressing commands written successfully!\n");

    // Update the display with the current/latest buffer contents
    clearOledDisplay();
}

// Set an x, y bit in the OLED display buffer
void setOledFrameBuffer(uint8_t x, uint8_t y) {
    // Carry out the change (Set a pixel to ON/1)
    
    // Switch statement on row value, to determine page, and then set bit based on the relevant bit within the page
    switch (y / 8) {
        case 0:
            oledDisplayFrameBuffer[0][x] |= (1 << y);
            break;
        case 1:
            oledDisplayFrameBuffer[1][x] |= (1 << (y - 8));
            break;
        case 2:
            oledDisplayFrameBuffer[2][x] |= (1 << (y - 16));
            break;
        case 3:
            oledDisplayFrameBuffer[3][x] |= (1 << (y - 24));
            break;
        case 4:
            oledDisplayFrameBuffer[4][x] |= (1 << (y - 32));
            break;
        case 5:
            oledDisplayFrameBuffer[5][x] |= (1 << (y - 40));
            break;
        case 6:
            oledDisplayFrameBuffer[6][x] |= (1 << (y - 48));
            break;
        case 7:
            oledDisplayFrameBuffer[7][x] |= (1 << (y - 56));
            break;
    }
}

// Clear an x, y bit in the OLED display buffer
void clearOledFrameBuffer(uint8_t x, uint8_t y) {
    // Carry out the change (Set a pixel to OFF/0)
    
    // Switch statement on row value, to determine page, and then set bit based on the relevant bit within the page
    switch (y / 8) {
        case 0:
            oledDisplayFrameBuffer[0][x] |= (0 << y);
            break;
        case 1:
            oledDisplayFrameBuffer[1][x] |= (0 << (y - 8));
            break;
        case 2:
            oledDisplayFrameBuffer[2][x] |= (0 << (y - 16));
            break;
        case 3:
            oledDisplayFrameBuffer[3][x] |= (0 << (y - 24));
            break;
        case 4:
            oledDisplayFrameBuffer[4][x] |= (0 << (y - 32));
            break;
        case 5:
            oledDisplayFrameBuffer[5][x] |= (0 << (y - 40));
            break;
        case 6:
            oledDisplayFrameBuffer[6][x] |= (0 << (y - 48));
            break;
        case 7:
            oledDisplayFrameBuffer[7][x] |= (0 << (y - 56));
            break;
    }
}

// Set an x, y bit in the OLED display buffer AND update the display if the buffer has changed as a result
void setOledPixel(uint8_t x, uint8_t y) {
    // Make a copy of the buffer before the 'change'
    uint8_t frameBufferCpy[8][128];
    memcpy(frameBufferCpy, oledDisplayFrameBuffer, sizeof(oledDisplayFrameBuffer));

    // Set the frame buffer accordingly
    setOledFrameBuffer(x, y);

    // If the newly modified buffer is different from the original copy (there is a valid change), update the display...
    if (memcmp(frameBufferCpy, oledDisplayFrameBuffer, sizeof(oledDisplayFrameBuffer)) != 0) {
        updateOledDisplay();
    }
}

// Clear an x, y bit in the OLED display buffer AND update the display if the buffer has changed as a result
void clearOledPixel(uint8_t x, uint8_t y) {
    // Make a copy of the buffer before the 'change'
    uint8_t frameBufferCpy[8][128];
    memcpy(frameBufferCpy, oledDisplayFrameBuffer, sizeof(oledDisplayFrameBuffer));

    // Carry out the change (Set a pixel to OFF/0x00)
    // NTS: if statement on row value, to determine page, and then set bit based on the relevant bit within the page 

    clearOledFrameBuffer(x, y);

    // If the newly modified buffer is different from the original copy (there is a valid change), update the display...
    if (memcmp(frameBufferCpy, oledDisplayFrameBuffer, sizeof(oledDisplayFrameBuffer)) != 0) {
        updateOledDisplay();
    }
}

// Draw a straight line between two x,y points on the OLED display (write to buiffer, update)
void drawOledLine(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end) {
    // Draw a straight line between (x_start, y_start) and (x_end, y_end)
    
    // Make a copy of the buffer before the 'change'
    uint8_t frameBufferCpy[8][128];
    memcpy(frameBufferCpy, oledDisplayFrameBuffer, sizeof(oledDisplayFrameBuffer));

    // Calculate the differences
    int dx = x_end - x_start;
    int dy = y_end - y_start;
    int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);

    if (steps == 0) {
        setOledFrameBuffer(x_start, y_start);
        return;
    }

    int x_inc = (dx << 8) / steps; // Shift left to simulate fixed-point
    int y_inc = (dy << 8) / steps;

    int x = x_start << 8;
    int y = y_start << 8;

    for (int i = 0; i <= steps; i++) {
        setOledFrameBuffer(x >> 8, y >> 8);
        x += x_inc;
        y += y_inc;
    }

    // If the newly modified buffer is different from the original copy (there is a valid change), update the display...
    if (memcmp(frameBufferCpy, oledDisplayFrameBuffer, sizeof(oledDisplayFrameBuffer)) != 0) {
        updateOledDisplay();
    }
}


//###################################################################################
// Write your additional code for Subtask 3 below here (and leave these separators in your code!)
// put functions, global variables, consts, #DEFINEs etc. that you need for Subtask 2 here
//

#define ACCEL_X_L_REG 0x28  // X Lo Register for Accelerometer
#define ACCEL_X_H_REG 0x29  // X Hi Register for Accelerometer

uint8_t rr;                 // The refresh rate, retrieved from the parameter given in graphData()
uint8_t data[2];            // Data for holding Hi and Lo bytes of x accel value
uint8_t currentColumn;      // The current column being used for a graph line on the display
short accel_x;              // The raw X accelerator value (-512 - 511)
uint16_t mapped_accel_x;    // Mapped x value to suit the display coords
uint16_t last_accel_x;      // The previous mapped x accel value                                (Jerk mode)
short difference;           // The difference between the new x mapped value and the previous   (Jerk mode)
uint8_t mapped_difference;  // That difference value mathematically mapped for use              (Jerk mode)

// Initialise the timer
void initTimer() {
    // Set up timer
    NRF_TIMER1->TASKS_STOP = 1;                           // Ensure any tasks are finished
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;             // Set the timer to timer mode
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;    // Set to 32 bit mode
    NRF_TIMER1->PRESCALER = 4;                            // Prescaler to 4
    NRF_TIMER1->TASKS_CLEAR = 1;                          // Clear tasks ready for timer use
}

// Initialise the MicroBit buttons A and B
void initButtons() {
    // Set buttons A and B as inputs
    NRF_P0->DIR = (GPIO_DIR_PIN14_Input << MICROBIT_PIN_BUTTON_A) |
                  (GPIO_DIR_PIN23_Input << MICROBIT_PIN_BUTTON_B);

    // Set the pin configuration for both pin 14 (Button A) and pin32 (Button B)
    NRF_GPIO->PIN_CNF[MICROBIT_PIN_BUTTON_A] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                               (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                               (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
    
    NRF_GPIO->PIN_CNF[MICROBIT_PIN_BUTTON_B] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                                               (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                               (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);
}

// Check if a specified MicroBit button is pressed
bool isButtonPressed(char button) {
    if (button == 'A') {
        // Check Button A
        if (!(NRF_P0->IN & (1 << MICROBIT_PIN_BUTTON_A))) { // active low check
            return true;
        }
        return false;
    }
    if (button == 'B') {
        // Check Button B
        if (!(NRF_P0->IN & (1 << MICROBIT_PIN_BUTTON_B))) { // active low check
            return true;
        }
        return false;
    }
    return false;
}

// Get the X accelerometer value through i2c (returns said X value as an int)
int getAccelXValue() {
    // Get x accelerometer data and format it for display use
    i2cInt.writeRegister(0x19 << 1, 0x20, 0b01010111);                  // Write the configuration to the accelerometer through easy i2c
    i2cInt.readRegister(0x19 << 1, ACCEL_X_L_REG, &data[0], 1, false);  // Read the Lo accel X register
    i2cInt.readRegister(0x19 << 1, ACCEL_X_H_REG, &data[1], 1, false);  // Read the Hi accel X register
    int result = ((int16_t)(data[1] << 8) | data[0]);                 // Store the combined bytes as a 16 bit signed integer
    return result >> 6;                                                 // Shift the value right by 6 places as it's naturally a 10 bit left justified value
}

// Scroll the OLED buffer to the left, leaving a column 127 blank for a new data plot
void scrollDisplayLeft() {
    // Shift all oled buffer contents one to the left
    for (size_t i = 0; i < 8; i++) {
        for (size_t j = 0; j < 128; j++) {
            if (j != 127) {
                // For every column that isnt the final column, 'shift left' (in a way)
                oledDisplayFrameBuffer[i][j] = oledDisplayFrameBuffer[i][j+1];
            } else {
                // If we are at the last column, this is unneccesary, but clear it ready for the next graph line (in this case)
                oledDisplayFrameBuffer[i][j] = 0x00;
            }
        }
    }
}

// Draw a straight line between two x,y points within the OLED display buffer but DONT update
void setOledLine(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end) {
    // Draw a straight line between (x_start, y_start) and (x_end, y_end)
    // NOTE: All variables used here are defined in the above, similar 'drawOledLine()' implementation (Task 2 section)

    // Calculate the differences
    int dx = x_end - x_start;
    int dy = y_end - y_start;
    int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);

    if (steps == 0) {
        setOledFrameBuffer(x_start, y_start);
        return;
    }

    int x_inc = (dx << 8) / steps; // Shift left to simulate fixed-point
    int y_inc = (dy << 8) / steps;

    int x = x_start << 8;
    int y = y_start << 8;

    for (int i = 0; i <= steps; i++) {
        setOledFrameBuffer(x >> 8, y >> 8);
        x += x_inc;
        y += y_inc;
    }
}

// Acceleration mode (default)
void modeAcceleration(uint8_t rr) {
    serial.printf("Running: Acceleration mode\n");

    clearMicroBitDisplay();
    setMicroBitPixel(0, 0);
    setMicroBitPixel(1, 0);
    setMicroBitPixel(2, 0);
    setMicroBitPixel(3, 0);
    setMicroBitPixel(4, 0);
    setMicroBitPixel(2, 2);

    // BIG NTS:
        // keep drawing the bars from left to right (x0-127) until full, when full shift all previous bars to the left and print new one at x127.
        // NTS: i think here i should keep a 128 row buffer 0 - 127 where i shift every value down to add the new one at position 127 
        // so i can redraw the lines one column to the left and add the new one in
    clearOledDisplay();
    currentColumn = 0;

    // Init timer
    initTimer();

    while (1) {
        // Clear and start the timer
        NRF_TIMER1->TASKS_CLEAR = 1;                            // Clear all current timer tasks
        NRF_TIMER1->CC[0] = (1000000 / rr);                     // Set the compare register to (1 second (1000000) / fps)
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;                      // Ensure that comparison register is essentially false
        NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;   // Configure the timer to clear upon matching compare
        NRF_TIMER1->TASKS_START = 1;                            // Start timer!

        // Get accel x value
        accel_x = getAccelXValue();


        // Based on x value, where x = 0, draw a bar from the bottom to y0, aand where x = 1023 draw from the bottom to y64
        accel_x = accel_x + 512; // conevert the x value to a range of 0-1023
        mapped_accel_x = (63 - (accel_x * 63 / 1023));
        
        setOledLine(currentColumn, 63, currentColumn, mapped_accel_x);

        // If current column is 127 (end of screen), no longer increment aas we will write out of display range
        if (currentColumn < 127) {
            currentColumn++;
        } else {
            scrollDisplayLeft();
        }

        if (isButtonPressed('A') || isButtonPressed('B')) {
            break;
        }

        // Wait for the timer to complete
        while (NRF_TIMER1->EVENTS_COMPARE[0] == 0);     // Wait for our (1000ms (1 sec) / refresh rate) ms timer
        updateOledDisplay();
        NRF_TIMER1->TASKS_STOP = 1;                     // Stop timer once finished
    }
}

// Jerk mode
void modeJerk(uint8_t rr) {
    serial.printf("Running: Jerk mode\n");
    
    clearMicroBitDisplay();
    setMicroBitPixel(0, 4);
    setMicroBitPixel(1, 4);
    setMicroBitPixel(2, 4);
    setMicroBitPixel(3, 4);
    setMicroBitPixel(4, 4);
    setMicroBitPixel(2, 2);

    clearOledDisplay();
    currentColumn = 0;

    // Set up timer
    initTimer();

    while (1) {
        // Clear and start the timer
        NRF_TIMER1->TASKS_CLEAR = 1;                            // Clear all current timer tasks
        NRF_TIMER1->CC[0] = (1000000 / rr);                     // Set the compare register to (1 second (1000000) / fps)
        NRF_TIMER1->EVENTS_COMPARE[0] = 0;                      // Ensure that comparison register is essentially false
        NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;   // Configure the timer to clear upon matching compare
        NRF_TIMER1->TASKS_START = 1;                            // Start timer!

        accel_x = getAccelXValue();
        accel_x = accel_x + 512; // conevert the x value to a range of 0-1023
        
        // find how different the current accel value is from the last, 
        // and then visualise that on teh graph from a range of 0 (+1023) to 63 (-1023)
        difference = accel_x - last_accel_x;
        mapped_difference = 63 + ((difference - -1023) * (0 - 63)) / (1023 - -1023);

        setOledLine(currentColumn, 32, currentColumn, mapped_difference);

        // If current column is 127 (end of screen), no longer increment aas we will write out of display range
        if (currentColumn < 127) {
            last_accel_x = accel_x;
            currentColumn++;
        } else {
            scrollDisplayLeft();
        }

        if (isButtonPressed('A') || isButtonPressed('B')) {
            break;
        }

        // Wait for the timer to complete
        while (NRF_TIMER1->EVENTS_COMPARE[0] == 0);     // Wait for our (1000ms (1 sec) / refresh rate) ms timer
        updateOledDisplay();
        NRF_TIMER1->TASKS_STOP = 1;                     // Stop timer once finished
    }
}

// The main function to enable the overall graphing application (this function doesn't return)
void graphData(uint8_t refreshRate){
    rr = refreshRate;
    // Initialise i2c + accelerometer
    i2cInt.setFrequency(100000);
    
    // Initialise buttons
    initButtons();
    // Initialise the OLED display
    initOledDisplay();
    // Initialise the MicroBit's on-board 'display'
    initMicroBitDisplay();

    // Run the 'Accelerationn mode' as it is default
    modeAcceleration(rr);

    // Indefinitely check for a button press to choose a new mode
    while (1) {
        if (isButtonPressed('A')) {
            // If Button A pressed, run 'Acceleration mode'
            modeAcceleration(rr);
        }
        if (isButtonPressed('B')) {
            // If Button B pressed, run 'Jerk mode'
            modeJerk(rr);
        }
    }
}
