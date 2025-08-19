//screen
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SDA A4
#define SCL A5

// Grid parameters for 9x9 grid
#define GRID_SIZE 9
#define CELL_SIZE 7     // Each cell is 7x7 pixels
#define GRID_WIDTH (GRID_SIZE * CELL_SIZE)   // 63 pixels
#define GRID_HEIGHT (GRID_SIZE * CELL_SIZE)  // 63 pixels
#define GRID_START_X ((SCREEN_WIDTH - GRID_WIDTH) / 2)  // Center horizontally
#define GRID_START_Y ((SCREEN_HEIGHT - GRID_HEIGHT) / 2) // Center vertically

void setup() {
    Serial.begin(9600);
    Wire.begin();
    while (!Serial);  // Wait for Serial (if needed)

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }

    // Clear the buffer
    display.clearDisplay();
    
    // Draw the 9x9 grid
    drawGrid();
    display.display();
}

void drawGrid() {
    display.clearDisplay();

    // Function to check if a cell should be removed
    auto isRemoved = [](int row, int col) -> bool {
        // Top-left
        if ((row == 0 && col == 0) || (row == 0 && col == 1) || (row == 1 && col == 0)) return true;
        // Top-right
        if ((row == 0 && col == 8) || (row == 0 && col == 7) || (row == 1 && col == 8)) return true;
        // Bottom-left
        if ((row == 8 && col == 0) || (row == 8 && col == 1) || (row == 7 && col == 0)) return true;
        // Bottom-right
        if ((row == 8 && col == 8) || (row == 8 && col == 7) || (row == 7 && col == 8)) return true;
        return false;
    };

    // Draw boundaries only for non-removed cells
    for (int row = 0; row < GRID_SIZE; ++row) {
        for (int col = 0; col < GRID_SIZE; ++col) {
            if (!isRemoved(row, col)) {
                int left = GRID_START_X + col * CELL_SIZE;
                int right = left + CELL_SIZE;
                int top = GRID_START_Y + row * CELL_SIZE;
                int bottom = top + CELL_SIZE;

                // Draw top
                display.drawLine(left, top, right, top, SSD1306_WHITE);
                // Draw bottom
                display.drawLine(left, bottom, right, bottom, SSD1306_WHITE);
                // Draw left
                display.drawLine(left, top, left, bottom, SSD1306_WHITE);
                // Draw right
                display.drawLine(right, top, right, bottom, SSD1306_WHITE);
            }
        }
    }
}

void loop() {
    // Grid is static, so we just maintain the display
    // You can add dynamic updates here if needed
    delay(1000);
}
