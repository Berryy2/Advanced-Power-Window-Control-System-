#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "FreeRTOS.h"
#include "tm4c123gh6pm.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "queue.h"
#include "semphr.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/lcd.h"
#include "TM4C123.h"
#include "LCD_I2C.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/qei.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#ifndef GPIO_PD6_PHA0
#define GPIO_PD6_PHA0 0x00031806
#endif

#ifndef GPIO_PD7_PHB0
#define GPIO_PD7_PHB0 0x00031C06
#endif

#define MAX_POS_CM 40
#define MIN_POS_CM 0

// Driver control pins
#define DRIVER_UP_PIN    GPIO_PIN_4  // PB4
#define DRIVER_DOWN_PIN  GPIO_PIN_5  // PB5

// Passenger control pins
#define PASSENGER_UP_PIN   GPIO_PIN_6  // PA6
#define PASSENGER_DOWN_PIN GPIO_PIN_7  // PA7

// Window lock pin
#define PASSENGER_LOCK_PIN GPIO_PIN_6  // PB6

// Limit switches
#define UPPER_LIMIT_PIN GPIO_PIN_0  // PB0
#define LOWER_LIMIT_PIN GPIO_PIN_1  // PB1

// IR obstacle sensor
#define IR_SENSOR_PIN GPIO_PIN_7  // PB7

// Buzzer
#define BUZZER_PIN GPIO_PIN_5  // PA5

// Debounce time for auto mode detection (in ms)
#define AUTO_MODE_THRESHOLD 1000

// Emergency reverse duration (in ms)
#define EMERGENCY_REVERSE_TIME_MS 500
#define REVERSE_SPEED_DELAY_MS 10
#define CM_PER_SECOND 15

// Semaphore for motor control
SemaphoreHandle_t xMotorSemaphore;

// Queue for emergency events
QueueHandle_t xEmergencyQueue;

// Global flag for emergency state
volatile bool emergencyStop = false;

// Position tracking
volatile uint32_t position_cm = 0;

/* Function: LCD_ClearLine
 * Type: void
 * Arguments: uint8_t line - the LCD line number to clear
 * Return: void
 * Description: Clears the specified line on the LCD screen. */
void LCD_ClearLine(uint8_t line) {
    // Move cursor to the beginning of the specified line
		LCD_SetCursor(line, 0);
		// Overwrite the entire line with spaces to clear it
    LCD_Print("                ");
		// Reset cursor to the start of the line again
    LCD_SetCursor(line, 0);
}

/* Function: delay_ms
 * Type: void
 * Arguments: int time - time in milliseconds to delay
 * Return: void
 * Description: Software-based delay loop approximating milliseconds. */
void delay_ms(int time) {
    int i, j;
	 // Nested loop approximates a 1ms delay based on CPU speed
    for (i = 0; i < time; i++)
        for (j = 0; j < 3180; j++) {}
}

/* Function: QEI0_Init
 * Type: void
 * Arguments: none
 * Return: void
 * Description: Initializes the Quadrature Encoder Interface (QEI0) for position feedback. */
void QEI0_Init(void) {
		// Enable peripherals for QEI and GPIO port D
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
		// Wait until both peripherals are ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
		// Unlock PD7 (a protected pin)
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
		// Configure PD6 and PD7 for QEI signals
#ifdef GPIO_PD6_PHA0
    GPIOPinConfigure(GPIO_PD6_PHA0);
#endif
#ifdef GPIO_PD7_PHB0
    GPIOPinConfigure(GPIO_PD7_PHB0);
#endif
		// Set the pins for QEI input type
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
		// Enable weak pull-ups for PD6 and PD7
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		// Disable QEI during setup
    QEIDisable(QEI0_BASE);
		// Configure QEI: count both signals, no reset on index, quadrature mode, swap phase A/B
    QEIConfigure(QEI0_BASE,
                 QEI_CONFIG_CAPTURE_A_B |
                 QEI_CONFIG_NO_RESET |
                 QEI_CONFIG_QUADRATURE |
                 QEI_CONFIG_SWAP,
                 0xFFFFFFFF); // Max position value
		// Reset position counter						 
    QEIPositionSet(QEI0_BASE, 0);
		// Enable QEI module
    QEIEnable(QEI0_BASE);
}

/* Function: Motor_Init
 * Type: void
 * Arguments: none
 * Return: void
 * Description: Initializes the motor control pins (PF2 and PF3) as outputs. */
void Motor_Init(void) {
		// Enable clock for GPIO Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
		// Unlock GPIO pins if locked (PF0 is often locked)
    GPIOUnlockPin(GPIO_PORTF_BASE, GPIO_PIN_0);
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x1F;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
		// Set PF2 and PF3 as outputs for motor control
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
}

/* Function: Buzzer_Init
 * Type: void
 * Arguments: none
 * Return: void
 * Description: Initializes the buzzer pin as output and sets it off. */
void Buzzer_Init(void) {
		// Enable GPIO port A for buzzer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    // Set buzzer pin (PA5) as output
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, BUZZER_PIN);
		// Ensure buzzer is off initiall
    GPIOPinWrite(GPIO_PORTA_BASE, BUZZER_PIN, 0);
}

/* Function: Buzzer_On
 * Type: void
 * Arguments: none
 * Return: void
 * Description: Activates the buzzer. */
void Buzzer_On(void) {
		// Set buzzer pin high (ON)
    GPIOPinWrite(GPIO_PORTA_BASE, BUZZER_PIN, BUZZER_PIN);
}

/* Function: Buzzer_Off
 * Type: void
 * Arguments: none
 * Return: void
 * Description: Deactivates the buzzer. */
void Buzzer_Off(void) {
		// Set buzzer pin low (OFF)
    GPIOPinWrite(GPIO_PORTA_BASE, BUZZER_PIN, 0);
}

/* Function: Motor_Up
 * Type: void
 * Arguments: none
 * Return: void
 * Description: Moves the motor in the upward direction if semaphore is available. */
void Motor_Up(void) {
		// Take motor semaphore to prevent race condition
    if (xSemaphoreTake(xMotorSemaphore, portMAX_DELAY) == pdTRUE) {
				// Clear LCD status line
        LCD_ClearLine(0);
        LCD_Print("Closing..   ");
				// Write signal to move motor up (PF2 high, PF3 low)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2);
				// Release semaphore
        xSemaphoreGive(xMotorSemaphore);
    }
}

/* Function: Motor_Down
 * Type: void
 * Arguments: none
 * Return: void
 * Description: Moves the motor in the downward direction if semaphore is available. */
void Motor_Down(void) {
		// Take motor semaphore to prevent race condition
    if (xSemaphoreTake(xMotorSemaphore, portMAX_DELAY) == pdTRUE) {
				// Clear LCD status line
        LCD_ClearLine(0);
        LCD_Print("Opening..   ");
				// Write signal to move motor down (PF2 low, PF3 high)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);
				// Release semaphore
        xSemaphoreGive(xMotorSemaphore);
    }
}

/* Function: Motor_Stop
 * Type: void
 * Arguments: none
 * Return: void
 * Description: Stops the motor if semaphore is available. */
void Motor_Stop(void) {
		// Take motor semaphore to prevent race condition
    if (xSemaphoreTake(xMotorSemaphore, portMAX_DELAY) == pdTRUE) {
				// Stop motor (set both PF2 and PF3 low)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3, 0);
				// Release semaphore
        xSemaphoreGive(xMotorSemaphore);
    }
}

/* Function: LCD_DisplayPosition
 * Type: void
 * Arguments: uint32_t cm - the current position in centimeters
 * Return: void
 * Description: Displays the current position on the second line of the LCD. */
void LCD_DisplayPosition(uint32_t cm) {
    char buffer[16];
		// Format the position text
    sprintf(buffer, "Pos: %lu cm   ", cm);
		// Set cursor to line 1, column 0
    LCD_SetCursor(1, 0);
		// Print formatted position
    LCD_Print(buffer);
}

/* Function: Buttons_Init
 * Type: void
 * Arguments: none
 * Return: void
 * Description: Initializes the input pins for driver and passenger buttons, limit switches, and sensors. */
void Buttons_Init(void) {
		// Enable clocks for GPIO ports B and A
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
		// Set PB4, PB5, PB6, PB7 as inputs with pull-up (driver buttons, lock, IR sensor)
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, DRIVER_UP_PIN | DRIVER_DOWN_PIN | PASSENGER_LOCK_PIN | IR_SENSOR_PIN);
    GPIOPadConfigSet(GPIO_PORTB_BASE, DRIVER_UP_PIN | DRIVER_DOWN_PIN | PASSENGER_LOCK_PIN | IR_SENSOR_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		// Set PB0 and PB1 as inputs for upper/lower limit switches
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, UPPER_LIMIT_PIN | LOWER_LIMIT_PIN);
    GPIOPadConfigSet(GPIO_PORTB_BASE, UPPER_LIMIT_PIN | LOWER_LIMIT_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		// Set PA6 and PA7 as inputs for passenger buttons
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, PASSENGER_UP_PIN | PASSENGER_DOWN_PIN);
    GPIOPadConfigSet(GPIO_PORTA_BASE, PASSENGER_UP_PIN | PASSENGER_DOWN_PIN,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

/* Function: IsButtonPressed
 * Type: bool
 * Arguments: uint32_t port - GPIO port base; uint8_t pin - GPIO pin
 * Return: true if the button is pressed; false otherwise
 * Description: Checks if the given button pin is currently pressed. */
bool IsButtonPressed(uint32_t port, uint8_t pin) {
    return (GPIOPinRead(port, pin) & pin) == 0;
}

/* Function: IsUpperLimitReached
 * Type: bool
 * Arguments: none
 * Return: true if upper limit switch is pressed; false otherwise
 * Description: Checks if the window has reached the upper mechanical limit. */
bool IsUpperLimitReached(void) {
    return IsButtonPressed(GPIO_PORTB_BASE, UPPER_LIMIT_PIN);
}

/* Function: IsLowerLimitReached
 * Type: bool
 * Arguments: none
 * Return: true if lower limit switch is pressed; false otherwise
 * Description: Checks if the window has reached the lower mechanical limit. */
bool IsLowerLimitReached(void) {
    return IsButtonPressed(GPIO_PORTB_BASE, LOWER_LIMIT_PIN);
}

/* Function: IsObstacleDetected
 * Type: bool
 * Arguments: none
 * Return: true if IR sensor detects an obstacle; false otherwise
 * Description: Checks for obstacle detection using the IR sensor. */
bool IsObstacleDetected(void) {
    return IsButtonPressed(GPIO_PORTB_BASE, IR_SENSOR_PIN);
}

/* Function: IsAutoMode
 * Type: bool
 * Arguments: uint32_t port, uint8_t pin - button port and pin
 * Return: true if auto mode is detected (press held short); false if long hold
 * Description: Determines if a button press qualifies as auto or manual mode. */
bool IsAutoMode(uint32_t port, uint8_t pin) {
    if (IsButtonPressed(port, pin)) {
				uint32_t start_time = xTaskGetTickCount();
				// Stay in loop while button remains pressed
        while (IsButtonPressed(port, pin)) {
						// If held too long, it's manual mode
            if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(AUTO_MODE_THRESHOLD)) {
                return false;
            }
        }
				// Button was pressed and released quickly
        return true;
    }
    return false;
}

/* Function: EmergencyTask
 * Type: void (FreeRTOS Task)
 * Arguments: void* pvParameters - task parameters (unused)
 * Return: void
 * Description: Responds to emergency events like obstacle detection, triggers motor stop/reverse and buzzer. */
void EmergencyTask(void *pvParameters) {
    uint8_t emergencyEvent;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    
    for (;;) {
				// Check if emergency event was queued
        if (xQueueReceive(xEmergencyQueue, &emergencyEvent, 0) == pdPASS) {
            if (emergencyEvent == 1) { // Obstacle detected
                emergencyStop = true;
                
                Motor_Stop();	// Immediately stop motor
                Buzzer_On();	// Alert with buzzer
                LCD_ClearLine(0);
                LCD_Print("OBSTACLE!!");
                vTaskDelay(pdMS_TO_TICKS(500)); 
                
                Motor_Down(); //Start reversing down
                
                TickType_t xStartTime = xTaskGetTickCount();
                while ((xTaskGetTickCount() - xStartTime) < pdMS_TO_TICKS(EMERGENCY_REVERSE_TIME_MS)) {
                    if (IsLowerLimitReached()) break;
                    
                    static uint32_t lastUpdate = 0;
                    uint32_t now = xTaskGetTickCount();
										// Update LCD every 100ms
                    if (now - lastUpdate >= pdMS_TO_TICKS(100)) {
                        if (position_cm > MIN_POS_CM) {
                            position_cm--;
                            LCD_DisplayPosition(position_cm);
                        }
                        lastUpdate = now;
                    }
                    
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                
                Motor_Stop(); //Stop Motor
                Buzzer_Off(); //Turn of buzzer
                emergencyStop = false; // Reset flag
                
                LCD_DisplayPosition(position_cm);
                LCD_ClearLine(0);  
                LCD_Print("SAFE           ");
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Repeat periodically
    }
}

/* Function: DriverControlTask
 * Type: void (FreeRTOS Task)
 * Arguments: void* pvParameters - task parameters (unused)
 * Return: void
 * Description: Handles driver inputs for window up/down in both manual and auto modes. */
void DriverControlTask(void *pvParameters) {
    for (;;) {
        if (emergencyStop) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
				// Handle UP button
        if (IsButtonPressed(GPIO_PORTB_BASE, DRIVER_UP_PIN)) {
            bool auto_mode = IsAutoMode(GPIO_PORTB_BASE, DRIVER_UP_PIN);
            
            if (auto_mode) {
                LCD_ClearLine(0);
                LCD_Print("Auto Closing");
                while (!IsUpperLimitReached() && position_cm < MAX_POS_CM && !emergencyStop) {
                    if (IsObstacleDetected()) {
                        uint8_t event = 1;
                        xQueueSend(xEmergencyQueue, &event, 0);
                        break;
                    }
                    Motor_Up();
                    position_cm++;
                    LCD_DisplayPosition(position_cm);
                    delay_ms(100);
                }
            } else {
                LCD_ClearLine(0);
                LCD_Print("Manual Closing");
                while (IsButtonPressed(GPIO_PORTB_BASE, DRIVER_UP_PIN) && 
                       !IsUpperLimitReached() && position_cm < MAX_POS_CM && !emergencyStop) {
                    if (IsObstacleDetected()) {
                        uint8_t event = 1;
                        xQueueSend(xEmergencyQueue, &event, 0);
                        break;
                    }
                    Motor_Up();
                    position_cm++;
                    LCD_DisplayPosition(position_cm);
                    delay_ms(100);
                }
            }
            Motor_Stop();
            
            if (IsUpperLimitReached()) {
                position_cm = MAX_POS_CM;
                LCD_DisplayPosition(position_cm);
                LCD_ClearLine(0);
                LCD_Print("Upper Limit   ");
            }
        }
				// Handle DOWN button
        if (IsButtonPressed(GPIO_PORTB_BASE, DRIVER_DOWN_PIN)) {
            bool auto_mode = IsAutoMode(GPIO_PORTB_BASE, DRIVER_DOWN_PIN);
            
            if (auto_mode) {
                LCD_ClearLine(0);
                LCD_Print("Auto Opening");
                while (!IsLowerLimitReached() && position_cm > MIN_POS_CM && !emergencyStop) {
                    Motor_Down();
                    position_cm--;
                    LCD_DisplayPosition(position_cm);
                    delay_ms(100);
                }
            } else {
                LCD_ClearLine(0);
                LCD_Print("Manual Opening");
                while (IsButtonPressed(GPIO_PORTB_BASE, DRIVER_DOWN_PIN) && 
                       !IsLowerLimitReached() && position_cm > MIN_POS_CM && !emergencyStop) {
                    Motor_Down();
                    position_cm--;
                    LCD_DisplayPosition(position_cm);
                    delay_ms(100);
                }
            }
            Motor_Stop();
            
            if (IsLowerLimitReached()) {
                position_cm = MIN_POS_CM;
                LCD_DisplayPosition(position_cm);
                LCD_ClearLine(0);
                LCD_Print("Lower Limit   ");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Task delay
    }
}

/* Function: PassengerControlTask
 * Type: void (FreeRTOS Task)
 * Arguments: void* pvParameters - task parameters (unused)
 * Return: void
 * Description: Handles passenger inputs for window up/down in both manual and auto modes, obeying lock. */
void PassengerControlTask(void *pvParameters) {
    static char lastMessage[16] = "";

    for (;;) {
        if (emergencyStop) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
	
        if (IsButtonPressed(GPIO_PORTB_BASE, PASSENGER_LOCK_PIN)) {
            if (strcmp(lastMessage, "Pass. Disable") != 0) {
                LCD_ClearLine(0);
                LCD_Print("Pass. Disable ");
                strcpy(lastMessage, "Pass. Disable");
                delay_ms(2000);
                LCD_ClearLine(0);
            }
        } else {
						// Passenger UP button
            if (IsButtonPressed(GPIO_PORTA_BASE, PASSENGER_UP_PIN)) {
                bool auto_mode = IsAutoMode(GPIO_PORTA_BASE, PASSENGER_UP_PIN);
                
                if (auto_mode) {
                    LCD_ClearLine(0);
                    LCD_Print("Auto Closing");
                    while (!IsUpperLimitReached() && position_cm < MAX_POS_CM && !emergencyStop) {
                        if (IsObstacleDetected()) {
                            uint8_t event = 1;
                            xQueueSend(xEmergencyQueue, &event, 0);
                            break;
                        }
                        Motor_Up();
                        position_cm++;
                        LCD_DisplayPosition(position_cm);
                        delay_ms(100);
                    }
                } else {
                    LCD_ClearLine(0);
                    LCD_Print("Manual Closing");
                    while (IsButtonPressed(GPIO_PORTA_BASE, PASSENGER_UP_PIN) && 
                           !IsUpperLimitReached() && position_cm < MAX_POS_CM && !emergencyStop) {
                        if (IsObstacleDetected()) {
                            uint8_t event = 1;
                            xQueueSend(xEmergencyQueue, &event, 0);
                            break;
                        }
                        Motor_Up();
                        position_cm++;
                        LCD_DisplayPosition(position_cm);
                        delay_ms(100);
                    }
                }
                Motor_Stop();
                
                if (IsUpperLimitReached()) {
                    position_cm = MAX_POS_CM;
                    LCD_DisplayPosition(position_cm);
                    LCD_ClearLine(0);
                    LCD_Print("Upper Limit   ");
                }
            }
						// Passenger DOWN button
            if (IsButtonPressed(GPIO_PORTA_BASE, PASSENGER_DOWN_PIN)) {
                bool auto_mode = IsAutoMode(GPIO_PORTA_BASE, PASSENGER_DOWN_PIN);
                
                if (auto_mode) {
                    LCD_ClearLine(0);
                    LCD_Print("Auto Opening");
                    while (!IsLowerLimitReached() && position_cm > MIN_POS_CM && !emergencyStop) {
                        Motor_Down();
                        position_cm--;
                        LCD_DisplayPosition(position_cm);
                        delay_ms(100);
                    }
                } else {
                    LCD_ClearLine(0);
                    LCD_Print("Manual Opening");
                    while (IsButtonPressed(GPIO_PORTA_BASE, PASSENGER_DOWN_PIN) && 
                           !IsLowerLimitReached() && position_cm > MIN_POS_CM && !emergencyStop) {
                        Motor_Down();
                        position_cm--;
                        LCD_DisplayPosition(position_cm);
                        delay_ms(100);
                    }
                }
                Motor_Stop();
                
                if (IsLowerLimitReached()) {
                    position_cm = MIN_POS_CM;
                    LCD_DisplayPosition(position_cm);
                    LCD_ClearLine(0);
                    LCD_Print("Lower Limit   ");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int main(void) {
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL |
                   SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Create semaphore for motor control
    xMotorSemaphore = xSemaphoreCreateMutex();
    
    // Create emergency queue (size for 5 events)
    xEmergencyQueue = xQueueCreate(5, sizeof(uint8_t));
    
    QEI0_Init();
    Motor_Init();
    Buzzer_Init();
    Buttons_Init();
    LCD_Init();
    LCD_SetBacklight(1);
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print("Power Window");

    // Create tasks
    xTaskCreate(EmergencyTask, "Emergency", 128, NULL, 3, NULL);
    xTaskCreate(DriverControlTask, "Driver", 128, NULL, 2, NULL);
    xTaskCreate(PassengerControlTask, "Passenger", 128, NULL, 1, NULL);

    vTaskStartScheduler(); // Start FreeRTOS
    while (1) {}
}

/* Function: vApplicationIdleHook
 * Type: void
 * Arguments: none
 * Return: void
 * Description: FreeRTOS idle hook that puts the processor to sleep using WFI instruction. */
void vApplicationIdleHook(void) {
    __asm("    WFI"); // Wait For Interrupt (low power idle)
}