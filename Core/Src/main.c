/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 64
#define CONFIRM_PORT GPIOB
#define CONFIRM_PIN GPIO_PIN_4
#define LEFT_PORT GPIOB
#define LEFT_PIN GPIO_PIN_5
#define RIGHT_PORT GPIOB
#define RIGHT_PIN GPIO_PIN_10
#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_5

uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_len = 0;
volatile uint8_t cmd_received = 0;
volatile uint8_t calibrate_state = 1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t modbus_crc(uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)data[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}


void send_modbus_command(uint8_t *data, uint16_t len) {
  uint16_t crc = modbus_crc(data, len);
  uint8_t crc_low = crc & 0xFF;
  uint8_t crc_high = (crc >> 8) & 0xFF;

  uint8_t buffer[len + 2];
  memcpy(buffer, data, len);
  buffer[len] = crc_low;
  buffer[len + 1] = crc_high;
  HAL_UART_Transmit(&huart1, buffer, len + 2, 1000);

  HAL_Delay(1); // Minimum 3.5 character delay (≈0.3ms at 115200)
}


void test_speed_control_1(uint8_t MOTOR_ADDRESS) {
    // Step 1: Motor enable
    uint8_t step1[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x06};
    send_modbus_command(step1, sizeof(step1));
    HAL_Delay(10);

    // Step 2
    uint8_t step2[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x07};
    send_modbus_command(step2, sizeof(step2));
    HAL_Delay(10);

    // Step 3: Set to speed mode
    uint8_t step3[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x0F};
    send_modbus_command(step3, sizeof(step3));
    HAL_Delay(10);

    // Step 4
    uint8_t step4[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x60, 0x00, 0x03};
    send_modbus_command(step4, sizeof(step4));
    HAL_Delay(10);

    // Step 5: Set acceleration to 1rps/s
    uint8_t step5[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x83, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x0A};
    send_modbus_command(step5, sizeof(step5));
    HAL_Delay(10);

    // Step 6: Set deceleration to 1rps/s
    uint8_t step6[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x84, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x0A};
    send_modbus_command(step6, sizeof(step6));
    HAL_Delay(10);

    // Step 7: Set target speed to 200rpm (modified from 100rpm)
    uint8_t step7[] = {MOTOR_ADDRESS, 0x10, 0x60, 0xFF, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0xC8};
    send_modbus_command(step7, sizeof(step7));
    HAL_Delay(10);
}

void stop_motor(uint8_t MOTOR_ADDRESS) {
	uint8_t stopCommand[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x06}; // Disable motor
	send_modbus_command(stopCommand, sizeof(stopCommand));
	HAL_Delay(10);
}

void test_absolute_position_control(uint8_t MOTOR_ADDRESS) {
    // Step 1: Motor enable (Shutdown command)
    uint8_t step1[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x06}; // CRC: 0x161C
    send_modbus_command(step1, sizeof(step1));
    HAL_Delay(10);

    // Step 2: Switch on
    uint8_t step2[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x07}; // CRC: 0xD7DC
    send_modbus_command(step2, sizeof(step2));
    HAL_Delay(10);

    // Step 3: Enable operation
    uint8_t step3[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x0F}; // CRC: 0xD61A
    send_modbus_command(step3, sizeof(step3));
    HAL_Delay(10);

    // Step 4: Set to position mode
    uint8_t step4[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x60, 0x00, 0x01}; // CRC: 0x5614
    send_modbus_command(step4, sizeof(step4));
    HAL_Delay(10);

    // Step 5: Set target speed to 900 rpm
    uint8_t step5[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x81, 0x00, 0x02, 0x04, 0x00, 0x00, 0x07, 0xD0}; // CRC: 0x93EA
    send_modbus_command(step5, sizeof(step5));
    HAL_Delay(10);

    // Step 6: Set acceleration to 5 rps/s
    uint8_t step6[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x83, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x64}; // CRC: 0x93DF
    send_modbus_command(step6, sizeof(step6));
    HAL_Delay(10);

    // Step 7: Set deceleration to 5 rps/s
    uint8_t step7[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x84, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x64}; // CRC: 0xD239
    send_modbus_command(step7, sizeof(step7));
    HAL_Delay(10);

    // Step 8: Set running distance to 10000 pulses
    uint8_t step8[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x7A, 0x00, 0x02, 0x04, 0x00, 0x00, 0x27, 0x10}; // CRC: 0xC6CA
    send_modbus_command(step8, sizeof(step8));
    HAL_Delay(10);

    // Step 9: Set absolute position mode
    uint8_t step9[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x0F};
    send_modbus_command(step9, sizeof(step9));
    HAL_Delay(10);

    // Step 10: Motion triggering
    uint8_t step10[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x1F};
    send_modbus_command(step10, sizeof(step10));
    HAL_Delay(10);
}

void set_pulses_per_revolution(uint8_t MOTOR_ADDRESS, uint32_t pulses_per_rev) {
    // Set electronic gear ratio: numerator = 131072, denominator = pulses_per_rev
    uint32_t numerator = 131072;  // Fixed numerator (encoder resolution)
    uint32_t denominator = pulses_per_rev;

    // Step 1: Write numerator to register 2034h
    uint8_t step1[] = {
        MOTOR_ADDRESS,
        0x10,               // Function code: Write multiple registers
        0x20, 0x34,         // Register address: 2034h
        0x00, 0x02,         // Number of registers to write (2 for 32-bit value)
        0x04,               // Byte count (4 bytes)
        (uint8_t)(numerator >> 24),
        (uint8_t)(numerator >> 16),
        (uint8_t)(numerator >> 8),
        (uint8_t)(numerator & 0xFF)
    };
    send_modbus_command(step1, sizeof(step1));
    HAL_Delay(10);

    // Step 2: Write denominator to register 2035h
    uint8_t step2[] = {
        MOTOR_ADDRESS,
        0x10,               // Function code: Write multiple registers
        0x20, 0x35,         // Register address: 2035h
        0x00, 0x02,         // Number of registers to write (2 for 32-bit value)
        0x04,               // Byte count (4 bytes)
        (uint8_t)(denominator >> 24),
        (uint8_t)(denominator >> 16),
        (uint8_t)(denominator >> 8),
        (uint8_t)(denominator & 0xFF)
    };
    send_modbus_command(step2, sizeof(step2));
    HAL_Delay(10);
}
int32_t degree_to_pulses(float degree) {
	int32_t pulses = (int32_t)(degree * 50 * 3.5 * 1000/360);
	return pulses;
}

void test_absolute_position_control_degree(uint8_t MOTOR_ADDRESS, float degree) {
    // Calculate the number of pulses based on the degree input
    int32_t pulses = degree_to_pulses(degree);

    // Step 1: Motor enable (Shutdown command)
    uint8_t step1[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x06}; // CRC: 0x161C
    send_modbus_command(step1, sizeof(step1));
    HAL_Delay(5);

    // Step 2: Switch on
    uint8_t step2[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x07}; // CRC: 0xD7DC
    send_modbus_command(step2, sizeof(step2));
    HAL_Delay(5);

    // Step 3: Enable operation
    uint8_t step3[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x0F}; // CRC: 0xD61A
    send_modbus_command(step3, sizeof(step3));
    HAL_Delay(5);

    // Step 4: Set to position mode
    uint8_t step4[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x60, 0x00, 0x01}; // CRC: 0x5614
    send_modbus_command(step4, sizeof(step4));
    HAL_Delay(5);

    // Step 5: Set target speed to 2000 rpm
//    uint8_t step5[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x81, 0x00, 0x02, 0x04, 0x00, 0x00, 0x07, 0xD0};
//    send_modbus_command(step5, sizeof(step5));
//    HAL_Delay(5);
//
    // Step 5: Set target speed to 600 rpm
    uint8_t step5[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x81, 0x00, 0x02, 0x04, 0x00, 0x00, 0x03, 0xE8};
    send_modbus_command(step5, sizeof(step5));
    HAL_Delay(5);

//    // Step 6: Set acceleration to 150 rps/s
//    uint8_t step6[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x83, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x96};
//    send_modbus_command(step6, sizeof(step6));
//    HAL_Delay(5);

    // Step 6: Set acceleration to 75 rps/s
    uint8_t step6[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x83, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x4B};
    send_modbus_command(step6, sizeof(step6));
    HAL_Delay(5);

    // Step 7: Set deceleration to 150 rps/s
    uint8_t step7[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x84, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x96}; // CRC: 0xD239
    send_modbus_command(step7, sizeof(step7));
    HAL_Delay(5);

    // Step 8: Set running distance to the calculated number of pulses
    uint8_t step8[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x7A, 0x00, 0x02, 0x04,
                       (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
                       (uint8_t)(pulses >> 8), (uint8_t)(pulses & 0xFF)}; // CRC needs to be calculated
    send_modbus_command(step8, sizeof(step8));
    HAL_Delay(5);

    // Step 9: Set absolute position mode
    uint8_t step9[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x0F};
    send_modbus_command(step9, sizeof(step9));
    HAL_Delay(5);

    // Step 10: Motion triggering
    uint8_t step10[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x1F};
    send_modbus_command(step10, sizeof(step10));
    HAL_Delay(5);
}
void test_relative_position_control_degree(uint8_t MOTOR_ADDRESS, float degree) {
    // Calculate the number of pulses based on the degree input
	int32_t pulses = degree_to_pulses(degree);

    // Step 1: Motor enable (Shutdown command)
    uint8_t step1[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x06}; // CRC: 0x161C
    send_modbus_command(step1, sizeof(step1));
    HAL_Delay(10);

    // Step 2: Switch on
    uint8_t step2[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x07}; // CRC: 0xD7DC
    send_modbus_command(step2, sizeof(step2));
    HAL_Delay(10);

    // Step 3: Enable operation
    uint8_t step3[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x0F}; // CRC: 0xD61A
    send_modbus_command(step3, sizeof(step3));
    HAL_Delay(10);

    // Step 4: Set to position mode
    uint8_t step4[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x60, 0x00, 0x01}; // CRC: 0x5614
    send_modbus_command(step4, sizeof(step4));
    HAL_Delay(10);

    // Step 5: Set target speed to 2000 rpm
    uint8_t step5[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x81, 0x00, 0x02, 0x04, 0x00, 0x00, 0x07, 0xD0};
    send_modbus_command(step5, sizeof(step5));
    HAL_Delay(10);

    // Step 6: Set acceleration to 150 rps/s
    uint8_t step6[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x83, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x96};
    send_modbus_command(step6, sizeof(step6));
    HAL_Delay(10);

    // Step 7: Set deceleration to 150 rps/s
    uint8_t step7[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x84, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x96};
    send_modbus_command(step7, sizeof(step7));
    HAL_Delay(10);

    // Step 8: Set running distance to the calculated number of pulses
    uint8_t step8[] = {MOTOR_ADDRESS, 0x10, 0x60, 0x7A, 0x00, 0x02, 0x04,
                      (uint8_t)((pulses >> 24) & 0xFF),
                      (uint8_t)((pulses >> 16) & 0xFF),
                      (uint8_t)((pulses >> 8) & 0xFF),
                      (uint8_t)(pulses & 0xFF)};
    send_modbus_command(step8, sizeof(step8));
    HAL_Delay(10);

    // Step 9: Enable operation (already enabled, but required in sequence)
    uint8_t step9[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x0F}; // CRC: 0xD61A
    send_modbus_command(step9, sizeof(step9));
    HAL_Delay(10);

    // Step 10: Set to relative position mode (bit 6 = 1 for relative mode)
    uint8_t step10[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x4F}; // CRC: 0xD7EA
    send_modbus_command(step10, sizeof(step10));
    HAL_Delay(10);

    // Step 11: Motion triggering
    uint8_t step11[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x5F}; // CRC: 0xD626
    send_modbus_command(step11, sizeof(step11));
    HAL_Delay(10);
}

void parse_command(char* cmd) {
  char *trimmed_cmd = cmd;
  char *p = trimmed_cmd;
  int motor_num, angle, chars_read;

  while (*p) {
      if (sscanf(p, " M%d %d%n", &motor_num, &angle, &chars_read) == 2) {
          if (motor_num >= 1 && motor_num <= 3) {
              test_absolute_position_control_degree(motor_num, (float)angle);
          }
          p += chars_read;
      } else {
          p++;
      }
  }

}
void set_current_position_as_zero(uint8_t MOTOR_ADDRESS) {
    // Step 1: Make sure the motor is in a stopped state first
    // Per the manual, register 2101h requires "Stop setting"

    // First ensure the motor is enabled but not moving
    // Shutdown command
    uint8_t cmd1[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x06};
    send_modbus_command(cmd1, sizeof(cmd1));
    HAL_Delay(20);

    // Switch on
    uint8_t cmd2[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x07};
    send_modbus_command(cmd2, sizeof(cmd2));
    HAL_Delay(20);

    // Enable operation
    uint8_t cmd3[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x40, 0x00, 0x0F};
    send_modbus_command(cmd3, sizeof(cmd3));
    HAL_Delay(20);

    // Set mode to position mode (to ensure proper operation)
    uint8_t cmd4[] = {MOTOR_ADDRESS, 0x06, 0x60, 0x60, 0x00, 0x01};
    send_modbus_command(cmd4, sizeof(cmd4));
    HAL_Delay(20);

    // Now clear the position to zero
    // Write value 1 to register 2101h to trigger the Clear Zero function
    uint8_t cmd5[] = {MOTOR_ADDRESS, 0x06, 0x21, 0x01, 0x00, 0x01};
    send_modbus_command(cmd5, sizeof(cmd5));
    HAL_Delay(50); // Longer delay to ensure the operation completes

    // Optional: Save parameters to EEPROM if needed
    // According to section 7, register 200Ah can be used to save parameters
    // uint8_t cmd6[] = {MOTOR_ADDRESS, 0x06, 0x20, 0x0A, 0x00, 0x02};
    // send_modbus_command(cmd6, sizeof(cmd6));
    // HAL_Delay(100); // Saving to EEPROM can take longer
}
typedef enum {
    BUTTON_RELEASED = 0,
    BUTTON_PRESSED = 1,
    BUTTON_DEBOUNCING = 2
} ButtonState;

typedef struct {
    GPIO_TypeDef* GPIOx;       /* GPIO Port */
    uint16_t GPIO_Pin;         /* GPIO Pin */
    ButtonState state;         /* Current button state */
    uint32_t debounce_time;    /* Time for debouncing in milliseconds */
    uint32_t last_tick;        /* Last time the button was checked */
    uint8_t pressed;           /* Flag to indicate a valid press was detected */
} Button_TypeDef;

void Button_Init(Button_TypeDef* button, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t debounce_time) {
    button->GPIOx = GPIOx;
    button->GPIO_Pin = GPIO_Pin;
    button->state = BUTTON_RELEASED;
    button->debounce_time = debounce_time;
    button->last_tick = 0;
    button->pressed = 0;
}

void Button_Update(Button_TypeDef* button) {
    uint32_t current_tick = HAL_GetTick();
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(button->GPIOx, button->GPIO_Pin);

    switch (button->state) {
        case BUTTON_RELEASED:
            if (pin_state == GPIO_PIN_SET) {
                button->state = BUTTON_DEBOUNCING;
                button->last_tick = current_tick;
            }
            break;

        case BUTTON_DEBOUNCING:
            if (current_tick - button->last_tick >= button->debounce_time) {
                /* Read the pin again after debounce period */
                if (HAL_GPIO_ReadPin(button->GPIOx, button->GPIO_Pin) == GPIO_PIN_SET) {
                    button->state = BUTTON_PRESSED;
                    button->pressed = 1; /* Set flag for a new press event */
                } else {
                    button->state = BUTTON_RELEASED;
                }
            }
            break;

        case BUTTON_PRESSED:
            if (pin_state == GPIO_PIN_RESET) {
                button->state = BUTTON_RELEASED;
            }
            break;
    }
}
uint8_t Button_IsPressed(Button_TypeDef* button) {
    if (button->pressed) {
        button->pressed = 0;
        return 1;
    }
    return 0;
}

void run_calibration(
		uint8_t MOTOR_ADDRESS,
		Button_TypeDef* confirm_button,
		Button_TypeDef* left_button,
		Button_TypeDef* right_button
) {
	if (Button_IsPressed(confirm_button)) {
		HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
		set_current_position_as_zero(MOTOR_ADDRESS);
		calibrate_state += 1;
		if (calibrate_state == 4) {
			calibrate_state = 0;
		}
	}
	else if (Button_IsPressed(left_button)) {
		HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
		test_relative_position_control_degree(MOTOR_ADDRESS, 5);
	}
	else if (Button_IsPressed(right_button)) {
		HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
		test_relative_position_control_degree(MOTOR_ADDRESS, -5);
	}
	else {
		HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
//		stop_motor(MOTOR_ADDRESS);
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

//	set_pulses_per_revolution(0x01, 1000);
//	HAL_Delay(1000);
  HAL_UART_Receive_IT(&huart2, rx_buffer, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Turn off LED

  Button_TypeDef confirmButton;
  Button_Init(&confirmButton, CONFIRM_PORT, CONFIRM_PIN, 200);
  Button_TypeDef leftButton;
  Button_Init(&leftButton, LEFT_PORT, LEFT_PIN, 200);
  Button_TypeDef rightButton;
  Button_Init(&rightButton, RIGHT_PORT, RIGHT_PIN, 200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Button_Update(&confirmButton);
	  Button_Update(&leftButton);
	  Button_Update(&rightButton);

	  if (calibrate_state == 0) {
		  if (cmd_received) {
			parse_command((char*)rx_buffer);
			rx_len = 0;
			cmd_received = 0;
		  }
	  }
	  else if (calibrate_state == 1) {
		  run_calibration(0x01, &confirmButton, &leftButton, &rightButton);
	  }
	  else if (calibrate_state == 2) {
		  run_calibration(0x02, &confirmButton, &leftButton, &rightButton);
	  }
	  else if (calibrate_state == 3) {
		  run_calibration(0x03, &confirmButton, &leftButton, &rightButton);
	  }
//	test_speed_control_1(0x01);
//	HAL_Delay(10000);
//	stop_motor(0x01);
//	HAL_Delay(1000);
//	test_absolute_position_control(0x01);
//	HAL_Delay(10000);
//	stop_motor(0x01);
//	HAL_Delay(1000);

//	test_absolute_position_control_degree(0x01, 0.0f);
//	HAL_Delay(5000);
//	test_absolute_position_control_degree(0x01, 120.0f);
//	HAL_Delay(5000);
//	test_absolute_position_control_degree(0x01, 240.0f);
//	HAL_Delay(5000);
//	test_absolute_position_control_degree(0x01, 360.0f);
//	HAL_Delay(10000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    // Check for end-of-command (e.g., newline)
    if (rx_buffer[rx_len] == '\n' || rx_len >= RX_BUFFER_SIZE - 1) {
      cmd_received = 1; // Flag to process command
      rx_buffer[rx_len] = '\0'; // Null-terminate
    } else {
      rx_len++;
    }
    // Restart interrupt-based reception
    HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_len], 1);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
