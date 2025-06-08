## Jason without Fan (Kitty Board)

### Motor

1. TIM3->CCR2 => PE3 (FL) (A) (PWM)
   - Prescale = 15-1
   - ARR = 65535-1
2. TIM2->CCR1 => PD3 (RL) (B) (PWM)
   - Prescale = 15-1
   - ARR = 65535-1
3. TIM3->CCR1 => PE2 (FR) (C) (PWM)
   - Prescale = 15-1
   - ARR = 65535-1
4. TIM2->CCR2 => PD4 (RR) (D) (PWM)
   - Prescale = 15-1
   - ARR = 65535-1

### Encoder

1. TIM1 => FL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
2. TIM5 => RL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
3. TIM8 => FR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
4. TIM4 => RR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535

### TB6612 (for left motor)

1. MOTOR_LEFT_ENABLE => 3.3V / 5V (shorted)
2. MOTOR_FL_IN1 => PB0 (GPIO) (GPIO1 left 2nd)
   - GPIO Output
   - Pull Down
3. MOTOR_FL_IN2 => PE6 (GPIO) (GPIO1 right 2nd)
   - GPIO Output
   - Pull Down
4. MOTOR_RL_IN1 => PE4 (GPIO) (DIR)
   - GPIO Output
   - Pull Down
5. MOTOR_RL_IN2 => PE5 (GPIO) (STEP)
   - GPIO Output
   - Pull Down

### TB6612 (for right motor)

1. MOTOR_RIGHT_ENABLE => 3.3V / 5V (shorted)
2. MOTOR_FR_IN1 => PD0 (GPIO) (GPIO2 up 2nd)
   - GPIO Output
   - Pull Down
3. MOTOR_FR_IN2 => PC12 (GPIO) (GPIO2 down 2nd)
   - GPIO Output
   - Pull Down
4. MOTOR_RR_IN1 => PA15 (GPIO) (I2C down 2nd)
   - GPIO Output
   - Pull Down
5. MOTOR_RR_IN2 => PB7 (GPIO) (I2C down 1st)
   - GPIO Output
   - Pull Down

### Hiwonder Board
1. UART4_TX => PC10 (UART)
   - 9600 Bits/s
2. UART4_RX => PC11 (UART)
   - 9600 Bits/s

### Remote Controller

1. UART1_TX => PC4 (UART)
   - 115200 Bits/s
2. UART1_RX => PC5 (UART)
   - 115200 Bits/s

## Jason's Car with Fan (F4)

### Motor

1. TIM2->CCR1 => PA5 (FL) (PWM) (C)
   - Prescale = 42-1
   - ARR = 10000-1
2. TIM2->CCR2 => PB3 (RL) (PWM) (D)
   - Prescale = 42-1
   - ARR = 10000-1
3. TIM2->CCR3 => PA2 (FR) (PWM) (A)
   - Prescale = 42-1
   - ARR = 10000-1
4. TIM2->CCR4 => PA3 (RR) (PWM) (B)
   - Prescale = 42-1
   - ARR = 10000-1

### Encoder

1. TIM1 => FL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
2. TIM5 => RL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
3. TIM8 => FR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
4. TIM4 => RR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535

### External TB6612 Module (for left motor)

1. EN_L => 3.3V / 5V (shorted)
2. C_IN1 => PA4 (GPIO) 
   - GPIO Output
   - Pull Down
3. C_IN2 => PA6 (GPIO) 
   - GPIO Output
   - Pull Down
4. D_IN1 => PA7 (GPIO)
   - GPIO Output
   - Pull Down
5. D_IN2 => PC4 (GPIO) 
   - GPIO Output
   - Pull Down

### External TB6612 Module (for right motor)

1. EN_R => 3.3V / 5V (shorted)
2. A_IN1 => PC5 (GPIO)
   - GPIO Output
   - Pull Down
3. A_IN2 => PB0 (GPIO) 
   - GPIO Output
   - Pull Down
4. B_IN1 => PB1 (GPIO) 
   - GPIO Output
   - Pull Down
5. B_IN2 => PB2 (GPIO) 
   - GPIO Output
   - Pull Down

### Fan

1. TIM3->CCR1 => PB4 (PWM)
   - Prescale = 42-1
   - ARR = 20000-1

### Hiwonder Board

1. UART4_TX => PC10 (UART)
   - 9600 Bits/s
2. UART4_RX => PC11 (UART)
   - 9600 Bits/s

### Remote Controller

1. UART1_TX => PA9 (UART)
   - 115200 Bits/s
2. UART1_RX => PA10 (UART)
   - 115200 Bits/s

## Jason's Car with Fan

### Motor

1. TIM3->CCR4 => PE5 (FL) (C) (PWM) (PWM2B)
   - Prescale = 170-1
   - ARR = 10000-1
2. TIM2->CCR4 => PD6 (RL) (D) (PWM) (GPIO right 2nd)
   - Prescale = 170-1
   - ARR = 10000-1
3. TIM3->CCR1 => PE2 (FR) (B) (PWM) (PWM1C)
   - Prescale = 170-1
   - ARR = 10000-1
4. TIM3->CCR2 => PE3 (RR) (A) (PWM) (PWM2C)
   - Prescale = 170-1
   - ARR = 10000-1

### Encoder

1. TIM1 => FL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
2. TIM5 => RL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
3. TIM8 => FR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
4. TIM4 => RR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535

### External TB6612 Module (for left motor)

1. EN_L => 3.3V / 5V (shorted)
2. C_IN1 => PB0 (GPIO) (GPIO1 left 2nd)
   - GPIO Output
   - Pull Down
3. C_IN2 => PE6 (GPIO) (GPIO1 right 2nd)
   - GPIO Output
   - Pull Down
4. D_IN1 => PD7 (GPIO) (GPIO2 left 2nd)
   - GPIO Output
   - Pull Down
5. D_IN2 => PB3 (GPIO) (UART4 left 2nd)
   - GPIO Output
   - Pull Down

### External TB6612 Module (for right motor)

1. EN_R => 3.3V / 5V (shorted)
2. A_IN1 => PA15 (GPIO) (I2C down 2nd)
   - GPIO Output
   - Pull Down
3. A_IN2 => PB7 (GPIO) (I2C down 1st)
   - GPIO Output
   - Pull Down
4. B_IN1 => PD2 (GPIO) (UART3 up 2nd)
   - GPIO Output
   - Pull Down
5. B_IN2 => PC12 (GPIO) (UART2 down 2nd)
   - GPIO Output
   - Pull Down

### Fan

1. TIM3->CCR3 => PE4 (PWM) (PWM1B)
   - Prescale = 170-1
   - ARR = 10000-1

### Hiwonder Board

1. UART4_TX => PC10 (UART)
   - 9600 Bits/s
2. UART4_RX => PC11 (UART)
   - 9600 Bits/s

### Remote Controller

1. UART1_TX => PC4 (UART)
   - 115200 Bits/s
2. UART1_RX => PC5 (UART)
   - 115200 Bits/s

## Jason's Car without Fan

### Motor

1. TIM3->CCR4 => PE5 (FL) (C) (PWM) (PWM2B)
   - Prescale = 1-1
   - ARR = 16800-1
2. TIM2->CCR4 => PD6 (RL) (D) (PWM) (GPIO2 right 2nd)
   - Prescale = 1-1
   - ARR = 16800-1
3. TIM3->CCR2 => PE3 (FR) (B) (PWM) (PWM2C)
   - Prescale = 1-1
   - ARR = 16800-1
4. TIM3->CCR1 => PE2 (RR) (A) (PWM) (PWM1C)
   - Prescale = 1-1
   - ARR = 16800-1

### Encoder

1. TIM1 => FL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
2. TIM5 => RL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
3. TIM8 => FR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
4. TIM4 => RR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535

### External TB6612 Module (for left motor)

1. EN_L => 3.3V / 5V (shorted)
2. C_IN1 => PB0 (GPIO) (GPIO1 left 2nd)
   - GPIO Output
   - Pull Down
3. C_IN2 => PE6 (GPIO) (GPIO1 right 2nd)
   - GPIO Output
   - Pull Down
4. D_IN1 => PD7 (GPIO) (GPIO2 left 2nd)
   - GPIO Output
   - Pull Down
5. D_IN2 => PB3 (GPIO) (UART4 left 2nd)
   - GPIO Output
   - Pull Down

### External TB6612 Module (for right motor)

1. EN_R => 3.3V / 5V (shorted)
2. A_IN1 => PA15 (GPIO) (I2C down 2nd)
   - GPIO Output
   - Pull Down
3. A_IN2 => PB7 (GPIO) (I2C down 1st)
   - GPIO Output
   - Pull Down
4. B_IN1 => PD2 (GPIO) (UART3 up 2nd)
   - GPIO Output
   - Pull Down
5. B_IN2 => PC12 (GPIO) (UART2 down 2nd)
   - GPIO Output
   - Pull Down

### Hiwonder Board

1. UART4_TX => PC10 (UART)
   - 9600 Bits/s
2. UART4_RX => PC11 (UART)
   - 9600 Bits/s

### Remote Controller

1. UART1_TX => PC4 (UART)
   - 115200 Bits/s
2. UART1_RX => PC5 (UART)
   - 115200 Bits/s

## Tommy's Car

### Motor

1. TIM3->CCR4 => PE5 (FL) (C) (PWM)
   - Prescale = 1-1
   - ARR = 65535
2. TIM3->CCR2 => PE3 (RL) (D) (PWM)
   - Prescale = 1-1
   - ARR = 65535
3. TIM3->CCR1 => PE2 (FR) (B) (PWM)
   - Prescale = 1-1
   - ARR = 65535
4. TIM2->CCR3 => PD7 (RR) (A) (PWM)
   - Prescale = 1-1
   - ARR = 65535

### Encoder

1. TIM1 => FL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
2. TIM5 => RL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
3. TIM8 => FR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
4. TIM4 => RR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535

### External TB6612 Module (for left motor)

1. EN_L => 3.3V / 5V (shorted)
2. C_IN1 => PB13 (GPIO)
   - GPIO Output
   - Pull Down
3. C_IN2 => PB15 (GPIO)
   - GPIO Output
   - Pull Down
4. D_IN1 => PE6 (GPIO)
   - GPIO Output
   - Pull Down
5. D_IN2 => PD8 (GPIO)
   - GPIO Output
   - Pull Down

### External TB6612 Module (for right motor)

1. EN_R => 3.3V / 5V (shorted)
2. A_IN1 => PE11 (GPIO)
   - GPIO Output
   - Pull Down
3. A_IN2 => PE12 (GPIO)
   - GPIO Output
   - Pull Down
4. B_IN1 => PB10 (GPIO)
   - GPIO Output
   - Pull Down
5. B_IN2 => PE13 (GPIO)
   - GPIO Output
   - Pull Down

### Linear Actuator

1. LINEAR_ACT_1_1 => PC10 (GPIO)
   - GPIO Output
   - Pull Down
1. LINEAR_ACT_1_2 => PF2 (GPIO)
   - GPIO Output
   - Pull Down
1. LINEAR_ACT_2_1 => PF9 (GPIO)
   - GPIO Output
   - Pull Down
1. LINEAR_ACT_2_2 => PE14 (GPIO)
   - GPIO Output
   - Pull Down

### Remote Controller

1. UART1_TX => PC4 (UART)
   - 115200 Bits/s
2. UART1_RX => PC5 (UART)
   - 115200 Bits/s

## Kitty's Car

### Motor

1. TIM3->CCR2 => PE3 (FL) (A) (PWM)
   - Prescale = 15-1
   - ARR = 65535-1
2. TIM2->CCR1 => PD3 (RL) (B) (PWM)
   - Prescale = 15-1
   - ARR = 65535-1
3. TIM3->CCR1 => PE2 (FR) (C) (PWM)
   - Prescale = 15-1
   - ARR = 65535-1
4. TIM2->CCR2 => PD4 (RR) (D) (PWM)
   - Prescale = 15-1
   - ARR = 65535-1

### Encoder

1. TIM1 => FL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
2. TIM5 => RL encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
3. TIM8 => FR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535
4. TIM4 => RR encoder (TIMER)
   - Prescale = 1-1
   - ARR = 65535

### TB6612 (for left motor)

1. MOTOR_LEFT_ENABLE => 3.3V / 5V (shorted)
2. MOTOR_FL_IN1 => PB0 (GPIO) (GPIO1 left 2nd)
   - GPIO Output
   - Pull Down
3. MOTOR_FL_IN2 => PE6 (GPIO) (GPIO1 right 2nd)
   - GPIO Output
   - Pull Down
4. MOTOR_RL_IN1 => PC10 (GPIO) (UART2 left 2nd)
   - GPIO Output
   - Pull Down
5. MOTOR_RL_IN2 => PC11 (GPIO) (UART2 right 2nd)
   - GPIO Output
   - Pull Down

### TB6612 (for right motor)

1. MOTOR_RIGHT_ENABLE => 3.3V / 5V (shorted)
2. MOTOR_FR_IN1 => PD0 (GPIO) (GPIO2 up 2nd)
   - GPIO Output
   - Pull Down
3. MOTOR_FR_IN2 => PC12 (GPIO) (GPIO2 down 2nd)
   - GPIO Output
   - Pull Down
4. MOTOR_RR_IN1 => PA15 (GPIO) (I2C down 2nd)
   - GPIO Output
   - Pull Down
5. MOTOR_RR_IN2 => PB7 (GPIO) (I2C down 1st)
   - GPIO Output
   - Pull Down

### Stepper Motor

1. TIM3->CCR4 => PE5 (PWM)
   - Prescale = 1-1
   - ARR = 65535
2. STEPPER_DIR_GPIO_Port => PE4 (GPIO)
   - GPIO Output
   - Pull Down

### Servo Motor

1. TIM2->CCR4 => PD6 (Big Wheel's Servo) (PWM)
   - Prescale = 1-1
   - ARR = 65535

### Remote Controller

1. UART1_TX => PC4 (UART)
   - 115200 Bits/s
2. UART1_RX => PC5 (UART)
   - 115200 Bits/s
