# STM32F407VGT6 with DC Motor Control using L293D Driver

This project demonstrates the interfacing of an STM32F407VGT6 microcontroller with a DC motor through an L293D motor driver. The setup uses PA0 and PA1 pins of the STM32F407VGT6 to control the direction of the motor, and PB1 is configured as Timer1 to control the motor's speed via PWM (Pulse Width Modulation) to the EnableA pin of the L293D.

# Features:

Direction Control:

PA0 (Input1 of L293D) and PA1 (Input2 of L293D) are used to set the direction of the motor.

By changing the states of these pins, the motor can rotate in either direction (clockwise or counterclockwise).

Speed Control:

PB1 (Timer1) is used to generate PWM signals to control the speed of the DC motor.

By adjusting the PWM duty cycle, the motor speed can be varied.

Motor Driver:

L293D motor driver is used to drive the DC motor.

EnableA pin is connected to PB1 (PWM output from Timer1), and Output1 and Output2 are connected to the DC motor.

# Pin Connections:

STM32F407VGT6 Pins:

PA0 → Input1 (L293D)

PA1 → Input2 (L293D)

PB1 → EnableA (L293D, PWM signal)

L293D Motor Driver Pins:

Input1 → PA0

Input2 → PA1

EnableA → PB1 (PWM)

Output1 → Motor Terminal 1

Output2 → Motor Terminal 2
