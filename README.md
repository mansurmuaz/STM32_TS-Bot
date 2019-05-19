# STM32_TS-Bot

## 1. Objective

We set out to experience all the development process of the embedded system design which includes electronic components as well as programming with C language and understand the working principle of touch screen, DC motors and to study communications on the board such as UART and SPI.

## 2. Project Description

The main purpose of this project is designing a car, which moves according to given directions from four button, that are represented with red rectangles, on a touchscreen or computer keyboard to be able to reach our objective. The car has two different drive modes, which are direct control mode and pre-programmed mode.

The car starts with direct control mode, and in this mode, the car will move forward, backward, left or right as soon as given any direction. And it will change the mode to pre-programmed mode when change mode button tapped. In the pre-programmed mode, the car will store the given directions, until tapping to the change mode button again. After tapping the change mode button, the car will follow the directions according to stored inputs. During the backward movement of the car, it will warn the surrounding with an alert sound in both modes.

Meanwhile, the car will measure the distance which it goes during its movement. It displays the distance data on the touchscreen, and also, it sends this data to the computer if data cable plugged in.

## 3. Technical Design

The main concept behind this project is to implement a car which drives to directions according to the inputs given from a touchscreen (2.83” 320x240 TFT LCD) on top of the EasyMX PRO V7 Micro Controller Unit (MCU) or a computer keyboard by UART communication through USB cable. A Smart Robot Car Kit which includes 2 DC motor and motor driver (L298N) has been used, in the project to control the movement of the car. For this purpose, PD8, PD9, PD10, PD11 pins are used to generate PWM signals for the motor driver. Table 1 explains the motor directions with given signals to mentioned pins.

Table 1 PWM Signals for Motor Directions:
![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/t1.png "Table 1 PWM Signals for Motor Directions")

The MCU board is assembled on top of the Car Kit. A 5V DC power bank was added to the car kit, to support MCU via USB cable. Additionally, a Lithium Polymer (Li-Po) 11.1V 850mAh 25C 3S battery was used to generate the power for motor driver. The buzzer on top of the MCU is used for giving an alert while moving backward. A button(PC0) added, to be able to change the drive mode between direct control mode and pre-programmed mode.

### 3.1 Metarials

* **Micro Controller Unit:** EasyMX PRO v7
* **Touch screen:** 2.83” 320x240 pixels TFT LCD
* **Buzzer:** PE14
* **Buttons:** PC0 pin
* **LEDs & Pins:** PD8, PD9, PD10, PD11, PB8, PB9
* **2 DC Motors:** 1.5V - 6V, max speed: 12000 rpm, current: 280mA
* **Motor Driver:** L298N input voltage: 11.1V, output voltage: 5V each motor
* **Switch:** Battery and motor driver connection
* **Battery:** 11.1V 850mAh 25C 3S Li-Po
* **Power Bank:** Output voltage: 5V via USB cable

## 4. Progress plan

![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/t2.png "Table 2 Progress Plan")
 
## 5. Work done

In general, all the work done by Burak Rüştü Duman and Mansur Muaz Ekici with together.

Since Burak is an Electrical & Electronics Engineering student, physical parts were under his responsibility. Those parts include assembling of the smart robot car kit, motor, motor driver, Li- Po battery, power bank, and EasyMX PRO v7 MCU board.

On the other hand, since Mansur is Computer Engineering student, algorithm design and code implementations were under his responsibility. For example, implementing the UART communication for both receiving and transmitting, displaying buttons with ILI9341 drivers on TFT LCD and getting input from the touchscreen by SPI communication, changing mode and movement algorithms, Systick timers, interrupts, GPIO inputs and outputs, etc.

The most challenging part was the fixing ILI9341 driver because it was so complicated and there was too much error to fix. We have tried to overcome it by searching examples from the internet but it did not help so much. Then, we worked hard on it and learnt how to solve and fix it. Additionally, we tried to speed down the motor by using resistors but the resistors are heated so much then, we connected them in parallel to reduce the power on each resistor. When we launched the car with minimum of 10 seconds, the resistors were again heated too much. Then, we decided to not reduce the speed of the motors by hardware. Then we speed down the motor with adding some delays as software.

## 6. Results and discussion

In the direct control mode, the accuracy is very high because the car was able to successfully move according to given directions. When the user press touchscreen or w, a, s, or d keys on the computer keyboard with connecting cable via UART connection, it is seen that there is a 100% success rate of moving the direction as forward, backward, left, or right.

In the pre-programmed mode, the car successfully stored the given directions, until tapping to the change mode button again. After tapping the change mode button, the car successfully followed the directions according to stored inputs. Also, as your finger is removed from the touch screen it stops its motion.

For safety and power saving, the turn on-off switch is added and worked successfully. All the cables and batteries are far from the user where under the board because the user can be able to control the car and see the board with ease.

## 7. Conclusion

The final performance of the project shows that it met all of our basic expectations and objectives. The car was able to move directions accordingly given inputs. MCU helps to convert directions to the necessary wheel rotations with using the motor driver. When beginning this project, the goal was just to move forward, backward, left or right according to given inputs from the touchscreen and as this was achieved we thought we could build upon it. In conclusion, we thought that, we have experienced most of the steps of embedded system design.

## 8. Figures

![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/1.png)
![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/2.png)
![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/3.png)
![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/4.png)
![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/5.png)
![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/6.png)
![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/7.png)
![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/8.png)
![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/9.png)
![alt text](https://github.com/mansurmuaz/STM32_TS-Bot/blob/master/Figures/10.png)
