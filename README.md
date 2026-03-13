# Introduction
This repository contains the firmware for a four-wheeled mobile robot controlled by an ESP32 and driven by an L298N motor driver. The project utilizes the Wemos Lolin32 Lite development board, selected for its compact form factor and integrated wireless capabilities (Wi-Fi and Bluetooth). By using the ESP32, the need for external communication modules is eliminated. Furthermore, the board's native support for Li-Po and Li-Ion battery management simplifies the power overhead.
This repository has three files in the `ITERATIONS` folder:
1. `WEMOS_LOLIN32_LITE_BOT_V1.ino`:
The robot operates in tank drive mode.
The left joystick Y-axis on the PS4 controller controls the left wheels.
The right joystick Y-axis controls the right wheels.
2. `WEMOS_LOLIN32_LITE_BOT_V2.ino`
The robot operates in arcade drive mode.
The left joystick Y-axis controls forward and backward movement.
The right joystick Y-axis controls clockwise and counterclockwise point turns.
3. `direct_mapping.ino`
This version uses the same arcade control scheme as V2, but replaces Arduino’s built-in map() function.
Instead of mapping joystick values from -512 to 512 into the range -1023 to 1023, the program performs direct value scaling by:
Taking the absolute value of the joystick input and doubling the magnitude. This approach reduces computational overhead slightly, making the program marginally faster.
# Chassis
The objective of the game is to maneuver a small rubber ball into your opponent's goal. A match lasts for 5 minutes. To assist in pushing the ball toward the opponent's goal, a small scoop is attached to the front of the robot.
The chassis utilized for this robot is an off-the-shelf model that includes four geared motors (TT 130) paired with four 60mm wheels. The chassis consists of two 2mm thick acrylic sheets, which are joined together using brass standoffs and M3 bolts. The acrylic plates feature cutouts designed to accommodate plastic brackets that hold the motors in place.
# Microcontroller
The bot is controlled by an ESP32 WEMOS LOLIN LITE 32, which I specifically chose for its wireless capabilities. Since the bot is operated using a PS4 controller, it was important to select a microcontroller that is Bluetooth-enabled. The Wemos LoLin Lite 32 also features a built-in battery management system (BMS), allowing it to be powered independently by a separate lithium cell. This setup prevents the microcontroller from resetting when the motors run at maximum speed and draw excessive current, which could cause the ESP32 to brown out if it shared the same power supply.
# Power supply
The robot is powered by a total of three lithium 18650 cells. Two of these cells are used to drive the motors and are connected to an L298N motor driver breakout board. However, the two lithium cells do not connect directly to the motor driver board. Instead, they are first connected to a generic 2S Battery Management System (BMS), which then connects to the motor driver board. The ESP32 development board receives its power from a separate lithium cell, which is connected via a JST connector.
