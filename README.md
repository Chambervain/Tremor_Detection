# Real-Time Tremor Detection System


## Project Description
This project measures, analyzes, and detects tremors from parkinson's patients, which simulated by a STM32F4-Discovery board. It uses gyroscopic data to perform FFT analysis and identifies tremor intensities.


## Usage Instruction
Led light:
The led light (LD3) on the board will light up when a tremor is detected. While the board is placed static and the tremor is not detected, the LD3 light will go off again.

Lcd screen:
The tremor intensity is range from 1 (lowest level tremor) to 100 (highest level tremor), and the intensity of tremor will be shown and presented on the screen of the board


## Setup Instruction
Connect the STM32F4-Discovery board with the computer. Compile and flash the provided code to the board using Mbed OS.


## Demo Video Link
Website of Youtube demo video: https://www.youtube.com/watch?v=vDDn2Coj7iM
