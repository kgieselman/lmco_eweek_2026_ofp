/******************************************************************************
 * @file pinout.h
 * @brief Header file containing pin definitions for the project
 * 
 * @note All 30 GPIOs are PWM capable (16 max in use though)
 * 
 * @note PWM
 * | Slice | Chan A | Chan B | Alt Chan A | Alt Chan B |
 * | :---: | :---:  | :---:  | :---:      | :---:      |
 * | 0     | 0      | 1      | 16         | 17         |
 * | 1     | 2      | 3      | 18         | 19         |
 * | 2     | 4      | 5      | 20         | 21         |
 * | 3     | 6      | 7      | 22         | 23         |
 * | 4     | 8      | 9      | 24         | 25         |
 * | 5     | 10     | 11     | 26         | 27         |
 * | 6     | 12     | 13     | 28         | 29         |
 * | 7     | 14     | 15     | n/a        | n/a        |
 * 
 * @note Pico Orientation has pins GP0 - GP15 facing right
 *                            pins GP16 - GP28 facing left
 *****************************************************************************/
#pragma once


/* Defines ------------------------------------------------------------------*/
#define PINOUT_TODO (-1) // Invalid pin


/* Constants ----------------------------------------------------------------*/
// FlySky IBus interface
const int PIN_IBUS_TX = 4;
const int PIN_IBUS_RX = 5;

// Drive Train

// Pick from pins GP0 - G15
const int PIN_MOTOR_RR_PWM   =  9; // PWM Slice 4 Channel B
const int PIN_MOTOR_RR_DIR_A =  7;
const int PIN_MOTOR_RR_DIR_B =  6;

// Pick from pins GP0 - GP15
const int PIN_MOTOR_FR_PWM   =  8; // PWM Slice 4 Channel A
const int PIN_MOTOR_FR_DIR_A = 10;
const int PIN_MOTOR_FR_DIR_B = 11;

// Pick from pins GP16 - GP28
const int PIN_MOTOR_FL_PWM   = 20; // PWM Slice 2 Channel A
const int PIN_MOTOR_FL_DIR_A = 18;
const int PIN_MOTOR_FL_DIR_B = 19;

// Pick from pins GP16 - GP28
const int PIN_MOTOR_RL_PWM   = 21; // PWM Slice 2 Channel B
const int PIN_MOTOR_RL_DIR_A = 27;
const int PIN_MOTOR_RL_DIR_B = 26;


// Collection Mechanism

// Deposit Mechanism

// Launcher Mechanism


/* EOF ----------------------------------------------------------------------*/
