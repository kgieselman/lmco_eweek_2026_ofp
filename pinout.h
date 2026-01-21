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
 *****************************************************************************/
#pragma once


/* Defines ------------------------------------------------------------------*/
#define PINOUT_TODO (-1) // Invalid pin


/* Constants ----------------------------------------------------------------*/
// FlySky IBus interface
const int PIN_IBUS_TX = 4;
const int PIN_IBUS_RX = 5;

// Drive Train
const int PIN_MOTOR_FR_PWM   = PINOUT_TODO;
const int PIN_MOTOR_FR_DIR_A = PINOUT_TODO;
const int PIN_MOTOR_FR_DIR_B = PINOUT_TODO;
const int PIN_MOTOR_FR_ENC   = PINOUT_TODO;

const int PIN_MOTOR_FL_PWM   = PINOUT_TODO;
const int PIN_MOTOR_FL_DIR_A = PINOUT_TODO;
const int PIN_MOTOR_FL_DIR_B = PINOUT_TODO;
const int PIN_MOTOR_FL_ENC   = PINOUT_TODO;

const int PIN_MOTOR_RR_PWM   = PINOUT_TODO;
const int PIN_MOTOR_RR_DIR_A = PINOUT_TODO;
const int PIN_MOTOR_RR_DIR_B = PINOUT_TODO;
const int PIN_MOTOR_RR_ENC   = PINOUT_TODO;

const int PIN_MOTOR_RL_PWM   = PINOUT_TODO;
const int PIN_MOTOR_RL_DIR_A = PINOUT_TODO;
const int PIN_MOTOR_RL_DIR_B = PINOUT_TODO;
const int PIN_MOTOR_RL_ENC   = PINOUT_TODO;

// Collection Mechanism

// Deposit Mechanism

// Launcher Mechanism


/* EOF ----------------------------------------------------------------------*/
