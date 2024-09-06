/*
 * pplib - a library for the Pico Held handheld
 *
 * Copyright (C) 2023 Daniel Kammer (daniel.kammer@web.de)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef BUTTONS_H
#define BUTTONS_H

/* ========================== includes ========================== */
#include <Arduino.h>

/* ========================= definitions ========================= */
/* --------------------- software assignment -------------------- */
#define DPAD_NONE     0b0000
#define DPAD_UP       0b0001
#define DPAD_LEFT     0b0010
#define DPAD_RIGHT    0b0100
#define DPAD_DOWN     0b1000

#define BUTTON_NONE   0b000
#define BUTTON_1      0b001
#define BUTTON_2      0b010
#define BUTTON_3      0b100

/* ====================== function declarations ====================== */
int ctrl_init();

void ctrl_recalib_analog();

uint16_t ctrl_dpad_state();
uint16_t ctrl_button_state();
int8_t ctrl_analog_x_state();
int8_t ctrl_analog_y_state();

#endif // BUTTONS_H