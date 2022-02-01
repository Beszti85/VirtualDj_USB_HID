/*
 * button.h
 *
 *  Created on: 2022. jan. 4.
 *      Author: drCsabesz
 */

#ifndef SRC_BUTTON_H_
#define SRC_BUTTON_H_

#include <stdbool.h>
#include <stdio.h>

typedef struct
{

	uint8_t       DebounceCtr;
	bool          ActiveState;
	bool          IsPressed;
	bool          WasPressed;
	uint8_t       DebounceOn;
	uint8_t       DebounceOff;
	uint16_t      Padding;
} ButtonHandler_t;

void ButtonPressed(ButtonHandler_t* ptrButtonHandler, bool actState);

void ButtonReleased(ButtonHandler_t* ptrButtonHandler, bool actState);

#endif /* SRC_BUTTON_H_ */
