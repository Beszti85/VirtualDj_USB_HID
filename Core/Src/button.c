/*
 * button.c
 *
 *  Created on: 2022. jan. 4.
 *      Author: drCsabesz
 */

#include "button.h"

void ButtonPressed(ButtonHandler_t* ptrButtonHandler, bool actState)
{
	// button not yet pressed
	if( ptrButtonHandler->IsPressed == false )
	{
		if( actState == ptrButtonHandler->ActiveState )
		{
			ptrButtonHandler->DebounceCtr++;
		}
		else
		if( ptrButtonHandler->DebounceCtr > 0u )
		{
			ptrButtonHandler->DebounceCtr--;
		}

		if( ptrButtonHandler->DebounceCtr > ptrButtonHandler->DebounceOn )
		{
			ptrButtonHandler->DebounceCtr = 0u;
			ptrButtonHandler->IsPressed = true;
		}
	}
}

void ButtonReleased(ButtonHandler_t* ptrButtonHandler, bool actState)
{
	// button pressed
	if ( ptrButtonHandler->IsPressed == true )
	{
		if( actState != ptrButtonHandler->ActiveState )
		{
			ptrButtonHandler->DebounceCtr++;
		}
		else
		if( ptrButtonHandler->DebounceCtr > 0u )
		{
			ptrButtonHandler->DebounceCtr--;
		}

		if( ptrButtonHandler->DebounceCtr > ptrButtonHandler->DebounceOff )
		{
			ptrButtonHandler->DebounceCtr = 0u;
			ptrButtonHandler->IsPressed = false;
		}
	}
}
