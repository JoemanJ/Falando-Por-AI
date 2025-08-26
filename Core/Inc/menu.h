#ifndef MENU_H
#define MENU_H

enum SELECT_FILTER {S_VADER,
			 S_PRIME,
			 S_LOW_PASS,
			 S_HIGH_PASS,
			 S_LOW_PITCH,
			 S_HIGH_PITCH,
//			 REVERB,
//			 DISTORTION,
//			 ECHO,
			 };

extern enum SELECT_FILTER SELECTED_FILTER;

void mainMenuOnLeft();
void mainMenuOnRight();
void mainMenuOnPress();
//void mainMenuDisplay();


#endif
