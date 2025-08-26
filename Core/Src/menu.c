#include "menu.h"

#include "LCD1602.h"
#include "filters.h"

static void mainMenuDisplay();

enum SELECT_FILTER SELECTED_FILTER = 0;

void mainMenuDisplay()
{
	lcd_put_cur(0, 0);
	//               0123456789abcdef
	if ((int) SELECTED_FILTER == (int) CURRENT_FILTER){
		lcd_send_string("  Filtro atual: ");
	}
	else{
		lcd_send_string("  Selecionar ?  ");
	}

	lcd_put_cur(1, 0);
	switch(SELECTED_FILTER)
	    {
	        case S_VADER:
	        	//               0123456789abcdef
	        	lcd_send_string("  Darth  Vader  ");
	            break;

	        case S_PRIME:
	        	//               0123456789abcdef
	        	lcd_send_string(" Optimus  Prime ");
	        	break;

	        case S_LOW_PASS:
	        	//               0123456789abcdef
	        	lcd_send_string("  Passa  Baixa  ");
	        	break;

	        case S_HIGH_PASS:
	        	//               0123456789abcdef
	        	lcd_send_string("   Passa Alta   ");
	        	break;

	        case S_LOW_PITCH:
	        	//               0123456789abcdef
	        	lcd_send_string("   Voz  Aguda   ");
	        	break;

	        case S_HIGH_PITCH:
	        	//               0123456789abcdef
	        	lcd_send_string("   Voz  Grave   ");
	        	break;
	        default:
	            break;
	    }
}

void mainMenuOnLeft()
{
	switch(SELECTED_FILTER)
		{
			case S_VADER:
				SELECTED_FILTER = S_HIGH_PITCH;
				break;

			case S_PRIME:
				SELECTED_FILTER = S_VADER;
				break;

			case S_LOW_PASS:
				SELECTED_FILTER = S_PRIME;
				break;

			case S_HIGH_PASS:
				SELECTED_FILTER = S_LOW_PASS;
				break;

			case S_LOW_PITCH:
				SELECTED_FILTER = S_HIGH_PASS;
				break;

			case S_HIGH_PITCH:
				SELECTED_FILTER = S_LOW_PITCH;
				break;
			default:
				break;
		}

	mainMenuDisplay();
}

void mainMenuOnRight()
{
	switch(SELECTED_FILTER)
		{
			case S_VADER:
				SELECTED_FILTER = S_PRIME;
				break;

			case S_PRIME:
				SELECTED_FILTER = S_LOW_PASS;
				break;

			case S_LOW_PASS:
				SELECTED_FILTER = S_HIGH_PASS;
				break;

			case S_HIGH_PASS:
				SELECTED_FILTER = S_LOW_PITCH;
				break;

			case S_LOW_PITCH:
				SELECTED_FILTER = S_HIGH_PITCH;
				break;

			case S_HIGH_PITCH:
				SELECTED_FILTER = S_VADER;
				break;
			default:
				break;
		}

	mainMenuDisplay();
}

void mainMenuOnPress()
{
	switch(SELECTED_FILTER)
		{
			case S_VADER:
				CURRENT_FILTER = DARTH_VADER;
				break;

			case S_PRIME:
				CURRENT_FILTER = OPTIMUS_PRIME;
				break;

			case S_LOW_PASS:
				CURRENT_FILTER = LOW_PASS;
				break;

			case S_HIGH_PASS:
				CURRENT_FILTER = HIGH_PASS;
				break;

			case S_LOW_PITCH:
				CURRENT_FILTER = LOW_PITCH;
				break;

			case S_HIGH_PITCH:
				CURRENT_FILTER = HIGH_PITCH;
				break;
			default:
				break;
		}

	mainMenuDisplay();
}

