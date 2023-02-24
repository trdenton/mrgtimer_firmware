#ifndef __LCD_H__
#define __LCD_H__
// assumes cursor is in the correct position
// converts t to MM:SS.sss
long unsigned int lcd_print_time(long unsigned int t);
void lcd_init();
void lcd_clear();
void lcd_message(const char* str);
#endif
