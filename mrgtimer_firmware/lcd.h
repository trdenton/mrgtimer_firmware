#ifndef __LCD_H__
#define __LCD_H__
// assumes cursor is in the correct position
// converts t to MM:SS.sss
void lcd_print_time(long unsigned int t);
void lcd_init();
void lcd_clear();
void lcd_message(const char* str);
void lcd_result(int lane, long unsigned int ms);
void lcd_elapsed(int row, long unsigned int t);
#endif
