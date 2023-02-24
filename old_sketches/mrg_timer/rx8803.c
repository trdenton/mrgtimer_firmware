#include <stdint.h>
#include "rx8803.h"

uint8_t bcd2dec(uint8_t in) {
	return 10*(in>>4) + (0x0F & in);
}

uint8_t dec2bcd(uint8_t in) {
	return ((in/10)<<4) + (0x0F & (in%10));
}

uint8_t rx8803_decode_1_100_S(uint8_t in)
{
	return bcd2dec(in);
}
uint8_t rx8803_decode_sec(uint8_t in)
{
	return bcd2dec(in & 0x7F);
}
uint8_t rx8803_decode_min(uint8_t in)
{
	return bcd2dec(in & 0x7F);
}
uint8_t rx8803_decode_hour(uint8_t in)
{
	return bcd2dec(in & 0x3F);
}
uint8_t rx8803_decode_week(uint8_t in)
{
	return (0x7F & in);
}
uint8_t rx8803_decode_day(uint8_t in)
{
	return bcd2dec(in & 0x3F);
}
uint8_t rx8803_decode_month(uint8_t in)
{
	return bcd2dec(in & 0x1F);
}
uint8_t rx8803_decode_year(uint8_t in)
{
	return bcd2dec(in);
}
uint16_t rx8803_decode_timer_counter(uint8_t tc1, uint8_t tc0)
{
	return tc0 + ((0x0f & tc1)<<8);
}

uint8_t rx8803_encode_1_100_S(uint8_t in)
{
	if (in <100)
		return dec2bcd(in);
	return 0;
}
uint8_t rx8803_encode_sec(uint8_t in)
{
	if (in < 60)
		return dec2bcd(in);
	return 0;
}
uint8_t rx8803_encode_min(uint8_t in)
{
	if (in < 60)
		return dec2bcd(in);
	return 0;
}
uint8_t rx8803_encode_hour(uint8_t in)
{
	if (in < 24)
		return dec2bcd(in);
	return 0;
}
uint8_t rx8803_encode_week(uint8_t in)
{
	return in & 0x7f;
}
uint8_t rx8803_encode_day(uint8_t in)
{
	if ( in < 31)
		return dec2bcd(in);
	return 0;
}
uint8_t rx8803_encode_month(uint8_t in)
{
	if (in < 12)
		return dec2bcd(in);
	return 0;
}
uint8_t rx8803_encode_year(uint8_t in)
{
	return dec2bcd(in);
}
uint16_t rx8803_encode_timer_counter(uint16_t in, uint8_t *tc1, uint8_t *tc0)
{
	if (tc0)
		*tc0 = in&0x00ff;
	if (tc1)
		*tc1 = (in >> 8);

	return 0;
}
