#ifndef __RX_8803_H__
#define __RX_8803_H__

// i2c address
# define RX8803_ADDR 

/*****
 *
 * REGISTERS
 *
 *****/

#define RX8803_REG_0_SEC 0x00
#define RX8803_REG_0_MIN 0x01
#define RX8803_REG_0_HOUR 0x02
#define RX8803_REG_0_WEEK 0x03
#define RX8803_REG_0_DAY 0x04
#define RX8803_REG_0_MONTH 0x05
#define RX8803_REG_0_YEAR 0x06
#define RX8803_REG_0_RAM 0x07
#define RX8803_REG_0_MIN_ALARM 0x08
#define RX8803_REG_0_HOUR_ALARM 0x09
#define RX8803_REG_0_WEEK_ALARM 0x0a
#define RX8803_REG_0_DAY_ALARM 0x0a
#define RX8803_REG_0_TIMER_COUNTER_0 0x0b
#define RX8803_REG_0_TIMER_COUNTER_1 0x0c
#define RX8803_REG_0_EXTENSION_REGISTER 0x0d
#define RX8803_REG_0_FLAG_REGISTER 0x0e
#define RX8803_REG_0_CONTROL_REGISTER 0x0f

#define RX8803_REG_1_1_100_SEC 0x10
#define RX8803_REG_1_SEC 0x11
#define RX8803_REG_1_MIN 0x12
#define RX8803_REG_1_HOUR 0x13
#define RX8803_REG_1_WEEK 0x14
#define RX8803_REG_1_DAY 0x15
#define RX8803_REG_1_MONTH 0x16
#define RX8803_REG_1_YEAR 0x17
#define RX8803_REG_1_MIN_ALARM 0x18
#define RX8803_REG_1_HOUR_ALARM 0x19
#define RX8803_REG_1_WEEK_ALARM 0x1a
#define RX8803_REG_1_DAY_ALARM 0x1a
#define RX8803_REG_1_TIMER_COUNTER_0 0x1b
#define RX8803_REG_1_TIMER_COUNTER_1 0x1c
#define RX8803_REG_1_EXTENSION_REGISTER 0x1d
#define RX8803_REG_1_FLAG_REGISTER 0x1e
#define RX8803_REG_1_CONTROL_REGISTER 0x1f

#define RX8803_REG_2_1_100_S_CP 0x20
#define RX8803_REG_2_SEC_CP 0x21
#define RX8803_REG_2_OSC_OFFSET 0x2c
#define RX8803_REG_2_EVENT_CONTROL 0x2f


// fields

typedef enum {
	TSEL0,
	TSEL1,
	FSEL0,
	FSEL1,
	TE,
	USEL,
	WADA,
	TEST
} EXTENSION_REGISTER_t;

typedef enum {
	VDET,
	VF,
	EVF,
	AF,
	TF,
	UF
} FLAG_REGISTER_t;

typedef enum {
	RESET,
	EIE=2,
	AIE,
	TIE,
	UIE,
	CSEL0,
	CSEL1
} CONTROL_REGISTER_t;

typedef enum {
	OFS0,
	OFS1,
	OFS2,
	OFS3
} OSC_OFFSET_REGISTER_t;

typedef enum {
	ERST,
	ET0=4,
	ET1,
	EHL,
	ECP
} EVENT_CONTOL_REGISTER_t;

typedef enum {
	AE=7
} ALARM_REGISTER_t;


uint8_t rx8803_decode_sec(uint8_t in);
uint8_t rx8803_decode_min(uint8_t in);
uint8_t rx8803_decode_hour(uint8_t in);
uint8_t rx8803_decode_week(uint8_t in);
uint8_t rx8803_decode_day(uint8_t in);
uint8_t rx8803_decode_month(uint8_t in);
uint8_t rx8803_decode_year(uint8_t in);
uint16_t rx8803_decode_timer_counter(uint8_t tc1, uint8_t tc0);

uint8_t rx8803_encode_1_100_S(uint8_t in);
uint8_t rx8803_encode_sec(uint8_t in);
uint8_t rx8803_encode_min(uint8_t in);
uint8_t rx8803_encode_hour(uint8_t in);
uint8_t rx8803_encode_week(uint8_t in);
uint8_t rx8803_encode_day(uint8_t in);
uint8_t rx8803_encode_month(uint8_t in);
uint8_t rx8803_encode_year(uint8_t in);
uint16_t rx8803_encode_timer_counter(uint16_t in, uint8_t *tc1, uint8_t *tc0);

uint8_t dec2bcd(uint8_t in);
uint8_t bcd2dec(uint8_t in);


#endif
