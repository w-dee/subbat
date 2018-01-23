#ifndef SETR_H_INCLUDED
#define SETR_H_INCLUDED

/* ------------------------------------------------------------------------- */

/*
	SYNOPSIS:
		SETR(register_name, bit, bit, ...)

	DESCRIPTION:
		register_name
			Any readable/writable AVR register
		bit
			Bit number (0..7) which you want to set or unset.
			If you want to set a bit, just put a bit number.
			Otherwise, if you want to UNSET the bit, put BITNOTed
			(ie. use ~ operator) bit number. See EXAMPLES below.
			You can specify bit numbers up to eight.
		
	EXAMPLES:
		To set bit:
			SETR(MCUCR, PUD); // set MCUCR PUD bit one
			SETR(GIMSK, INT1, INT0); // set both INT1 INT0 one
		To unset bit:
			SETR(MCUCR, ~PUD); // set MCUCR PUD bit zero
			SETR(GIMSK, ~INT1, ~INT0); // unset both INT1 INT0 
		You can mix these set/unset specifier at once:
			SETR(TCCR0B, CS02, ~CS01, CS00); // set CSO2:CS01:CS00 = 1:0:1

	NOTE:
		This macro heavily expects gcc's optimization. Gcc 4.3 is known to
		work well with this. Other versions will or will not.
*/


#define SETR_BIT_(n) \
	if(sizeof(bits) > (n)) { \
		if(bits[n] < 0) \
			and_bits += 1<<(~bits[n]); \
		else \
			or_bits += 1<<(bits[n]); \
	}

#define SETR(REG, ...) \
	do { \
		static const signed char bits[]= \
			{ __VA_ARGS__ }; \
		unsigned char and_bits = 0, or_bits = 0; \
		unsigned char p; \
		SETR_BIT_(0); \
		SETR_BIT_(1); \
		SETR_BIT_(2); \
		SETR_BIT_(3); \
		SETR_BIT_(4); \
		SETR_BIT_(5); \
		SETR_BIT_(6); \
		SETR_BIT_(7); \
		and_bits = ~and_bits; \
		if(or_bits == 0xff) { \
			(REG) = 0xff; \
		} else if(and_bits == 0x00) { \
			(REG) = 0x00; \
		} else { \
			p = (REG); \
			if(and_bits != 0xff) p &= and_bits; \
			if(or_bits  != 0x00) p |= or_bits;   \
			(REG) = p; \
		} \
	} while(0)

#define MDI 0
#define MDIP 1
#define MDO0 2
#define MDO1 3
#define MAI 4


#define MAP_START(FN) __attribute__((always_inline)) static void FN(const char port, const char num, const char mode) {

#define MAP_END }

#define MAP_CONCAT(A, B) A##B

#define MAP_PORT_A 'A'
#define MAP_PORT_B 'B'
#define MAP_PORT_C 'C'
#define MAP_PORT_D 'D'
#define MAP_PORT_E 'E'
#define MAP_PORT_F 'F'
#define MAP_PORT_G 'G'
#define MAP_PORT_H 'H'


#define MAP_MATCH(P, N, DIDR, DIDRB)  \
	if(port == MAP_CONCAT(MAP_PORT_, P) && num == N) \
	{ \
		if(mode == MDI)   { SETR(MAP_CONCAT(PORT, P), ~N); SETR(MAP_CONCAT(DDR, P), ~N); if(DIDRB != -1) SETR(DIDR, ~DIDRB); } \
		else \
		if(mode == MDIP)  { SETR(MAP_CONCAT(PORT, P), N);  SETR(MAP_CONCAT(DDR, P), ~N); if(DIDRB != -1) SETR(DIDR, ~DIDRB); } \
		else \
		if(mode == MDO0)  { SETR(MAP_CONCAT(PORT, P), ~N); SETR(MAP_CONCAT(DDR, P), N);  if(DIDRB != -1) SETR(DIDR, ~DIDRB); } \
		else \
		if(mode == MDO1)  { SETR(MAP_CONCAT(PORT, P), N);  SETR(MAP_CONCAT(DDR, P), N);  if(DIDRB != -1) SETR(DIDR, ~DIDRB); } \
		else \
		if(mode == MAI)   { SETR(MAP_CONCAT(PORT, P), ~N); SETR(MAP_CONCAT(DDR, P), ~N); if(DIDRB != -1) SETR(DIDR, DIDRB);  } \
	}


#endif
