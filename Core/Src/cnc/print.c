#include "cnc/grbl.h"

void print_uint8_base10(uint8_t n){
  uint8_t digit_a = 0;
  uint8_t digit_b = 0;
  if (n >= 100) { // 100-255
    digit_a = '0' + n % 10;
    n /= 10;
  }
  if (n >= 10) { // 10-99
    digit_b = '0' + n % 10;
    n /= 10;
  }
  TelnetWrite('0' + n);
  if (digit_b) { TelnetWrite(digit_b); }
  if (digit_a) { TelnetWrite(digit_a); }
}

void print_uint8_base2_ndigit(uint8_t n, uint8_t digits) {
  unsigned char buf[digits];
  uint8_t i = 0;

  for (; i < digits; i++) {
      buf[i] = n % 2 ;
      n /= 2;
  }

  for (; i > 0; i--)
	  TelnetWrite('0' + buf[i - 1]);
}

void print_uint32_base10(uint32_t n){
  if (n == 0) {
	  TelnetWrite('0');
    return;
  }

  unsigned char buf[10];
  uint8_t i = 0;

  while (n > 0) {
    buf[i++] = n % 10;
    n /= 10;
  }

  for (; i > 0; i--)
	  TelnetWrite('0' + buf[i-1]);
}


void printInteger(long n){
  if (n < 0) {
	  TelnetWrite('-');
    print_uint32_base10(-n);
  } else {
    print_uint32_base10(n);
  }
}

void printFloat(float n, uint8_t decimal_places){
  if (n < 0) {
	  TelnetWrite('-');
    n = -n;
  }

  uint8_t decimals = decimal_places;
  while (decimals >= 2) { // Quickly convert values expected to be E0 to E-4.
    n *= 100;
    decimals -= 2;
  }
  if (decimals) { n *= 10; }
  n += 0.5; // Add rounding factor. Ensures carryover through entire value.

  // Generate digits backwards and store in string.
  unsigned char buf[13];
  uint8_t i = 0;
  uint32_t a = (long)n;
  while(a > 0) {
    buf[i++] = (a % 10) + '0'; // Get digit
    a /= 10;
  }
  while (i < decimal_places) {
     buf[i++] = '0'; // Fill in zeros to decimal point for (n < 1)
  }
  if (i == decimal_places) { // Fill in leading zero, if needed.
    buf[i++] = '0';
  }

  // Print the generated string.
  for (; i > 0; i--) {
    if (i == decimal_places) {
    	TelnetWrite('.');
   }
    TelnetWrite(buf[i-1]);
  }
}
