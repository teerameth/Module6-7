#include <stdint.h>

#include "PoorManFloat.h"

//
// This file can be renamed to a .ino and compiled as sketch to be run on the
// target e.g. arduino nano.
//

#ifdef TEST
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#define xprintf printf
#define test(x, msg) \
  if (!(x)) {        \
    puts(msg);       \
    assert(false);   \
  };
#else
#include <Arduino.h>
uint16_t error_cnt = 0;
char buffer[256];
#define xprintf(...)              \
  {                               \
    sprintf(buffer, __VA_ARGS__); \
    Serial.print(buffer);         \
  }
#define test(x, msg)          \
  if (!(x)) {                 \
    error_cnt++;              \
    Serial.print("ERROR: ");  \
    Serial.println(__LINE__); \
  } else {                    \
    Serial.print("OK: ");     \
    Serial.println(__LINE__); \
  };
#endif

void perform_test() {
  upm_float x;

  x = upm_from((uint8_t)1);
  test(x == 0x8080, "conversion error from uint8_t 1");
  x = upm_from((uint8_t)2);
  test(x == 0x8180, "conversion error from uint8_t 2");
  x = upm_from((uint8_t)3);
  test(x == 0x81c0, "conversion error from uint8_t 3");
  x = upm_from((uint8_t)4);
  xprintf("%x\n", x);
  test(x == 0x8280, "conversion error from uint8_t 4");
  x = upm_from((uint8_t)8);
  test(x == 0x8380, "conversion error from uint8_t 8");
  x = upm_from((uint8_t)16);
  test(x == 0x8480, "conversion error from uint8_t 16");
  x = upm_from((uint8_t)32);
  test(x == 0x8580, "conversion error from uint8_t 32");
  x = upm_from((uint8_t)64);
  test(x == 0x8680, "conversion error from uint8_t 64");
  x = upm_from((uint8_t)128);
  test(x == 0x8780, "conversion error from uint8_t 128");

  x = upm_from((uint16_t)1);
  test(x == 0x8080, "conversion error from uint16_t 1");
  x = upm_from((uint16_t)256);
  test(x == 0x8880, "conversion error from uint16_t 256");
  x = upm_from((uint16_t)512);
  test(x == 0x8980, "conversion error from uint16_t 512");
  x = upm_from((uint16_t)1024);
  test(x == 0x8a80, "conversion error from uint16_t 1024");
  x = upm_from((uint16_t)2048);
  test(x == 0x8b80, "conversion error from uint16_t 2048");
  x = upm_from((uint16_t)4096);
  test(x == 0x8c80, "conversion error from uint16_t 4096");
  x = upm_from((uint16_t)8192);
  test(x == 0x8d80, "conversion error from uint16_t 8192");
  x = upm_from((uint16_t)16384);
  test(x == 0x8e80, "conversion error from uint16_t 16384");
  x = upm_from((uint16_t)32768);
  test(x == 0x8f80, "conversion error from uint16_t 32768");

  x = upm_from((uint32_t)1);
  test(x == 0x8080, "conversion error from uint32_t 1");
  x = upm_from((uint32_t)3);
  xprintf("%x\n", x);
  test(x == 0x81c0, "conversion error from uint32_t 3");
  x = upm_from((uint32_t)5);
  test(x == 0x82a0, "conversion error from uint32_t 5");
  x = upm_from((uint32_t)65536);
  test(x == 0x9080, "conversion error from uint32_t 65536");
  x = upm_from((uint32_t)131072);
  test(x == 0x9180, "conversion error from uint32_t 131072");
  x = upm_from((uint32_t)262144);
  test(x == 0x9280, "conversion error from uint32_t 262144");
  x = upm_from((uint32_t)524288);
  test(x == 0x9380, "conversion error from uint32_t 524288");
  x = upm_from((uint32_t)1048576);
  test(x == 0x9480, "conversion error from uint32_t 1048576");
  x = upm_from((uint32_t)2097152);
  test(x == 0x9580, "conversion error from uint32_t 2097152");
  x = upm_from((uint32_t)4194304);
  test(x == 0x9680, "conversion error from uint32_t 4194304");
  x = upm_from((uint32_t)8388608);
  test(x == 0x9780, "conversion error from uint32_t 8388608");
  x = upm_from((uint32_t)16777216);
  test(x == 0x9880, "conversion error from uint32_t 16777216");
  x = upm_from((uint32_t)33554432);
  test(x == 0x9980, "conversion error from uint32_t 33554432");
  x = upm_from((uint32_t)67108864);
  test(x == 0x9a80, "conversion error from uint32_t 67108864");
  x = upm_from((uint32_t)134217728);
  test(x == 0x9b80, "conversion error from uint32_t 134217728");
  x = upm_from((uint32_t)268435456);
  test(x == 0x9c80, "conversion error from uint32_t 268435456");
  x = upm_from((uint32_t)536870912);
  test(x == 0x9d80, "conversion error from uint32_t 536870912");
  x = upm_from((uint32_t)1073741824);
  test(x == 0x9e80, "conversion error from uint32_t 1073741824");
  x = upm_from((uint32_t)2147483648);
  test(x == 0x9f80, "conversion error from uint32_t 2147483648");

  upm_float x1, x2, x3;
  x1 = upm_from((uint32_t)3);
  x2 = upm_from((uint32_t)5);
  x3 = upm_from((uint32_t)15);
  x = multiply(x1, x2);
  xprintf("%x*%x=%x\n", x1, x2, x);
  test(x == x3, "multiply error 3*5");
  test(x == 0x83f0, "multiply error 3*5");

  for (uint64_t i = 1; i < (1L << 16); i *= 5) {
    x = upm_from((uint16_t)i);
    uint16_t back = upm_to_u16(x);

    test(i >= back, "conversion error to/back");
    test((i & back) == back, "conversion error to/back");
  }

  for (uint64_t i = 1; i < (1L << 32); i *= 3) {
    x = upm_from((uint32_t)i);
    uint32_t back = upm_to_u32(x);

    test(i >= back, "conversion error to/back");
    test((i & back) == back, "conversion error to/back");
  }

  // Check overflows
  x1 = upm_from((uint32_t)0x0ffff);
  x2 = upm_from((uint32_t)0x1fffe);
  x = multiply(x1, x2);
  uint32_t back = upm_to_u32(x);
  xprintf("%x*%x=%x (back=%d)\n", x1, x2, x, back);
  test(x == 0xa0fe, "multiply error");
  test(back == 0xffffffff, "overflow not catched");

  x1 = upm_from((uint32_t)0x5555);
  x2 = upm_from((uint32_t)0x0055);
  x = divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (back=%d)\n", x1, x2, x, back);
  test(back == 0x0100, "wrong division");

  x1 = upm_from((uint32_t)0xf455);
  x2 = upm_from((uint32_t)0x0030);
  x = divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (%d)\n", x1, x2, x, back);
  test((back * 0x0030) <= 0xf455, "wrong division");
  test((back * 0x0031) > 0xf455, "wrong division");

  x = upm_from((uint32_t)16000000);
  xprintf("%x\n", x);
  test(x == 0x97f4, "conversion error from uint32_t 16000000");
  x = multiply(x, x);
  xprintf("%x\n", x);
  test(x == 0xafe8, "multiply error from uint32_t 16000000²");

  x1 = upm_from((uint32_t)0xf455);
  x2 = upm_from((uint32_t)0xf455);
  x = abs_diff(x1, x2);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%d)\n", x1, x2, x, back);
  test(back == 0, "wrong abs_diff");

  x1 = upm_from((uint32_t)0xf455);
  x2 = upm_from((uint32_t)0xf400);
  x = abs_diff(x1, x2);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%d)\n", x1, x2, x, back);
  test(back == 0, "wrong abs_diff");
  x = abs_diff(x2, x1);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%d)\n", x1, x2, x, back);
  test(back == 0, "wrong abs_diff");
  x = sum(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%d)\n", x1, x2, x, back);
  test(back == 0x1e800, "wrong sum");
  x = sum(x2, x1);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%d)\n", x1, x2, x, back);
  test(back == 0x1e800, "wrong sum");

  x1 = upm_from((uint32_t)0xf455);
  x2 = upm_from((uint32_t)0xf3ff);
  x = abs_diff(x1, x2);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%d)\n", x1, x2, x, back);
  test(back == 0x0100, "wrong abs_diff");
  x = abs_diff(x2, x1);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%d)\n", x1, x2, x, back);
  test(back == 0x0100, "wrong abs_diff");
  x = sum(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%d)\n", x1, x2, x, back);
  test(back == 0x1e600, "wrong sum");
  x = sum(x2, x1);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%d)\n", x1, x2, x, back);
  test(back == 0x1e600, "wrong sum");

  x1 = upm_from((uint32_t)0xf4555);
  x2 = upm_from((uint32_t)0x0f3ff);
  x = abs_diff(x1, x2);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%d)\n", x1, x2, x, back);
  test(back == 0xe5000, "wrong abs_diff");
  x = abs_diff(x2, x1);
  back = upm_to_u32(x);
  xprintf("|%x-%x|=%x (%d)\n", x1, x2, x, back);
  test(back == 0xe5000, "wrong abs_diff");
  x = sum(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%x)\n", x1, x2, x, back);
  test(back == 0x102000, "wrong sum");
  x = sum(x2, x1);
  back = upm_to_u32(x);
  xprintf("%x+%x=%x (%x)\n", x1, x2, x, back);
  test(back == 0x102000, "wrong sum");

  x1 = upm_from((uint32_t)0xf4555);
  x = shl(x1, 4);
  back = upm_to_u32(x);
  test(back == 0xf40000, "wrong shl");
  x1 = upm_from((uint32_t)0xf4555);
  x = shr(x1, 4);
  back = upm_to_u32(x);
  test(back == 0xf400, "wrong shr");

  x1 = upm_from((uint32_t)0x20000);
  x = sqrt(x1);
  back = upm_to_u32(x);
  xprintf("sqrt(%x)=0x%x (%d)  = 362 ?\n", x1, x, back);
  test(back == 362, "sqrt");

  x1 = upm_from((uint32_t)0x10000);
  x = sqrt(x1);
  back = upm_to_u32(x);
  xprintf("sqrt(%x)=0x%x (%x)\n", x1, x, back);
  test(back == 0x100, "sqrt");

  x1 = upm_from((uint16_t)59536);
  test(x1 == 0x8fe8, "upm_from");
  x1 = upm_from((uint32_t)59536);
  test(x1 == 0x8fe8, "upm_from");
  x1 = upm_from((uint32_t)(244L * 244L));
  test(x1 == 0x8fe8, "upm_from");
  x = sqrt(x1);
  back = upm_to_u32(x);
  xprintf("sqrt(%x)=0x%x (%x)\n", x1, x, back);
  test(back == 244, "sqrt");

  x1 = upm_from((uint32_t)(122 * 122));
  x = sqrt(x1);
  back = upm_to_u32(x);
  xprintf("sqrt(%x)=0x%x (%x)\n", x1, x, back);
  test(back == 122, "sqrt");

  x1 = upm_from((uint32_t)(174L * 174));
  x = sqrt(x1);
  back = upm_to_u32(x);
  xprintf("sqrt(%x)=0x%x (%x)\n", x1, x, back);
  test(back == 174, "sqrt");

  x1 = upm_from((uint32_t)4);
  x = sqrt(x1);
  back = upm_to_u32(x);
  xprintf("sqrt(%x)=0x%x (%x)\n", x1, x, back);
  test(back == 2, "sqrt");
  x = shl(sqrt(shr(x1, 2)), 1);
  back = upm_to_u32(x);
  xprintf("shl(sqrt(shr(%x,2)),1)=0x%x (%x)\n", x1, x, back);
  test(back == 2, "sqrt");
  x = shl(sqrt(shr(x1, 4)), 2);
  back = upm_to_u32(x);
  xprintf("shl(sqrt(shr(%x,4)),2)=0x%x (%x)\n", x1, x, back);
  x = shl(sqrt(shr(x1, 42)), 21);
  back = upm_to_u32(x);
  xprintf("shl(sqrt(shr(%x,42)),21)=0x%x (%x)\n", x1, x, back);
  test(back == 2, "sqrt");

  x1 = upm_from((uint32_t)250);
  x2 = upm_from((uint32_t)10000);
  x = divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (%d)\n", x1, x2, x, back);
  test(back == 0, "divide");
  x = multiply(x, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x*%x=%x (%d)\n", x1, x2, x2, x, back);
  test(back == 251, "divide");

  x1 = upm_from((uint32_t)250);
  x2 = upm_from((uint32_t)10000);
  x = divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (%d)\n", x1, x2, x, back);
  test(back == 0, "divide");
  x = shl(x, 10);
  back = upm_to_u32(x);
  xprintf("shl(%x/%x,10)=%x (%d)\n", x1, x2, x, back);
  test(back == 25, "divide/shl");

  x1 = upm_from((uint32_t)1600);
  x2 = upm_from((uint32_t)1000000);
  x = divide(x1, x2);
  back = upm_to_u32(x);
  xprintf("%x/%x=%x (%d)\n", x1, x2, x, back);
  test(back == 0, "divide");
  x = shl(x, 20);
  back = upm_to_u32(x);
  xprintf("shl(%x/%x,20)=%x (%d)\n", x1, x2, x, back);
  test(back == 1680, "divide/shl");
  x = shr(x, 20);
  back = upm_to_u32(x);
  xprintf("shr(shl(%x/%x,20),20)=%x (%d)\n", x1, x2, x, back);
  test(back == 0, "divide/shl");
  x = sqrt(x);
  back = upm_to_u32(x);
  xprintf("sqrt(%x/%x)=%x (%d)\n", x1, x2, x, back);
  test(back == 0, "divide/sqrt");
  x = multiply(x, x2);
  back = upm_to_u32(x);
  xprintf("sqrt(%x/%x)*%x=%x (%d)\n", x1, x2, x2, x, back);
  test(back == 39936, "divide/sqrt/multiply");

  x1 = upm_from((uint32_t)(174));
  x = multiply(x1, x1);
  back = upm_to_u32(x);
  xprintf("multiply(%x,%x)=0x%x (%d)\n", x1, x1, x, back);
  test(back == 30208, "multiply x*x");
  x = square(x1);
  back = upm_to_u32(x);
  xprintf("square(%x)=0x%x (%d)\n", x1, x, back);
  test(back == 30208, "square");

  x1 = upm_from((uint32_t)(6400));
  x = multiply(x1, x1);
  back = upm_to_u32(x);
  xprintf("multiply(%x,%x)=0x%x (%d)\n", x1, x1, x, back);
  test(back == 40894464, "multiply x*x");
  x = square(x1);
  back = upm_to_u32(x);
  xprintf("square(%x)=0x%x (%d)\n", x1, x, back);
  test(back == 40894464, "square");

#ifdef TEST
  xprintf("TEST_03 PASSED\n");
#else
  if (error_cnt == 0) {
    Serial.println("TEST PASSED");
  } else {
    Serial.print("TEST FAILED: ");
    Serial.print(error_cnt);
    Serial.println(" errors");
  }
#endif
}
#ifdef TEST
int main() { perform_test(); }
#else
void setup() {
  Serial.begin(115200);
  Serial.println("Start test...");
  perform_test();
}

void loop() {}
#endif
