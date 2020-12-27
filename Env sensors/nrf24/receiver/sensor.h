struct clisensor {
  char id[7] = "RCV001";
  int16_t temp = 240; // C * 10
  int16_t rh = 570; // RH * 10
  int16_t p = 1000; // hPa
  int16_t r = 100; // 0-100 %
  int16_t g = 100; // 0-100 %
  int16_t b = 100; // 0-100 %
  int16_t c = 100; // 0-100 %
} clisensor1;
