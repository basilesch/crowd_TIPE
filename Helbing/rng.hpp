//RNG / HASH THAT RETURN A VALUE IN [0..max[

#ifndef RNG_HPP
#define RNG_HPP

unsigned long wyhash64_x = 0xc0ff33; 
long rng(int max) {
  wyhash64_x += 0x60bee2bee120fc15;
  __uint128_t tmp;
  tmp = (__uint128_t) wyhash64_x * 0xa3b195354a39b70d;
  unsigned long m1 = (tmp >> 64) ^ tmp;
  tmp = (__uint128_t)m1 * 0x1b03738712fad5c9;
  unsigned long m2 = (tmp >> 64) ^ tmp;
  //return m2;
  return m2%max;
}

long whash(long h)  {
  h += 0x60bee2bee120fc15;
  __uint128_t tmp;
  tmp = (__uint128_t) h * 0xa3b195354a39b70d;
  unsigned long m1 = (tmp >> 64) ^ tmp;
  tmp = (__uint128_t)m1 * 0x1b03738712fad5c9;
  unsigned long m2 = (tmp >> 64) ^ tmp;
  return m2;
}

long wnext(long *h) {
  *h += 0x60bee2bee120fc15;
  __uint128_t tmp;
  tmp = (__uint128_t) *h * 0xa3b195354a39b70d;
  unsigned long m1 = (tmp >> 64) ^ tmp;
  tmp = (__uint128_t)m1 * 0x1b03738712fad5c9;
  unsigned long m2 = (tmp >> 64) ^ tmp;
  return m2;
}

long wseed() {
  wyhash64_x += 0x60bee2bee120fc15;
  __uint128_t tmp;
  tmp = (__uint128_t) wyhash64_x * 0xa3b195354a39b70d;
  unsigned long m1 = (tmp >> 64) ^ tmp;
  tmp = (__uint128_t)m1 * 0x1b03738712fad5c9;
  unsigned long m2 = (tmp >> 64) ^ tmp;
  return m2;
}

#endif
