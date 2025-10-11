#ifndef UTIL_H
#define UTIL_H

#define BITS_PER_LONG 64
#define UL (unsigned long)
#define BIT(nr)         ((1UL) << (nr))
// #define GENMASK(h, l)   (((~((unsigned int)0)) << (l)) & (~(0) >> (32 - 1 - (h))))
#define GENMASK(h, l) \
	(((~UL(0)) - (UL(1) << (l)) + 1) & \
	 (~UL(0) >> (BITS_PER_LONG - 1 - (h))))

//  #define MIN(a,b) \
//    ({ __typeof__ (a) _a = (a); \
//        __typeof__ (b) _b = (b); \
//      _a > _b ? _b : _a; })

#endif
