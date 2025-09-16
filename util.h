#ifndef UTIL_H
#define UTIL_H

#define BIT(nr)			((1) << (nr))
#define GENMASK(h, l)   (((~((unsigned int)0)) << (l)) & (~(0) >> (32 - 1 - (h))))

#endif
