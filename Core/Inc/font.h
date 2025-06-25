#ifndef __FONT_H
#define __FONT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Declare your font array
// Example for a 5x7 font. Each character is 5 bytes wide (columns), 7 pixels high.
// The array should cover ASCII characters from space (' ') to tilde ('~').
// (ASCII 32 to 126) -> 95 characters.
extern const unsigned char Font5x7[95][5];

#ifdef __cplusplus
}
#endif

#endif /* __FONT_H */
