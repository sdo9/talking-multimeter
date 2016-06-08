#ifdef __cplusplus
extern "C" {
#endif

#include "words_def.h"

#if WORD_LEN < 256
typedef unsigned char utter_t;
#else
typedef unsigned int utter_t;
#endif

#define NO_WORD 0  // A word code that means to speak nothing.

struct utter_buffer {
#define NUM_UTTERS 15
  utter_t utterances[NUM_UTTERS];  // word codes.
  char utter_pos;  // index in utterances of next free slot.
};

void appendUtterance(utter_t utt, struct utter_buffer *utterbuf);

#define UTTER(u) appendUtterance(u, utterbuf)

void sayNumber(long n, struct utter_buffer *utterbuf);

#ifdef __cplusplus
}
#endif
