#ifdef __cplusplus
extern "C" {
#endif

#include "words_def.h"

#define NO_WORD 0  // A word code that means to speak nothing.

struct utter_buffer {
#define NUM_UTTERS 15
  int utterances[NUM_UTTERS];  // word codes.
  int utter_pos;  // index in utterances of next free slot.
};

void appendUtterance(int utt, struct utter_buffer *utterbuf) ;

#define UTTER(u) appendUtterance(u, utterbuf)

void sayNumber(long n, struct utter_buffer *utterbuf);

#ifdef __cplusplus
}
#endif
