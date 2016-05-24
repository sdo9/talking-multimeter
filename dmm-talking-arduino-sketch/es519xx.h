#ifdef __cplusplus
extern "C" {
#endif

int handle_packet(
  const uint8_t *buf,
  int verbose,
  int already_talking,
  long now_millis,
  struct utter_buffer *utterbuf);

void reset_on_powercycle();

#ifdef __cplusplus
}
#endif
