#ifdef __cplusplus
extern "C" {
#endif

int8_t handle_packet(
  const uint8_t *buf,
  bool verbose,
  bool already_talking,
  unsigned long now_millis,
  struct utter_buffer *utterbuf);

void reset_on_powercycle();

#ifdef __cplusplus
}
#endif
