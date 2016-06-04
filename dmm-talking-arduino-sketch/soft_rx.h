#ifdef __cplusplus
extern "C" {
#endif

void soft_rx_start(void);
void soft_rx_stop(void);
int soft_rx_get(void);
bool soft_rx_empty(void);

/* special codes returned by soft_rx_get() */
#define SOFT_RX_EMPTY       -1
#define SOFT_RX_INCOMPLETE  -2
#define SOFT_RX_BREAK       -3
#define SOFT_RX_ERROR       -4

/* Error counters */
extern unsigned int soft_rx_err_overflow;
extern unsigned int soft_rx_err_irq_sync;
extern unsigned int soft_rx_err_irq_time;
extern unsigned int soft_rx_err_stop_sync;

/* this must be provided somewhere else */
void _dbg_(char *string, bool do_val, long val, bool do_ln);

#ifdef __cplusplus
}
#endif
