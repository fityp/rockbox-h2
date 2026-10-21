#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

bool bt_scan(void);
void bt_autostart(void);
void bt_set_enabled(bool enabled);
bool bt_is_available(void);
int bt_paired_devices(void);
int bt_pair_new_device(void);
bool bt_heap_info(void);
size_t bt_read_pcm(uint8_t *buf, size_t len);