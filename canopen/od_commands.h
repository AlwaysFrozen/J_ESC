#ifndef OD_COMMANDS_H
#define OD_COMMANDS_H
#include "od.h"

OD_Result od_commands_bind(void);
/* Called after a successful configuration write, in the same critical section. */
void od_commands_config_changed(void);
bool od_commands_config_pending(void);
#endif
