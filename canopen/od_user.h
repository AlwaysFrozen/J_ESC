#ifndef CANOPEN_OD_USER_H
#define CANOPEN_OD_USER_H

#include "od_generated.h"

/* User-owned mapping. Call before starting communication. */
OD_Result od_user_init(const OD_Entry **first_missing);

#endif
