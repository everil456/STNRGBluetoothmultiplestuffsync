#ifndef SYNCR_H__
#define SYNCR_H__

#include <stdio.h>
#include "link_layer.h"

/* Function Declarations */
uint32_t get_time();
uint32_t update_offsetp(uint32_t offsetp, uint32_t offseti);

#endif