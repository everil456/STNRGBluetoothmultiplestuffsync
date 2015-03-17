#ifndef SYNCR_H__
#define SYNCR_H__

#include <stdio.h>
#include "link_layer.h"



/* Function Declarations */


uint32_t get_time();

uint32_t update_offsetp(uint32_t offsetp, uint32_t offseti);

uint32_t f_bstimei();

uint32_t s2i(char array[]);

uint32_t f_hhtimei();

uint32_t  calculate(uint32_t bstimei, uint32_t hhtimei, uint32_t offsetp);

#endif