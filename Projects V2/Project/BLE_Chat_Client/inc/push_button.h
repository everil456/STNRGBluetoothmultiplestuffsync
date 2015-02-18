#ifndef PUSH_BUTTON_H__
#define PUSH_BUTTON_H__

/* Parameter */
typedef int boolean;
#define true   1
#define false  0
#define BUTTON   GPIO_Pin_6     //Part of GPIOC
extern boolean buttonOK;

void initButton();

#endif