#ifndef PWM_CTRL_H
#define PWM_CTRL_H

//#define PWM_PIN GPIO_NUM_3
#define GPIN1 GPIO_NUM_3
#define GPIN2 GPIO_NUM_4


void motor_init(void);
void setSpeed(uint8_t speed);
void setDirection(uint8_t direction);
void brake(void);
void motor_set(int8_t speed);
#endif