#ifndef  __keyboard_control
#define  __keyboard_control

#include "robomaster_common.h"
#include "stm32f4xx_hal.h"
void Keyboard_Init(void);
void Keyboard_value_Init(void);
void keyboard_chassis(void);
void keyboard_mouse(void);


typedef struct {
	
	uint8_t r;
	uint8_t r_flag;
	
	uint8_t f;
	uint8_t f_flag;
	
	uint8_t g;
	uint8_t g_flag;
	
	uint8_t z;
	uint8_t z_flag;
	
	uint8_t x;
	uint8_t x_flag;
	
	uint8_t c;
	uint8_t c_flag;
	
	uint8_t v;
  uint8_t v_flag;	
	
	uint8_t b;
	uint8_t b_flag;
	
	uint8_t Ctrl;
	uint8_t Ctrl_flag;
	
	uint8_t mousel;
	uint8_t mousel_flag;
	
	uint8_t mouser;
	uint8_t mouser_flag;
	
	uint16_t speed;
	
	float yaw;
	float pitch;

	
}Keybroad_Control;

extern Keybroad_Control kb;

extern float kbshootspeed;

#endif


