#ifndef MATH_OPS_H
#define MATH_OPS_H

#define PI 3.14159265359f

#include "math.h"
#include "leg_Message.h"
// length of receive/transmit buffers
#define RX_LEN 66
#define TX_LEN 66

// length of outgoing/incoming messages
#define DATA_LEN 42
#define CMD_LEN  66

// Master CAN ID ///
#define CAN_ID 0x0


/// Value Limits ///
 #define P_MIN -95.5f
 #define P_MAX 95.5f
 #define V_MIN -45.0f
 #define V_MAX 45.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -18.0f
 #define T_MAX 18.0f
 

 
 /// Joint Soft Stops ///
 #define A_LIM_P 4.f
 #define A_LIM_N -4.f
 #define H_LIM_P 4.0f
 #define H_LIM_N -6.0f
 #define K_LIM_P 99.0f
 #define K_LIM_N -99.0f
 #define KP_SOFTSTOP 100.0f
 #define KD_SOFTSTOP 0.4f;

#define ENABLE_CMD 0xFFFF
#define DISABLE_CMD 0x1F1F

float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
void limit_norm(float *x, float *y, float limit);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
#endif
