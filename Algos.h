//Algos Header File
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
uint32_t get_xyz_data( void );
signed int Kalman(signed int LED_val,signed int LED_val1,int order);
signed int Kalman2(signed int LED_val,signed int LED_val1,int order);
uint32_t Peak_Detect(uint32_t LED_val,int min_max);
signed char  physical_activity_count(signed char x,signed char  y,signed char  z);
signed int RLS(uint32_t LED_val,signed char accel);
signed int Peak_Detect2(signed int LED_val,int min_max);
signed int Peak_Detect3(signed int LED_val,int min_max);
signed int Peak_Detect4(signed int LED_val,int min_max);
extern int deltad;
extern int deltad2;
extern int deltad3;
extern int deltad4;