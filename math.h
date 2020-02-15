#ifndef _MATH_H
#define _MATH_H

#define ANGLE_MAX		180.0f
#define ANGLE_MIN		-180.0f
#define PI   				(3.14159265f)

#define LIMIT( x,min,max )  ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ))
#define ABS(x) 							((x)>0?(x):-(x))
#define DIV(Number, Prescaler, Threshold) ((Prescaler == 0.0f)? Threshold: (Number/Prescaler))
#define Power2(x) 					(x * x)
#define MAX_2(x,y)					((x >= y)? x: y)
#define MAX_3(x,y,z)				((z >= MAX_2(x,y))? z: MAX_2(x,y))

float sqrt(float number);
float translateAngle(float angle);
float sin(float angle);
float cos(float angle);
float tan(float angle);
float arctan1(float tan);
float arctan2(float x, float y);
float arcsin(float i);
float exp(float x);
#endif

