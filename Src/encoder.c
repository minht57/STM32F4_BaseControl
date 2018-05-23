/**
 *	Author: minht57
 *  version 0.0.1
 */

/**
 * @file	encoder.c
 * @brief	encoder controller
 */
 
#include "encoder.h"

#define PI  3.141592654

void Encoder_CalVelocity(TARGET_VELOCITY_PARAMETERS * TargetVelocity)
{
  static float f_AngleVelocityLeft;
  static float f_AngleVelocityRight;
  
  f_AngleVelocityLeft = (2 * TargetVelocity->linear_velocity + TargetVelocity->angle_velocity * TargetVelocity->length) / 
                        (2 * TargetVelocity->radius);
  f_AngleVelocityRight = (2 * TargetVelocity->linear_velocity - TargetVelocity->angle_velocity * TargetVelocity->length) / 
                        (2 * TargetVelocity->radius);
  
  TargetVelocity->i16_Pulse_Left = (int16_t)(TargetVelocity->ratio * f_AngleVelocityLeft * PULSE_ENCODER_PER_ROUND / (2 * PI) * T_SAMPLE);
  TargetVelocity->i16_Pulse_Right = (int16_t)(TargetVelocity->ratio * f_AngleVelocityRight * PULSE_ENCODER_PER_ROUND / (2 * PI) * T_SAMPLE);
}


void Encoder_ReCalVelocity(TARGET_VELOCITY_PARAMETERS * ResultVelocity)
{
//  static float f_AngleVelocityLeft;
//  static float f_AngleVelocityRight;
//  
//  f_AngleVelocityLeft = (float)(ResultVelocity->i16_Pulse_Left * 2 * PI) / (ResultVelocity->ratio * PULSE_ENCODER_PER_ROUND);
//  f_AngleVelocityRight = (float)(ResultVelocity->i16_Pulse_Right * 2 * PI) / (ResultVelocity->ratio * PULSE_ENCODER_PER_ROUND);
//  
//  ResultVelocity->angle_velocity = (f_AngleVelocityRight - f_AngleVelocityLeft) * ResultVelocity->radius / ResultVelocity->length;
//  ResultVelocity->linear_velocity = (f_AngleVelocityLeft + f_AngleVelocityRight) * ResultVelocity->radius / 2;
  static float f_RatioAngleVelocity;
  static float f_RatioLinearVelocity;
  f_RatioAngleVelocity = (2 * PI * ResultVelocity->radius)/(ResultVelocity->ratio * PULSE_ENCODER_PER_ROUND * ResultVelocity->length  * T_SAMPLE);
  f_RatioLinearVelocity = (2 * PI * ResultVelocity->radius)/(ResultVelocity->ratio * PULSE_ENCODER_PER_ROUND * 2  * T_SAMPLE);
  
  ResultVelocity->angle_velocity = (-ResultVelocity->i16_Pulse_Right + ResultVelocity->i16_Pulse_Left) * f_RatioAngleVelocity;
  ResultVelocity->linear_velocity = (ResultVelocity->i16_Pulse_Right + ResultVelocity->i16_Pulse_Left) * f_RatioLinearVelocity;
}

//void Encoder_CopyVelocity(TARGET_VELOCITY_PARAMETERS * VelocitySrc, TARGET_VELOCITY_PARAMETERS * VelocityDest)
//{
//  VelocityDest->radius = VelocitySrc->radius;
//  VelocityDest->ratio = VelocitySrc->ratio;
//  VelocityDest->length = VelocitySrc->length;
//  VelocityDest->i16_Pulse_Left = VelocitySrc->i16_Pulse_Left;
//  VelocityDest->i16_Pulse_Right = VelocitySrc->i16_Pulse_Right;
//}
