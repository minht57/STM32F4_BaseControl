/**
 *	Author: minht57
 *  version 0.0.1
 */

/**
 * @file	Encoder.h
 * @brief	Encoder
 */


#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

#define PULSE_ENCODER_PER_ROUND   3250

typedef struct {
  int16_t counter;
  int16_t result;
  float alpha;
}LPF_PARAMETERS;

typedef struct {
  float angle_velocity;
  float linear_velocity;
  float radius;
  float length;
  float ratio;
  int16_t i16_Pulse_Left;
  int16_t i16_Pulse_Right;
}TARGET_VELOCITY_PARAMETERS;

void Encoder_CalVelocity(TARGET_VELOCITY_PARAMETERS * TargetVelocity);
void Encoder_ReCalVelocity(TARGET_VELOCITY_PARAMETERS * ResultVelocity);
//void Encoder_CopyVelocity(TARGET_VELOCITY_PARAMETERS * VelocitySrc, TARGET_VELOCITY_PARAMETERS * VelocityDest);

#endif /* ENCODER_H_ */
