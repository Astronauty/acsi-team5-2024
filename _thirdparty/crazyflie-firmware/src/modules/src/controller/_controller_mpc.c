
#include "stabilizer_types.h"

#include "controller_mpc.h"

#include "log.h"
#include "param.h"
#include "math3d.h"


void controllerMPCInit(void)
{
}

bool controllerMPCTest(void)
{
  return true;
}


void controllerMPC(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const stabilizerStep_t stabilizerStep)
{
  control->controlMode = controlModeForceTorque;
  control->thrustSi = 0.05; // force to provide control_thrust
  control->torqueX = 0;
  control->torqueY = 0;
  control->torqueZ = 0;
  
}
