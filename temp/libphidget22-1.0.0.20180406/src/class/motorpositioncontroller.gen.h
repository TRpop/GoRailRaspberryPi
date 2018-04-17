#ifndef _MOTORPOSITIONCONTROLLER_H_
#define _MOTORPOSITIONCONTROLLER_H_

/* Generated by WriteClassHeaderVisitor: Fri Apr 06 2018 16:02:34 GMT-0600 (MDT) */

typedef struct _PhidgetMotorPositionController *PhidgetMotorPositionControllerHandle;

/* Methods */
API_PRETURN_HDR PhidgetMotorPositionController_create(PhidgetMotorPositionControllerHandle *ch);
API_PRETURN_HDR PhidgetMotorPositionController_delete(PhidgetMotorPositionControllerHandle *ch);
API_PRETURN_HDR PhidgetMotorPositionController_addPositionOffset(PhidgetMotorPositionControllerHandle ch,
  double positionOffset);

/* Properties */
API_PRETURN_HDR PhidgetMotorPositionController_setAcceleration(PhidgetMotorPositionControllerHandle ch,
  double acceleration);
API_PRETURN_HDR PhidgetMotorPositionController_getAcceleration(PhidgetMotorPositionControllerHandle ch,
  double *acceleration);
API_PRETURN_HDR PhidgetMotorPositionController_getMinAcceleration(PhidgetMotorPositionControllerHandle ch,
  double *minAcceleration);
API_PRETURN_HDR PhidgetMotorPositionController_getMaxAcceleration(PhidgetMotorPositionControllerHandle ch,
  double *maxAcceleration);
API_PRETURN_HDR PhidgetMotorPositionController_setCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double currentLimit);
API_PRETURN_HDR PhidgetMotorPositionController_getCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *currentLimit);
API_PRETURN_HDR PhidgetMotorPositionController_getMinCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *minCurrentLimit);
API_PRETURN_HDR PhidgetMotorPositionController_getMaxCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *maxCurrentLimit);
API_PRETURN_HDR PhidgetMotorPositionController_setCurrentRegulatorGain(PhidgetMotorPositionControllerHandle ch, double currentRegulatorGain);
API_PRETURN_HDR PhidgetMotorPositionController_getCurrentRegulatorGain(PhidgetMotorPositionControllerHandle ch, double *currentRegulatorGain);
API_PRETURN_HDR PhidgetMotorPositionController_getMinCurrentRegulatorGain(PhidgetMotorPositionControllerHandle ch, double *minCurrentRegulatorGain);
API_PRETURN_HDR PhidgetMotorPositionController_getMaxCurrentRegulatorGain(PhidgetMotorPositionControllerHandle ch, double *maxCurrentRegulatorGain);
API_PRETURN_HDR PhidgetMotorPositionController_setDataInterval(PhidgetMotorPositionControllerHandle ch,
  uint32_t dataInterval);
API_PRETURN_HDR PhidgetMotorPositionController_getDataInterval(PhidgetMotorPositionControllerHandle ch,
  uint32_t *dataInterval);
API_PRETURN_HDR PhidgetMotorPositionController_getMinDataInterval(PhidgetMotorPositionControllerHandle ch,
  uint32_t *minDataInterval);
API_PRETURN_HDR PhidgetMotorPositionController_getMaxDataInterval(PhidgetMotorPositionControllerHandle ch,
  uint32_t *maxDataInterval);
API_PRETURN_HDR PhidgetMotorPositionController_setDeadBand(PhidgetMotorPositionControllerHandle ch,
  double deadBand);
API_PRETURN_HDR PhidgetMotorPositionController_getDeadBand(PhidgetMotorPositionControllerHandle ch,
  double *deadBand);
API_PRETURN_HDR PhidgetMotorPositionController_getDutyCycle(PhidgetMotorPositionControllerHandle ch,
  double *dutyCycle);
API_PRETURN_HDR PhidgetMotorPositionController_setEngaged(PhidgetMotorPositionControllerHandle ch,
  int engaged);
API_PRETURN_HDR PhidgetMotorPositionController_getEngaged(PhidgetMotorPositionControllerHandle ch,
  int *engaged);
API_PRETURN_HDR PhidgetMotorPositionController_setFanMode(PhidgetMotorPositionControllerHandle ch,
  Phidget_FanMode fanMode);
API_PRETURN_HDR PhidgetMotorPositionController_getFanMode(PhidgetMotorPositionControllerHandle ch,
  Phidget_FanMode *fanMode);
API_PRETURN_HDR PhidgetMotorPositionController_setIOMode(PhidgetMotorPositionControllerHandle ch,
  Phidget_EncoderIOMode IOMode);
API_PRETURN_HDR PhidgetMotorPositionController_getIOMode(PhidgetMotorPositionControllerHandle ch,
  Phidget_EncoderIOMode *IOMode);
API_PRETURN_HDR PhidgetMotorPositionController_setKd(PhidgetMotorPositionControllerHandle ch,
  double kd);
API_PRETURN_HDR PhidgetMotorPositionController_getKd(PhidgetMotorPositionControllerHandle ch,
  double *kd);
API_PRETURN_HDR PhidgetMotorPositionController_setKi(PhidgetMotorPositionControllerHandle ch,
  double ki);
API_PRETURN_HDR PhidgetMotorPositionController_getKi(PhidgetMotorPositionControllerHandle ch,
  double *ki);
API_PRETURN_HDR PhidgetMotorPositionController_setKp(PhidgetMotorPositionControllerHandle ch,
  double kp);
API_PRETURN_HDR PhidgetMotorPositionController_getKp(PhidgetMotorPositionControllerHandle ch,
  double *kp);
API_PRETURN_HDR PhidgetMotorPositionController_getPosition(PhidgetMotorPositionControllerHandle ch,
  double *position);
API_PRETURN_HDR PhidgetMotorPositionController_getMinPosition(PhidgetMotorPositionControllerHandle ch,
  double *minPosition);
API_PRETURN_HDR PhidgetMotorPositionController_getMaxPosition(PhidgetMotorPositionControllerHandle ch,
  double *maxPosition);
API_PRETURN_HDR PhidgetMotorPositionController_setRescaleFactor(PhidgetMotorPositionControllerHandle ch,
  double rescaleFactor);
API_PRETURN_HDR PhidgetMotorPositionController_getRescaleFactor(PhidgetMotorPositionControllerHandle ch,
  double *rescaleFactor);
API_PRETURN_HDR PhidgetMotorPositionController_setStallVelocity(PhidgetMotorPositionControllerHandle ch,
  double stallVelocity);
API_PRETURN_HDR PhidgetMotorPositionController_getStallVelocity(PhidgetMotorPositionControllerHandle ch,
  double *stallVelocity);
API_PRETURN_HDR PhidgetMotorPositionController_getMinStallVelocity(PhidgetMotorPositionControllerHandle ch, double *minStallVelocity);
API_PRETURN_HDR PhidgetMotorPositionController_getMaxStallVelocity(PhidgetMotorPositionControllerHandle ch, double *maxStallVelocity);
API_PRETURN_HDR PhidgetMotorPositionController_setTargetPosition(PhidgetMotorPositionControllerHandle ch,
  double targetPosition);
API_VRETURN_HDR PhidgetMotorPositionController_setTargetPosition_async(PhidgetMotorPositionControllerHandle ch, double targetPosition, Phidget_AsyncCallback fptr, void *ctx);
API_PRETURN_HDR PhidgetMotorPositionController_getTargetPosition(PhidgetMotorPositionControllerHandle ch,
  double *targetPosition);
API_PRETURN_HDR PhidgetMotorPositionController_setVelocityLimit(PhidgetMotorPositionControllerHandle ch,
  double velocityLimit);
API_PRETURN_HDR PhidgetMotorPositionController_getVelocityLimit(PhidgetMotorPositionControllerHandle ch,
  double *velocityLimit);
API_PRETURN_HDR PhidgetMotorPositionController_getMinVelocityLimit(PhidgetMotorPositionControllerHandle ch, double *minVelocityLimit);
API_PRETURN_HDR PhidgetMotorPositionController_getMaxVelocityLimit(PhidgetMotorPositionControllerHandle ch, double *maxVelocityLimit);

/* Events */
typedef void (CCONV *PhidgetMotorPositionController_OnDutyCycleUpdateCallback)(PhidgetMotorPositionControllerHandle ch, void *ctx, double dutyCycle);

API_PRETURN_HDR PhidgetMotorPositionController_setOnDutyCycleUpdateHandler(PhidgetMotorPositionControllerHandle ch, PhidgetMotorPositionController_OnDutyCycleUpdateCallback fptr, void *ctx);
typedef void (CCONV *PhidgetMotorPositionController_OnPositionChangeCallback)(PhidgetMotorPositionControllerHandle ch, void *ctx, double position);

API_PRETURN_HDR PhidgetMotorPositionController_setOnPositionChangeHandler(PhidgetMotorPositionControllerHandle ch, PhidgetMotorPositionController_OnPositionChangeCallback fptr, void *ctx);

#endif /* _MOTORPOSITIONCONTROLLER_H_ */
