#ifndef __MOTOR_DRIVER_CONFIG_H
#define __MOTOR_DRIVER_CONFIG_H

/* 底盘驱动器类型 */
#define DRIVER_TYPE_ODRIVE  0
#define DRIVER_TYPE_MSSD    1
#define DRIVER_TYPE_OID     2

/*
 * Current chassis firmware uses the OID motor driver. Override this only for
 * an intentional driver migration in the compiler definitions.
 */
#ifndef MOTOR_DRIVER_USED
#define MOTOR_DRIVER_USED DRIVER_TYPE_OID
#endif

#if (MOTOR_DRIVER_USED != DRIVER_TYPE_ODRIVE) && \
    (MOTOR_DRIVER_USED != DRIVER_TYPE_MSSD) && \
    (MOTOR_DRIVER_USED != DRIVER_TYPE_OID)
#error "Unsupported MOTOR_DRIVER_USED"
#endif

#endif /* __MOTOR_DRIVER_CONFIG_H */
