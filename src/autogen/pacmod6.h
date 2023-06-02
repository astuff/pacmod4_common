#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// DBC file version
#define VER_PACMOD6_MAJ (0U)
#define VER_PACMOD6_MIN (0U)

// include current dbc-driver compilation config
#include "pacmod6-config.h"

#ifdef PACMOD6_USE_DIAG_MONITORS
// This file must define:
// base monitor struct
// function signature for HASH calculation: (@GetFrameHash)
// function signature for getting system tick value: (@GetSystemTick)
#include "canmonitorutil.h"

#endif // PACMOD6_USE_DIAG_MONITORS


// def @GLOBAL_RPT CAN Message (16   0x10)
#define GLOBAL_RPT_IDE (0U)
#define GLOBAL_RPT_DLC (8U)
#define GLOBAL_RPT_CANID (0x10)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "Control Disabled"
  //  1 : "Control Enabled"
  uint8_t PACMOD_SYSTEM_ENABLED : 1;           //      Bits= 1

  //  0 : "Not Overridden"
  //  1 : "Overridden"
  uint8_t PACMOD_SYSTEM_OVERRIDE_ACTIVE : 1;   //      Bits= 1

  uint8_t USR_CAN_TIMEOUT : 1;                 //      Bits= 1

  uint8_t STR_CAN_TIMEOUT : 1;                 //      Bits= 1

  //  0 : "No Active CAN Timeout"
  //  1 : "Active CAN Timeout"
  uint8_t BRK_CAN_TIMEOUT : 1;                 //      Bits= 1

  uint8_t PACMOD_SUBSYSTEM_TIMEOUT : 1;        //      Bits= 1

  uint8_t VEH_CAN_TIMEOUT : 1;                 //      Bits= 1

  uint8_t PACMOD_SYSTEM_FAULT_ACTIVE : 1;      //      Bits= 1

  uint8_t CONFIG_FAULT_ACTIVE : 1;             //      Bits= 1

  uint16_t USR_CAN_READ_ERRORS;                //      Bits=16

#else

  //  0 : "Control Disabled"
  //  1 : "Control Enabled"
  uint8_t PACMOD_SYSTEM_ENABLED;               //      Bits= 1

  //  0 : "Not Overridden"
  //  1 : "Overridden"
  uint8_t PACMOD_SYSTEM_OVERRIDE_ACTIVE;       //      Bits= 1

  uint8_t USR_CAN_TIMEOUT;                     //      Bits= 1

  uint8_t STR_CAN_TIMEOUT;                     //      Bits= 1

  //  0 : "No Active CAN Timeout"
  //  1 : "Active CAN Timeout"
  uint8_t BRK_CAN_TIMEOUT;                     //      Bits= 1

  uint8_t PACMOD_SUBSYSTEM_TIMEOUT;            //      Bits= 1

  uint8_t VEH_CAN_TIMEOUT;                     //      Bits= 1

  uint8_t PACMOD_SYSTEM_FAULT_ACTIVE;          //      Bits= 1

  uint8_t CONFIG_FAULT_ACTIVE;                 //      Bits= 1

  uint16_t USR_CAN_READ_ERRORS;                //      Bits=16

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} GLOBAL_RPT_t;

// def @COMPONENT_RPT_00 CAN Message (32   0x20)
#define COMPONENT_RPT_00_IDE (0U)
#define COMPONENT_RPT_00_DLC (6U)
#define COMPONENT_RPT_00_CANID (0x20)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "PACMod"
  //  1 : "PACMini"
  //  2 : "PACMicro"
  uint8_t COMPONENT_TYPE : 4;                  //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t ACCEL : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t BRAKE : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t CRUISE_CONTROL_BUTTONS : 1;          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_LEFT : 1;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_RIGHT : 1;             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HAZARD_LIGHTS : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HEADLIGHT : 1;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HORN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t MEDIA_CONTROLS : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t PARKING_BRAKE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t SHIFT : 1;                           //      Bits= 1

  uint8_t SPRAY : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t STEERING : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t TURN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WIPER : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WATCHDOG : 1;                        //      Bits= 1

  uint8_t BRAKE_DECCEL : 1;                    //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                         //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                      //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CONFIG_FAULT : 1;                    //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CAN_TIMEOUT_FAULT : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT : 1;   //      Bits= 1

#else

  //  0 : "PACMod"
  //  1 : "PACMini"
  //  2 : "PACMicro"
  uint8_t COMPONENT_TYPE;                      //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t ACCEL;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t BRAKE;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t CRUISE_CONTROL_BUTTONS;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_LEFT;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_RIGHT;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HAZARD_LIGHTS;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HEADLIGHT;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HORN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t MEDIA_CONTROLS;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t PARKING_BRAKE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t SHIFT;                               //      Bits= 1

  uint8_t SPRAY;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t STEERING;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t TURN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WIPER;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WATCHDOG;                            //      Bits= 1

  uint8_t BRAKE_DECCEL;                        //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                             //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                          //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CONFIG_FAULT;                        //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CAN_TIMEOUT_FAULT;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT;       //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} COMPONENT_RPT_00_t;

// def @COMPONENT_RPT_01 CAN Message (33   0x21)
#define COMPONENT_RPT_01_IDE (0U)
#define COMPONENT_RPT_01_DLC (6U)
#define COMPONENT_RPT_01_CANID (0x21)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "PACMod"
  //  1 : "PACMini"
  //  2 : "PACMicro"
  uint8_t COMPONENT_TYPE : 4;                  //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t ACCEL : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t BRAKE : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t CRUISE_CONTROL_BUTTONS : 1;          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_LEFT : 1;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_RIGHT : 1;             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HAZARD_LIGHTS : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HEADLIGHT : 1;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HORN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t MEDIA_CONTROLS : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t PARKING_BRAKE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t SHIFT : 1;                           //      Bits= 1

  uint8_t SPRAY : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t STEERING : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t TURN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WIPER : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WATCHDOG : 1;                        //      Bits= 1

  uint8_t BRAKE_DECCEL : 1;                    //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                         //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                      //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CONFIG_FAULT : 1;                    //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CAN_TIMEOUT_FAULT : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT : 1;   //      Bits= 1

#else

  //  0 : "PACMod"
  //  1 : "PACMini"
  //  2 : "PACMicro"
  uint8_t COMPONENT_TYPE;                      //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t ACCEL;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t BRAKE;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t CRUISE_CONTROL_BUTTONS;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_LEFT;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_RIGHT;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HAZARD_LIGHTS;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HEADLIGHT;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HORN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t MEDIA_CONTROLS;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t PARKING_BRAKE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t SHIFT;                               //      Bits= 1

  uint8_t SPRAY;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t STEERING;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t TURN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WIPER;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WATCHDOG;                            //      Bits= 1

  uint8_t BRAKE_DECCEL;                        //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                             //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                          //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CONFIG_FAULT;                        //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CAN_TIMEOUT_FAULT;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT;       //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} COMPONENT_RPT_01_t;

// def @COMPONENT_RPT_02 CAN Message (34   0x22)
#define COMPONENT_RPT_02_IDE (0U)
#define COMPONENT_RPT_02_DLC (6U)
#define COMPONENT_RPT_02_CANID (0x22)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "PACMod"
  //  1 : "PACMini"
  //  2 : "PACMicro"
  uint8_t COMPONENT_TYPE : 4;                  //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t ACCEL : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t BRAKE : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t CRUISE_CONTROL_BUTTONS : 1;          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_LEFT : 1;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_RIGHT : 1;             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HAZARD_LIGHTS : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HEADLIGHT : 1;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HORN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t MEDIA_CONTROLS : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t PARKING_BRAKE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t SHIFT : 1;                           //      Bits= 1

  uint8_t SPRAY : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t STEERING : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t TURN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WIPER : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WATCHDOG : 1;                        //      Bits= 1

  uint8_t BRAKE_DECCEL : 1;                    //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                         //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                      //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CONFIG_FAULT : 1;                    //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CAN_TIMEOUT_FAULT : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT : 1;   //      Bits= 1

#else

  //  0 : "PACMod"
  //  1 : "PACMini"
  //  2 : "PACMicro"
  uint8_t COMPONENT_TYPE;                      //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t ACCEL;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t BRAKE;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t CRUISE_CONTROL_BUTTONS;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_LEFT;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_RIGHT;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HAZARD_LIGHTS;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HEADLIGHT;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HORN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t MEDIA_CONTROLS;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t PARKING_BRAKE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t SHIFT;                               //      Bits= 1

  uint8_t SPRAY;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t STEERING;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t TURN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WIPER;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WATCHDOG;                            //      Bits= 1

  uint8_t BRAKE_DECCEL;                        //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                             //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                          //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CONFIG_FAULT;                        //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CAN_TIMEOUT_FAULT;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT;       //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} COMPONENT_RPT_02_t;

// def @COMPONENT_RPT_03 CAN Message (35   0x23)
#define COMPONENT_RPT_03_IDE (0U)
#define COMPONENT_RPT_03_DLC (6U)
#define COMPONENT_RPT_03_CANID (0x23)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "PACMod"
  //  1 : "PACMini"
  //  2 : "PACMicro"
  uint8_t COMPONENT_TYPE : 4;                  //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t ACCEL : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t BRAKE : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t CRUISE_CONTROL_BUTTONS : 1;          //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_LEFT : 1;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_RIGHT : 1;             //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HAZARD_LIGHTS : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HEADLIGHT : 1;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HORN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t MEDIA_CONTROLS : 1;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t PARKING_BRAKE : 1;                   //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t SHIFT : 1;                           //      Bits= 1

  uint8_t SPRAY : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t STEERING : 1;                        //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t TURN : 1;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WIPER : 1;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WATCHDOG : 1;                        //      Bits= 1

  uint8_t BRAKE_DECCEL : 1;                    //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER : 4;                         //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT : 4;                      //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CONFIG_FAULT : 1;                    //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CAN_TIMEOUT_FAULT : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT : 1;   //      Bits= 1

#else

  //  0 : "PACMod"
  //  1 : "PACMini"
  //  2 : "PACMicro"
  uint8_t COMPONENT_TYPE;                      //      Bits= 4

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t ACCEL;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t BRAKE;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t CRUISE_CONTROL_BUTTONS;              //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_LEFT;                  //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t DASH_CONTROLS_RIGHT;                 //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HAZARD_LIGHTS;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HEADLIGHT;                           //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t HORN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t MEDIA_CONTROLS;                      //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t PARKING_BRAKE;                       //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t SHIFT;                               //      Bits= 1

  uint8_t SPRAY;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t STEERING;                            //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t TURN;                                //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WIPER;                               //      Bits= 1

  //  0 : "ABSENT"
  //  1 : "PRESENT"
  uint8_t WATCHDOG;                            //      Bits= 1

  uint8_t BRAKE_DECCEL;                        //      Bits= 1

  // Counter shall have the value of 0 with the first message transmission.  It shall increase by 1 with each subsequent message transmission up to and including the value of 0xF.  The next message transmission shall be 0, and this pattern shall repeat.
  uint8_t COUNTER;                             //      Bits= 4

  // The COMPLEMENT shall be the complement of the COUNTER.  For example, if COUNTER is 0x1011, then the COMPLEMENT is 0x0100.
  uint8_t COMPLEMENT;                          //      Bits= 4

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CONFIG_FAULT;                        //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t CAN_TIMEOUT_FAULT;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INTERNAL_SUPPLY_VOLTAGE_FAULT;       //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} COMPONENT_RPT_03_t;

// def @GLOBAL_CMD CAN Message (128  0x80)
#define GLOBAL_CMD_IDE (0U)
#define GLOBAL_CMD_DLC (1U)
#define GLOBAL_CMD_CANID (0x80)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t CLEAR_FAULTS : 1;                  //      Bits= 1

#else

  uint8_t CLEAR_FAULTS;                      //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} GLOBAL_CMD_t;

// def @ACCEL_CMD CAN Message (256  0x100)
#define ACCEL_CMD_IDE (0U)
#define ACCEL_CMD_DLC (3U)
#define ACCEL_CMD_CANID (0x100)
// signal: @ACCEL_CMD_ro
#define PACMOD6_ACCEL_CMD_ro_CovFactor (0.001000)
#define PACMOD6_ACCEL_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_ACCEL_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint16_t ACCEL_CMD_ro;                     //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ACCEL_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint16_t ACCEL_CMD_ro;                     //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ACCEL_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} ACCEL_CMD_t;

// def @BRAKE_CMD CAN Message (260  0x104)
#define BRAKE_CMD_IDE (0U)
#define BRAKE_CMD_DLC (3U)
#define BRAKE_CMD_CANID (0x104)
// signal: @BRAKE_CMD_ro
#define PACMOD6_BRAKE_CMD_ro_CovFactor (0.001000)
#define PACMOD6_BRAKE_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_BRAKE_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint16_t BRAKE_CMD_ro;                     //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t BRAKE_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint16_t BRAKE_CMD_ro;                     //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t BRAKE_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} BRAKE_CMD_t;

// def @CRUISE_CONTROL_BUTTONS_CMD CAN Message (264  0x108)
#define CRUISE_CONTROL_BUTTONS_CMD_IDE (0U)
#define CRUISE_CONTROL_BUTTONS_CMD_DLC (2U)
#define CRUISE_CONTROL_BUTTONS_CMD_CANID (0x108)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  uint8_t CRUISE_CONTROL_BUTTON;             //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  uint8_t CRUISE_CONTROL_BUTTON;             //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} CRUISE_CONTROL_BUTTONS_CMD_t;

// def @DASH_CONTROLS_LEFT_CMD CAN Message (268  0x10c)
#define DASH_CONTROLS_LEFT_CMD_IDE (0U)
#define DASH_CONTROLS_LEFT_CMD_DLC (2U)
#define DASH_CONTROLS_LEFT_CMD_CANID (0x10c)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} DASH_CONTROLS_LEFT_CMD_t;

// def @DASH_CONTROLS_RIGHT_CMD CAN Message (272  0x110)
#define DASH_CONTROLS_RIGHT_CMD_IDE (0U)
#define DASH_CONTROLS_RIGHT_CMD_DLC (2U)
#define DASH_CONTROLS_RIGHT_CMD_CANID (0x110)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t DASH_CONTROLS_BUTTON;              //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} DASH_CONTROLS_RIGHT_CMD_t;

// def @HAZARD_LIGHTS_CMD CAN Message (276  0x114)
#define HAZARD_LIGHTS_CMD_IDE (0U)
#define HAZARD_LIGHTS_CMD_DLC (2U)
#define HAZARD_LIGHTS_CMD_CANID (0x114)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t HAZARD_LIGHTS_CMD : 1;             //      Bits= 1

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t HAZARD_LIGHTS_CMD;                 //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} HAZARD_LIGHTS_CMD_t;

// def @HEADLIGHT_CMD CAN Message (280  0x118)
#define HEADLIGHT_CMD_IDE (0U)
#define HEADLIGHT_CMD_DLC (2U)
#define HEADLIGHT_CMD_CANID (0x118)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  uint8_t HEADLIGHT_CMD;                     //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  uint8_t HEADLIGHT_CMD;                     //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} HEADLIGHT_CMD_t;

// def @HORN_CMD CAN Message (284  0x11c)
#define HORN_CMD_IDE (0U)
#define HORN_CMD_DLC (2U)
#define HORN_CMD_CANID (0x11c)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  uint8_t HORN_CMD : 1;                      //      Bits= 1

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  uint8_t HORN_CMD;                          //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} HORN_CMD_t;

// def @MEDIA_CONTROLS_CMD CAN Message (288  0x120)
#define MEDIA_CONTROLS_CMD_IDE (0U)
#define MEDIA_CONTROLS_CMD_DLC (2U)
#define MEDIA_CONTROLS_CMD_CANID (0x120)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  uint8_t MEDIA_CONTROLS_CMD;                //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  uint8_t MEDIA_CONTROLS_CMD;                //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} MEDIA_CONTROLS_CMD_t;

// def @PARKING_BRAKE_CMD CAN Message (292  0x124)
#define PARKING_BRAKE_CMD_IDE (0U)
#define PARKING_BRAKE_CMD_DLC (2U)
#define PARKING_BRAKE_CMD_CANID (0x124)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint8_t PARKING_BRAKE_CMD : 1;             //      Bits= 1

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint8_t PARKING_BRAKE_CMD;                 //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} PARKING_BRAKE_CMD_t;

// def @SHIFT_CMD CAN Message (296  0x128)
#define SHIFT_CMD_IDE (0U)
#define SHIFT_CMD_DLC (2U)
#define SHIFT_CMD_CANID (0x128)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  // FORWARD is also HIGH on vehicles with LOW/HIGH, PARK and LOW only available on certain Vehicles.
  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  7 : "NONE"
  uint8_t SHIFT_CMD;                         //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  // FORWARD is also HIGH on vehicles with LOW/HIGH, PARK and LOW only available on certain Vehicles.
  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  7 : "NONE"
  uint8_t SHIFT_CMD;                         //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} SHIFT_CMD_t;

// def @STEERING_CMD CAN Message (300  0x12c)
#define STEERING_CMD_IDE (0U)
#define STEERING_CMD_DLC (5U)
#define STEERING_CMD_CANID (0x12c)
// signal: @POSITION_ro
#define PACMOD6_POSITION_ro_CovFactor (0.001000)
#define PACMOD6_POSITION_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @ROTATION_RATE_ro
#define PACMOD6_ROTATION_RATE_ro_CovFactor (0.001000)
#define PACMOD6_ROTATION_RATE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_ROTATION_RATE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  int16_t POSITION_ro;                       //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t POSITION_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                 //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  int16_t POSITION_ro;                       //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t POSITION_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                 //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} STEERING_CMD_t;

// def @TURN_CMD CAN Message (304  0x130)
#define TURN_CMD_IDE (0U)
#define TURN_CMD_DLC (2U)
#define TURN_CMD_CANID (0x130)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  uint8_t TURN_SIGNAL_CMD;                   //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  uint8_t TURN_SIGNAL_CMD;                   //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} TURN_CMD_t;

// def @WIPER_CMD CAN Message (308  0x134)
#define WIPER_CMD_IDE (0U)
#define WIPER_CMD_DLC (2U)
#define WIPER_CMD_CANID (0x134)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  255 : "High"
  //  254 : "Medium"
  //  253 : "Low"
  //  10 : "Intermittent 10"
  //  9 : "Intermittent 9"
  //  8 : "Intermittent 8"
  //  7 : "Intermittent 7"
  //  6 : "Intermittent 6"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  uint8_t WIPER_CMD;                         //      Bits= 8

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  255 : "High"
  //  254 : "Medium"
  //  253 : "Low"
  //  10 : "Intermittent 10"
  //  9 : "Intermittent 9"
  //  8 : "Intermittent 8"
  //  7 : "Intermittent 7"
  //  6 : "Intermittent 6"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  uint8_t WIPER_CMD;                         //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} WIPER_CMD_t;

// def @SPRAYER_CMD CAN Message (312  0x138)
#define SPRAYER_CMD_IDE (0U)
#define SPRAYER_CMD_DLC (2U)
#define SPRAYER_CMD_CANID (0x138)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  uint8_t SPRAYER_CMD : 1;                   //      Bits= 1

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  //  0 : "NOT_SPRAYING"
  //  1 : "SPRAYING"
  uint8_t SPRAYER_CMD;                       //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} SPRAYER_CMD_t;

// def @BRAKE_DECCEL_CMD CAN Message (316  0x13c)
#define BRAKE_DECCEL_CMD_IDE (0U)
#define BRAKE_DECCEL_CMD_DLC (4U)
#define BRAKE_DECCEL_CMD_CANID (0x13c)
// signal: @BRAKE_DECCEL_CMD_ro
#define PACMOD6_BRAKE_DECCEL_CMD_ro_CovFactor (0.001000)
#define PACMOD6_BRAKE_DECCEL_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_BRAKE_DECCEL_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLE : 1;                        //      Bits= 1

  uint8_t IGNORE_OVERRIDES : 1;              //      Bits= 1

  uint8_t CLEAR_OVERRIDE : 1;                //      Bits= 1

  uint16_t BRAKE_DECCEL_CMD_ro;              //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t BRAKE_DECCEL_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

  //  0 : "NO_ENDURANCE_BRAKE_INTEGRATION_ALLOWED"
  //  1 : "ONLY_ENDURANCE_BRAKES_ALLOWED"
  //  2 : "ENDURANCE_BRAKE_INTEGRATION_ALLOWED"
  uint8_t XBR_EBI_MODE : 2;                  //      Bits= 2

  //  0 : "HIGHEST_PRIORITY"
  //  1 : "HIGH_PRIORITY"
  //  2 : "MEDIUM_PRIORITY"
  //  3 : "LOW_PRIORITY"
  uint8_t XBR_PRIORITY : 2;                  //      Bits= 2

  //  0 : "OVERRIDE_DISABLE"
  //  1 : "ACCELERATION_CONTROL_WITH_ADDITION_MODE"
  //  2 : "ACCELERATION_CONTROL_WITH_MAXIMUM_MODE"
  uint8_t XBR_CONTROL_MODE : 2;              //      Bits= 2

#else

  uint8_t ENABLE;                            //      Bits= 1

  uint8_t IGNORE_OVERRIDES;                  //      Bits= 1

  uint8_t CLEAR_OVERRIDE;                    //      Bits= 1

  uint16_t BRAKE_DECCEL_CMD_ro;              //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t BRAKE_DECCEL_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

  //  0 : "NO_ENDURANCE_BRAKE_INTEGRATION_ALLOWED"
  //  1 : "ONLY_ENDURANCE_BRAKES_ALLOWED"
  //  2 : "ENDURANCE_BRAKE_INTEGRATION_ALLOWED"
  uint8_t XBR_EBI_MODE;                      //      Bits= 2

  //  0 : "HIGHEST_PRIORITY"
  //  1 : "HIGH_PRIORITY"
  //  2 : "MEDIUM_PRIORITY"
  //  3 : "LOW_PRIORITY"
  uint8_t XBR_PRIORITY;                      //      Bits= 2

  //  0 : "OVERRIDE_DISABLE"
  //  1 : "ACCELERATION_CONTROL_WITH_ADDITION_MODE"
  //  2 : "ACCELERATION_CONTROL_WITH_MAXIMUM_MODE"
  uint8_t XBR_CONTROL_MODE;                  //      Bits= 2

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} BRAKE_DECCEL_CMD_t;

// def @ACCEL_RPT CAN Message (512  0x200)
#define ACCEL_RPT_IDE (0U)
#define ACCEL_RPT_DLC (8U)
#define ACCEL_RPT_CANID (0x200)
// signal: @MANUAL_INPUT_ro
#define PACMOD6_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD6_MANUAL_INPUT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD6_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD6_COMMANDED_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD6_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD6_OUTPUT_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} ACCEL_RPT_t;

// def @ACCEL_CMD_LIMIT_RPT CAN Message (513  0x201)
#define ACCEL_CMD_LIMIT_RPT_IDE (0U)
#define ACCEL_CMD_LIMIT_RPT_DLC (4U)
#define ACCEL_CMD_LIMIT_RPT_CANID (0x201)
// signal: @ACCEL_CMD_LIMIT_ro
#define PACMOD6_ACCEL_CMD_LIMIT_ro_CovFactor (0.001000)
#define PACMOD6_ACCEL_CMD_LIMIT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_ACCEL_CMD_LIMIT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @LIMITED_ACCEL_CMD_ro
#define PACMOD6_LIMITED_ACCEL_CMD_ro_CovFactor (0.001000)
#define PACMOD6_LIMITED_ACCEL_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_LIMITED_ACCEL_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint16_t ACCEL_CMD_LIMIT_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ACCEL_CMD_LIMIT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t LIMITED_ACCEL_CMD_ro;             //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LIMITED_ACCEL_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint16_t ACCEL_CMD_LIMIT_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ACCEL_CMD_LIMIT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t LIMITED_ACCEL_CMD_ro;             //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LIMITED_ACCEL_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} ACCEL_CMD_LIMIT_RPT_t;

// def @BRAKE_RPT CAN Message (516  0x204)
#define BRAKE_RPT_IDE (0U)
#define BRAKE_RPT_DLC (8U)
#define BRAKE_RPT_CANID (0x204)
// signal: @MANUAL_INPUT_ro
#define PACMOD6_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD6_MANUAL_INPUT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD6_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD6_COMMANDED_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD6_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD6_OUTPUT_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} BRAKE_RPT_t;

// def @BRAKE_CMD_LIMIT_RPT CAN Message (517  0x205)
#define BRAKE_CMD_LIMIT_RPT_IDE (0U)
#define BRAKE_CMD_LIMIT_RPT_DLC (4U)
#define BRAKE_CMD_LIMIT_RPT_CANID (0x205)
// signal: @BRAKE_CMD_LIMIT_ro
#define PACMOD6_BRAKE_CMD_LIMIT_ro_CovFactor (0.001000)
#define PACMOD6_BRAKE_CMD_LIMIT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_BRAKE_CMD_LIMIT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @LIMITED_BRAKE_CMD_ro
#define PACMOD6_LIMITED_BRAKE_CMD_ro_CovFactor (0.001000)
#define PACMOD6_LIMITED_BRAKE_CMD_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_LIMITED_BRAKE_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint16_t BRAKE_CMD_LIMIT_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t BRAKE_CMD_LIMIT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t LIMITED_BRAKE_CMD_ro;             //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LIMITED_BRAKE_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint16_t BRAKE_CMD_LIMIT_ro;               //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t BRAKE_CMD_LIMIT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t LIMITED_BRAKE_CMD_ro;             //      Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LIMITED_BRAKE_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} BRAKE_CMD_LIMIT_RPT_t;

// def @CRUISE_CONTROL_BUTTONS_RPT CAN Message (520  0x208)
#define CRUISE_CONTROL_BUTTONS_RPT_IDE (0U)
#define CRUISE_CONTROL_BUTTONS_RPT_DLC (4U)
#define CRUISE_CONTROL_BUTTONS_RPT_CANID (0x208)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  6 : "CRUISE_CONTROL_ON_OFF"
  //  5 : "CRUISE_CONTROL_RES_INC"
  //  4 : "CRUISE_CONTROL_SET_DEC"
  //  3 : "CRUISE_CONTROL_ACC_CLOSER"
  //  2 : "CRUISE_CONTROL_ACC_FURTHER"
  //  1 : "CRUISE_CONTROL_CNCL"
  //  0 : "CRUISE_CONTROL_NONE"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} CRUISE_CONTROL_BUTTONS_RPT_t;

// def @DASH_CONTROLS_LEFT_RPT CAN Message (524  0x20c)
#define DASH_CONTROLS_LEFT_RPT_IDE (0U)
#define DASH_CONTROLS_LEFT_RPT_DLC (4U)
#define DASH_CONTROLS_LEFT_RPT_CANID (0x20c)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} DASH_CONTROLS_LEFT_RPT_t;

// def @DASH_CONTROLS_RIGHT_RPT CAN Message (528  0x210)
#define DASH_CONTROLS_RIGHT_RPT_IDE (0U)
#define DASH_CONTROLS_RIGHT_RPT_DLC (4U)
#define DASH_CONTROLS_RIGHT_RPT_CANID (0x210)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  5 : "DASH_CONTROL_DOWN"
  //  4 : "DASH_CONTROL_UP"
  //  3 : "DASH_CONTROL_RIGHT"
  //  2 : "DASH_CONTROL_LEFT"
  //  1 : "DASH_CONTROL_OK"
  //  0 : "DASH_CONTROL_NONE"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} DASH_CONTROLS_RIGHT_RPT_t;

// def @HAZARD_LIGHTS_RPT CAN Message (532  0x214)
#define HAZARD_LIGHTS_RPT_IDE (0U)
#define HAZARD_LIGHTS_RPT_DLC (4U)
#define HAZARD_LIGHTS_RPT_CANID (0x214)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  uint8_t MANUAL_INPUT : 1;                  //      Bits= 1

  uint8_t COMMANDED_VALUE : 1;               //      Bits= 1

  uint8_t OUTPUT_VALUE : 1;                  //      Bits= 1

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  uint8_t MANUAL_INPUT;                      //      Bits= 1

  uint8_t COMMANDED_VALUE;                   //      Bits= 1

  uint8_t OUTPUT_VALUE;                      //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} HAZARD_LIGHTS_RPT_t;

// def @HEADLIGHT_RPT CAN Message (536  0x218)
#define HEADLIGHT_RPT_IDE (0U)
#define HEADLIGHT_RPT_DLC (4U)
#define HEADLIGHT_RPT_CANID (0x218)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  2 : "High Beams"
  //  1 : "Low Beams"
  //  0 : "Headlights Off"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} HEADLIGHT_RPT_t;

// def @HORN_RPT CAN Message (540  0x21c)
#define HORN_RPT_IDE (0U)
#define HORN_RPT_DLC (4U)
#define HORN_RPT_CANID (0x21c)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "OFF"
  //  1 : "ON"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "OFF"
  //  1 : "ON"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  0 : "OFF"
  //  1 : "ON"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "OFF"
  //  1 : "ON"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "OFF"
  //  1 : "ON"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} HORN_RPT_t;

// def @MEDIA_CONTROLS_RPT CAN Message (544  0x220)
#define MEDIA_CONTROLS_RPT_IDE (0U)
#define MEDIA_CONTROLS_RPT_DLC (4U)
#define MEDIA_CONTROLS_RPT_CANID (0x220)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  6 : "MEDIA_CONTROL_VOL_DOWN"
  //  5 : "MEDIA_CONTROL_VOL_UP"
  //  4 : "MEDIA_CONTROL_NEXT_TRACK_HANG_UP"
  //  3 : "MEDIA_CONTROL_PREV_TRACK_ANSWER"
  //  2 : "MEDIA_CONTROL_MUTE"
  //  1 : "MEDIA_CONTROL_VOICE_COMMAND"
  //  0 : "MEDIA_CONTROL_NONE"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} MEDIA_CONTROLS_RPT_t;

// def @PARKING_BRAKE_RPT CAN Message (548  0x224)
#define PARKING_BRAKE_RPT_IDE (0U)
#define PARKING_BRAKE_RPT_DLC (4U)
#define PARKING_BRAKE_RPT_CANID (0x224)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  uint8_t MANUAL_INPUT : 1;                  //      Bits= 1

  uint8_t COMMANDED_VALUE : 1;               //      Bits= 1

  uint8_t OUTPUT_VALUE : 1;                  //      Bits= 1

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  uint8_t MANUAL_INPUT;                      //      Bits= 1

  uint8_t COMMANDED_VALUE;                   //      Bits= 1

  uint8_t OUTPUT_VALUE;                      //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} PARKING_BRAKE_RPT_t;

// def @SHIFT_RPT CAN Message (552  0x228)
#define SHIFT_RPT_IDE (0U)
#define SHIFT_RPT_DLC (4U)
#define SHIFT_RPT_CANID (0x228)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  5 : "BETWEEN_GEARS"
  //  6 : "ERROR"
  //  7 : "NONE"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  7 : "NONE"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  5 : "BETWEEN_GEARS"
  //  6 : "ERROR"
  //  7 : "NONE"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  5 : "BETWEEN_GEARS"
  //  6 : "ERROR"
  //  7 : "NONE"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  7 : "NONE"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "PARK"
  //  1 : "REVERSE"
  //  2 : "NEUTRAL"
  //  3 : "FORWARD/HIGH"
  //  4 : "LOW"
  //  5 : "BETWEEN_GEARS"
  //  6 : "ERROR"
  //  7 : "NONE"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} SHIFT_RPT_t;

// def @STEERING_RPT CAN Message (556  0x22c)
#define STEERING_RPT_IDE (0U)
#define STEERING_RPT_DLC (8U)
#define STEERING_RPT_CANID (0x22c)
// signal: @MANUAL_INPUT_ro
#define PACMOD6_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD6_MANUAL_INPUT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD6_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD6_COMMANDED_VALUE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD6_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD6_OUTPUT_VALUE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  int16_t MANUAL_INPUT_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t COMMANDED_VALUE_ro;                //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t OUTPUT_VALUE_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  int16_t MANUAL_INPUT_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t COMMANDED_VALUE_ro;                //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t OUTPUT_VALUE_ro;                   //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} STEERING_RPT_t;

// def @STEERING_CMD_LIMIT_RPT CAN Message (557  0x22d)
#define STEERING_CMD_LIMIT_RPT_IDE (0U)
#define STEERING_CMD_LIMIT_RPT_DLC (8U)
#define STEERING_CMD_LIMIT_RPT_CANID (0x22d)
// signal: @POSITION_CMD_LIMIT_ro
#define PACMOD6_POSITION_CMD_LIMIT_ro_CovFactor (0.001000)
#define PACMOD6_POSITION_CMD_LIMIT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_POSITION_CMD_LIMIT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @LIMITED_POSITION_CMD_ro
#define PACMOD6_LIMITED_POSITION_CMD_ro_CovFactor (0.001000)
#define PACMOD6_LIMITED_POSITION_CMD_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_LIMITED_POSITION_CMD_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @ROTATION_RATE_CMD_LIMIT_ro
#define PACMOD6_ROTATION_RATE_CMD_LIMIT_ro_CovFactor (0.001000)
#define PACMOD6_ROTATION_RATE_CMD_LIMIT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_ROTATION_RATE_CMD_LIMIT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @LIMITED_ROTATION_RATE_ro
#define PACMOD6_LIMITED_ROTATION_RATE_ro_CovFactor (0.001000)
#define PACMOD6_LIMITED_ROTATION_RATE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_LIMITED_ROTATION_RATE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int16_t POSITION_CMD_LIMIT_ro;             //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t POSITION_CMD_LIMIT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t LIMITED_POSITION_CMD_ro;           //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LIMITED_POSITION_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t ROTATION_RATE_CMD_LIMIT_ro;       //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_CMD_LIMIT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t LIMITED_ROTATION_RATE_ro;         //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LIMITED_ROTATION_RATE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  int16_t POSITION_CMD_LIMIT_ro;             //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t POSITION_CMD_LIMIT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t LIMITED_POSITION_CMD_ro;           //  [-] Bits=16 Factor= 0.001000        Unit:'rad'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LIMITED_POSITION_CMD_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t ROTATION_RATE_CMD_LIMIT_ro;       //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_CMD_LIMIT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t LIMITED_ROTATION_RATE_ro;         //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LIMITED_ROTATION_RATE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} STEERING_CMD_LIMIT_RPT_t;

// def @TURN_RPT CAN Message (560  0x230)
#define TURN_RPT_IDE (0U)
#define TURN_RPT_DLC (4U)
#define TURN_RPT_CANID (0x230)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  0 : "RIGHT"
  //  1 : "NONE"
  //  2 : "LEFT"
  //  3 : "HAZARD"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} TURN_RPT_t;

// def @WIPER_RPT CAN Message (564  0x234)
#define WIPER_RPT_IDE (0U)
#define WIPER_RPT_DLC (4U)
#define WIPER_RPT_CANID (0x234)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  //  255 : "High"
  //  254 : "Medium"
  //  253 : "Low"
  //  10 : "Intermittent 10"
  //  9 : "Intermittent 9"
  //  8 : "Intermittent 8"
  //  7 : "Intermittent 7"
  //  6 : "Intermittent 6"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  255 : "High"
  //  254 : "Medium"
  //  253 : "Low"
  //  10 : "Intermittent 10"
  //  9 : "Intermittent 9"
  //  8 : "Intermittent 8"
  //  7 : "Intermittent 7"
  //  6 : "Intermittent 6"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  255 : "High"
  //  254 : "Medium"
  //  253 : "Low"
  //  10 : "Intermittent 10"
  //  9 : "Intermittent 9"
  //  8 : "Intermittent 8"
  //  7 : "Intermittent 7"
  //  6 : "Intermittent 6"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t PACMOD_FAULT;                      //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  //  0 : "NO_TIMEOUT"
  //  1 : "TIMEOUT"
  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  //  255 : "High"
  //  254 : "Medium"
  //  253 : "Low"
  //  10 : "Intermittent 10"
  //  9 : "Intermittent 9"
  //  8 : "Intermittent 8"
  //  7 : "Intermittent 7"
  //  6 : "Intermittent 6"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  uint8_t MANUAL_INPUT;                      //      Bits= 8

  //  255 : "High"
  //  254 : "Medium"
  //  253 : "Low"
  //  10 : "Intermittent 10"
  //  9 : "Intermittent 9"
  //  8 : "Intermittent 8"
  //  7 : "Intermittent 7"
  //  6 : "Intermittent 6"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  uint8_t COMMANDED_VALUE;                   //      Bits= 8

  //  255 : "High"
  //  254 : "Medium"
  //  253 : "Low"
  //  10 : "Intermittent 10"
  //  9 : "Intermittent 9"
  //  8 : "Intermittent 8"
  //  7 : "Intermittent 7"
  //  6 : "Intermittent 6"
  //  5 : "Intermittent 5"
  //  4 : "Intermittent 4"
  //  3 : "Intermittent 3"
  //  2 : "Intermittent 2"
  //  1 : "Intermittent 1"
  //  0 : "Wipers Off"
  uint8_t OUTPUT_VALUE;                      //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} WIPER_RPT_t;

// def @SPRAYER_RPT CAN Message (568  0x238)
#define SPRAYER_RPT_IDE (0U)
#define SPRAYER_RPT_DLC (4U)
#define SPRAYER_RPT_CANID (0x238)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  uint8_t MANUAL_INPUT : 1;                  //      Bits= 1

  uint8_t COMMANDED_VALUE : 1;               //      Bits= 1

  uint8_t OUTPUT_VALUE : 1;                  //      Bits= 1

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  uint8_t MANUAL_INPUT;                      //      Bits= 1

  uint8_t COMMANDED_VALUE;                   //      Bits= 1

  uint8_t OUTPUT_VALUE;                      //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} SPRAYER_RPT_t;

// def @BRAKE_DECCEL_RPT CAN Message (572  0x23c)
#define BRAKE_DECCEL_RPT_IDE (0U)
#define BRAKE_DECCEL_RPT_DLC (7U)
#define BRAKE_DECCEL_RPT_CANID (0x23c)
// signal: @MANUAL_INPUT_ro
#define PACMOD6_MANUAL_INPUT_ro_CovFactor (0.001000)
#define PACMOD6_MANUAL_INPUT_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_MANUAL_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @COMMANDED_VALUE_ro
#define PACMOD6_COMMANDED_VALUE_ro_CovFactor (0.001000)
#define PACMOD6_COMMANDED_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_COMMANDED_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @OUTPUT_VALUE_ro
#define PACMOD6_OUTPUT_VALUE_ro_CovFactor (0.001000)
#define PACMOD6_OUTPUT_VALUE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_OUTPUT_VALUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t ENABLED : 1;                       //      Bits= 1

  uint8_t OVERRIDE_ACTIVE : 1;               //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT : 1;          //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT : 1;            //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT : 1;         //      Bits= 1

  uint8_t PACMOD_FAULT : 1;                  //      Bits= 1

  uint8_t VEHICLE_FAULT : 1;                 //      Bits= 1

  uint8_t COMMAND_TIMEOUT : 1;               //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint8_t ENABLED;                           //      Bits= 1

  uint8_t OVERRIDE_ACTIVE;                   //      Bits= 1

  uint8_t COMMAND_OUTPUT_FAULT;              //      Bits= 1

  uint8_t INPUT_OUTPUT_FAULT;                //      Bits= 1

  uint8_t OUTPUT_REPORTED_FAULT;             //      Bits= 1

  uint8_t PACMOD_FAULT;                      //      Bits= 1

  uint8_t VEHICLE_FAULT;                     //      Bits= 1

  uint8_t COMMAND_TIMEOUT;                   //      Bits= 1

  uint16_t MANUAL_INPUT_ro;                  //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MANUAL_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t COMMANDED_VALUE_ro;               //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t COMMANDED_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t OUTPUT_VALUE_ro;                  //      Bits=16 Factor= 0.001000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t OUTPUT_VALUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} BRAKE_DECCEL_RPT_t;

// def @ACCEL_AUX_RPT CAN Message (768  0x300)
#define ACCEL_AUX_RPT_IDE (0U)
#define ACCEL_AUX_RPT_DLC (6U)
#define ACCEL_AUX_RPT_CANID (0x300)
// signal: @RAW_PEDAL_POS_ro
#define PACMOD6_RAW_PEDAL_POS_ro_CovFactor (0.001000)
#define PACMOD6_RAW_PEDAL_POS_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_RAW_PEDAL_POS_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @RAW_PEDAL_FORCE_ro
#define PACMOD6_RAW_PEDAL_FORCE_ro_CovFactor (0.001000)
#define PACMOD6_RAW_PEDAL_FORCE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_RAW_PEDAL_FORCE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int16_t RAW_PEDAL_POS_ro;                  //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t RAW_PEDAL_POS_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t RAW_PEDAL_FORCE_ro;                //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t RAW_PEDAL_FORCE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  uint8_t OPERATOR_INTERACTION : 1;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_PEDAL_POS_AVAIL : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_PEDAL_FORCE_AVAIL : 1;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t OPERATOR_INTERACTION_AVAIL : 1;    //      Bits= 1

#else

  int16_t RAW_PEDAL_POS_ro;                  //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t RAW_PEDAL_POS_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t RAW_PEDAL_FORCE_ro;                //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t RAW_PEDAL_FORCE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  uint8_t OPERATOR_INTERACTION;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_PEDAL_POS_AVAIL;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_PEDAL_FORCE_AVAIL;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t OPERATOR_INTERACTION_AVAIL;        //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} ACCEL_AUX_RPT_t;

// def @BRAKE_AUX_RPT CAN Message (772  0x304)
#define BRAKE_AUX_RPT_IDE (0U)
#define BRAKE_AUX_RPT_DLC (8U)
#define BRAKE_AUX_RPT_CANID (0x304)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int16_t RAW_PEDAL_POS;                     //  [-] Bits=16

  int16_t RAW_PEDAL_FORCE;                   //  [-] Bits=16

  int16_t RAW_BRAKE_PRESSURE;                //  [-] Bits=16

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  uint8_t OPERATOR_INTERACTION : 1;          //      Bits= 1

  uint8_t BRAKE_ON_OFF : 1;                  //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_PEDAL_POS_AVAIL : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_PEDAL_FORCE_AVAIL : 1;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_BRAKE_PRESSURE_AVAIL : 1;      //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t OPERATOR_INTERACTION_AVAIL : 1;    //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t BRAKE_ON_OFF_AVAIL : 1;            //      Bits= 1

#else

  int16_t RAW_PEDAL_POS;                     //  [-] Bits=16

  int16_t RAW_PEDAL_FORCE;                   //  [-] Bits=16

  int16_t RAW_BRAKE_PRESSURE;                //  [-] Bits=16

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  uint8_t OPERATOR_INTERACTION;              //      Bits= 1

  uint8_t BRAKE_ON_OFF;                      //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_PEDAL_POS_AVAIL;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_PEDAL_FORCE_AVAIL;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_BRAKE_PRESSURE_AVAIL;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t OPERATOR_INTERACTION_AVAIL;        //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t BRAKE_ON_OFF_AVAIL;                //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} BRAKE_AUX_RPT_t;

// def @HEADLIGHT_AUX_RPT CAN Message (792  0x318)
#define HEADLIGHT_AUX_RPT_IDE (0U)
#define HEADLIGHT_AUX_RPT_DLC (3U)
#define HEADLIGHT_AUX_RPT_CANID (0x318)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t HEADLIGHTS_ON : 1;                 //      Bits= 1

  uint8_t HEADLIGHTS_ON_BRIGHT : 1;          //      Bits= 1

  uint8_t FOG_LIGHTS_ON : 1;                 //      Bits= 1

  //  3 : "Headlights On - Auto Mode"
  //  2 : "Headlights On - Manual Mode"
  //  1 : "Parking Lights Only"
  //  0 : "Headlights Off"
  uint8_t HEADLIGHTS_MODE;                   //      Bits= 8

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t HEADLIGHTS_ON_AVAIL : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t HEADLIGHTS_ON_BRIGHT_AVAIL : 1;    //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t FOG_LIGHTS_ON_AVAIL : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t HEADLIGHTS_MODE_AVAIL : 1;         //      Bits= 1

#else

  uint8_t HEADLIGHTS_ON;                     //      Bits= 1

  uint8_t HEADLIGHTS_ON_BRIGHT;              //      Bits= 1

  uint8_t FOG_LIGHTS_ON;                     //      Bits= 1

  //  3 : "Headlights On - Auto Mode"
  //  2 : "Headlights On - Manual Mode"
  //  1 : "Parking Lights Only"
  //  0 : "Headlights Off"
  uint8_t HEADLIGHTS_MODE;                   //      Bits= 8

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t HEADLIGHTS_ON_AVAIL;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t HEADLIGHTS_ON_BRIGHT_AVAIL;        //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t FOG_LIGHTS_ON_AVAIL;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t HEADLIGHTS_MODE_AVAIL;             //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} HEADLIGHT_AUX_RPT_t;

// def @SHIFT_AUX_RPT CAN Message (808  0x328)
#define SHIFT_AUX_RPT_IDE (0U)
#define SHIFT_AUX_RPT_DLC (8U)
#define SHIFT_AUX_RPT_CANID (0x328)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t BETWEEN_GEARS : 1;                  //      Bits= 1

  uint8_t STAY_IN_NEUTRAL_MODE : 1;           //      Bits= 1

  uint8_t BRAKE_INTERLOCK_ACTIVE : 1;         //      Bits= 1

  uint8_t SPEED_INTERLOCK_ACTIVE : 1;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t BETWEEN_GEARS_AVAIL : 1;            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t STAY_IN_NEUTRAL_MODE_AVAIL : 1;     //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t BRAKE_INTERLOCK_ACTIVE_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t SPEED_INTERLOCK_ACTIVE_AVAIL : 1;   //      Bits= 1

  //  0 : "SILENT"
  //  1 : "BEEP"
  uint8_t WRITE_TO_CONFIG : 1;                //      Bits= 1

#else

  uint8_t BETWEEN_GEARS;                      //      Bits= 1

  uint8_t STAY_IN_NEUTRAL_MODE;               //      Bits= 1

  uint8_t BRAKE_INTERLOCK_ACTIVE;             //      Bits= 1

  uint8_t SPEED_INTERLOCK_ACTIVE;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t BETWEEN_GEARS_AVAIL;                //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t STAY_IN_NEUTRAL_MODE_AVAIL;         //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t BRAKE_INTERLOCK_ACTIVE_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t SPEED_INTERLOCK_ACTIVE_AVAIL;       //      Bits= 1

  //  0 : "SILENT"
  //  1 : "BEEP"
  uint8_t WRITE_TO_CONFIG;                    //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} SHIFT_AUX_RPT_t;

// def @STEERING_AUX_RPT CAN Message (812  0x32c)
#define STEERING_AUX_RPT_IDE (0U)
#define STEERING_AUX_RPT_DLC (8U)
#define STEERING_AUX_RPT_CANID (0x32c)
// signal: @RAW_POSITION_ro
#define PACMOD6_RAW_POSITION_ro_CovFactor (0.001000)
#define PACMOD6_RAW_POSITION_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_RAW_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @RAW_TORQUE_ro
#define PACMOD6_RAW_TORQUE_ro_CovFactor (0.001000)
#define PACMOD6_RAW_TORQUE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_RAW_TORQUE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @ROTATION_RATE_ro
#define PACMOD6_ROTATION_RATE_ro_CovFactor (0.001000)
#define PACMOD6_ROTATION_RATE_ro_toS(x) ( (uint16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_ROTATION_RATE_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int16_t RAW_POSITION_ro;                   //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t RAW_POSITION_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t RAW_TORQUE_ro;                     //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t RAW_TORQUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                 //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  uint8_t OPERATOR_INTERACTION : 1;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_POSITION_AVAIL : 1;            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_TORQUE_AVAIL : 1;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t ROTATION_RATE_AVAIL : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t OPERATOR_INTERACTION_AVAIL : 1;    //      Bits= 1

#else

  int16_t RAW_POSITION_ro;                   //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t RAW_POSITION_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t RAW_TORQUE_ro;                     //  [-] Bits=16 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t RAW_TORQUE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint16_t ROTATION_RATE_ro;                 //      Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ROTATION_RATE_phys;
#endif // PACMOD6_USE_SIGFLOAT

  // OPERATOR_INTERACTION shall have the value of 1 if the driver is moving, changing, or otherwise touching the operator control(s) that relates to this signal to an extent that is detectable. Otherwise, the value shall be 0.
  //  0 : "NO_INTERACTION"
  //  1 : "INTERACTION"
  uint8_t OPERATOR_INTERACTION;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_POSITION_AVAIL;                //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t RAW_TORQUE_AVAIL;                  //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t ROTATION_RATE_AVAIL;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t OPERATOR_INTERACTION_AVAIL;        //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} STEERING_AUX_RPT_t;

// def @TURN_AUX_RPT CAN Message (816  0x330)
#define TURN_AUX_RPT_IDE (0U)
#define TURN_AUX_RPT_DLC (2U)
#define TURN_AUX_RPT_CANID (0x330)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t DRIVER_BLINKER_BULB_ON : 1;         //      Bits= 1

  uint8_t PASS_BLINKER_BULB_ON : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t DRIVER_BLINKER_BULB_ON_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t PASS_BLINKER_BULB_ON_AVAIL : 1;     //      Bits= 1

#else

  uint8_t DRIVER_BLINKER_BULB_ON;             //      Bits= 1

  uint8_t PASS_BLINKER_BULB_ON;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t DRIVER_BLINKER_BULB_ON_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t PASS_BLINKER_BULB_ON_AVAIL;         //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} TURN_AUX_RPT_t;

// def @WIPER_AUX_RPT CAN Message (820  0x334)
#define WIPER_AUX_RPT_IDE (0U)
#define WIPER_AUX_RPT_DLC (2U)
#define WIPER_AUX_RPT_CANID (0x334)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t FRONT_WIPING : 1;                  //      Bits= 1

  uint8_t FRONT_SPRAYING : 1;                //      Bits= 1

  uint8_t REAR_WIPING : 1;                   //      Bits= 1

  uint8_t REAR_SPRAYING : 1;                 //      Bits= 1

  uint8_t SPRAY_NEAR_EMPTY : 1;              //      Bits= 1

  uint8_t SPRAY_EMPTY : 1;                   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t FRONT_WIPING_AVAIL : 1;            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t FRONT_SPRAYING_AVAIL : 1;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t REAR_WIPING_AVAIL : 1;             //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t REAR_SPRAYING_AVAIL : 1;           //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t SPRAY_NEAR_EMPTY_AVAIL : 1;        //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t SPRAY_EMPTY_AVAIL : 1;             //      Bits= 1

#else

  uint8_t FRONT_WIPING;                      //      Bits= 1

  uint8_t FRONT_SPRAYING;                    //      Bits= 1

  uint8_t REAR_WIPING;                       //      Bits= 1

  uint8_t REAR_SPRAYING;                     //      Bits= 1

  uint8_t SPRAY_NEAR_EMPTY;                  //      Bits= 1

  uint8_t SPRAY_EMPTY;                       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t FRONT_WIPING_AVAIL;                //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t FRONT_SPRAYING_AVAIL;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t REAR_WIPING_AVAIL;                 //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t REAR_SPRAYING_AVAIL;               //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t SPRAY_NEAR_EMPTY_AVAIL;            //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t SPRAY_EMPTY_AVAIL;                 //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} WIPER_AUX_RPT_t;

// def @BRAKE_DECCEL_AUX_RPT CAN Message (824  0x338)
#define BRAKE_DECCEL_AUX_RPT_IDE (0U)
#define BRAKE_DECCEL_AUX_RPT_DLC (3U)
#define BRAKE_DECCEL_AUX_RPT_CANID (0x338)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "NO_BRAKE_DEMAND"
  //  1 : "DRIVERS_BRAKE_DEMAND"
  //  2 : "ADDITION_MODE_OF_XBR_ACCELERATION_CONTROL"
  //  3 : "MAXIMUM_MODE_OF_XBR_ACCELERATION_CONTROL"
  uint8_t XBR_ACTIVE_CONTROL_MODE : 4;         //      Bits= 4

  //  0 : "ANY_EXTERNAL_BRAKE_DEMAND_WILL_BE_ACCEPTED"
  //  2 : "NO_EXTERNAL_BRAKE_DEMAND_WILL_BE_ACCEPTED"
  uint8_t XBR_SYSTEM_STATE : 2;                //      Bits= 2

  //  0 : "FOUNDATION_BRAKES_NOT_IN_USE"
  //  1 : "FOUNDATION_BRAKES_IN_USE"
  uint8_t FOUNDATION_BRAKE_USE : 2;            //      Bits= 2

  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "ACTIVE_BUT_INACTIVE_SOON"
  //  6 : "ERROR"
  uint8_t HILL_HOLDER_MODE : 3;                //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t XBR_ACTIVE_CONTROL_MODE_AVAIL : 1;   //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t XBR_SYSTEM_STATE_AVAIL : 1;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t FOUNDATION_BRAKE_USE_AVAIL : 1;      //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t HILL_HOLDER_MODE_AVAIL : 1;          //      Bits= 1

#else

  //  0 : "NO_BRAKE_DEMAND"
  //  1 : "DRIVERS_BRAKE_DEMAND"
  //  2 : "ADDITION_MODE_OF_XBR_ACCELERATION_CONTROL"
  //  3 : "MAXIMUM_MODE_OF_XBR_ACCELERATION_CONTROL"
  uint8_t XBR_ACTIVE_CONTROL_MODE;             //      Bits= 4

  //  0 : "ANY_EXTERNAL_BRAKE_DEMAND_WILL_BE_ACCEPTED"
  //  2 : "NO_EXTERNAL_BRAKE_DEMAND_WILL_BE_ACCEPTED"
  uint8_t XBR_SYSTEM_STATE;                    //      Bits= 2

  //  0 : "FOUNDATION_BRAKES_NOT_IN_USE"
  //  1 : "FOUNDATION_BRAKES_IN_USE"
  uint8_t FOUNDATION_BRAKE_USE;                //      Bits= 2

  //  0 : "INACTIVE"
  //  1 : "ACTIVE"
  //  2 : "ACTIVE_BUT_INACTIVE_SOON"
  //  6 : "ERROR"
  uint8_t HILL_HOLDER_MODE;                    //      Bits= 3

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t XBR_ACTIVE_CONTROL_MODE_AVAIL;       //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t XBR_SYSTEM_STATE_AVAIL;              //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t FOUNDATION_BRAKE_USE_AVAIL;          //      Bits= 1

  // PACMod sets this value as a function of which vehicle platform is under test and, therefore, this value does not change during operation.
  uint8_t HILL_HOLDER_MODE_AVAIL;              //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} BRAKE_DECCEL_AUX_RPT_t;

// def @VEHICLE_SPEED_RPT CAN Message (1024 0x400)
#define VEHICLE_SPEED_RPT_IDE (0U)
#define VEHICLE_SPEED_RPT_DLC (3U)
#define VEHICLE_SPEED_RPT_CANID (0x400)
// signal: @VEHICLE_SPEED_ro
#define PACMOD6_VEHICLE_SPEED_ro_CovFactor (0.010000)
#define PACMOD6_VEHICLE_SPEED_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD6_VEHICLE_SPEED_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int16_t VEHICLE_SPEED_ro;                  //  [-] Bits=16 Factor= 0.010000        Unit:'m/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t VEHICLE_SPEED_phys;
#endif // PACMOD6_USE_SIGFLOAT

  //  0 : "INVALID"
  //  1 : "VALID"
  uint8_t VEHICLE_SPEED_VALID : 1;           //      Bits= 1

#else

  int16_t VEHICLE_SPEED_ro;                  //  [-] Bits=16 Factor= 0.010000        Unit:'m/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t VEHICLE_SPEED_phys;
#endif // PACMOD6_USE_SIGFLOAT

  //  0 : "INVALID"
  //  1 : "VALID"
  uint8_t VEHICLE_SPEED_VALID;               //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} VEHICLE_SPEED_RPT_t;

// def @BRAKE_MOTOR_RPT_1 CAN Message (1025 0x401)
#define BRAKE_MOTOR_RPT_1_IDE (0U)
#define BRAKE_MOTOR_RPT_1_DLC (8U)
#define BRAKE_MOTOR_RPT_1_CANID (0x401)
// signal: @MOTOR_CURRENT_ro
#define PACMOD6_MOTOR_CURRENT_ro_CovFactor (0.001000)
#define PACMOD6_MOTOR_CURRENT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_MOTOR_CURRENT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @SHAFT_POSITION_ro
#define PACMOD6_SHAFT_POSITION_ro_CovFactor (0.001000)
#define PACMOD6_SHAFT_POSITION_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_SHAFT_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} BRAKE_MOTOR_RPT_1_t;

// def @BRAKE_MOTOR_RPT_2 CAN Message (1026 0x402)
#define BRAKE_MOTOR_RPT_2_IDE (0U)
#define BRAKE_MOTOR_RPT_2_DLC (8U)
#define BRAKE_MOTOR_RPT_2_CANID (0x402)
// signal: @ANGULAR_SPEED_ro
#define PACMOD6_ANGULAR_SPEED_ro_CovFactor (0.100000)
#define PACMOD6_ANGULAR_SPEED_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.100000)) )
#define PACMOD6_ANGULAR_SPEED_ro_fromS(x) ( (((x) * (0.100000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} BRAKE_MOTOR_RPT_2_t;

// def @BRAKE_MOTOR_RPT_3 CAN Message (1027 0x403)
#define BRAKE_MOTOR_RPT_3_IDE (0U)
#define BRAKE_MOTOR_RPT_3_DLC (8U)
#define BRAKE_MOTOR_RPT_3_CANID (0x403)
// signal: @TORQUE_OUTPUT_ro
#define PACMOD6_TORQUE_OUTPUT_ro_CovFactor (0.001000)
#define PACMOD6_TORQUE_OUTPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_TORQUE_OUTPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @TORQUE_INPUT_ro
#define PACMOD6_TORQUE_INPUT_ro_CovFactor (0.001000)
#define PACMOD6_TORQUE_INPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_TORQUE_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} BRAKE_MOTOR_RPT_3_t;

// def @STEERING_MOTOR_RPT_1 CAN Message (1028 0x404)
#define STEERING_MOTOR_RPT_1_IDE (0U)
#define STEERING_MOTOR_RPT_1_DLC (8U)
#define STEERING_MOTOR_RPT_1_CANID (0x404)
// signal: @MOTOR_CURRENT_ro
#define PACMOD6_MOTOR_CURRENT_ro_CovFactor (0.001000)
#define PACMOD6_MOTOR_CURRENT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_MOTOR_CURRENT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @SHAFT_POSITION_ro
#define PACMOD6_SHAFT_POSITION_ro_CovFactor (0.001000)
#define PACMOD6_SHAFT_POSITION_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_SHAFT_POSITION_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  int32_t MOTOR_CURRENT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'amps'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t MOTOR_CURRENT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int32_t SHAFT_POSITION_ro;                 //  [-] Bits=32 Factor= 0.001000        Unit:'radians'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t SHAFT_POSITION_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} STEERING_MOTOR_RPT_1_t;

// def @STEERING_MOTOR_RPT_2 CAN Message (1029 0x405)
#define STEERING_MOTOR_RPT_2_IDE (0U)
#define STEERING_MOTOR_RPT_2_DLC (8U)
#define STEERING_MOTOR_RPT_2_CANID (0x405)
// signal: @ANGULAR_SPEED_ro
#define PACMOD6_ANGULAR_SPEED_ro_CovFactor (0.100000)
#define PACMOD6_ANGULAR_SPEED_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.100000)) )
#define PACMOD6_ANGULAR_SPEED_ro_fromS(x) ( (((x) * (0.100000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  int16_t ENCODER_TEMPERATURE;               //  [-] Bits=16 Unit:'deg_C'

  int16_t MOTOR_TEMPERATURE;                 //  [-] Bits=16 Unit:'deg_C'

  int32_t ANGULAR_SPEED_ro;                  //  [-] Bits=32 Factor= 0.100000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ANGULAR_SPEED_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} STEERING_MOTOR_RPT_2_t;

// def @STEERING_MOTOR_RPT_3 CAN Message (1030 0x406)
#define STEERING_MOTOR_RPT_3_IDE (0U)
#define STEERING_MOTOR_RPT_3_DLC (8U)
#define STEERING_MOTOR_RPT_3_CANID (0x406)
// signal: @TORQUE_OUTPUT_ro
#define PACMOD6_TORQUE_OUTPUT_ro_CovFactor (0.001000)
#define PACMOD6_TORQUE_OUTPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_TORQUE_OUTPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @TORQUE_INPUT_ro
#define PACMOD6_TORQUE_INPUT_ro_CovFactor (0.001000)
#define PACMOD6_TORQUE_INPUT_ro_toS(x) ( (int32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_TORQUE_INPUT_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  int32_t TORQUE_OUTPUT_ro;                  //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t TORQUE_OUTPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int32_t TORQUE_INPUT_ro;                   //  [-] Bits=32 Factor= 0.001000        Unit:'N-m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t TORQUE_INPUT_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} STEERING_MOTOR_RPT_3_t;

// def @WHEEL_SPEED_RPT CAN Message (1031 0x407)
#define WHEEL_SPEED_RPT_IDE (0U)
#define WHEEL_SPEED_RPT_DLC (8U)
#define WHEEL_SPEED_RPT_CANID (0x407)
// signal: @WHEEL_SPD_FRONT_LEFT_ro
#define PACMOD6_WHEEL_SPD_FRONT_LEFT_ro_CovFactor (0.010000)
#define PACMOD6_WHEEL_SPD_FRONT_LEFT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD6_WHEEL_SPD_FRONT_LEFT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_FRONT_RIGHT_ro
#define PACMOD6_WHEEL_SPD_FRONT_RIGHT_ro_CovFactor (0.010000)
#define PACMOD6_WHEEL_SPD_FRONT_RIGHT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD6_WHEEL_SPD_FRONT_RIGHT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_REAR_LEFT_ro
#define PACMOD6_WHEEL_SPD_REAR_LEFT_ro_CovFactor (0.010000)
#define PACMOD6_WHEEL_SPD_REAR_LEFT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD6_WHEEL_SPD_REAR_LEFT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @WHEEL_SPD_REAR_RIGHT_ro
#define PACMOD6_WHEEL_SPD_REAR_RIGHT_ro_CovFactor (0.010000)
#define PACMOD6_WHEEL_SPD_REAR_RIGHT_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD6_WHEEL_SPD_REAR_RIGHT_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int16_t WHEEL_SPD_FRONT_LEFT_ro;           //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_FRONT_LEFT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t WHEEL_SPD_FRONT_RIGHT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_FRONT_RIGHT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t WHEEL_SPD_REAR_LEFT_ro;            //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_REAR_LEFT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t WHEEL_SPD_REAR_RIGHT_ro;           //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_REAR_RIGHT_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  int16_t WHEEL_SPD_FRONT_LEFT_ro;           //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_FRONT_LEFT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t WHEEL_SPD_FRONT_RIGHT_ro;          //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_FRONT_RIGHT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t WHEEL_SPD_REAR_LEFT_ro;            //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_REAR_LEFT_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t WHEEL_SPD_REAR_RIGHT_ro;           //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t WHEEL_SPD_REAR_RIGHT_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} WHEEL_SPEED_RPT_t;

// def @SOFTWARE_VERSION_RPT_00 CAN Message (1032 0x408)
#define SOFTWARE_VERSION_RPT_00_IDE (0U)
#define SOFTWARE_VERSION_RPT_00_DLC (7U)
#define SOFTWARE_VERSION_RPT_00_CANID (0x408)
#define SOFTWARE_VERSION_RPT_00_CYC (1000U)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#else

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} SOFTWARE_VERSION_RPT_00_t;

// def @SOFTWARE_VERSION_RPT_01 CAN Message (1033 0x409)
#define SOFTWARE_VERSION_RPT_01_IDE (0U)
#define SOFTWARE_VERSION_RPT_01_DLC (7U)
#define SOFTWARE_VERSION_RPT_01_CANID (0x409)
#define SOFTWARE_VERSION_RPT_01_CYC (1000U)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#else

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} SOFTWARE_VERSION_RPT_01_t;

// def @SOFTWARE_VERSION_RPT_02 CAN Message (1034 0x40a)
#define SOFTWARE_VERSION_RPT_02_IDE (0U)
#define SOFTWARE_VERSION_RPT_02_DLC (7U)
#define SOFTWARE_VERSION_RPT_02_CANID (0x40a)
#define SOFTWARE_VERSION_RPT_02_CYC (1000U)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#else

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} SOFTWARE_VERSION_RPT_02_t;

// def @SOFTWARE_VERSION_RPT_03 CAN Message (1035 0x40b)
#define SOFTWARE_VERSION_RPT_03_IDE (0U)
#define SOFTWARE_VERSION_RPT_03_DLC (7U)
#define SOFTWARE_VERSION_RPT_03_CANID (0x40b)
#define SOFTWARE_VERSION_RPT_03_CYC (1000U)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#else

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MAJOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t MINOR;                             //      Bits= 8

  // MAJOR, MINOR, and PATCH shall be the software version number and shall be defined by Sematic Versioning 2.0.  For build numbers other than 0000, it shall be the most recent software release.
  uint8_t PATCH;                             //      Bits= 8

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD0;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD1;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD2;                            //      Bits= 8 Unit:'ASCII'

  // BUILD0-BUILD3 shall represent the software build number and shall be constrained to characters A-Z and 0-9.
  uint8_t BUILD3;                            //      Bits= 8 Unit:'ASCII'

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} SOFTWARE_VERSION_RPT_03_t;

// def @YAW_RATE_RPT CAN Message (1037 0x40d)
#define YAW_RATE_RPT_IDE (0U)
#define YAW_RATE_RPT_DLC (2U)
#define YAW_RATE_RPT_CANID (0x40d)
// signal: @YAW_RATE_ro
#define PACMOD6_YAW_RATE_ro_CovFactor (0.010000)
#define PACMOD6_YAW_RATE_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD6_YAW_RATE_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int16_t YAW_RATE_ro;                       //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t YAW_RATE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  int16_t YAW_RATE_ro;                       //  [-] Bits=16 Factor= 0.010000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t YAW_RATE_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} YAW_RATE_RPT_t;

// def @LAT_LON_HEADING_RPT CAN Message (1038 0x40e)
#define LAT_LON_HEADING_RPT_IDE (0U)
#define LAT_LON_HEADING_RPT_DLC (8U)
#define LAT_LON_HEADING_RPT_CANID (0x40e)
// signal: @HEADING_ro
#define PACMOD6_HEADING_ro_CovFactor (0.010000)
#define PACMOD6_HEADING_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD6_HEADING_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  int8_t LATITUDE_DEGREES;                   //  [-] Bits= 8 Unit:'deg'

  int8_t LATITUDE_MINUTES;                   //  [-] Bits= 8 Unit:'min'

  int8_t LATITUDE_SECONDS;                   //  [-] Bits= 8 Unit:'sec'

  int8_t LONGITUDE_DEGREES;                  //  [-] Bits= 8 Unit:'deg'

  int8_t LONGITUDE_MINUTES;                  //  [-] Bits= 8 Unit:'min'

  int8_t LONGITUDE_SECONDS;                  //  [-] Bits= 8 Unit:'sec'

  int16_t HEADING_ro;                        //  [-] Bits=16 Factor= 0.010000        Unit:'deg'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t HEADING_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  int8_t LATITUDE_DEGREES;                   //  [-] Bits= 8 Unit:'deg'

  int8_t LATITUDE_MINUTES;                   //  [-] Bits= 8 Unit:'min'

  int8_t LATITUDE_SECONDS;                   //  [-] Bits= 8 Unit:'sec'

  int8_t LONGITUDE_DEGREES;                  //  [-] Bits= 8 Unit:'deg'

  int8_t LONGITUDE_MINUTES;                  //  [-] Bits= 8 Unit:'min'

  int8_t LONGITUDE_SECONDS;                  //  [-] Bits= 8 Unit:'sec'

  int16_t HEADING_ro;                        //  [-] Bits=16 Factor= 0.010000        Unit:'deg'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t HEADING_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} LAT_LON_HEADING_RPT_t;

// def @DATE_TIME_RPT CAN Message (1039 0x40f)
#define DATE_TIME_RPT_IDE (0U)
#define DATE_TIME_RPT_DLC (6U)
#define DATE_TIME_RPT_CANID (0x40f)
// signal: @DATE_YEAR_ro
#define PACMOD6_DATE_YEAR_ro_CovFactor (1)
#define PACMOD6_DATE_YEAR_ro_toS(x) ( (uint8_t) ((x) - (2000)) )
#define PACMOD6_DATE_YEAR_ro_fromS(x) ( ((x) + (2000)) )
// signal: @DATE_MONTH_ro
#define PACMOD6_DATE_MONTH_ro_CovFactor (1)
#define PACMOD6_DATE_MONTH_ro_toS(x) ( (uint8_t) ((x) - (1)) )
#define PACMOD6_DATE_MONTH_ro_fromS(x) ( ((x) + (1)) )
// signal: @DATE_DAY_ro
#define PACMOD6_DATE_DAY_ro_CovFactor (1)
#define PACMOD6_DATE_DAY_ro_toS(x) ( (uint8_t) ((x) - (1)) )
#define PACMOD6_DATE_DAY_ro_fromS(x) ( ((x) + (1)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t DATE_YEAR_ro;                      //      Bits= 8 Offset= 2000               Unit:'yr'

#ifdef PACMOD6_USE_SIGFLOAT
  uint16_t DATE_YEAR_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint8_t DATE_MONTH_ro;                     //      Bits= 8 Offset= 1                  Unit:'mon'

#ifdef PACMOD6_USE_SIGFLOAT
  uint16_t DATE_MONTH_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint8_t DATE_DAY_ro;                       //      Bits= 8 Offset= 1                  Unit:'dy'

#ifdef PACMOD6_USE_SIGFLOAT
  uint16_t DATE_DAY_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint8_t TIME_HOUR;                         //      Bits= 8 Unit:'hr'

  uint8_t TIME_MINUTE;                       //      Bits= 8 Unit:'min'

  uint8_t TIME_SECOND;                       //      Bits= 8 Unit:'sec'

#else

  uint8_t DATE_YEAR_ro;                      //      Bits= 8 Offset= 2000               Unit:'yr'

#ifdef PACMOD6_USE_SIGFLOAT
  uint16_t DATE_YEAR_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint8_t DATE_MONTH_ro;                     //      Bits= 8 Offset= 1                  Unit:'mon'

#ifdef PACMOD6_USE_SIGFLOAT
  uint16_t DATE_MONTH_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint8_t DATE_DAY_ro;                       //      Bits= 8 Offset= 1                  Unit:'dy'

#ifdef PACMOD6_USE_SIGFLOAT
  uint16_t DATE_DAY_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint8_t TIME_HOUR;                         //      Bits= 8 Unit:'hr'

  uint8_t TIME_MINUTE;                       //      Bits= 8 Unit:'min'

  uint8_t TIME_SECOND;                       //      Bits= 8 Unit:'sec'

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} DATE_TIME_RPT_t;

// def @DETECTED_OBJECT_RPT CAN Message (1041 0x411)
#define DETECTED_OBJECT_RPT_IDE (0U)
#define DETECTED_OBJECT_RPT_DLC (6U)
#define DETECTED_OBJECT_RPT_CANID (0x411)
// signal: @FRONT_OBJECT_DISTANCE_LOW_RES_ro
#define PACMOD6_FRONT_OBJECT_DISTANCE_LOW_RES_ro_CovFactor (0.001000)
#define PACMOD6_FRONT_OBJECT_DISTANCE_LOW_RES_ro_toS(x) ( (uint32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_FRONT_OBJECT_DISTANCE_LOW_RES_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @FRONT_OBJECT_DISTANCE_HIGH_RES_ro
#define PACMOD6_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_CovFactor (0.001000)
#define PACMOD6_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_toS(x) ( (uint32_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_FRONT_OBJECT_DISTANCE_HIGH_RES_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint32_t FRONT_OBJECT_DISTANCE_LOW_RES_ro;       //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_LOW_RES_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint32_t FRONT_OBJECT_DISTANCE_HIGH_RES_ro;      //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_HIGH_RES_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint32_t FRONT_OBJECT_DISTANCE_LOW_RES_ro;       //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_LOW_RES_phys;
#endif // PACMOD6_USE_SIGFLOAT

  uint32_t FRONT_OBJECT_DISTANCE_HIGH_RES_ro;      //      Bits=24 Factor= 0.001000        Unit:'m'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t FRONT_OBJECT_DISTANCE_HIGH_RES_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} DETECTED_OBJECT_RPT_t;

// def @VEH_SPECIFIC_RPT_1 CAN Message (1042 0x412)
#define VEH_SPECIFIC_RPT_1_IDE (0U)
#define VEH_SPECIFIC_RPT_1_DLC (2U)
#define VEH_SPECIFIC_RPT_1_CANID (0x412)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t SHIFT_POS_1;                       //      Bits= 8

  uint8_t SHIFT_POS_2;                       //      Bits= 8

#else

  uint8_t SHIFT_POS_1;                       //      Bits= 8

  uint8_t SHIFT_POS_2;                       //      Bits= 8

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} VEH_SPECIFIC_RPT_1_t;

// def @VEH_DYNAMICS_RPT CAN Message (1043 0x413)
#define VEH_DYNAMICS_RPT_IDE (0U)
#define VEH_DYNAMICS_RPT_DLC (1U)
#define VEH_DYNAMICS_RPT_CANID (0x413)
// signal: @VEH_G_FORCES_ro
#define PACMOD6_VEH_G_FORCES_ro_CovFactor (0.001000)
#define PACMOD6_VEH_G_FORCES_ro_toS(x) ( (uint8_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_VEH_G_FORCES_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t VEH_G_FORCES_ro;                   //      Bits= 8 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t VEH_G_FORCES_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  uint8_t VEH_G_FORCES_ro;                   //      Bits= 8 Factor= 0.001000       

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t VEH_G_FORCES_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} VEH_DYNAMICS_RPT_t;

// def @VIN_RPT CAN Message (1044 0x414)
#define VIN_RPT_IDE (0U)
#define VIN_RPT_DLC (7U)
#define VIN_RPT_CANID (0x414)
#define VIN_RPT_CYC (1000U)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint32_t VEH_MFG_CODE;                     //      Bits=24

  uint8_t VEH_MY_CODE;                       //      Bits= 8

  uint32_t VEH_SERIAL;                       //      Bits=24

#else

  uint32_t VEH_MFG_CODE;                     //      Bits=24

  uint8_t VEH_MY_CODE;                       //      Bits= 8

  uint32_t VEH_SERIAL;                       //      Bits=24

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} VIN_RPT_t;

// def @OCCUPANCY_RPT CAN Message (1045 0x415)
#define OCCUPANCY_RPT_IDE (0U)
#define OCCUPANCY_RPT_DLC (2U)
#define OCCUPANCY_RPT_CANID (0x415)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t DRIVER_SEAT_OCCUPIED : 1;               //      Bits= 1

  uint8_t PASS_SEAT_OCCUPIED : 1;                 //      Bits= 1

  uint8_t REAR_SEAT_OCCUPIED : 1;                 //      Bits= 1

  uint8_t DRIVER_SEATBELT_BUCKLED : 1;            //      Bits= 1

  uint8_t PASS_SEATBELT_BUCKLED : 1;              //      Bits= 1

  uint8_t REAR_SEATBELT_BUCKLED : 1;              //      Bits= 1

  uint8_t DRIVER_SEAT_OCCUPIED_IS_VALID : 1;      //      Bits= 1

  uint8_t PASS_SEAT_OCCUPIED_IS_VALID : 1;        //      Bits= 1

  uint8_t REAR_SEAT_OCCUPIED_IS_VALID : 1;        //      Bits= 1

  uint8_t DRIVER_SEATBELT_BUCKLED_IS_VALID : 1;   //      Bits= 1

  uint8_t PASS_SEATBELT_BUCKLED_IS_VALID : 1;     //      Bits= 1

  uint8_t REAR_SEATBELT_BUCKLED_IS_VALID : 1;     //      Bits= 1

#else

  uint8_t DRIVER_SEAT_OCCUPIED;                   //      Bits= 1

  uint8_t PASS_SEAT_OCCUPIED;                     //      Bits= 1

  uint8_t REAR_SEAT_OCCUPIED;                     //      Bits= 1

  uint8_t DRIVER_SEATBELT_BUCKLED;                //      Bits= 1

  uint8_t PASS_SEATBELT_BUCKLED;                  //      Bits= 1

  uint8_t REAR_SEATBELT_BUCKLED;                  //      Bits= 1

  uint8_t DRIVER_SEAT_OCCUPIED_IS_VALID;          //      Bits= 1

  uint8_t PASS_SEAT_OCCUPIED_IS_VALID;            //      Bits= 1

  uint8_t REAR_SEAT_OCCUPIED_IS_VALID;            //      Bits= 1

  uint8_t DRIVER_SEATBELT_BUCKLED_IS_VALID;       //      Bits= 1

  uint8_t PASS_SEATBELT_BUCKLED_IS_VALID;         //      Bits= 1

  uint8_t REAR_SEATBELT_BUCKLED_IS_VALID;         //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} OCCUPANCY_RPT_t;

// def @INTERIOR_LIGHTS_RPT CAN Message (1046 0x416)
#define INTERIOR_LIGHTS_RPT_IDE (0U)
#define INTERIOR_LIGHTS_RPT_DLC (3U)
#define INTERIOR_LIGHTS_RPT_CANID (0x416)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t FRONT_DOME_LIGHTS_ON : 1;            //      Bits= 1

  uint8_t REAR_DOME_LIGHTS_ON : 1;             //      Bits= 1

  uint8_t MOOD_LIGHTS_ON : 1;                  //      Bits= 1

  //  12 : "DIM_LEVEL_MAX"
  //  11 : "DIM_LEVEL_11"
  //  10 : "DIM_LEVEL_10"
  //  9 : "DIM_LEVEL_9"
  //  8 : "DIM_LEVEL_8"
  //  7 : "DIM_LEVEL_7"
  //  6 : "DIM_LEVEL_6"
  //  5 : "DIM_LEVEL_5"
  //  4 : "DIM_LEVEL_4"
  //  3 : "DIM_LEVEL_3"
  //  2 : "DIM_LEVEL_2"
  //  1 : "DIM_LEVEL_1"
  //  0 : "DIM_LEVEL_MIN"
  uint8_t DIM_LEVEL;                           //      Bits= 8

  uint8_t FRONT_DOME_LIGHTS_ON_IS_VALID : 1;   //      Bits= 1

  uint8_t REAR_DOME_LIGHTS_ON_IS_VALID : 1;    //      Bits= 1

  uint8_t MOOD_LIGHTS_ON_IS_VALID : 1;         //      Bits= 1

  uint8_t DIM_LEVEL_IS_VALID : 1;              //      Bits= 1

#else

  uint8_t FRONT_DOME_LIGHTS_ON;                //      Bits= 1

  uint8_t REAR_DOME_LIGHTS_ON;                 //      Bits= 1

  uint8_t MOOD_LIGHTS_ON;                      //      Bits= 1

  //  12 : "DIM_LEVEL_MAX"
  //  11 : "DIM_LEVEL_11"
  //  10 : "DIM_LEVEL_10"
  //  9 : "DIM_LEVEL_9"
  //  8 : "DIM_LEVEL_8"
  //  7 : "DIM_LEVEL_7"
  //  6 : "DIM_LEVEL_6"
  //  5 : "DIM_LEVEL_5"
  //  4 : "DIM_LEVEL_4"
  //  3 : "DIM_LEVEL_3"
  //  2 : "DIM_LEVEL_2"
  //  1 : "DIM_LEVEL_1"
  //  0 : "DIM_LEVEL_MIN"
  uint8_t DIM_LEVEL;                           //      Bits= 8

  uint8_t FRONT_DOME_LIGHTS_ON_IS_VALID;       //      Bits= 1

  uint8_t REAR_DOME_LIGHTS_ON_IS_VALID;        //      Bits= 1

  uint8_t MOOD_LIGHTS_ON_IS_VALID;             //      Bits= 1

  uint8_t DIM_LEVEL_IS_VALID;                  //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} INTERIOR_LIGHTS_RPT_t;

// def @DOOR_RPT CAN Message (1047 0x417)
#define DOOR_RPT_IDE (0U)
#define DOOR_RPT_DLC (2U)
#define DOOR_RPT_CANID (0x417)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t DRIVER_DOOR_OPEN : 1;                 //      Bits= 1

  uint8_t PASS_DOOR_OPEN : 1;                   //      Bits= 1

  uint8_t REAR_DRIVER_DOOR_OPEN : 1;            //      Bits= 1

  uint8_t REAR_PASS_DOOR_OPEN : 1;              //      Bits= 1

  uint8_t HOOD_OPEN : 1;                        //      Bits= 1

  uint8_t TRUNK_OPEN : 1;                       //      Bits= 1

  uint8_t FUEL_DOOR_OPEN : 1;                   //      Bits= 1

  uint8_t DRIVER_DOOR_OPEN_IS_VALID : 1;        //      Bits= 1

  uint8_t PASS_DOOR_OPEN_IS_VALID : 1;          //      Bits= 1

  uint8_t REAR_DRIVER_DOOR_OPEN_IS_VALID : 1;   //      Bits= 1

  uint8_t REAR_PASS_DOOR_OPEN_IS_VALID : 1;     //      Bits= 1

  uint8_t HOOD_OPEN_IS_VALID : 1;               //      Bits= 1

  uint8_t TRUNK_OPEN_IS_VALID : 1;              //      Bits= 1

  uint8_t FUEL_DOOR_OPEN_IS_VALID : 1;          //      Bits= 1

#else

  uint8_t DRIVER_DOOR_OPEN;                     //      Bits= 1

  uint8_t PASS_DOOR_OPEN;                       //      Bits= 1

  uint8_t REAR_DRIVER_DOOR_OPEN;                //      Bits= 1

  uint8_t REAR_PASS_DOOR_OPEN;                  //      Bits= 1

  uint8_t HOOD_OPEN;                            //      Bits= 1

  uint8_t TRUNK_OPEN;                           //      Bits= 1

  uint8_t FUEL_DOOR_OPEN;                       //      Bits= 1

  uint8_t DRIVER_DOOR_OPEN_IS_VALID;            //      Bits= 1

  uint8_t PASS_DOOR_OPEN_IS_VALID;              //      Bits= 1

  uint8_t REAR_DRIVER_DOOR_OPEN_IS_VALID;       //      Bits= 1

  uint8_t REAR_PASS_DOOR_OPEN_IS_VALID;         //      Bits= 1

  uint8_t HOOD_OPEN_IS_VALID;                   //      Bits= 1

  uint8_t TRUNK_OPEN_IS_VALID;                  //      Bits= 1

  uint8_t FUEL_DOOR_OPEN_IS_VALID;              //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} DOOR_RPT_t;

// def @REAR_LIGHTS_RPT CAN Message (1048 0x418)
#define REAR_LIGHTS_RPT_IDE (0U)
#define REAR_LIGHTS_RPT_DLC (2U)
#define REAR_LIGHTS_RPT_CANID (0x418)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  uint8_t BRAKE_LIGHTS_ON : 1;               //      Bits= 1

  uint8_t REVERSE_LIGHTS_ON : 1;             //      Bits= 1

  uint8_t BRAKE_LIGHTS_ON_IS_VALID : 1;      //      Bits= 1

  uint8_t REVERSE_LIGHTS_ON_IS_VALID : 1;    //      Bits= 1

#else

  uint8_t BRAKE_LIGHTS_ON;                   //      Bits= 1

  uint8_t REVERSE_LIGHTS_ON;                 //      Bits= 1

  uint8_t BRAKE_LIGHTS_ON_IS_VALID;          //      Bits= 1

  uint8_t REVERSE_LIGHTS_ON_IS_VALID;        //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} REAR_LIGHTS_RPT_t;

// def @LINEAR_ACCEL_RPT CAN Message (1049 0x419)
#define LINEAR_ACCEL_RPT_IDE (0U)
#define LINEAR_ACCEL_RPT_DLC (7U)
#define LINEAR_ACCEL_RPT_CANID (0x419)
// signal: @LATERAL_ACCEL_ro
#define PACMOD6_LATERAL_ACCEL_ro_CovFactor (0.010000)
#define PACMOD6_LATERAL_ACCEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD6_LATERAL_ACCEL_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @LONGITUDNAL_ACCEL_ro
#define PACMOD6_LONGITUDNAL_ACCEL_ro_CovFactor (0.010000)
#define PACMOD6_LONGITUDNAL_ACCEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD6_LONGITUDNAL_ACCEL_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )
// signal: @VERTICAL_ACCEL_ro
#define PACMOD6_VERTICAL_ACCEL_ro_CovFactor (0.010000)
#define PACMOD6_VERTICAL_ACCEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.010000)) )
#define PACMOD6_VERTICAL_ACCEL_ro_fromS(x) ( (((x) * (0.010000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t LATERAL_NEW_DATA_RX : 1;           //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t LONGITUDNAL_NEW_DATA_RX : 1;       //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t VERTICAL_NEW_DATA_RX : 1;          //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t LATERAL_VALID : 1;                 //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t LONGITUDNAL_VALID : 1;             //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t VERTICAL_VALID : 1;                //      Bits= 1

  int16_t LATERAL_ACCEL_ro;                  //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LATERAL_ACCEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t LONGITUDNAL_ACCEL_ro;              //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LONGITUDNAL_ACCEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t VERTICAL_ACCEL_ro;                 //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t VERTICAL_ACCEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t LATERAL_NEW_DATA_RX;               //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t LONGITUDNAL_NEW_DATA_RX;           //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t VERTICAL_NEW_DATA_RX;              //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t LATERAL_VALID;                     //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t LONGITUDNAL_VALID;                 //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t VERTICAL_VALID;                    //      Bits= 1

  int16_t LATERAL_ACCEL_ro;                  //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LATERAL_ACCEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t LONGITUDNAL_ACCEL_ro;              //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t LONGITUDNAL_ACCEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t VERTICAL_ACCEL_ro;                 //  [-] Bits=16 Factor= 0.010000        Unit:'m/s^2'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t VERTICAL_ACCEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} LINEAR_ACCEL_RPT_t;

// def @ANG_VEL_RPT CAN Message (1050 0x41a)
#define ANG_VEL_RPT_IDE (0U)
#define ANG_VEL_RPT_DLC (7U)
#define ANG_VEL_RPT_CANID (0x41a)
// signal: @PITCH_VEL_ro
#define PACMOD6_PITCH_VEL_ro_CovFactor (0.001000)
#define PACMOD6_PITCH_VEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_PITCH_VEL_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @ROLL_VEL_ro
#define PACMOD6_ROLL_VEL_ro_CovFactor (0.001000)
#define PACMOD6_ROLL_VEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_ROLL_VEL_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )
// signal: @YAW_VEL_ro
#define PACMOD6_YAW_VEL_ro_CovFactor (0.001000)
#define PACMOD6_YAW_VEL_ro_toS(x) ( (int16_t) (((x) - (0.000000)) / (0.001000)) )
#define PACMOD6_YAW_VEL_ro_fromS(x) ( (((x) * (0.001000)) + (0.000000)) )

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t PITCH_NEW_DATA_RX : 1;             //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t ROLL_NEW_DATA_RX : 1;              //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t YAW_NEW_DATA_RX : 1;               //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t PITCH_VALID : 1;                   //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t ROLL_VALID : 1;                    //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t YAW_VALID : 1;                     //      Bits= 1

  int16_t PITCH_VEL_ro;                      //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t PITCH_VEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t ROLL_VEL_ro;                       //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ROLL_VEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t YAW_VEL_ro;                        //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t YAW_VEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

#else

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t PITCH_NEW_DATA_RX;                 //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t ROLL_NEW_DATA_RX;                  //      Bits= 1

  //  0 : "NEW_DATA_NOT_RX"
  //  1 : "NEW_DATA_RX"
  uint8_t YAW_NEW_DATA_RX;                   //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t PITCH_VALID;                       //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t ROLL_VALID;                        //      Bits= 1

  //  0 : "NOT_VALID"
  //  1 : "VALID"
  uint8_t YAW_VALID;                         //      Bits= 1

  int16_t PITCH_VEL_ro;                      //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t PITCH_VEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t ROLL_VEL_ro;                       //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t ROLL_VEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

  int16_t YAW_VEL_ro;                        //  [-] Bits=16 Factor= 0.001000        Unit:'rad/s'

#ifdef PACMOD6_USE_SIGFLOAT
  sigfloat_t YAW_VEL_phys;
#endif // PACMOD6_USE_SIGFLOAT

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} ANG_VEL_RPT_t;

// def @NOTIFICATION_CMD CAN Message (1051 0x41b)
#define NOTIFICATION_CMD_IDE (0U)
#define NOTIFICATION_CMD_DLC (1U)
#define NOTIFICATION_CMD_CANID (0x41b)
#define NOTIFICATION_CMD_CYC (250U)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "NOT_MUTED"
  //  1 : "MUTED"
  uint8_t BUZZER_MUTE : 1;                   //      Bits= 1

  //  0 : "NO_ACTION"
  //  1 : "WHITE"
  uint8_t UNDERDASH_LIGHTS_WHITE : 1;        //      Bits= 1

#else

  //  0 : "NOT_MUTED"
  //  1 : "MUTED"
  uint8_t BUZZER_MUTE;                       //      Bits= 1

  //  0 : "NO_ACTION"
  //  1 : "WHITE"
  uint8_t UNDERDASH_LIGHTS_WHITE;            //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} NOTIFICATION_CMD_t;

// def @ESTOP_RPT CAN Message (1052 0x41c)
#define ESTOP_RPT_IDE (0U)
#define ESTOP_RPT_DLC (1U)
#define ESTOP_RPT_CANID (0x41c)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "RELEASED"
  //  1 : "PRESSED"
  uint8_t ESTOP : 1;                         //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t ESTOP_FAULT : 1;                   //      Bits= 1

#else

  //  0 : "RELEASED"
  //  1 : "PRESSED"
  uint8_t ESTOP;                             //      Bits= 1

  //  0 : "NO_FAULT"
  //  1 : "FAULT"
  uint8_t ESTOP_FAULT;                       //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} ESTOP_RPT_t;

// def @WATCHDOG_RPT CAN Message (1536 0x600)
#define WATCHDOG_RPT_IDE (0U)
#define WATCHDOG_RPT_DLC (8U)
#define WATCHDOG_RPT_CANID (0x600)
#define WATCHDOG_RPT_CYC (100U)

typedef struct
{
#ifdef PACMOD6_USE_BITS_SIGNAL

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_ENABLE_FLAG : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_OVERRIDE_ACTIVE : 1;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_COMMAND_TIMEOUT_ERROR : 1;         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_PACMOD_SUBSYSTEM_TIMEOUT : 1;      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_VEHICLE_CAN_TIMEOUT : 1;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_PACMOD_SYSTEM_FAULT_ACTIVE : 1;    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_CONFIG_FAULT_ACTIVE : 1;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_TIMEOUT : 1;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_ENABLED : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_OVERRIDE_ACTIVE : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_COMMAND_OUTPUT_FAULT : 1;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_INPUT_OUTPUT_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_OUTPUT_REPORTED_FAULT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_PACMOD_FAULT : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_VEHICLE_FAULT : 1;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_TIMEOUT : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_ENABLED : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_OVERRIDE_ACTIVE : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_COMMAND_OUTPUT_FAULT : 1;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_INPUT_OUTPUT_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_OUTPUT_REPORTED_FAULT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_PACMOD_FAULT : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_VEHICLE_FAULT : 1;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_TIMEOUT : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_ENABLED : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_OVERRIDE_ACTIVE : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_COMMAND_OUTPUT_FAULT : 1;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_INPUT_OUTPUT_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_OUTPUT_REPORTED_FAULT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_PACMOD_FAULT : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_VEHICLE_FAULT : 1;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_TIMEOUT : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_ENABLED : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_OVERRIDE_ACTIVE : 1;                //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_COMMAND_OUTPUT_FAULT : 1;           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_INPUT_OUTPUT_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_OUTPUT_REPORTED_FAULT : 1;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_PACMOD_FAULT : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_VEHICLE_FAULT : 1;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_TIMEOUT : 1;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD1_CONFIG_FAULT : 1;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD1_CAN_TIMEOUT : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD1_COUNTER_FAULT : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD2_CONFIG_FAULT : 1;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD2_CAN_TIMEOUT : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD2_COUNTER_FAULT : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD3_CONFIG_FAULT : 1;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD3_CAN_TIMEOUT : 1;                     //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD3_COUNTER_FAULT : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI1_RPT_TIMEOUT : 1;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI1_CONFIG_FAULT : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI1_CAN_TIMEOUT : 1;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI1_COUNTER_FAULT : 1;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI2_RPT_TIMEOUT : 1;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI2_CONFIG_FAULT : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI2_CAN_TIMEOUT : 1;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI2_COUNTER_FAULT : 1;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI3_RPT_TIMEOUT : 1;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI3_CONFIG_FAULT : 1;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI3_CAN_TIMEOUT : 1;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI3_COUNTER_FAULT : 1;                  //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD_SYSTEM_PRESENT_FAULT : 1;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI_SYSTEM_PRESENT_FAULT : 1;            //      Bits= 1

  uint8_t GLOBAL_INTERNAL_POWER_SUPPLY_FAULT : 1;   //      Bits= 1

#else

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_ENABLE_FLAG;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_OVERRIDE_ACTIVE;                   //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_COMMAND_TIMEOUT_ERROR;             //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_PACMOD_SUBSYSTEM_TIMEOUT;          //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_VEHICLE_CAN_TIMEOUT;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_PACMOD_SYSTEM_FAULT_ACTIVE;        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_CONFIG_FAULT_ACTIVE;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t GLOBAL_TIMEOUT;                           //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_ENABLED;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_OVERRIDE_ACTIVE;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_COMMAND_OUTPUT_FAULT;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_INPUT_OUTPUT_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_OUTPUT_REPORTED_FAULT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_PACMOD_FAULT;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_VEHICLE_FAULT;                      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t ACCEL_TIMEOUT;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_ENABLED;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_OVERRIDE_ACTIVE;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_COMMAND_OUTPUT_FAULT;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_INPUT_OUTPUT_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_OUTPUT_REPORTED_FAULT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_PACMOD_FAULT;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_VEHICLE_FAULT;                      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t BRAKE_TIMEOUT;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_ENABLED;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_OVERRIDE_ACTIVE;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_COMMAND_OUTPUT_FAULT;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_INPUT_OUTPUT_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_OUTPUT_REPORTED_FAULT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_PACMOD_FAULT;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_VEHICLE_FAULT;                      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t SHIFT_TIMEOUT;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_ENABLED;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_OVERRIDE_ACTIVE;                    //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_COMMAND_OUTPUT_FAULT;               //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_INPUT_OUTPUT_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_OUTPUT_REPORTED_FAULT;              //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_PACMOD_FAULT;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_VEHICLE_FAULT;                      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t STEER_TIMEOUT;                            //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD1_CONFIG_FAULT;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD1_CAN_TIMEOUT;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD1_COUNTER_FAULT;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD2_CONFIG_FAULT;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD2_CAN_TIMEOUT;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD2_COUNTER_FAULT;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD3_CONFIG_FAULT;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD3_CAN_TIMEOUT;                         //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD3_COUNTER_FAULT;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI1_RPT_TIMEOUT;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI1_CONFIG_FAULT;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI1_CAN_TIMEOUT;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI1_COUNTER_FAULT;                      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI2_RPT_TIMEOUT;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI2_CONFIG_FAULT;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI2_CAN_TIMEOUT;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI2_COUNTER_FAULT;                      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI3_RPT_TIMEOUT;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI3_CONFIG_FAULT;                       //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI3_CAN_TIMEOUT;                        //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI3_COUNTER_FAULT;                      //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MOD_SYSTEM_PRESENT_FAULT;                 //      Bits= 1

  //  0 : "FALSE"
  //  1 : "TRUE"
  uint8_t MINI_SYSTEM_PRESENT_FAULT;                //      Bits= 1

  uint8_t GLOBAL_INTERNAL_POWER_SUPPLY_FAULT;       //      Bits= 1

#endif // PACMOD6_USE_BITS_SIGNAL

#ifdef PACMOD6_USE_DIAG_MONITORS

  FrameMonitor_t mon1;

#endif // PACMOD6_USE_DIAG_MONITORS

} WATCHDOG_RPT_t;

// Function signatures

uint32_t Unpack_GLOBAL_RPT_pacmod6(GLOBAL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_GLOBAL_RPT_pacmod6(GLOBAL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_GLOBAL_RPT_pacmod6(GLOBAL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_00_pacmod6(COMPONENT_RPT_00_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_COMPONENT_RPT_00_pacmod6(COMPONENT_RPT_00_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_COMPONENT_RPT_00_pacmod6(COMPONENT_RPT_00_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_01_pacmod6(COMPONENT_RPT_01_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_COMPONENT_RPT_01_pacmod6(COMPONENT_RPT_01_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_COMPONENT_RPT_01_pacmod6(COMPONENT_RPT_01_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_02_pacmod6(COMPONENT_RPT_02_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_COMPONENT_RPT_02_pacmod6(COMPONENT_RPT_02_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_COMPONENT_RPT_02_pacmod6(COMPONENT_RPT_02_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_COMPONENT_RPT_03_pacmod6(COMPONENT_RPT_03_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_COMPONENT_RPT_03_pacmod6(COMPONENT_RPT_03_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_COMPONENT_RPT_03_pacmod6(COMPONENT_RPT_03_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_GLOBAL_CMD_pacmod6(GLOBAL_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_GLOBAL_CMD_pacmod6(GLOBAL_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_GLOBAL_CMD_pacmod6(GLOBAL_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_ACCEL_CMD_pacmod6(ACCEL_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_ACCEL_CMD_pacmod6(ACCEL_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_CMD_pacmod6(ACCEL_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_BRAKE_CMD_pacmod6(BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_BRAKE_CMD_pacmod6(BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_CMD_pacmod6(BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_CRUISE_CONTROL_BUTTONS_CMD_pacmod6(CRUISE_CONTROL_BUTTONS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_CRUISE_CONTROL_BUTTONS_CMD_pacmod6(CRUISE_CONTROL_BUTTONS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CRUISE_CONTROL_BUTTONS_CMD_pacmod6(CRUISE_CONTROL_BUTTONS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_LEFT_CMD_pacmod6(DASH_CONTROLS_LEFT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_LEFT_CMD_pacmod6(DASH_CONTROLS_LEFT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_LEFT_CMD_pacmod6(DASH_CONTROLS_LEFT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_RIGHT_CMD_pacmod6(DASH_CONTROLS_RIGHT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_RIGHT_CMD_pacmod6(DASH_CONTROLS_RIGHT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_RIGHT_CMD_pacmod6(DASH_CONTROLS_RIGHT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_HAZARD_LIGHTS_CMD_pacmod6(HAZARD_LIGHTS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_HAZARD_LIGHTS_CMD_pacmod6(HAZARD_LIGHTS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HAZARD_LIGHTS_CMD_pacmod6(HAZARD_LIGHTS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_CMD_pacmod6(HEADLIGHT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_HEADLIGHT_CMD_pacmod6(HEADLIGHT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HEADLIGHT_CMD_pacmod6(HEADLIGHT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_HORN_CMD_pacmod6(HORN_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_HORN_CMD_pacmod6(HORN_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HORN_CMD_pacmod6(HORN_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_MEDIA_CONTROLS_CMD_pacmod6(MEDIA_CONTROLS_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_MEDIA_CONTROLS_CMD_pacmod6(MEDIA_CONTROLS_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MEDIA_CONTROLS_CMD_pacmod6(MEDIA_CONTROLS_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_PARKING_BRAKE_CMD_pacmod6(PARKING_BRAKE_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_PARKING_BRAKE_CMD_pacmod6(PARKING_BRAKE_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_PARKING_BRAKE_CMD_pacmod6(PARKING_BRAKE_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_SHIFT_CMD_pacmod6(SHIFT_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_SHIFT_CMD_pacmod6(SHIFT_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SHIFT_CMD_pacmod6(SHIFT_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_STEERING_CMD_pacmod6(STEERING_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_STEERING_CMD_pacmod6(STEERING_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_CMD_pacmod6(STEERING_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_TURN_CMD_pacmod6(TURN_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_TURN_CMD_pacmod6(TURN_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TURN_CMD_pacmod6(TURN_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_WIPER_CMD_pacmod6(WIPER_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_WIPER_CMD_pacmod6(WIPER_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WIPER_CMD_pacmod6(WIPER_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_SPRAYER_CMD_pacmod6(SPRAYER_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_SPRAYER_CMD_pacmod6(SPRAYER_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SPRAYER_CMD_pacmod6(SPRAYER_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_BRAKE_DECCEL_CMD_pacmod6(BRAKE_DECCEL_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_BRAKE_DECCEL_CMD_pacmod6(BRAKE_DECCEL_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_DECCEL_CMD_pacmod6(BRAKE_DECCEL_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_ACCEL_RPT_pacmod6(ACCEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_ACCEL_RPT_pacmod6(ACCEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_RPT_pacmod6(ACCEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_ACCEL_CMD_LIMIT_RPT_pacmod6(ACCEL_CMD_LIMIT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_ACCEL_CMD_LIMIT_RPT_pacmod6(ACCEL_CMD_LIMIT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_CMD_LIMIT_RPT_pacmod6(ACCEL_CMD_LIMIT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_BRAKE_RPT_pacmod6(BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_BRAKE_RPT_pacmod6(BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_RPT_pacmod6(BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_BRAKE_CMD_LIMIT_RPT_pacmod6(BRAKE_CMD_LIMIT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_BRAKE_CMD_LIMIT_RPT_pacmod6(BRAKE_CMD_LIMIT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_CMD_LIMIT_RPT_pacmod6(BRAKE_CMD_LIMIT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_CRUISE_CONTROL_BUTTONS_RPT_pacmod6(CRUISE_CONTROL_BUTTONS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_CRUISE_CONTROL_BUTTONS_RPT_pacmod6(CRUISE_CONTROL_BUTTONS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_CRUISE_CONTROL_BUTTONS_RPT_pacmod6(CRUISE_CONTROL_BUTTONS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_LEFT_RPT_pacmod6(DASH_CONTROLS_LEFT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_LEFT_RPT_pacmod6(DASH_CONTROLS_LEFT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_LEFT_RPT_pacmod6(DASH_CONTROLS_LEFT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_DASH_CONTROLS_RIGHT_RPT_pacmod6(DASH_CONTROLS_RIGHT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_DASH_CONTROLS_RIGHT_RPT_pacmod6(DASH_CONTROLS_RIGHT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DASH_CONTROLS_RIGHT_RPT_pacmod6(DASH_CONTROLS_RIGHT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_HAZARD_LIGHTS_RPT_pacmod6(HAZARD_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_HAZARD_LIGHTS_RPT_pacmod6(HAZARD_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HAZARD_LIGHTS_RPT_pacmod6(HAZARD_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_RPT_pacmod6(HEADLIGHT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_HEADLIGHT_RPT_pacmod6(HEADLIGHT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HEADLIGHT_RPT_pacmod6(HEADLIGHT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_HORN_RPT_pacmod6(HORN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_HORN_RPT_pacmod6(HORN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HORN_RPT_pacmod6(HORN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_MEDIA_CONTROLS_RPT_pacmod6(MEDIA_CONTROLS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_MEDIA_CONTROLS_RPT_pacmod6(MEDIA_CONTROLS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_MEDIA_CONTROLS_RPT_pacmod6(MEDIA_CONTROLS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_PARKING_BRAKE_RPT_pacmod6(PARKING_BRAKE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_PARKING_BRAKE_RPT_pacmod6(PARKING_BRAKE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_PARKING_BRAKE_RPT_pacmod6(PARKING_BRAKE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_SHIFT_RPT_pacmod6(SHIFT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_SHIFT_RPT_pacmod6(SHIFT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SHIFT_RPT_pacmod6(SHIFT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_STEERING_RPT_pacmod6(STEERING_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_STEERING_RPT_pacmod6(STEERING_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_RPT_pacmod6(STEERING_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_STEERING_CMD_LIMIT_RPT_pacmod6(STEERING_CMD_LIMIT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_STEERING_CMD_LIMIT_RPT_pacmod6(STEERING_CMD_LIMIT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_CMD_LIMIT_RPT_pacmod6(STEERING_CMD_LIMIT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_TURN_RPT_pacmod6(TURN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_TURN_RPT_pacmod6(TURN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TURN_RPT_pacmod6(TURN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_WIPER_RPT_pacmod6(WIPER_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_WIPER_RPT_pacmod6(WIPER_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WIPER_RPT_pacmod6(WIPER_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_SPRAYER_RPT_pacmod6(SPRAYER_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_SPRAYER_RPT_pacmod6(SPRAYER_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SPRAYER_RPT_pacmod6(SPRAYER_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_BRAKE_DECCEL_RPT_pacmod6(BRAKE_DECCEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_BRAKE_DECCEL_RPT_pacmod6(BRAKE_DECCEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_DECCEL_RPT_pacmod6(BRAKE_DECCEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_ACCEL_AUX_RPT_pacmod6(ACCEL_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_ACCEL_AUX_RPT_pacmod6(ACCEL_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ACCEL_AUX_RPT_pacmod6(ACCEL_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_BRAKE_AUX_RPT_pacmod6(BRAKE_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_BRAKE_AUX_RPT_pacmod6(BRAKE_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_AUX_RPT_pacmod6(BRAKE_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_HEADLIGHT_AUX_RPT_pacmod6(HEADLIGHT_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_HEADLIGHT_AUX_RPT_pacmod6(HEADLIGHT_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_HEADLIGHT_AUX_RPT_pacmod6(HEADLIGHT_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_SHIFT_AUX_RPT_pacmod6(SHIFT_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_SHIFT_AUX_RPT_pacmod6(SHIFT_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SHIFT_AUX_RPT_pacmod6(SHIFT_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_STEERING_AUX_RPT_pacmod6(STEERING_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_STEERING_AUX_RPT_pacmod6(STEERING_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_AUX_RPT_pacmod6(STEERING_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_TURN_AUX_RPT_pacmod6(TURN_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_TURN_AUX_RPT_pacmod6(TURN_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_TURN_AUX_RPT_pacmod6(TURN_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_WIPER_AUX_RPT_pacmod6(WIPER_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_WIPER_AUX_RPT_pacmod6(WIPER_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WIPER_AUX_RPT_pacmod6(WIPER_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_BRAKE_DECCEL_AUX_RPT_pacmod6(BRAKE_DECCEL_AUX_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_BRAKE_DECCEL_AUX_RPT_pacmod6(BRAKE_DECCEL_AUX_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_DECCEL_AUX_RPT_pacmod6(BRAKE_DECCEL_AUX_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_VEHICLE_SPEED_RPT_pacmod6(VEHICLE_SPEED_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_VEHICLE_SPEED_RPT_pacmod6(VEHICLE_SPEED_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEHICLE_SPEED_RPT_pacmod6(VEHICLE_SPEED_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_1_pacmod6(BRAKE_MOTOR_RPT_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_BRAKE_MOTOR_RPT_1_pacmod6(BRAKE_MOTOR_RPT_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_MOTOR_RPT_1_pacmod6(BRAKE_MOTOR_RPT_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_2_pacmod6(BRAKE_MOTOR_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_BRAKE_MOTOR_RPT_2_pacmod6(BRAKE_MOTOR_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_MOTOR_RPT_2_pacmod6(BRAKE_MOTOR_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_BRAKE_MOTOR_RPT_3_pacmod6(BRAKE_MOTOR_RPT_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_BRAKE_MOTOR_RPT_3_pacmod6(BRAKE_MOTOR_RPT_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_BRAKE_MOTOR_RPT_3_pacmod6(BRAKE_MOTOR_RPT_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_1_pacmod6(STEERING_MOTOR_RPT_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_STEERING_MOTOR_RPT_1_pacmod6(STEERING_MOTOR_RPT_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_MOTOR_RPT_1_pacmod6(STEERING_MOTOR_RPT_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_2_pacmod6(STEERING_MOTOR_RPT_2_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_STEERING_MOTOR_RPT_2_pacmod6(STEERING_MOTOR_RPT_2_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_MOTOR_RPT_2_pacmod6(STEERING_MOTOR_RPT_2_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_STEERING_MOTOR_RPT_3_pacmod6(STEERING_MOTOR_RPT_3_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_STEERING_MOTOR_RPT_3_pacmod6(STEERING_MOTOR_RPT_3_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_STEERING_MOTOR_RPT_3_pacmod6(STEERING_MOTOR_RPT_3_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_WHEEL_SPEED_RPT_pacmod6(WHEEL_SPEED_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_WHEEL_SPEED_RPT_pacmod6(WHEEL_SPEED_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WHEEL_SPEED_RPT_pacmod6(WHEEL_SPEED_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_00_pacmod6(SOFTWARE_VERSION_RPT_00_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_SOFTWARE_VERSION_RPT_00_pacmod6(SOFTWARE_VERSION_RPT_00_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SOFTWARE_VERSION_RPT_00_pacmod6(SOFTWARE_VERSION_RPT_00_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_01_pacmod6(SOFTWARE_VERSION_RPT_01_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_SOFTWARE_VERSION_RPT_01_pacmod6(SOFTWARE_VERSION_RPT_01_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SOFTWARE_VERSION_RPT_01_pacmod6(SOFTWARE_VERSION_RPT_01_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_02_pacmod6(SOFTWARE_VERSION_RPT_02_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_SOFTWARE_VERSION_RPT_02_pacmod6(SOFTWARE_VERSION_RPT_02_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SOFTWARE_VERSION_RPT_02_pacmod6(SOFTWARE_VERSION_RPT_02_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_SOFTWARE_VERSION_RPT_03_pacmod6(SOFTWARE_VERSION_RPT_03_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_SOFTWARE_VERSION_RPT_03_pacmod6(SOFTWARE_VERSION_RPT_03_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_SOFTWARE_VERSION_RPT_03_pacmod6(SOFTWARE_VERSION_RPT_03_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_YAW_RATE_RPT_pacmod6(YAW_RATE_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_YAW_RATE_RPT_pacmod6(YAW_RATE_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_YAW_RATE_RPT_pacmod6(YAW_RATE_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_LAT_LON_HEADING_RPT_pacmod6(LAT_LON_HEADING_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_LAT_LON_HEADING_RPT_pacmod6(LAT_LON_HEADING_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LAT_LON_HEADING_RPT_pacmod6(LAT_LON_HEADING_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_DATE_TIME_RPT_pacmod6(DATE_TIME_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_DATE_TIME_RPT_pacmod6(DATE_TIME_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DATE_TIME_RPT_pacmod6(DATE_TIME_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_DETECTED_OBJECT_RPT_pacmod6(DETECTED_OBJECT_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_DETECTED_OBJECT_RPT_pacmod6(DETECTED_OBJECT_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DETECTED_OBJECT_RPT_pacmod6(DETECTED_OBJECT_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_VEH_SPECIFIC_RPT_1_pacmod6(VEH_SPECIFIC_RPT_1_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_VEH_SPECIFIC_RPT_1_pacmod6(VEH_SPECIFIC_RPT_1_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEH_SPECIFIC_RPT_1_pacmod6(VEH_SPECIFIC_RPT_1_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_VEH_DYNAMICS_RPT_pacmod6(VEH_DYNAMICS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_VEH_DYNAMICS_RPT_pacmod6(VEH_DYNAMICS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VEH_DYNAMICS_RPT_pacmod6(VEH_DYNAMICS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_VIN_RPT_pacmod6(VIN_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_VIN_RPT_pacmod6(VIN_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_VIN_RPT_pacmod6(VIN_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_OCCUPANCY_RPT_pacmod6(OCCUPANCY_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_OCCUPANCY_RPT_pacmod6(OCCUPANCY_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_OCCUPANCY_RPT_pacmod6(OCCUPANCY_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_INTERIOR_LIGHTS_RPT_pacmod6(INTERIOR_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_INTERIOR_LIGHTS_RPT_pacmod6(INTERIOR_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_INTERIOR_LIGHTS_RPT_pacmod6(INTERIOR_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_DOOR_RPT_pacmod6(DOOR_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_DOOR_RPT_pacmod6(DOOR_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_DOOR_RPT_pacmod6(DOOR_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_REAR_LIGHTS_RPT_pacmod6(REAR_LIGHTS_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_REAR_LIGHTS_RPT_pacmod6(REAR_LIGHTS_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_REAR_LIGHTS_RPT_pacmod6(REAR_LIGHTS_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_LINEAR_ACCEL_RPT_pacmod6(LINEAR_ACCEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_LINEAR_ACCEL_RPT_pacmod6(LINEAR_ACCEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_LINEAR_ACCEL_RPT_pacmod6(LINEAR_ACCEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_ANG_VEL_RPT_pacmod6(ANG_VEL_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_ANG_VEL_RPT_pacmod6(ANG_VEL_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ANG_VEL_RPT_pacmod6(ANG_VEL_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_NOTIFICATION_CMD_pacmod6(NOTIFICATION_CMD_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_NOTIFICATION_CMD_pacmod6(NOTIFICATION_CMD_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_NOTIFICATION_CMD_pacmod6(NOTIFICATION_CMD_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_ESTOP_RPT_pacmod6(ESTOP_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_ESTOP_RPT_pacmod6(ESTOP_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_ESTOP_RPT_pacmod6(ESTOP_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

uint32_t Unpack_WATCHDOG_RPT_pacmod6(WATCHDOG_RPT_t* _m, const uint8_t* _d, uint8_t dlc_);
#ifdef PACMOD6_USE_CANSTRUCT
uint32_t Pack_WATCHDOG_RPT_pacmod6(WATCHDOG_RPT_t* _m, __CoderDbcCanFrame_t__* cframe);
#else
uint32_t Pack_WATCHDOG_RPT_pacmod6(WATCHDOG_RPT_t* _m, uint8_t* _d, uint8_t* _len, uint8_t* _ide);
#endif // PACMOD6_USE_CANSTRUCT

#ifdef __cplusplus
}
#endif
