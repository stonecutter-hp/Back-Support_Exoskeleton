/***********************************************************************
 * The Finite State Machine Alogrithm
 **********************************************************************/

#include "FSM.h"

// motion type: 1-Standing;       2-Walking; 
//              3-Lowering;       4-Grasping;
//              5-Lifting;        0-Exit state.
MotionType mode;
MotionType PreMode;         // last time's motion mode
// tech type: 1-Stoop bending; 2-Squat bending; 3-Semi-squat bending
uint8_t tech;            // bending tech flag
// 1-left; 2-right; 0-none
uint8_t side;            // Asymmetric side flag
HLCont Subject1;         // High-level control parameters for specific subjects
