#ifndef SRC_CHANNELS_H_
#define SRC_CHANNELS_H_

/*---------------------------------------------------
 *                      Channels
 * --------------------------------------------------
 */

//-------------------MOTORS------------------
const static int CH_LEFTDRIVE = 1;
const static int CH_RIGHTDRIVE = 2;

const static int CH_SHOOTERA = 1;
const static int CH_SHOOTERB = 0;

const static int CH_ANGLEMOTOR = 2;

const static int CH_LIFTMOTOR = 3;
//-------------------LIMIT SWITCHES------------
const static int CH_SHOOTER_ANGLE_BOTTOM = 0;
const static int CH_SHOOTER_ANGLE_TOP = 1;

//--------------------HUMAN INPUT--------------
const static int CH_DRIVESTICK = 0;
const static int CH_SPECIALSTICK = 1;

//--------------------MISC---------------------
const static int CH_PDP = 0;

//-------------------PNEUMATICS------------------
const static int CH_PCMA = 0;
const static int CH_PCMB = 1;

const static int CH_SHIFTER_PCM = CH_PCMA;
const static int CH_SHIFTER_FW = 0;
const static int CH_SHIFTER_RV = 1;

const static int CH_HANG_PCM = CH_PCMA;
const static int CH_HANG_FW = 2;
const static int CH_HANG_RV = 3;

const static int CH_SHOOTSTICK_PCM = CH_PCMA;
const static int CH_SHOOTSTICK_FW = 6;
const static int CH_SHOOTSTICK_RV = 7;

const static int CH_ARMMAIN = 4;
const static int CH_ARMSECONDARY = 2;

/* ----------------------------------------------------------------
 *                          Buttons
 * ----------------------------------------------------------------
 */

//------------------Driver Stick-------------------------
const static int BUT_SHIFTER = 1;
const static int BUT_HANG = 2;
const static int BUT_BREACH2 = 10;
const static int BUT_BREACH3 = 7;
const static int BUT_BREACH4 = 11;
const static int BUT_BREACH5 = 8;

//-----------------Specials Stick-------------------------
const static int BUT_FIRE = 1;
const static int BUT_REVUP = 3;
const static int BUT_SUCKIN = 2;
const static int BUT_AUTOAIMA = 4;
const static int BUT_AUTOAIMB = 5;

const static int BUT_ARMMAIN_FW = 6;
const static int BUT_ARMMAIN_RV = 7;
const static int BUT_ARMSEC_FW = 11;
const static int BUT_ARMSEC_RV = 10;

#endif
