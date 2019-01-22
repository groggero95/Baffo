#ifndef CONFIG_H
#define CONFIG_H
/******************************************************************************/
/*                                    DEFINE                                  */
/******************************************************************************/

#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))
#define PI 3.141592653589793
#define DIAM 5.6
#define RAD 2.8
#define CF_RAMP_UP 2/3
#define CF_RAMP_DW 2/3
#define MAX_SPEED 1050
#define AXE_WHEELS 9.85
#define ROT_ADJ 3.0/5
#define ROBOT_WIDTH 120
#define ROBOT_LENGTH 320
#define ROBOT_SENSOR_DIST 60
#define FIELD_WIDTH 1200 - ROBOT_WIDTH/2
#define FIELD_LENGTH_FRONT 800 - ROBOT_LENGTH/2
#define FIELD_LENGTH_BACK 200 - ROBOT_LENGTH/2
#define START_POS_X 0
#define START_POS_Y 0
#define ANGLE_X FIELD_WIDTH/2-ROBOT_SENSOR_DIST
#define ANGLE_Y 100-10-ROBOT_SENSOR_DIST-ROBOT_LENGTH/2
#define X0 0
#define Y0 0
#define N_AREAS 7
#define THROWBALL_OFFSET 70
#define CALIBRATION_ROT 0

/* Needed for Bluetooth communication */
//#define SERV_ADDR	"aa:bb:cc:dd:ee:ff"     /* Whatever the address of the server is */
//#define SERV_ADDR	"AC:7B:A1:A5:34:E8"	/* Ale PC */
#define SERV_ADDR	  "34:41:5D:D9:AE:7B"	/* Giulio PC */
//#define SERV_ADDR	  "14:AB:C5:34:E7:FB"

	/* Giulio PC */
#define TEAM_ID     1                   /* Your team ID */

#define MSG_ACK     0
#define MSG_START   1
#define MSG_STOP    2
#define MSG_KICK    3
#define MSG_SCORE   4
#define MSG_CUSTOM  8

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
#include <windows.h>
// UNIX //////////////////////////////////////////
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
//////////////////////////////////////////////////
#endif

enum Elaboration_Code {BALL_SHOT,NOT_FOUND,OBJ_IN_AREA,AREA_326,AREA_FREE};
enum Search_Type {RADIUS, ELLIPTIC};
enum Dir {N = 0,E = 90,S = 180,W = 270};
enum Mode {DEFAULT, AGGRESSIVE};

#endif /* CONFIG_H */
