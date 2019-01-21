#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include <pthread.h>
#include "config.h"
#include "lib.h"
#include "bt.h"

const char const *color[] = {"?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };

/****************************************************************************************************/
/*                                   GLOBAL VARIABLES                                               */
/****************************************************************************************************/
struct Position pos;
uint8_t sn_ball;		  //tacho to throw the ball
uint8_t sn_lift;      //tacho to lift the ball
uint8_t sn_tacho[3]={DESC_LIMIT, DESC_LIMIT, DESC_LIMIT};  //2 tacho motors, 3rd one for closing the multi
uint8_t sn_gyro;      //gyroscope
uint8_t sn_us;        //ultrasonic distance sensor
uint8_t sn_touch;
uint8_t sn_color;
uint8_t sn_sonar;
uint8_t sn_mag;

FLAGS_T state;
char s[ 256 ];
int val;
float value;

pthread_mutex_t sem_gyro;
pthread_mutex_t sem_us;
pthread_t thread[2];

volatile int gyro_dir = 0.0;
volatile int us_dist = 0.0;
volatile int flag_kill = 0;

int s_bt;
uint16_t msgId = 0;
char to_bt[6];

/****************************************************************************************************/
/****************************************************************************************************/
/*
int elaborate_dist(uint8_t *sn_tacho, uint8_t sn_ball, uint8_t sn_lift, struct Position pos, int dist){
  int dist_tmp;
  int balls;

  if(dist>0){
    //ball found
    //if distance is > 300 cm go even closer and search again
    if(dist<300){
      //TBI check factor dist*... is different from zero
      go_straight_mm(dist-100, sn_tacho, 0);
      //There is a ball?
      //check with color sensor or distance sensor
      dist_tmp=read_us(sn_us,5);
      printf("Distance after moving towards the ball: %d\n", dist_tmp);
      if(read_us(sn_us,5)<=110){
        //ball near enought
        //liftball(sn_lift, sn_ball);
        //return_to_center(sn_tacho);
        //throwball(sn_ball, 0.8);
        //balls++;
        //send scored
        return BALL_SHOT;
      } else {
        //return_to_center(sn_tacho);
        //probably wrong or simple search from that position
        //rotate(-(pos.deg-180));
        //simple_search();
        //return NOT_FOUND;
        return OBJ_IN_AREA;
      }
    } else {
      //go near that area and search again
      go_straight_mm(dist-200, sn_tacho, 0);
      return OBJ_IN_AREA;
      //return_to_center(distance, sn_tacho);
    }
  } else if(dist<0){
    //in this case something is found but not really detected
    dist = 130;
    do{
        dist -= 20;

    } while(!go_straight_mm(dist, sn_tacho, 0));
    return AREA_326;
  } else {
    //set the area as free

  }

  return AREA_FREE;

}
*/
//this is the function that search a ball and score
void alg_flow(uint8_t *sn_tacho, uint8_t sn_ball, uint8_t sn_lift, struct Position pos, struct Search_Areas *areas, enum Mode mode){
  int dist, dist_tmp, lift;
  int balls, i;
  int found_ball = 0;
  //at start robot has two balls
  //score 3 points line and score the two balls
  go_straight_mm(120, sn_tacho, 0);
  start_throwball(sn_ball);
  liftball(sn_lift, sn_ball);
  throwball(sn_ball, 1);
  go_straight_mm(-120, sn_tacho, 0);
  /*
  //scan front area to verify to have scored
  dist=read_us(sn_us, 10);
  if(dist<=120){
    //lucky case in which the ball returns between the baffi
    liftball(sn_lift, sn_ball);
    throwball(sn_ball, 1);
  }
  */
  balls=2;

  //TODO hopefully send 6 points scored message
  if(mode==DEFAULT){
    //Scanning phase
    for(i=0; i<N_AREAS; i++){
      //go to the scan Position
      go_to_point90(areas[i].posx, areas[i].posy, sn_tacho, areas[i].dir);
      Sleep(300);
      dist=continous_search(areas[i]);
      if(dist>0){
        //go towards ball
        go_straight_mm(dist-90, sn_tacho, 1);
        Sleep(300);
        // check if the ball is still in front of the robot
        dist_tmp=read_us(sn_us, 10);
        if (dist_tmp > 200){
          // the robot was not aligned with the ball and lost it, search closer
          printf("\ndist was %d --> CLOSE_RANGE\n\n", dist_tmp);
          // back up a little to have the ball in range
        	go_straight_mm(-50, sn_tacho, 1);
          dist = closerange_search();
          if (dist > 0) {
            go_straight_mm(dist-90, sn_tacho, 1);
          }
        }
        if (dist > 0){
          lift = liftball(sn_lift, sn_ball);
          if(lift > 0){
            found_ball = 1;
            // calibrate before throwing in area 3 and 4
            if (i==2 || i==4){
              calibrate();
            }
            //return_to_center(sn_tacho);
          	go_to_point90(0, 120, sn_tacho, N);
          	Sleep(300);
            //go_straight_mm(120, sn_tacho, 1);
          	Sleep(300);
            throwball(sn_ball, 1);

            // TODO send message BT to server!!! **************

            Sleep(2000);
            //scan front area to verify to have scored
            dist_tmp=read_us(sn_us, 10);
            if(dist_tmp<=120){
              //lucky case in which the ball returns between the baffi
              liftball(sn_lift, sn_ball);
              throwball(sn_ball, 1);
            }
            balls++;
          } else {
            //return to the position of the research
            go_straight_mm(-dist+90, sn_tacho, 1);
          }
        }
      }
      if((i==2 || i==4 || i==5) && (!found_ball) ){
        printf("\n CALIBRATING \n\n");
        calibrate();
      }
      found_ball = 0;
      printf("\n\n\tNEW AREA\n\n");
    }

  } else if(mode==AGGRESSIVE){

    rotate(90, sn_tacho);
    go_straight_fullsped(440, sn_tacho);
    rotate(90, sn_tacho);
    go_straight_fullsped(1380, sn_tacho);
    go_straight_fullsped(-5, sn_tacho);
    rotate(-90, sn_tacho);
    go_straight_fullsped(300, sn_tacho);
    go_straight_fullsped(-500, sn_tacho);

/*
    rotate_with_adjustment(150, sn_tacho);
    go_straight_fullsped(740, sn_tacho);
    rotate_with_adjustment(50, sn_tacho);
    go_straight_fullsped(700, sn_tacho);
    rotate_with_adjustment(30, sn_tacho);
*/
  }
    /*
    switch( elaborate_dist(sn_tacho,sn_ball,sn_lift,pos,dist) ){

      case BALL_SHOT:
        //simple_search(CENTERING, 0, 0, 0);
        liftball(sn_lift, sn_ball);
        return_to_center(sn_tacho);
        //throwball(sn_ball, 1);
        balls++;
        // to new area
        printf("To new area\n");
        break;

      case NOT_FOUND:
        // free but maybe check again
        printf("To new area, but maybe there is something\n");
        break;

      case OBJ_IN_AREA:
        printf("Something found but distant\n");
        dist = simple_search(CENTERING, 30, -30, 200);
        elaborate_dist(sn_tacho,sn_ball,sn_lift,pos,dist);
        liftball(sn_lift, sn_ball);
        return_to_center(sn_tacho);
        //go_straight_mm(200, sn_tacho);
        //throwball(sn_ball, 1);
        balls++;
        break;

      case AREA_326:
        printf("Search better, maybe something\n");
        dist = simple_search(SECTOR, 40, -40, 300);
        elaborate_dist(sn_tacho,sn_ball,sn_lift,pos,dist);
        break;

      case AREA_FREE:
        printf("Trust me, nothing here\n");
        break;

      default:
        fprintf(stderr,"*************************ERROR IN SEARCH*************************\n");
        break;
    }
  }
  */

}


/*****************************************MAIN**********************************************/

int main(int argc, char *argv[]) {
  struct CornerAngles c_angles;
  struct Search_Areas areas[N_AREAS];
  int i;
  int dist;
  int flg;
  enum Mode mode;

#ifndef __ARM_ARCH_4T__
  /* Disable auto-detection of the brick (you have to set the correct address below) */
  //ev3_brick_addr = "192.168.0.204";
#endif

  if ( ev3_init() == -1 ) return 1;

#ifndef __ARM_ARCH_4T__
  //printf( "The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr );
#else
#endif

  /* Input work mode */
  if(argc > 1){
    if(argv[1][0]=='d'){
      /* DEFAULT MODE */
      mode=DEFAULT;
    } else if(argv[1][0]=='a'){
      /* AGGRESSIVE MODE */
      mode=AGGRESSIVE;
    } else {
      printf("You should select between: 'd' for DEFAULT or 'a' for AGGRESSIVE");
      /* MOVED TO DEFAULT */
      mode=DEFAULT;
    }
  } else {
    /* DEFAULT MODE */
    mode=DEFAULT;
  }

  if (signal(SIGINT, kill_all) == SIG_ERR)
      printf("Kill signal handler not set\n");
  //initialize sensors
	sensors_init();
  if( initialize_bt() == -1){
    return -1;
  }

  initialize_areas(areas);
  printf("In main\n");

  Sleep(1000);

  send_bt(to_bt);
  
  //go_to_point90(areas[0].posx, areas[0].posy, sn_tacho, N);
  alg_flow(sn_tacho, sn_ball, sn_lift, pos, areas, mode);
  //go_straight_mm(1600, sn_tacho, 0);
  //pos.x = 250;
  //pos.y = 350; 
  //calibrate();

  // rotate_with_adjustment(90,sn_tacho);
  // rotate_with_adjustment(180,sn_tacho);
  // rotate_with_adjustment(90,sn_tacho);
  // Sleep(5000);
  // rotate_with_adjustment(-90,sn_tacho);
  // rotate_with_adjustment(-180,sn_tacho);
  // rotate_with_adjustment(-90,sn_tacho);
  

/*
  if(liftball(sn_lift,sn_ball))
  	throwball(sn_ball, 1);*/
/*
  dist=continous_search(areas[0]);
  if(dist>0){
    go_straight_mm(dist-100, sn_tacho, 1);
    if(liftball(sn_lift, sn_ball)){
      return_to_center(sn_tacho);
      throwball(sn_ball, 1);
    }
  }
*/


  ev3_uninit();

  printf( "*** ( EV3 ) Bye! ***\n" );

  return ( 0 );
}
