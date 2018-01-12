#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"
#include "coroutine.h"
#include "ev3_servo.h"

#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>


#define Sleep( msec ) usleep(( msec ) * 1000 )


#define SERV_ADDR  "40:E2:30:8E:5D:DC"     /* Whatever the address of the server is */
#define TEAM_ID     5                       /* Your team ID */

#define MSG_ACK     0
#define MSG_START    1
#define MSG_STOP   2
#define MSG_KICK    3
#define MSG_POSITION 4
#define MSG_MAPDATA 	5
#define MSG_MAPDONE 6
#define MSG_OBSTACLE 7




//////////////////////////////////////////////////
#endif
const char const *color[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
#define COLOR_COUNT  (( int )( sizeof( color ) / sizeof( color[ 0 ])))

// GLOBAL VARIABLES




char ROBOT_DIRECTION = 'E';
int ROBOT_X = 10;
int ROBOT_Y = 2;



bool releaseobject = true;
char map[256][256] = { ' ' };
char string[58];

int s;
uint16_t msgId = 0;

// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
#include <windows.h>
// UNIX //////////////////////////////////////////
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif
#define L_MOTOR_PORT      OUTPUT_C
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define IR_CHANNEL        0
#define SPEED_LINEAR      75  /* Motor speed for linear motion, in percents */
#define SPEED_CIRCULAR    50  /* ... for circular motion */
int max_speed;  /* Motor maximal speed */
#define DEGREE_TO_COUNT( d )  (( d ) * 260 / 90 )
int app_alive;
enum {
	MODE_REMOTE,  /* IR remote control */
	MODE_AUTO,    /* Self-driving */
};
int mode;  /* Driving mode */
enum {
	MOVE_NONE,
	MOVE_FORWARD,
	MOVE_BACKWARD,
	TURN_LEFT,
	TURN_RIGHT,
	TURN_ANGLE,
	STEP_BACKWARD,
	MOVE_BACKWARDS_RANDOMLY,
	RELEASE_OBJECT,
	TURN_TO_TARGET,
};
int moving;   /* Current moving */
int command;  /* Command for the 'drive' coroutine */
int angle;    /* Angle of rotation */
enum { L, R };
uint8_t motor[3] = { DESC_LIMIT, DESC_LIMIT, DESC_LIMIT };  /* Sequence numbers of motors */


FLAGS_T state;
uint8_t sn;
uint8_t sn_touch;
uint8_t sn_color;
uint8_t sn_compass;
uint8_t sn_sonar;
uint8_t sn_mag;
uint8_t sn_gyro;


int read_from_server(int sock, char *buffer, size_t maxSize) {
	int bytes_read = read(sock, buffer, maxSize);

	if (bytes_read <= 0) {
		fprintf(stderr, "Server unexpectedly closed connection...\n");
		close(s);
		exit(EXIT_FAILURE);
	}

	printf("[DEBUG] received %d bytes\n", bytes_read);

	return bytes_read;
}



static void _run_forever(int l_speed, int r_speed)

{
	set_tacho_speed_sp(motor[L], l_speed);
	set_tacho_speed_sp(motor[R], r_speed);
	multi_set_tacho_command_inx(motor, TACHO_RUN_FOREVER);

}
static void _run_to_rel_pos(int l_speed, int l_pos, int r_speed, int r_pos)
{
	set_tacho_speed_sp(motor[L], l_speed);
	set_tacho_speed_sp(motor[R], r_speed);
	set_tacho_position_sp(motor[L], l_pos);
	set_tacho_position_sp(motor[R], r_pos);
	multi_set_tacho_command_inx(motor, TACHO_RUN_TO_REL_POS);
}
static void _run_timed(int l_speed, int r_speed, int ms)
{
	set_tacho_speed_sp(motor[L], l_speed);
	set_tacho_speed_sp(motor[R], r_speed);
	multi_set_tacho_time_sp(motor, ms);
	multi_set_tacho_command_inx(motor, TACHO_RUN_TIMED);
}
static int _is_running(void)
{
	FLAGS_T state = TACHO_STATE__NONE_;
	get_tacho_state_flags(motor[L], &state);
	if (state != TACHO_STATE__NONE_) return (1);
	get_tacho_state_flags(motor[R], &state);
	if (state != TACHO_STATE__NONE_) return (1);
	return (0);
}
static void _stop(void)
{
	multi_set_tacho_command_inx(motor, TACHO_STOP);
}

void turnright()
{
	printf("turning right\n");
	float current_pos;
	get_sensor_value0(sn_gyro, &current_pos);
	float final_pos = current_pos + 90.0;

	int diff = (int)(final_pos - current_pos);
	bool turn_right;
	//printf("CURRENTPOS:%f FINALPOS:%f DIFF:%d\n", current_pos, final_pos, diff);

	while (abs(current_pos - final_pos) != 0)
	{

		turn_right = (diff>0);

		uint8_t right_wheel, left_wheel;
		

		int max_speed;
		ev3_search_tacho_plugged_in(67, 0, &right_wheel, 0);
		ev3_search_tacho_plugged_in(66, 0, &left_wheel, 0);

		get_tacho_max_speed(right_wheel, &max_speed);

		set_tacho_stop_action_inx(right_wheel, TACHO_COAST);
		set_tacho_stop_action_inx(left_wheel, TACHO_COAST);

		set_tacho_speed_sp(right_wheel, max_speed * (turn_right ? 1 : -1) * 0.1);
		set_tacho_speed_sp(left_wheel, max_speed * (turn_right ? -1 : 1) * 0.1);

		set_tacho_time_sp(right_wheel, 50);
		set_tacho_time_sp(left_wheel, 50);

		set_tacho_command_inx(right_wheel, TACHO_RUN_TIMED);
		set_tacho_command_inx(left_wheel, TACHO_RUN_TIMED);

		get_sensor_value0(sn_gyro, &current_pos);
		diff = (int)(final_pos - current_pos);
		//printf("CURRENTPOS:%f FINALPOS:%f DIFF:%d\n", current_pos, final_pos, diff);
	}

}

void turnleft()
{
	printf("turning left\n");
	float current_pos;
	get_sensor_value0(sn_gyro, &current_pos);
	float final_pos = current_pos - 90.0;

	int diff = (int)(final_pos - current_pos);
	bool turn_right;
	//printf("CURRENTPOS:%f FINALPOS:%f DIFF:%d\n", current_pos, final_pos, diff);

	while (abs(current_pos - final_pos) != 0)
	{

		turn_right = (diff>0);

		uint8_t right_wheel, left_wheel;
		
		int max_speed;
		ev3_search_tacho_plugged_in(67, 0, &right_wheel, 0);
		ev3_search_tacho_plugged_in(66, 0, &left_wheel, 0);

		get_tacho_max_speed(right_wheel, &max_speed);
		//		get_tacho_max_speed( left_wheel, &max_speed );

		set_tacho_stop_action_inx(right_wheel, TACHO_COAST);
		set_tacho_stop_action_inx(left_wheel, TACHO_COAST);

		set_tacho_speed_sp(right_wheel, max_speed * (turn_right ? 1 : -1) * 0.1);
		set_tacho_speed_sp(left_wheel, max_speed * (turn_right ? -1 : 1) * 0.1);

		set_tacho_time_sp(right_wheel, 50);
		set_tacho_time_sp(left_wheel, 50);
		//		set_tacho_ramp_up_sp( right_wheel, 100 );
		//		set_tacho_ramp_up_sp( left_wheel,100 );

		set_tacho_command_inx(right_wheel, TACHO_RUN_TIMED);
		set_tacho_command_inx(left_wheel, TACHO_RUN_TIMED);


		get_sensor_value0(sn_gyro, &current_pos);
		diff = (int)(final_pos - current_pos);
		//printf("CURRENTPOS:%f FINALPOS:%f DIFF:%d\n", current_pos, final_pos, diff);
	}

}


int app_init(void)
{
	char s[16];
	if (ev3_search_tacho_plugged_in(L_MOTOR_PORT, L_MOTOR_EXT_PORT, motor + L, 0)) {
		get_tacho_max_speed(motor[L], &max_speed);
		/* Reset the motor */
		set_tacho_command_inx(motor[L], TACHO_RESET);
	}
	else {
		printf("LEFT motor (%s) is NOT found.\n", ev3_port_name(L_MOTOR_PORT, L_MOTOR_EXT_PORT, 0, s));
		/* Inoperative without left motor */
		return (0);
	}
	if (ev3_search_tacho_plugged_in(R_MOTOR_PORT, R_MOTOR_EXT_PORT, motor + R, 0)) {
		/* Reset the motor */
		set_tacho_command_inx(motor[R], TACHO_RESET);
	}
	else {
		printf("RIGHT motor (%s) is NOT found.\n", ev3_port_name(R_MOTOR_PORT, R_MOTOR_EXT_PORT, 0, s));
		/* Inoperative without right motor */
		return (0);
	}
	command = moving = MOVE_NONE;
	return(1);	
}

CORO_CONTEXT(handle_ir_proximity);
CORO_CONTEXT(drive);
CORO_CONTEXT(handle_brick_control);
CORO_CONTEXT(handle_touch);

/* Coroutine of the TOUCH sensor handling */
CORO_DEFINE(handle_touch)
{
	CORO_LOCAL int val;
	CORO_BEGIN();
	for (; ; ) {
		/* Waiting the button is pressed */
		get_sensor_value(0, sn_touch, &val);
		/* Stop the vehicle */
		printf("touched something %d",val);
		command = MOVE_NONE;
		/* Switch mode */
		//  _set_mode(( mode == MODE_REMOTE ) ? MODE_AUTO : MODE_REMOTE );
		/* Waiting the button is released */
		//CORO_WAIT(get_sensor_value(0, touch, &val) && (!val));
	}
	CORO_END();
}

/* Coroutine of control the motors */
CORO_DEFINE(drive)
{
	CORO_LOCAL float value_gyro;
	CORO_LOCAL int speed_linear, speed_circular, speed_release, speed_linear2;
	CORO_LOCAL int _wait_stopped, value_color;
	CORO_LOCAL uint8_t release_engine;
	CORO_LOCAL uint8_t proximitysensor, gyrosensor, colorsensor;

	CORO_LOCAL char positionmessage[58];
	CORO_LOCAL struct sockaddr_rc addr = { 0 };
	CORO_LOCAL int ss;
	
	CORO_LOCAL float minimumdistance, currentdistance, currentpoint, targetpoint, startgyro;
	CORO_BEGIN();
	speed_linear = max_speed * SPEED_LINEAR / 400;
	speed_circular = max_speed * SPEED_CIRCULAR / 400;
	speed_linear2 = max_speed * SPEED_LINEAR / 1600;

	for (; ; ) {
		/* Waiting new command */
		CORO_WAIT(moving != command);
		_wait_stopped = 0;

		switch (command) {
		case MOVE_NONE:
			_stop();
			_wait_stopped = 1;
			break;
		case MOVE_FORWARD:
			_run_forever(-speed_linear, -speed_linear);
			break;
		case MOVE_BACKWARD:
			_run_forever(speed_linear, speed_linear);
			break;
		case MOVE_BACKWARDS_RANDOMLY:
			_run_forever(0.5*speed_linear, speed_linear);
			break;
		case TURN_LEFT:
			get_sensor_value0(sn_gyro, &value_gyro);
			turnleft();
			break;
		case TURN_RIGHT:
			get_sensor_value0(sn_gyro, &value_gyro);
			turnright();
			break;
		case TURN_ANGLE:
			_run_to_rel_pos(speed_circular, DEGREE_TO_COUNT(-angle)
				, speed_circular, DEGREE_TO_COUNT(angle));
			_wait_stopped = 1;
			break;
		case STEP_BACKWARD:
			_run_timed(speed_linear, speed_linear, 1000);
			_wait_stopped = 1;
			break;
		
		case RELEASE_OBJECT:
			ev3_search_tacho_plugged_in(65, 0, &release_engine, 0);

			get_tacho_max_speed(release_engine, &speed_release);

			set_tacho_stop_action_inx(release_engine, TACHO_COAST);

			set_tacho_speed_sp(release_engine, -(0.5)*speed_release);

			set_tacho_time_sp(release_engine, 500);

			set_tacho_command_inx(release_engine, TACHO_RUN_TIMED);

			Sleep(500);

			set_tacho_stop_action_inx(release_engine, TACHO_COAST);

			set_tacho_speed_sp(release_engine, (0.5)*speed_release);

			set_tacho_time_sp(release_engine, 500);

			set_tacho_command_inx(release_engine, TACHO_RUN_TIMED);


			
		/*	
	ss = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t)1;
	str2ba(SERV_ADDR, &addr.rc_bdaddr);

	connect(ss, (struct sockaddr *)&addr, sizeof(addr));



		
		
		*((uint16_t *)positionmessage) = msgId++;
		positionmessage[2] = TEAM_ID;
		positionmessage[3] = 0xFF;
		positionmessage[4] = MSG_OBSTACLE;
		positionmessage[5] = 0;
		positionmessage[6] = ROBOT_X;         
		positionmessage[7] = 0x00;
		positionmessage[8] = ROBOT_Y;		
		positionmessage[9] = 0x00;
		write(ss, positionmessage, 10);
	*/
			break;


		case TURN_TO_TARGET:
			
			ev3_search_sensor(LEGO_EV3_GYRO, &gyrosensor, 0);
			ev3_search_sensor(LEGO_EV3_US, &proximitysensor, 0);
			ev3_search_sensor(LEGO_EV3_COLOR, &colorsensor, 0);

			get_sensor_value0(gyrosensor, &currentpoint);
			startgyro = currentpoint;
			Sleep(100);
			
			targetpoint = currentpoint - 5.0;

			//turn 5 degree left
			while (currentpoint > targetpoint)
			{
				get_sensor_value0(gyrosensor, &currentpoint);
				_run_forever(-speed_linear2, speed_linear2);
			}
			
			//turn 10 degree right
			get_sensor_value0(gyrosensor, &currentpoint);
			targetpoint = currentpoint + 10.0;

			minimumdistance = 99999;
			currentdistance = 99999;

			while (currentpoint < targetpoint)
			{
				get_sensor_value0(gyrosensor, &currentpoint);
				_run_forever(speed_linear2, -speed_linear2);

				get_sensor_value0(proximitysensor, &currentdistance);
				if (currentdistance < minimumdistance)
				{
					minimumdistance = currentdistance;
				}

			}

			printf("Minimum distance: %f \n", minimumdistance);
			
			printf("Turning until minimum distance is achieved\n");
			get_sensor_value0(gyrosensor, &targetpoint);
			get_sensor_value0(proximitysensor, &currentdistance);

			targetpoint = currentpoint - 10;
			while ((currentdistance > minimumdistance + 2) || (currentpoint < startgyro-7.0))
			{
				//printf("CurrentDistance: %f DesiredDistance: %f Diff: %f\n", currentdistance, minimumdistance, currentdistance - minimumdistance);
				get_sensor_value0(gyrosensor, &currentpoint);
				get_sensor_value0(proximitysensor, &currentdistance);
				_run_forever(-(0.4)*speed_linear2, (0.4)*speed_linear2);

			}
			
			//go forward to get closer
			while (currentdistance > 50.0 && currentdistance < 120)
			{
				//printf("Forward\n");
				//printf("CurrentDistance: %f DesiredDistance: %f Diff: %f\n", currentdistance, minimumdistance, currentdistance - minimumdistance);
				get_sensor_value0(proximitysensor, &currentdistance);
				_run_forever(-(0.4)*speed_linear2, -(0.4)*speed_linear2);
			}

			if (!get_sensor_value(0, sn_color, &value_color) || (value_color < 0) || (value_color >= COLOR_COUNT)) {
				value_color = 0;
			}

			if (value_color == 5)
			{
				printf("RED Movable Obstacle detected.\n");
				map[ROBOT_X][ROBOT_Y] = 'R'
			}
			else
			{
				printf("NON Movable Obstacle detected.\n");
				map[ROBOT_X][ROBOT_Y] = 'B'
			}

			while (currentdistance < minimumdistance)
			{
				//printf("Backwards\n");
				//printf("CurrentDistance: %f DesiredDistance: %f Diff: %f\n", currentdistance, minimumdistance, currentdistance - minimumdistance);
				get_sensor_value0(proximitysensor, &currentdistance);
				_run_forever((0.6)*speed_linear2, (0.6)*speed_linear2);
			}

			/*
			targetpoint = startgyro

			if(currentpoint > targetpoint)
			{
			while (currentpoint > targetpoint)
			{
				get_sensor_value0(gyrosensor, &currentpoint);
				_run_forever(-speed_linear2, speed_linear2);
			}
			}
			else{
			while (currentpoint < targetpoint)
			{
				get_sensor_value0(gyrosensor, &currentpoint);
				_run_forever(speed_linear2, -speed_linear2);
			}	
				
			}
*/
			_run_forever(0, 0);
			break;

	}

		moving = command;
		if (_wait_stopped) {
			/* Waiting the command is completed */
			CORO_WAIT(!_is_running());
			command = moving = MOVE_NONE;
		}
	}
	CORO_END();
}

CORO_DEFINE(handle_brick_control)
{
	CORO_LOCAL uint8_t keys, pressed = EV3_KEY__NONE_;
	CORO_BEGIN();
	for (; ; ) {

		/* Waiting any key is pressed or released */
		CORO_WAIT(ev3_read_keys(&keys) && (keys != pressed));
		pressed = keys;
		if (pressed & EV3_KEY_BACK) {
			printf("brick command given\n");//test
			/* Stop the vehicle */
			command = MOVE_NONE;
			/* Quit */
			app_alive = 0;
		}
		CORO_YIELD();
	}
	CORO_END();
}

CORO_DEFINE(handle_ir_proximity)
{
	CORO_LOCAL float prox;
	CORO_LOCAL int touch_state,randoma,value_color;
	CORO_LOCAL const char const *colorarray[] = { "?", "BLACK", "BLUE", "GREEN", "YELLOW", "RED", "WHITE", "BROWN" };
	CORO_BEGIN();
	ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0);
	for (; ; ) {
		get_sensor_value(0, sn_touch, &touch_state);
		if (touch_state != 0)
		{
			printf("Touched something I am going one step backwards.\n");
			command = MOVE_BACKWARDS_RANDOMLY;
		}

		else
		{
		
			if (!get_sensor_value(0, sn_color, &value_color) || (value_color < 0) || (value_color >= COLOR_COUNT)) {
				value_color = 0;
			}


		get_sensor_value0(sn_sonar, &prox);

		printf("\r  distance: %f color: (%s) \n", prox, colorarray[value_color]);

		if (prox == 0) {
			command = MOVE_NONE;
		}
		else if (prox < 120)
		{
	
if(releaseobject)
{	
randoma = rand() % 100;
			if (randoma<30) 
			{
	command = RELEASE_OBJECT;
	CORO_CALL(drive);
	Sleep(200);
	command = MOVE_NONE;
	CORO_CALL(drive);
releaseobject = false;
			}
			else
			{
				
			}	

	}			
			command = TURN_TO_TARGET;
			CORO_CALL(drive);
			
if((ROBOT_X < 6) && (ROBOT_Y < 7))
			{
			
if(ROBOT_DIRECTION=='E')
{
	command = TURN_LEFT;
}	
else if (ROBOT_DIRECTION=='W')
{
	command = TURN_RIGHT;
}
else if(ROBOT_DIRECTION=='N')
{
	command = TURN_RIGHT;
}
else if (ROBOT_DIRECTION=='S')
{
	command = TURN_LEFT;
}
				
			}
else if	((ROBOT_X < 6) && (ROBOT_Y > 25))
{
	
	if(ROBOT_DIRECTION=='E')
{
	command = TURN_RIGHT;
}	
else if (ROBOT_DIRECTION=='W')
{
	command = TURN_LEFT;
}
else if(ROBOT_DIRECTION=='N')
{
	command = TURN_RIGHT;
}
else if (ROBOT_DIRECTION=='S')
{
	command = TURN_LEFT;
}
	
	
	
}
else if	((ROBOT_X > 18) && (ROBOT_Y < 25))
{
	
	
	if(ROBOT_DIRECTION=='E')
{
	command = TURN_LEFT;
}	
else if (ROBOT_DIRECTION=='W')
{
	command = TURN_RIGHT;
}
else if(ROBOT_DIRECTION=='N')
{
	command = TURN_LEFT;
}
else if (ROBOT_DIRECTION=='S')
{
	command = TURN_RIGHT;
}
	
	
	
	
}
else if	((ROBOT_X > 18) && (ROBOT_Y > 25))
{
	
	if(ROBOT_DIRECTION=='E')
{
	command = TURN_RIGHT;
}	
else if (ROBOT_DIRECTION=='W')
{
	command = TURN_LEFT;
}
else if(ROBOT_DIRECTION=='N')
{
	command = TURN_LEFT;
}
else if (ROBOT_DIRECTION=='S')
{
	command = TURN_RIGHT;
}
	
	
}
else
{
randoma = rand() % 100;
			if (randoma<50) 
			{

				command = TURN_RIGHT;
			}
			else
			{
				command = TURN_LEFT;
			}	
}
	

	
			
			
			
			



			
		}
		else
		{
									
			
			map[ROBOT_X][ROBOT_Y] = 'G'
			
				if (ROBOT_Y > 2 && ROBOT_DIRECTION !='S')
				{
					command = MOVE_FORWARD;
				}
				else
				{	
					randoma = rand() % 100;
					if (randoma<50)
					{
						command = TURN_RIGHT;
					}
					else
					{
						command = TURN_LEFT;
					}

				}

		}

		}

		CORO_YIELD();
	}
	CORO_END();
}





void setCoordinates()
{
	if (command == MOVE_FORWARD)
	{

		if (ROBOT_DIRECTION == 'N')
		{
			ROBOT_Y++; if (ROBOT_Y > 255) { ROBOT_Y = 255; }
		}
		else if (ROBOT_DIRECTION == 'S')
		{
			ROBOT_Y--;		if (ROBOT_Y < 1) { ROBOT_Y = 0; }
		}
		else if (ROBOT_DIRECTION == 'E')
		{
			ROBOT_X++; if (ROBOT_X > 255) { ROBOT_X = 255; }
		}
		else if (ROBOT_DIRECTION == 'W')
		{
			ROBOT_X--; 			if (ROBOT_X < 1) { ROBOT_X = 0; }
		}
	}
	else if (command == MOVE_BACKWARD || command == MOVE_BACKWARDS_RANDOMLY)
	{
		if (ROBOT_DIRECTION == 'N')
		{
			ROBOT_Y--;   if (ROBOT_Y < 1) { ROBOT_Y = 0; }
		}
		else if (ROBOT_DIRECTION == 'S')
		{
			ROBOT_Y++; if (ROBOT_Y > 255) { ROBOT_Y = 255; }
		}
		else if (ROBOT_DIRECTION == 'E')
		{
			ROBOT_X--;   if (ROBOT_X < 1) { ROBOT_X = 0; }
		}
		else if (ROBOT_DIRECTION == 'W')
		{
			ROBOT_X++; (ROBOT_X > 255) { ROBOT_X = 255; }
		}
	}

}

void changeDirection()
{
	if (command == TURN_LEFT) {
		if (ROBOT_DIRECTION == 'N')
		{
			ROBOT_DIRECTION = 'W';
		}
		else if (ROBOT_DIRECTION == 'S')
		{
			ROBOT_DIRECTION = 'E';
		}
		else if (ROBOT_DIRECTION == 'E')
		{
			ROBOT_DIRECTION = 'N';
		}
		else if (ROBOT_DIRECTION == 'W')
		{
			ROBOT_DIRECTION = 'S';
		}
	}
	if (command == TURN_RIGHT) {
		if (ROBOT_DIRECTION == 'N')
		{
			ROBOT_DIRECTION = 'E';
		}
		else if (ROBOT_DIRECTION == 'S')
		{
			ROBOT_DIRECTION = 'W';
		}
		else if (ROBOT_DIRECTION == 'E')
		{
			ROBOT_DIRECTION = 'S';
		}
		else if (ROBOT_DIRECTION == 'W')
		{
			ROBOT_DIRECTION = 'N';
		}
	}
}

int main(void)
{
	srand(time(NULL));
	int i;
	char s[256];
	int ss;
	int val;
	uint32_t n, ii;
	float value_compass, value_sonar, value_gyro;
	int val_color,val_touch;
#ifndef __ARM_ARCH_4T__
	/* Disable auto-detection of the brick (you have to set the correct address below) */
	ev3_brick_addr = "192.168.0.204";
#endif
	if (ev3_init() == -1) return (1);

#ifndef __ARM_ARCH_4T__
	printf("The EV3 brick auto-detection is DISABLED,\nwaiting %s online with plugged tacho...\n", ev3_brick_addr);
#else
	printf("Waiting tacho is plugged...\n");

#endif
	while (ev3_tacho_init() < 1) Sleep(1000);

	printf("*** ( EV3 ) Hello! ***\n");


	printf("Found tacho motors:\n");
	for (i = 0; i < DESC_LIMIT; i++) {
		if (ev3_tacho[i].type_inx != TACHO_TYPE__NONE_) {
			printf("  type = %s\n", ev3_tacho_type(ev3_tacho[i].type_inx));
			printf("  port = %s\n", ev3_tacho_port_name(i, s));
			printf("  port = %d %d\n", ev3_tacho_desc_port(i), ev3_tacho_desc_extport(i));
		}
	}

	//Run all sensors
	ev3_sensor_init();


	printf("Found sensors:\n");
	for (i = 0; i < DESC_LIMIT; i++) {
		if (ev3_sensor[i].type_inx != SENSOR_TYPE__NONE_) {
			printf("  type = %s\n", ev3_sensor_type(ev3_sensor[i].type_inx));
			printf("  port = %s\n", ev3_sensor_port_name(i, s));
			if (get_sensor_mode(i, s, sizeof(s))) {
				printf("  mode = %s\n", s);
			}
			if (get_sensor_num_values(i, &n)) {
				for (ii = 0; ii < n; ii++) {
					if (get_sensor_value(ii, i, &val)) {
						printf("  value%d = %d\n", ii, val);
					}
				}
			}
		}
	}

	if (ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0)) {
		//printf("TOUCH sensor is found, press BUTTON for EXIT...\n");
	}


	if (ev3_search_sensor(LEGO_EV3_COLOR, &sn_color, 0)) {
		printf("COLOR sensor is found, reading COLOR...\n");
		if (!get_sensor_value(0, sn_color, &val_color) || (val_color < 0) || (val_color >= COLOR_COUNT)) {
			val_color = 0;
		}
		printf("\r(%s) \n", color[val_color]);
		fflush(stdout);
	}

	if (ev3_search_sensor(HT_NXT_COMPASS, &sn_compass, 0)) {
		printf("COMPASS found, reading compass...\n");
		if (!get_sensor_value0(sn_compass, &value_compass)) {
			value_compass = 0;
		}
		printf("\r(%f) \n", value_compass);
		fflush(stdout);
	}

	if (ev3_search_sensor(LEGO_EV3_US, &sn_sonar, 0)) {
		printf("SONAR found, reading sonar...\n");
		if (!get_sensor_value0(sn_sonar, &value_sonar)) {
			value_sonar = 0;
		}
		//printf( "\r(%f) \n", value_sonar);
		fflush(stdout);
	}

	if (ev3_search_sensor(LEGO_EV3_GYRO, &sn_gyro, 0)) {
		printf("SONAR found, reading gyro...\n");
		if (!get_sensor_value0(sn_gyro, &value_gyro)) {
			value_gyro = 0;
		}
		printf("\rvalue gyro: (%f) \n", value_gyro);
		fflush(stdout);
	}

	if (ev3_search_sensor(LEGO_EV3_TOUCH, &sn_touch, 0)) {
		printf("TOUCH sensor is found, reading COLOR...\n");
		if (!get_sensor_value(0, sn_touch, &val_touch)) {
			val_color = 0;
		}
		printf("\r(%d) \n", val_touch);
		fflush(stdout);
	}

	struct sockaddr_rc addr = { 0 };
	int status;

	ev3_tacho_init();
	
	ss = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t)1;
	str2ba(SERV_ADDR, &addr.rc_bdaddr);

	status = connect(ss, (struct sockaddr *)&addr, sizeof(addr));
	while (status != 0)
	{
		status = connect(ss, (struct sockaddr *)&addr, sizeof(addr));
		printf("Connection Attempt\n");
		Sleep(100);
	}

	printf("Connected!\n");

	read_from_server(ss, string, 9);
	while (string[4] != MSG_START)
	{
		read_from_server(ss, string, 9);
	}

	printf("Start message received!\n");
	app_alive = app_init();
	
	while (app_alive) {
		
		CORO_CALL(handle_brick_control);
		CORO_CALL(handle_ir_proximity);
		CORO_CALL(drive);

		if(command == RELEASE_OBJECT)	
			{
		char obstaclemessage[58];
		*((uint16_t *)obstaclemessage) = msgId++;
		obstaclemessage[2] = TEAM_ID;
		obstaclemessage[3] = 0xFF;
		obstaclemessage[4] = MSG_OBSTACLE;
		obstaclemessage[5] = 0;
		obstaclemessage[6] = ROBOT_X;
		obstaclemessage[7] = 0x00;
		obstaclemessage[8] = ROBOT_Y;
		obstaclemessage[9] = 0x00;
		write(ss, obstaclemessage, 10);
			}
		
		setCoordinates();
		changeDirection();

		Sleep(450);
		command = MOVE_NONE;
		CORO_CALL(drive);

		printf("Sending position: ROBOT_DIRECTION: %c, ROBOT_X: %d, ROBOT_Y: %d\n", ROBOT_DIRECTION, ROBOT_X, ROBOT_Y);
		char positionmessage[58];
		*((uint16_t *)positionmessage) = msgId++;
		positionmessage[2] = TEAM_ID;
		positionmessage[3] = 0xFF;
		positionmessage[4] = MSG_POSITION;
		positionmessage[5] = ROBOT_X;          /* x */
		positionmessage[6] = 0x00;
		positionmessage[7] = ROBOT_Y;		/* y */
		positionmessage[8] = 0x00;
		write(ss, positionmessage, 9);
		
		
		if(msgId%10 == 0)
		{
			//send mapdone
			
			char mapmessage[58];
			
			
			for(int i = 0; i< 256; i++)
			{
				for(int j = 0; j< 256; j++)
			{
				
				if(map[i][j]!=' ')
				{
						
		*((uint16_t *)mapmessage) = msgId++;
		mapmessage[2] = TEAM_ID;
		mapmessage[3] = 0xFF;
		mapmessage[4] = MSG_MAPDATA;
		mapmessage[5] = ROBOT_X;          
		mapmessage[6] = 0x00;
		mapmessage[7] = ROBOT_Y;		
		mapmessage[8] = 0x00;
		
		mapmessage[9] = 0x00;
		mapmessage[10] = 0x00;
		mapmessage[11] = 0x00;
		
		if(map[i][j]=='R')
		{
			mapmessage[9] = 0xFF;
		}
		else if(map[i][j]=='G')
		{
			mapmessage[10] = 0xFF;
		}
	else if(map[i][j]=='B')
		{
			mapmessage[11] = 0xFF;
		}		
		
		write(ss, mapmessage, 12);
					
				}				
			}
				
			}
		
		char mapdonemessage[58];
		*((uint16_t *)mapdonemessage) = msgId++;
		mapdonemessage[2] = TEAM_ID;
		mapdonemessage[3] = 0xFF;
		mapdonemessage[4] = MSG_MAPDONE;
		write(ss, mapdonemessage, 5);
		
			
		}
		

	/*
	read_from_server(ss, string, 6);
	if (string[4] != MSG_KICK && string[5] == TEAM_ID)
	{
	printf("I am kicked :( \n");
	close(ss);
	ev3_uninit();
	printf("*** ( EV3 ) Bye! ***\n");
		return (0);
	}	
	*/
	
	
	}


	close(ss);
	ev3_uninit();
	printf("*** ( EV3 ) Bye! ***\n");

	return (0);
}
