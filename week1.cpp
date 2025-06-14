#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>

//gcc -o week1 week1.cpp -lwiringPi  -lm
// rsync -avz week1.cpp pi@10.42.0.1:FlightController/week1.cpp

#define calibration 1000.0
#define max_gyro_rate 300
#define max_absolute_roll_angle 45
#define max_absolute_pitch_angle 45
#define time_out 0.75
#define A 0.02
// #define thrust_neutral 1725
#define thrust_neutral 1650
#define thrust_amplitude 150
#define thrust_max 2000
#define thrust_min 0
#define joystick_max 255

#define pitch_amplitude 4
#define pitch_gain 12.0
#define pitch_derivative_gain 1.9
#define pitch_integral_gain 0.3
#define pitch_integral_saturated 100

#define roll_amplitude 4
#define roll_gain 12.0
#define roll_derivative_gain 1.9
#define roll_integral_gain 0.3
#define roll_integral_saturated 100

#define yaw_amplitude 50
#define yaw_gain 5

#define autonomous_yaw_gain 20.0
#define autonomous_roll_gain 0.25
#define autonomous_roll_derivative_gain 0.1
// #define autonomous_roll_gain 0.0
// #define autonomous_roll_derivative_gain 0.0
#define autonomous_pitch_gain 0.25
#define autonomous_pitch_derivative_gain 0.1



int setup_imu();
void calibrate_imu();
void read_imu();
void update_filter();
void setup_joystick();
void trap(int signal);
void safety_check(int key1);
void joystick_check(int key0, int key3, int key2);
void update_time_elapsed(int sequence_num);
void pid_pitch_control();
void pid_roll_control();
void yaw_control();
void set_pitch(int pitch);
void set_roll(int roll);
void set_yaw(int yaw);
void set_thrust(int thrust);
void set_motors(int motor1, int motor2, int motor3, int motor4);
void motor_enable();

void setup_camera();
void high_yaw_control(int yaw);
void high_roll_control(int y);
void high_pitch_control(int x);

//global variables
int motor_address,accel_address,gyro_address;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //accel xyz,  gyro xyz,
long time_curr;
long time_prev;
struct timespec te;
float yaw=0.0;
float pitch_angle=0.0;
float roll_angle=0.0;
float pitch_t = 0.0;
float roll_t = 0.0;
float roll_angle_calibrated = 0.0;
float pitch_angle_calibrated = 0.0;
float integrated_roll_gyro = 0.0;
float integrated_pitch_gyro = 0.0;
float x_gyro_calibrated = 0.0;
float y_gyro_calibrated = 0.0;
float z_gyro_calibrated = 0.0;

int prev_sequence_number = 0;
float time_elapsed = 0.0;
long time_curr2;
long time_prev2;


int motor_commands[4];
float thrust = 0.0;
float thrust_scaler = 0.0;
float thrust_desired = 0.0;

float pitch_desired = 0.0;
float pitch_error = 0.0;
float integral_pitch = 0.0;

float roll_desired = 0.0;
float roll_error = 0.0;
float integral_roll = 0.0;

float yaw_desired = 0.0;
float yaw_error = 0.0;

int autonomous = 0;
int prev_camera_sequence = 0;
float prev_camera_y_estimated = 0.0;
float camera_y_estimated = 0.0;
float prev_camera_x_estimated = 0.0;
float camera_x_estimated = 0.0;



//global variables to add
struct Joystick
{
	int key0;
	int key1;
	int key2;
	int key3;
	int pitch;
	int roll;
	int yaw;
	int thrust;
	int sequence_num;
};
Joystick* shared_memory;

struct Camera
{
  int x;
  int y;
  int z;
  int yaw;
  int sequence_num;
};

Camera* camera_memory; 
Camera camera_data;

int run_program=1;
int paused = 0;

int main (int argc, char *argv[]) {
	setup_imu();
	calibrate_imu();
	motor_enable();

	setup_joystick();
	signal(SIGINT, &trap);

	setup_camera();

	for (int i = 0; i<700; i++){
		set_motors(4,4,4,4);
	}

	while(run_program == 1)
	{
		//to refresh values from shared memory first
		Joystick joystick_data=*shared_memory;

		read_imu();   
		update_filter();
		update_time_elapsed(joystick_data.sequence_num);
		safety_check(joystick_data.key1);
		joystick_check(joystick_data.key0, joystick_data.key3, joystick_data.key2);

		set_pitch(joystick_data.pitch);
		set_roll(joystick_data.roll);
		set_yaw(joystick_data.yaw);

		Camera camera_data=*camera_memory;
		// printf("camera=%d %d %d %d %d\n\r",camera_data.x,camera_data.y,camera_data.z,camera_data.yaw,camera_data.sequence_num);

		if (run_program == 0){
			motor_commands[0] = 0;
			motor_commands[1] = 0;
			motor_commands[2] = 0;
			motor_commands[3] = 0;

		} else if (paused == 1){
			motor_commands[0] = 2;
			motor_commands[1] = 2;
			motor_commands[2] = 2;
			motor_commands[3] = 2;
			set_motors(motor_commands[3], motor_commands[2], motor_commands[1], motor_commands[0]);
			// printf("%d; %d; %d; %d; %10.5f; %10.5f;%10.5f; %10.5f\n", motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3], pitch_t, roll_t, pitch_angle_calibrated, roll_angle_calibrated);

		} else {
			set_thrust(joystick_data.thrust);
			pid_pitch_control();
			pid_roll_control();
			yaw_control();

			// printf("%d\n", autonomous);

			// printf("%d\n\r", camera_data.yaw);


			if (autonomous==1) {
				high_yaw_control(camera_data.yaw);
				high_roll_control(camera_data.y);
				high_pitch_control(camera_data.x);
				// printf("%d; %10.5f\n\r", camera_data.yaw, x_gyro_calibrated);
			}
			
			// printf("%d; %d; %d; %d; %10.5f; %10.5f;%10.5f; %10.5f\n", motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3], pitch_t, roll_t, pitch_angle_calibrated, roll_angle_calibrated);
			// motor_commands[0] = 0;
			// motor_commands[1] = 0;
			// motor_commands[2] = 0;
			// motor_commands[3] = 0;
			set_motors(motor_commands[3], motor_commands[2], motor_commands[1], motor_commands[0]);
		}
		prev_camera_sequence = camera_data.sequence_num;

	}

	return 0;
}

void calibrate_imu() {
	x_gyro_calibration=0;
	y_gyro_calibration=0;
	z_gyro_calibration=0;
	roll_calibration=0;
	pitch_calibration=0;
	accel_z_calibration=0;

	for(int i=0;i<calibration;i++)
	{
		read_imu();
		x_gyro_calibration+=imu_data[3];
		y_gyro_calibration+=imu_data[4];
		z_gyro_calibration+=imu_data[5];
		roll_calibration+=roll_angle;
		pitch_calibration+=pitch_angle;
		accel_z_calibration+=accel_z_calibration;
	}
		x_gyro_calibration=x_gyro_calibration/calibration;
		y_gyro_calibration=y_gyro_calibration/calibration;
		z_gyro_calibration=z_gyro_calibration/calibration;
		roll_calibration=roll_calibration/calibration;
		pitch_calibration=pitch_calibration/calibration;
		accel_z_calibration=accel_z_calibration/calibration;

	printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);
}

void read_imu() {
	uint8_t address=0;//todo: set address value for accel x value
	float ax=0;
	float az=0;
	float ay=0;
	int vh=0;
	int vl=0;
	int vw=0;

	//accel reads

	address=0x12;//use 0x00 format for hex
	vw=wiringPiI2CReadReg16(accel_address,address);
	//convert from 2's complement
	if(vw>0x8000)
	{
		vw=vw ^ 0xffff;
		vw=-vw-1;
	}

	imu_data[0]=3.0/32768.0 * vw;//convert to g's  
	
	address=0x14;//use 0x00 format for hex
	vw=wiringPiI2CReadReg16(accel_address,address);   
	//convert from 2's complement
	if(vw>0x8000)
	{
		vw=vw ^ 0xffff;
		vw=-vw-1;
	}          
	imu_data[1]=3.0/32768.0 * vw;//convert to g's  
	
	address=0x16;//use 0x00 format for hex
	vw=wiringPiI2CReadReg16(accel_address,address);   
	//convert from 2's complement    
	if(vw>0x8000)
	{
		vw=vw ^ 0xffff;
		vw=-vw-1;
	}          
	imu_data[2]=3.0/32768.0 * vw;//convert to g's  
	imu_data[2] = imu_data[2]; 

	//gyro reads

	address=0x02;//use 0x00 format for hex
	vw=wiringPiI2CReadReg16(gyro_address,address);   
	//convert from 2's complement          
	if(vw>0x8000)
	{
		vw=vw ^ 0xffff;
		vw=-vw-1;
	}          
	imu_data[3]=vw*1000.0/32768.0;//convert to degrees/sec
	
	address=0x04;//use 0x00 format for hex
	vw=wiringPiI2CReadReg16(gyro_address,address);
	//convert from 2's complement            
	if(vw>0x8000)
	{
		vw=vw ^ 0xffff;
		vw=-vw-1;
	}          
	imu_data[4]=vw*1000.0/32768.0;//convert to degrees/sec
	
	address=0x06;//use 0x00 format for hex
	vw=wiringPiI2CReadReg16(gyro_address,address);
	//convert from 2's complement            
	if(vw>0x8000)
	{
		vw=vw ^ 0xffff;
		vw=-vw-1;
	}          
	imu_data[5]=-1.0*vw*1000.0/32768.0;//convert to degrees/sec  

	pitch_angle = (atan2(imu_data[1], imu_data[0]) * (180.0/M_PI));
	pitch_angle_calibrated = pitch_angle - pitch_calibration;
	roll_angle = atan2(imu_data[2], imu_data[0]) * (180.0/M_PI);
	roll_angle_calibrated = roll_angle - roll_calibration;

	x_gyro_calibrated = imu_data[3] - x_gyro_calibration;
	y_gyro_calibrated = imu_data[4] - y_gyro_calibration;
	z_gyro_calibrated = imu_data[5] - z_gyro_calibration;
}

void update_filter() {
	//get current time in nanoseconds
	timespec_get(&te,TIME_UTC);
	time_curr=te.tv_nsec;

	//compute time since last execution
	float imu_diff=time_curr-time_prev;

	//check for rollover
	if(imu_diff<=0)
	{
	imu_diff+=1000000000;
	}
	//convert to seconds
	imu_diff=imu_diff/1000000000;
	time_prev=time_curr;

	//comp. filter for roll, pitch here:
	integrated_roll_gyro = integrated_roll_gyro + (y_gyro_calibrated * imu_diff);
	integrated_pitch_gyro = integrated_pitch_gyro + (z_gyro_calibrated * imu_diff);
	roll_t = roll_angle_calibrated * A + (1 - A) * (y_gyro_calibrated * imu_diff + roll_t);
	pitch_t = pitch_angle_calibrated * A + (1 - A) * (z_gyro_calibrated * imu_diff + pitch_t);
}

void update_time_elapsed(int sequence_num) {
		
	//get current time in nanoseconds
	timespec_get(&te,TIME_UTC);
	time_curr2=te.tv_nsec;
	//compute time since last execution
	float imu_diff=time_curr2-time_prev2;
	//check for rollover
	if(imu_diff<=0)
	{
	imu_diff+=1000000000;
	}
	//convert to seconds
	imu_diff=imu_diff/1000000000;
	time_prev2=time_curr2;

	// implement a float variable that will calculate time elapsed since last change in sequence number

		if (prev_sequence_number == sequence_num) {
			time_elapsed = time_elapsed + imu_diff;
		}
		else {
			time_elapsed = 0.0;
		}

		prev_sequence_number = sequence_num;
	
}

int setup_imu() {
	wiringPiSetup ();
	motor_address=wiringPiI2CSetup (0x56) ; 
	
	//setup imu on I2C
	accel_address=wiringPiI2CSetup (0x19) ;
	
	
	gyro_address=wiringPiI2CSetup (0x69) ;
	
	if(accel_address==-1)
	{
		printf("-----cant connect to accel I2C device %d --------\n",accel_address);
		return -1;
	}
	else if(gyro_address==-1)
	{
		printf("-----cant connect to gyro I2C device %d --------\n",gyro_address);
		return -1;
	}
	else
	{
		// printf("all i2c devices detected\n");
		sleep(1);
		wiringPiI2CWriteReg8(accel_address, 0x7d, 0x04); //power on accel    
		wiringPiI2CWriteReg8(accel_address, 0x41, 0x00); //accel range to +_3g    
		wiringPiI2CWriteReg8(accel_address, 0x40, 0x89); //high speed filtered accel with osr4
		
		wiringPiI2CWriteReg8(gyro_address, 0x11, 0x00);//power on gyro
		wiringPiI2CWriteReg8(gyro_address, 0x0F, 0x01);//set gyro to +-1000dps
		wiringPiI2CWriteReg8(gyro_address, 0x10, 0x03);//set data rate and bandwith
		
		
		sleep(1);
	}
	return 0;
}

//function to add
void setup_joystick() {
	int segment_id;
	struct shmid_ds shmbuffer;
	int segment_size;
	const int shared_segment_size = 0x6400;
	int smhkey=33222;
	/* Allocate a shared memory segment. */
	segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
	/* Attach the shared memory segment. */
	shared_memory = (Joystick*) shmat (segment_id, 0, 0);
	printf ("shared memory attached at address %p\n", shared_memory);
	/* Determine the segment's size. */
	shmctl (segment_id, IPC_STAT, &shmbuffer);
	segment_size = shmbuffer.shm_segsz;
	printf ("segment size: %d\n", segment_size);
	/* Write a string to the shared memory segment. */
	//sprintf (shared_memory, "test!!!!.");
}

//when cntrl+c pressed, kill motors
void trap(int signal) {
	printf("ending program\n\r");
	printf("Control+C pressed\n");
	run_program=0;
}

void safety_check(int key1) {

	// if Joystick "B" is pressed
	if (key1 == 1) {
		printf("Joystick B pressed\n");
		run_program = 0;
	}
	// Any gyro rate > 300 degrees/sec
	if (fabs(imu_data[3] - x_gyro_calibration) > max_gyro_rate) {
		printf("Max x gyro rate exceeded\n");
		run_program = 0;
	}
	if (fabs(imu_data[4] - y_gyro_calibration) > max_gyro_rate) {
		printf("Max y gyro rate exceeded\n");
		run_program = 0;
	}
	if (fabs(imu_data[5] - z_gyro_calibration) > max_gyro_rate) {
		printf("Max z gyro rate exceeded\n");
		run_program = 0;
	}
	// Roll angle > 45 or <-45
	if (fabs(roll_t) > max_absolute_roll_angle) {
		printf("Max roll angle exceeded\n");
		run_program = 0;
	}

	// Pitch angle >45 or <-45
	if (fabs(pitch_t) > max_absolute_pitch_angle) {
		printf("Max pitch angle exceeded\n");
		run_program = 0;
	}
	// slides instruct 0.35 seconds since last joystick update 
	if (time_elapsed > time_out) {
		printf("Joystick timeout\n");
		run_program = 0;
	}
}

void joystick_check(int key0, int key3, int key2) {
	if (key0 == 1) {
		paused = 1;
	} else if (key3 == 1) {
		paused = 0;
	}

	if (key2 == 1) {
		autonomous = 1-autonomous;
		// printf("%d", autonomous);
	}
}

void set_thrust(int thrust) {
	thrust_scaler = -1.0*((thrust - joystick_max/2.0))/(joystick_max / 2.0);

	thrust_desired = thrust_neutral + (thrust_scaler * thrust_amplitude);

	motor_commands[0] = (thrust_neutral + (thrust_scaler * thrust_amplitude));
	motor_commands[1] = (thrust_neutral + (thrust_scaler * thrust_amplitude));
	motor_commands[2] = (thrust_neutral + (thrust_scaler * thrust_amplitude));
	motor_commands[3] = (thrust_neutral + (thrust_scaler * thrust_amplitude));
}

void set_pitch(int pitch) {
	float pitch_scaler = 1.0*((pitch - joystick_max/2.0))/(joystick_max / 2.0);

	pitch_desired = pitch_scaler * pitch_amplitude;
	pitch_error = pitch_desired - pitch_t;
}

void pid_pitch_control() {

	// [0] and [2] are front left and front right motors respectively
	// [1] and [3] are back left and back right motors respectively

	// Proportional control
	motor_commands[0] = motor_commands[0] + pitch_error * pitch_gain;
	motor_commands[2] = motor_commands[2] + pitch_error * pitch_gain;
	motor_commands[1] = motor_commands[1] - pitch_error * pitch_gain;
	motor_commands[3] = motor_commands[3] - pitch_error * pitch_gain;

	// Derivative control
	motor_commands[0] = motor_commands[0] - z_gyro_calibrated * pitch_derivative_gain;
	motor_commands[2] = motor_commands[2] - z_gyro_calibrated * pitch_derivative_gain;
	motor_commands[1] = motor_commands[1] + z_gyro_calibrated * pitch_derivative_gain;
	motor_commands[3] = motor_commands[3] + z_gyro_calibrated * pitch_derivative_gain;

	// Integral control

	integral_pitch += pitch_integral_gain * pitch_error;
	if (integral_pitch > pitch_integral_saturated) {
		integral_pitch = pitch_integral_saturated;
	} else if (integral_pitch < -1.0 * pitch_integral_saturated) {
		integral_pitch = -1.0 * pitch_integral_saturated;
	}

	motor_commands[0] = (motor_commands[0] + integral_pitch);
	motor_commands[2] = (motor_commands[2] + integral_pitch);
	motor_commands[1] = (motor_commands[1] - integral_pitch);
	motor_commands[3] = (motor_commands[3] - integral_pitch);
}

void pid_roll_control() {	
	// Proportional control

	// [0] and [2] are front left and front right motors respectively
	// [1] and [3] are back left and back right motors respectively

	// [0] and [1] are front left and back left motors respectively
	// [2] and [3] are front right and back right motors respectively

	motor_commands[0] = motor_commands[0] + roll_error * roll_gain;
	motor_commands[1] = motor_commands[1] + roll_error * roll_gain;
	motor_commands[2] = motor_commands[2] - roll_error * roll_gain;
	motor_commands[3] = motor_commands[3] - roll_error * roll_gain;

	// Derivative control
	motor_commands[0] = motor_commands[0] - y_gyro_calibrated * roll_derivative_gain;
	motor_commands[1] = motor_commands[1] - y_gyro_calibrated * roll_derivative_gain;
	motor_commands[2] = motor_commands[2] + y_gyro_calibrated * roll_derivative_gain;
	motor_commands[3] = motor_commands[3] + y_gyro_calibrated * roll_derivative_gain;

	// Integral control
	integral_roll += roll_integral_gain * roll_error;
	if (integral_roll > roll_integral_saturated) {
		integral_roll = roll_integral_saturated;
	} else if (integral_roll < -1.0 * roll_integral_saturated) {
		integral_roll = -1.0 * roll_integral_saturated;
	}

	motor_commands[0] = (motor_commands[0] + integral_roll);
	motor_commands[1] = (motor_commands[1] + integral_roll);
	motor_commands[2] = (motor_commands[2] - integral_roll);
	motor_commands[3] = (motor_commands[3] - integral_roll);
}

void yaw_control() {
	motor_commands[0] = motor_commands[0] - yaw_error * yaw_gain;
	motor_commands[1] = motor_commands[1] + yaw_error * yaw_gain;
	motor_commands[2] = motor_commands[2] + yaw_error * yaw_gain;
	motor_commands[3] = motor_commands[3] - yaw_error * yaw_gain;
}

void set_roll(int roll) {
	float roll_scaler = 1.0*((roll - joystick_max/2.0))/(joystick_max / 2.0);

	roll_desired = roll_scaler * roll_amplitude;
	roll_error = roll_desired - roll_t;
}

void set_yaw(int yaw) {
	float yaw_scaler = -1.0*((yaw - joystick_max/2.0))/(joystick_max / 2.0);

	yaw_desired = yaw_scaler * yaw_amplitude;
	yaw_error = yaw_desired - x_gyro_calibrated;
}


void motor_enable() {

    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;    
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2]; 
    
    int cal_delay=50;
    
    for(int i=0;i<1000;i++) {
      motor_id=0;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);
    }
     
    for(int i=0;i<2000;i++) {
      motor_id=0;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=50;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);
    }
    
     
    for(int i=0;i<500;i++) {
      motor_id=0;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]); 
      
      usleep(cal_delay);   
      motor_id=1;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);      
      
      usleep(cal_delay); 
      motor_id=2;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);   
   
      usleep(cal_delay);   
      motor_id=3;
      commanded_speed=0;
      data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
      data[1]=commanded_speed&0x7f;    
      wiringPiI2CWrite(motor_address,data[0]);     
      usleep(cal_delay);    
      wiringPiI2CWrite(motor_address,data[1]);       
      usleep(cal_delay);
    }
}

void set_motors(int motor0, int motor1, int motor2, int motor3) {
    if(motor0<0)
      motor0=0;
    if(motor0>thrust_max)
      motor0=thrust_max;
    if(motor1<0)
      motor1=0;
    if(motor1>thrust_max)
      motor1=thrust_max;
    if(motor2<0)
      motor2=0;
    if(motor2>thrust_max)
      motor2=thrust_max;
    if(motor3<0)
      motor3=0;
    if(motor3>thrust_max)
      motor3=thrust_max;
      
    uint8_t motor_id=0;
    uint8_t special_command=0;
    uint16_t commanded_speed_0=1000;    
    uint16_t commanded_speed_1=0;
    uint16_t commanded_speed=0;
    uint8_t data[2]; 
    
   // wiringPiI2CWriteReg8(motor_address, 0x00,data[0] );
    //wiringPiI2CWrite (motor_address,data[0]) ;
    int com_delay=500;
   
    motor_id=0;
    commanded_speed=motor0;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);  
 
    
    usleep(com_delay);   
    motor_id=1;
    commanded_speed=motor1;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);      
  
    usleep(com_delay); 
    motor_id=2;
    commanded_speed=motor2;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);   

    
    usleep(com_delay);   
    motor_id=3;
    commanded_speed=motor3;
    data[0]=0x80+(motor_id<<5)+(special_command<<4)+((commanded_speed>>7)&0x0f);
    data[1]=commanded_speed&0x7f;    
    wiringPiI2CWrite(motor_address,data[0]);     
    usleep(com_delay);    
    wiringPiI2CWrite(motor_address,data[1]);    
    usleep(com_delay);
}

void setup_camera() {

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = sizeof(struct Camera);
  int smhkey=123456;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  camera_memory = (Camera*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", camera_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  // sprintf (shared_memory, "test!!!!."); 
}

void high_yaw_control(int yaw) {
	// printf("%d\n\r", yaw);
	int yaw_autonomous_error = -yaw;
	// printf("%10.5f\n\r", yaw_autonomous_error);
	motor_commands[0] = motor_commands[0] - yaw_autonomous_error * autonomous_yaw_gain;
	motor_commands[1] = motor_commands[1] + yaw_autonomous_error * autonomous_yaw_gain;
	motor_commands[2] = motor_commands[2] + yaw_autonomous_error * autonomous_yaw_gain;
	motor_commands[3] = motor_commands[3] - yaw_autonomous_error * autonomous_yaw_gain;
}

void high_roll_control(int y) {

	printf("%d; %10.5f\n\r", y, camera_y_estimated);

	if (camera_data.sequence_num != prev_camera_sequence) {
		prev_camera_y_estimated = camera_y_estimated;
		camera_y_estimated = (camera_y_estimated * 0.6) + (y * 0.4);
	}

	float autonomous_roll_error = camera_y_estimated;

	motor_commands[0] = motor_commands[0] + autonomous_roll_error * autonomous_roll_gain;
	motor_commands[1] = motor_commands[1] + autonomous_roll_error * autonomous_roll_gain;
	motor_commands[2] = motor_commands[2] - autonomous_roll_error * autonomous_roll_gain;
	motor_commands[3] = motor_commands[3] - autonomous_roll_error * autonomous_roll_gain;

	// Derivative control
	float y_change = camera_y_estimated - prev_camera_y_estimated;

	motor_commands[0] = motor_commands[0] - y_change * autonomous_roll_derivative_gain;
	motor_commands[1] = motor_commands[1] - y_change * autonomous_roll_derivative_gain;
	motor_commands[2] = motor_commands[2] + y_change * autonomous_roll_derivative_gain;
	motor_commands[3] = motor_commands[3] + y_change * autonomous_roll_derivative_gain;

}

void high_pitch_control(int x) {

	if (camera_data.sequence_num != prev_camera_sequence) {
		prev_camera_x_estimated = camera_x_estimated;
		camera_x_estimated = (camera_x_estimated * 0.6) + (x * 0.4);
	}

	float autonomous_pitch_error = camera_x_estimated - 100;

	// Proportional control
	motor_commands[0] = motor_commands[0] + autonomous_pitch_error * autonomous_pitch_gain;
	motor_commands[2] = motor_commands[2] + autonomous_pitch_error * autonomous_pitch_gain;
	motor_commands[1] = motor_commands[1] - autonomous_pitch_error * autonomous_pitch_gain;
	motor_commands[3] = motor_commands[3] - autonomous_pitch_error * autonomous_pitch_gain;

	float x_change = camera_x_estimated - prev_camera_x_estimated;
	// Derivative control
	motor_commands[0] = motor_commands[0] - x_change * autonomous_pitch_derivative_gain;
	motor_commands[2] = motor_commands[2] - x_change * autonomous_pitch_derivative_gain;
	motor_commands[1] = motor_commands[1] + x_change * autonomous_pitch_derivative_gain;
	motor_commands[3] = motor_commands[3] + x_change * autonomous_pitch_derivative_gain;

}