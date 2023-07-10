
 /*
  * Team Id: 1461     
  * Author List: Ankit Kumar, Piyush Goenka, Ramanath Amith, Nikhil Bhat      
  * Filename:MAIN_FILE       
  * Theme: Biped Patrol      
  * Functions: setup, loop, TImer_1_ISR, timer_interrupt_setup        
  */


 
///////////////////////  MPU 6050 libraries /////////////////////////
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif  
/////////////////////// END MPU 6050 libraries /////////////////////////


///////////////////////// EM buzzer LED pins ///////////////////////
# define EM_front 34
# define EM_back 36
# define Red_LED 38
# define Common_Anode 40
# define Green_LED 42
# define VCC 46
# define buzzer 48
# define GND 50
///////////////////////// END EM buzzer LED pins /////////////////////


///////////////////////// Motor pins ///////////////////////

# define RM1 7
# define RM2 6
# define LM1 4
# define LM2 5
# define LMEN 8
# define RMEN 9

///////////////////////// END Motor pins //////////////////



///////////////////////// ENCODER pins ///////////////////

#define RH_ENCODER_A 3 
#define RH_ENCODER_B 11
#define LH_ENCODER_A 2
#define LH_ENCODER_B 10

///////////////////////// END ENCODER pins /////////////



///////////////////////// MPU6050 variables ////////////

MPU6050 accelgyro;
int16_t ax=0, ay=0, az=0 ,ax_l=0, ay_l=0, az_l=0;
int16_t gx=0, gy=0, gz=0 ,gx0=0, gy0=0, gz0=0, gx_h=0 , gy_h=0, gz_h=0;
float angle_0=0, angle_1=0, angle_1_l=0,d_angle=0,gz_l=0, t_dot =0 , t_dot_l=0;

///////////////////////// END MPU6050 variables ///////////////////////



///////////////////////// Encoder variables ///////////////////////////////

signed long rightCount = 0,leftCount = 0;
double angle_r = 0,angle_l = 0; 
double angle_r0 = 0,angle_l0 = 0;
float angle_r_dt = 0,angle_l_dt = 0;
float angle_r_dt_0=0,angle_l_dt_0=0; 

///////////////////////// END Encoder variables ///////////////////////////////




//---------------------------------PID---------------------------------------------------------------------------------------------------------------//

////////////////////////////// PID variables-tilt_angle ///////////////////////
float integralSum_tilt = 0; 
float Kp_tilt = 80.0;
float Kd_tilt= 5.2;
float Ki_tilt =0;
float lastError_tilt =0;
float rateError_tilt =0;
//////////////////////////// END PID variables-tilt_angle ///////////////////////

///////////////////////// PID variables-encoder_pos ///////////////////////
float integralSum_encoder = 0; 
float Kp_encoder = 45.67;
float Kd_encoder= 12.9;
float Ki_encoder =0;
float lastError_encoder =0;
float rateError_encoder =0;
int loop_count_enc = 0;
///////////////////////// END PID variables_encoder_pos ///////////////////////

/////////////////////////START drift control variables/////////////////////////////////
int loop_count_drift_control=0;
float Kp_left_drift=10;
float Kp_right_drift=7;
/////////////////////////END driftcontrol variables//////////////////////////////

/////////////////////////START velocity control variables/////////////////////////////////
float Kp_vel = 27.3;
float Kd_vel = 6.98;
float Ki_vel = 0;
/////////////////////////END velocity variables//////////////////////////////

//------------------------------------END PID--------------------------------------------------------------------

//xbee
byte discard = 0;              //variable to store useless bytes
byte Joystick_X = 0;          // variable to store Forward-Reverse Joystick state
byte Joystick_Y = 0;          // variable to store Right-Left Joystick state
int xbee_output =0;           // 0 -> stop , 1 -> Forward , 2 -> Back , 3-> right , 4-> left


//end xbee

///////////////////// General variables /////////////////////////////////
float pwm_op =0;
float pwm_l = 0;
float theta = 0.0;
float theta_dot =0.0;
float phi = 0;
float phi_dot =0;

float left_offset=0;
float right_offset=0;

int joystick_state;
float dtime = 0.005;

///////////////////// END General variables /////////////////////////////////



//////////////////// setpoint variables ////////////////////////////////////////

float setpoint_tilt_angle =0.0;
float setpoint_encoder_pos =0.0;
float setpoint_PHI_Ang_velocity=0.0;

//////////////////// END setpoint variables //////////////////////////////





// ************************************************************************************ Functions ********************************************************************************************************************************




////////////////////////// SETUP ///////////////////////////////////////////////// 

/*  

 * Function Name: setup
 * Input: None     
 * Output: None     
 * Logic: Initializses the code      
 * Example Call: setup()      

 */
 
void setup() 
{
   motor_setup();
   encoder_setup();
   sensor_initialize();
   other_devices_setup();
   timer_interrupt_setup ();
   Serial2.begin (115200);  
}
///////////////////////////// END setup /////////////////////////////////////////////





//////////////////// LOOP ////////////////////////////////////////////////// 

/*  

 * Function Name: loop
 * Input: None      
 * Output: None     
 * Logic: Runs the code inside it continuously forever     
 * Example Call: loop()      

 */
 
 void loop() 
  {
      
      delay(10);

      joystick_state=xbee_read();
   }
 //////////////////////////////// END LOOP ////////////////////////////////////////////////////// 




///////////////////////////////// START ISR-for timer1 ////////////////////////////////////////////


/*  

 * Function Name: ISR()
 * Input: None      
 * Output: None    
 * Logic: Fuction is called when TImer 1 interrupt occurs      
 * Example Call: ISR(TIMER1_COMPA_vect)      

 */
 
ISR(TIMER1_COMPA_vect)
{
  theta =     get_theta_value();
  theta_dot = get_theta_dot_value();
  phi =       get_phi_value();
  phi_dot =   get_phi_dot_value();

    //counter to determine PID_control_encoder freq
   loop_count_enc=loop_count_enc+1;
   if (loop_count_enc==2){
     setpoint_tilt_angle=  PID_control_encoder(setpoint_encoder_pos,((leftCount+rightCount)/2) ,Kp_encoder, Kd_encoder, Ki_encoder , dtime );   
    loop_count_enc =0; 
   }
  
    //counter to determine PID_control_drift_control freq
    loop_count_drift_control=loop_count_drift_control+1;                 
    if (loop_count_drift_control==2){
    left_offset=  PID_leftDrift_Control(rightCount,leftCount);                                                              
    right_offset=  PID_rightDrift_Control(rightCount,leftCount);   
    loop_count_drift_control =0; 
   }

    
    if(joystick_state>0)
    {
       joystick_actuate_motors(joystick_state);
    }

    else  // joystick sate is 0 means bot is in static condition
    {
      pwm_op = PID_control_tilt (setpoint_tilt_angle, theta, theta_dot ,Kp_tilt, Kd_tilt, Ki_tilt , dtime );
      motors_actuate ( pwm_op,left_offset,right_offset);
      
    }

    sei();
 }
///////////////////////////////// END ISR-for timer1 ////////////////////////////////////////////





///////////////////////////////// TIMER INTERRUPT FUNCTION ///////////////////////////////////////

/*  

 * Function Name: timer_interrupt_setup
 * Input: None      
 * Output: None     
 * Logic: Setup timer 1 Interrupt settings      
 * Example Call: timer_interrupt_setup ()       

 */

void timer_interrupt_setup (void)
 {
  cli();//stop interrupts

//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 2000000*dtime - 1 ; // = (16*10^6) / (freq*8) - 1 (must be <65536) 10ms - 19999, 5ms - 9999
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bits for 1024 prescaler = 8
 
  TCCR1B |= (1 << CS11) ;   
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
 }


///////////////////////////////// END TIMER INTERRUPT FUNCTION ///////////////////////////////////////
