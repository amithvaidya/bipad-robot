
 /*
  * Team Id: 1461     
  * Author List: Ankit Kumar, Piyush Goenka, Ramanath Amith, Nikhil Bhat      
  * Filename: xbee_joystick_LED_EM_buzzer_functions      
  * Theme: Biped Patrol      
  * Functions: xbee_read, joystick_actuate_motors, other_devices_setup     
  * Global Variables: None  
 */ 



/*  

 * Function Name: xbee_read
 * Input: None      
 * Output: Joystick state    
 * Logic: Reads serial data from xbee Rx through Serial2.read function of  UART2     
 * Example Call: xbee_read ()      

 */

int xbee_read (void)
{ 
  int remote_state, EM_values, LED_buzzer_values ;
  
 if (Serial2.available()>25)      // Remote XBee sends a frame of 26 bytes, so we wait till we recieve one complete Frame and then read the Frame 
{
  if (Serial2.read()==0x7E)      // 0x7E is the Start byte
  {
  for (int i=2; i<20 ; i++)
  {
    discard=Serial2.read();     // Read the next 18 bytes (they are not important to us)
  }

   EM_values= Serial2.read();  // 20th Byte contains EM digital values
   LED_buzzer_values= Serial2.read();        // 21st Byte contains Red and Green  LEDs digital values
   Joystick_X = Serial2.read();         // 22nd Byte contains upper byte for Forward-Reverse states of the Motor in an Analog value ( we only consider the upper byte)
   discard=Serial2.read();           // 23rd Byte contains lower byte for Forward-Reverse states of the Motor in an Analog value (we ignore the lower byte)
   Joystick_Y = Serial2.read();         // 24td Byte contains upper byte for Right-Left states of the Motor in an Analog value ( we only consider the upper byte)
   discard=Serial2.read();           // 25th Byte contains lower byte for Right-Left states of the Motor in an Analog value ( we ignore the lower byte)
   discard=Serial2.read();           // We ignore the checksum



      if((EM_values&0x08)>0)  
     digitalWrite(EM_front , HIGH);
   else
     digitalWrite(EM_back , HIGH);
     

   if((EM_values&0x10)>0) 
   digitalWrite(EM_back, HIGH);
   else
   digitalWrite(EM_back, LOW);

   if((LED_buzzer_values&0x04)>0)      
   digitalWrite(Red_LED, LOW);
   else
   digitalWrite(Red_LED, HIGH);

   if((LED_buzzer_values&0x08)>0)     
   digitalWrite(Green_LED, LOW);
   else
   digitalWrite(Green_LED, HIGH);


   if((LED_buzzer_values&0x10)>0)  
   digitalWrite(buzzer, HIGH);
   else
   digitalWrite(buzzer, LOW);
   

// The upper byte analog value for fowrard and right is equal to 3
// The upper byte analog value for stop is equal to 2
// The upper byte analog value for reverse and left is equal to 0

        if (Joystick_X == 3 && Joystick_Y == 2 )   
            remote_state= 1;                                               //FORWARD STATE
    
        else if (Joystick_X == 0 && Joystick_Y == 2 )          
            remote_state= 2 ;                                              //BACKWARD STATE
    
        else if (Joystick_X == 2 && Joystick_Y == 3 )                      //RIGHT STATE
           remote_state= 3 ;               
    
        else if (Joystick_X == 2 && Joystick_Y == 0 )                     //LEFT STATE
          remote_state= 4 ;
   
        else 
       remote_state= 0 ;

    
  }
  } 
  return remote_state;
}





/*  

 * Function Name: joystick_actuate_motors
 * Input: Joystick state      
 * Output: None     
 * Logic: Based on Joystick state, bot is moved in a different way      
 * Example Call: joystick_actuate_motors(1)       

 */

 
void joystick_actuate_motors(int state)
{
  float current_PHI_Ang_velocity= phi;
  float sp_angle_1;
  float sp_angle_2;
  switch(state)
 {
  case 1: //foward 
        setpoint_encoder_pos =setpoint_encoder_pos + 100; //inc encoder pos
        setpoint_PHI_Ang_velocity=  5;                   //rpm-this must be const
        
        sp_angle_1= PID_control_encoder(setpoint_encoder_pos,((leftCount+rightCount)/2) ,Kp_encoder, Kd_encoder, Ki_encoder , dtime );  
        sp_angle_2= PID_velocity_Control(setpoint_PHI_Ang_velocity, current_PHI_Ang_velocity, Kp_vel, Kd_vel,  Ki_vel,  dtime);
        
        setpoint_tilt_angle= sp_angle_2 + sp_angle_1;
        
        pwm_op = PID_control_tilt (setpoint_tilt_angle, theta, theta_dot ,Kp_tilt, Kd_tilt, Ki_tilt , dtime );
        motors_actuate ( pwm_op,0,0);
        break;

  case 2: //backward
  
        setpoint_encoder_pos =setpoint_encoder_pos -100; //inc encoder pos
        setpoint_PHI_Ang_velocity=  5;                   //rpm-this must be const
        
        sp_angle_1= PID_control_encoder(setpoint_encoder_pos,((leftCount+rightCount)/2) ,Kp_encoder, Kd_encoder, Ki_encoder , dtime );  
        sp_angle_2= PID_velocity_Control(setpoint_PHI_Ang_velocity, current_PHI_Ang_velocity, Kp_vel,  Kd_vel,  Ki_vel,  dtime);
        setpoint_tilt_angle= sp_angle_2 + sp_angle_1;
        
        pwm_op = PID_control_tilt (setpoint_tilt_angle, theta, theta_dot ,Kp_tilt, Kd_tilt, Ki_tilt , dtime );
        motors_actuate ( pwm_op,0,0);

        break;

  case 3:      //rotate RIGHT

        motors_actuate ( 150,-10,10);
        break;

  case 4:     //rotate left
        motors_actuate ( 150,10,-10);

        break;
        
  default: 
           break;

 }
  
}




 ///////////////////////////////// BUZZER EM LED FUNCTIONS ///////////////////////////////////////


 /*  

 * Function Name: other_devices_setup
 * Input: None      
 * Output: None     
 * Logic: Sets up and initializes the EMs, buzzer and LEDs      
 * Example Call: other_devices_setup()       

 */

 
void other_devices_setup(void)
{
    pinMode (EM_front , OUTPUT);
    pinMode (EM_back , OUTPUT);
    pinMode (Red_LED , OUTPUT);
    pinMode (Common_Anode , OUTPUT);
    pinMode (Green_LED , OUTPUT);
    pinMode (VCC , OUTPUT);
    pinMode (GND , OUTPUT);
    pinMode (buzzer , OUTPUT);   
        
    digitalWrite(EM_front , 0);
    digitalWrite(EM_back , 0);
    digitalWrite(Red_LED , 1);
    digitalWrite(Common_Anode , 0);
    digitalWrite(Green_LED , 1);
    digitalWrite(VCC , 1);
    digitalWrite(GND , 0);
    digitalWrite(buzzer , 0);
 
}


///////////////////////////////// END BUZZER EM LED FUNCTIONS ///////////////////////////////////////
