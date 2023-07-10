
 /*
  * Team Id: 1461     
  * Author List: Ankit Kumar, Piyush Goenka, Ramanath Amith, Nikhil Bhat      
  * Filename: Motor_and_Encoder_functions      
  * Theme: Biped Patrol      
  * Functions: motors_actuate, motor_setup, rightEncoderEvent, leftEncoderEvent, encoder_setup, get_phi_value, get_phi_dot_value      
  * Global Variables: None  
 */


///////////////////////////////// MOTORS FUNCTIONS ///////////////////////////////////////


 /*  

 * Function Name: motors_actuate 
 * Input: pwm_value, left_motor_pwm_offset, right_motor_pwm_offset      
 * Output: Nonw    
 * Logic: We give power to the motors based on pwm values     
 * Example Call: motors_actuate (255, 10, 5);       

 */
 
 void motors_actuate (int pwm_value,int left_offset,int right_offset )
  { 

    int left_pwm;
    int right_pwm;
    pwm_l = lowpassfilter ( pwm_value,pwm_l,10 );    // THis low-pass filter reduces Jitter

    pwm_value = pwm_l;                               //pwm value is low pass filter o/p     

  
    
    if (abs(theta)>40)  /////////////////////////////// For turning off motors if robot cannot recover
    {
     pwm_value =0;
    }

   
    
//-----------------------------------------------------%%%%% left-start-----------------------------------------------------------------------------------------
    if(pwm_value+left_offset < 0 )
    { left_pwm = constrain(pwm_value+left_offset ,-50,-255);}    
    if(pwm_value+left_offset >= 0 )
    { left_pwm = constrain(pwm_value+left_offset ,50, 255) ;}   
    
    if (left_pwm<0)
    {
      left_pwm = left_pwm*(-1);
      digitalWrite(LM1 , 0);
      digitalWrite(LM2 , 1);
      analogWrite(LMEN , left_pwm);
    }
    
    else if (left_pwm>0)
    {
  
    digitalWrite(LM1 , 1);
    digitalWrite(LM2 , 0);
    analogWrite(LMEN ,left_pwm);
    }
  
    else  // if (left_pwm==0)
    {
    analogWrite(LMEN , 0);
    }

//-----------------------------------------------------%%%%% left end------------------------------------------------------------------------------------------
//-----------------------------------------------------%%%%% right start------------------------------------------------------------------------------------------
    if(pwm_value+right_offset < 0 )
    { right_pwm = constrain(pwm_value+right_offset ,-50,-255);}    
    if(pwm_value+right_offset >= 0 )
    { right_pwm = constrain(pwm_value+right_offset ,50, 255) ;}   
    
    if (right_pwm<0)
    {
      right_pwm = right_pwm*(-1);
      digitalWrite(RM1 , 0);
      digitalWrite(RM2 , 1);
      analogWrite(RMEN , right_pwm);
    }
    
    else if (right_pwm>0)
    {
  
    digitalWrite(RM1 , 1);
    digitalWrite(RM2 , 0);
    analogWrite(RMEN ,right_pwm);
    }
  
    else  // if (right_pwm==0)
    {
    analogWrite(RMEN , 0);
    }

//-----------------------------------------------------%%%%% right end------------------------------------------------------------------------------------------



}



 /*  

 * Function Name: motor_setup
 * Input: None      
 * Output: None     
 * Logic: Sets up and initializes the motor pins     
 * Example Call: motor_setup()      

 */

 
void motor_setup(void)
{
  pinMode (LM1 , OUTPUT);
  pinMode (LM2 , OUTPUT);
  pinMode (RM1 , OUTPUT);
  pinMode (RM2 , OUTPUT);
  pinMode (LMEN , OUTPUT);
  pinMode (RMEN , OUTPUT);
  
  digitalWrite(LM1 , 0);
  digitalWrite(LM2 , 0);
  digitalWrite(RM1 , 0);
  digitalWrite(RM2 , 0);
  digitalWrite(LMEN , 0);
  digitalWrite(RMEN , 0);
}


/////////////////////////////////END MOTORS FUNCTIONS ///////////////////////////////////////



///////////////////////////////// ENCODER FUNCTIONS ///////////////////////////////////////



 /*  

 * Function Name: rightEncoderEvent
 * Input: None      
 * Output: None     
 * Logic: Increments or decrements the right motor encoder count by 1      
 * Example Call: rightEncoderEvent()       

 */
 
void rightEncoderEvent() {
  
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;                           // Clockwise
    } else {
      rightCount--;                           // Anti-Clockwise
    }
  
  
}



 /*  

 * Function Name: leftEncoderEvent
 * Input: None      
 * Output: None     
 * Logic: Increments or decrements the left motor encoder count by 1      
 * Example Call: leftEncoderEvent()       

 */

 
void leftEncoderEvent() {
  
    if (digitalRead(LH_ENCODER_B) == LOW) 
    {
      leftCount--;                            // Clockwise
    } else
    {
      leftCount++;                            // Anti-Clockwise
    }
}



 /*  

 * Function Name: encoder_setup
 * Input: None      
 * Output: None    
 * Logic: Ses up the Encoder pins      
 * Example Call: encoder_setup()       

 */

 
void encoder_setup(void)
{
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  
 attachInterrupt(1, rightEncoderEvent, RISING);  // pin 3 right motor channel A
 attachInterrupt(0, leftEncoderEvent, RISING);   // pin 2 left motor channel A
}


 /*  

 * Function Name: get_phi_value
 * Input: None     
 * Output: Angle travelled by the wheels in radians
 * Logic: We take the encoder counts of the motors and convert it into radians and average them    
 * Example Call: phi_value = get_phi_value()      

 */

double get_phi_value(void)
{
  angle_r = (rightCount/270.0)*6.283;    // 2 * pi = 6.283 .... angle_r is in radians 
  angle_l = (leftCount/270.0)*6.283;

  return ((angle_r + angle_l) / 2.0 );
}



 /*  

 * Function Name: get_phi_dot_value
 * Input: None     
 * Output: Angular velocity of the wheel     
 * Logic: We take difference between the angles made by wheels in consecutive time instants and we average them together     
 * Example Call: phi_dot = get_phi_dot_value()       

 */
 
double get_phi_dot_value(void)
{
  angle_r_dt = (angle_r - angle_r0 )/ dtime;                // angle_r_dt is in degrees per second
  angle_l_dt = (angle_l - angle_l0 )/ dtime;

  angle_r_dt_0 = lowpassfilter ( angle_r_dt,angle_r_dt_0,5 ); // smoothens the d_phi
  angle_l_dt_0 = lowpassfilter ( angle_l_dt,angle_l_dt_0,5 ); // smoothens the d_phi

  angle_r0= angle_r;
  angle_l0= angle_l;

  return ((angle_r_dt_0 + angle_l_dt_0) / 2.0 );
}



///////////////////////////////// END ENCODER FUNCTIONS ///////////////////////////////////////
