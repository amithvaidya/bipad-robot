 
 /*
  * Team Id: 1461     
  * Author List: Ankit Kumar, Piyush Goenka, Ramanath Amith, Nikhil Bhat      
  * Filename: PID_functions       
  * Theme: Biped Patrol      
  * Functions: PID_control_tilt, PID_control_encoder, PID_leftDrift_Control, PID_rightDrift_Control, PID_velocity_Control        
  * Global Variables: None  
 */ 


  ///////////////////////////////// PID FUNCTIONS ///////////////////////////////////////


  
/**********************************
Function name  : PID_control_tilt
Functionality : To compute the PWM value to set the motor speed using a PID Controller
                Process variable is tilt angle
Return Value  : pwm value
Example Call  : PID_control_tilt()
***********************************/


float PID_control_tilt (float setpoint, float theta, float theta_dot, float Kp, float Kd, float Ki, float elapsedTime) 
{
  
  float error,cumError;
  float output;
  error = theta - setpoint -2.6;                                              ///////////////////////////// error 
  float proportionalTerm =  Kp*error ;                                       ////////////////////////// Proportional
  float derivativeTerm   =  Kd*theta_dot;
  output =  proportionalTerm +  derivativeTerm  ;                          
  return (output);

}




/**********************************
Function name  : PID_control_encoder
Functionality : To compute the angle set point using a Proportional controller.
                Process variables is position (encoder count).
                This is the dominant controller when robot is still and is 
                used to hold the position of the robot and prevent drifting.
Return Value  : Angle setpoint
Example Call  : PID_control_encoder()
***********************************/


float PID_control_encoder (float setpoint, float encoderPos,  float Kp, float Kd, float Ki, float elapsedTime) 
{
  
  float error,cumError,rateError,lastError;
  float output;
  
  error = encoderPos - setpoint;                                             
  rateError = (error - lastError)/elapsedTime;
  cumError += error * elapsedTime;        
  
  float proportionalTerm =  -Kp * error      *0.0001;                       
  float derivativeTerm   =  -Kd * rateError  *0.0001;

 
  output =  proportionalTerm+  derivativeTerm ;                             
  output = constrain (output, -5, 5);                                     

  lastError=error;
  return (output);                // Angle set-point
  

}



/**********************************
Function name  : PID_leftDrift_Control and PID_rightDrift_Control
Functionality : To prevent the rotational drift of the robot during static balance using a 
                simple Proportional controller to minimize difference in encoder counts
Return Value  : Left and Right rotational offset.
Example Call  : PID_leftDrift_Control(), PID_rightDrift_Control()
***********************************/


 float PID_leftDrift_Control(int right_encoder_count,int left_encoder_count)
 {
  float rotation_left = -1 * (left_encoder_count - right_encoder_count) * 0.05 * Kp_left_drift;
  return(rotation_left);
 }

  float PID_rightDrift_Control(int right_encoder_count,int left_encoder_count)
 {
   float rotation_right = (left_encoder_count - right_encoder_count) * 0.05 * Kp_right_drift;
   return(rotation_right);
 }





/**********************************
Function name  : PID_velocity_Control
Functionality : To compute the angle set point using a PID Controller.
                Process variable is velocity in RPM of the motors.
                Velocity is measured to control the angle set point in order to
                prevent the robot from accelerating too much and falling down and 
                to control the target velocity of the robot while in motion.
                This is the dominant controller when robot is moving.
Return Value  : angle setpoint_
Example Call  : PID_velocity_Control()
***********************************/


float PID_velocity_Control(float setpoint, float PHI_dot, float Kp, float Kd, float Ki, float elapsedTime)
{

  float error,cumError,rateError,lastError;
  float output;
  
  error = PHI_dot - setpoint;                                             
  rateError = (error - lastError)/elapsedTime;
  cumError += error * elapsedTime;        
  
  float proportionalTerm =  -Kp * error      *0.001;                       
  float derivativeTerm   =  -Kd * rateError  *0.001;

 
  output =  proportionalTerm+  derivativeTerm ;                             
  output = constrain (output, -5, 5);             //constrain is for for rpm o/p of motors                               

  lastError=error;
  return (output);                                //output is angle setpoint
  

}



 ///////////////////////////////// END PID FUNCTIONS ///////////////////////////////////////


 
