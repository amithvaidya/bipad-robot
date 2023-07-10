
 /*
  * Team Id: 1461     
  * Author List: Ankit Kumar, Piyush Goenka, Ramanath Amith, Nikhil Bhat      
  * Filename: MPU_6050_functions      
  * Theme: Biped Patrol      
  * Functions: lowpassfilter, highpassfilter, comp_filter, sesnor_initialize, get_theta_value, get_theta_dot_value     
  * Global Variables: None  
 */ 

 
///////////////////////////////// MPU6050 FUNCTIONS ///////////////////////////////////////



 /*  

 * Function Name: lowpassfilter 
 * Input: unfiltered_value,  filtered_value,  f_cut  
 * Output: filtered_output    
 * Logic: We pass the signal through a low pass filter by using a difference equation     
 * Example Call: singal = lowpassfilter(new_signal, filtered_signal, 5)      

 */

float lowpassfilter ( float unfiltered_value, float filtered_value, int16_t f_cut)
{
    float dT = dtime; 
    float Tau = 1/(2*f_cut*3.14);
    float alpha = Tau/(Tau+dT);                
    filtered_value = (1-alpha)*unfiltered_value + alpha*filtered_value;
    return (filtered_value);
}



 /*  

 * Function Name: highpassfilter 
 * Input: unfiltered_value,  filtered_value,  f_cut  
 * Output: filtered_output    
 * Logic: We pass the signal through a high pass filter by using a difference equation     
 * Example Call: singal = highpassfilter(new_signal, filtered_signal, 5)      

 */
 
int16_t highpassfilter ( int16_t unfiltered_value0,int16_t unfiltered_value, int16_t filtered_value, int16_t f_cut)
{
  
    float dT = dtime;
    float Tau = 1/(2*f_cut*3.14);
    float alpha = Tau/(Tau+dT);  
                  
    filtered_value = (1-alpha)*filtered_value+(1-alpha)*(unfiltered_value - unfiltered_value0);
    
    return(filtered_value);
}



 /*  

 * Function Name: comp_filter 
 * Input: unfiltered_value,  filtered_value,  f_cut  
 * Output: filtered_output    
 * Logic: We pass the signal through a complimentary filter to combine both gyro and accelerometer readings for better output      
 * Example Call: singal = comp_filter(ax, ay, az, gx, gy, gz, filtered_signal, 5)      

 */
 
double comp_filter(int16_t Ax,int16_t Ay,int16_t Az,int16_t Gx,int16_t Gy,int16_t Gz, float angle_old,int16_t f_cut)
{  
    
    float dT = dtime;
    float Tau = 1/(2*f_cut*3.14);
    float alpha = 0.1;                  // alpha =0.1 gives best results
    
    double ax = Ax;
    double ay = Ay;
    double az = Az;
    double gx = Gx/ 131.0;
    double gy = Gy/ 131.0;
    double gz = Gz/ 131.0;

    double acc = -atan(ax/ay);             // -ve sign because sensor is mounted in oppoite direction
   
    double gyro=gz;
    double angle = (1-alpha)*(angle_old + gyro*dT) + (alpha)*(180/3.14)*(acc);
    
   return (angle); 
}


 /*  

 * Function Name: sensor_initialize
 * Input: None     
 * Output: None    
 * Logic: Initialises the sensor for the robot      
 * Example Call: sensor_initialize ()       

 */
 
void sensor_initialize (void)
{
      // join I2C bus (I2Cdev library doesn't do this automatically)
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
      #endif
      accelgyro.initialize();

}




 /*  

 * Function Name: get_theta_value 
 * Input: None     
 * Output: Gives value of angle in degrees    
 * Logic: We read sensor data and process the signals through complimentary filter and get the value of theta     
 * Example Call: theta = get_theta_value()      

 */

 
double get_theta_value (void)
{ 
  sei();
  
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  cli();        // clear interrupt
   
   gz = -1*gz; 

  gz_h= highpassfilter (gz0,gz,gz_h,10);
  
  
  ax_l= lowpassfilter ( ax,ax_l,3);
  ay_l= lowpassfilter ( ay,ay_l,3 );
  gz0=gz;

  angle_1 = comp_filter(ax_l,ay_l,az,gx_h,gy_h,gz_h,angle_0,5);
  angle_1 = lowpassfilter ( angle_1,angle_0,10 );                 // This low-pass filter smoothens the signal
  angle_0=angle_1;
  
  return (round((angle_1)*10.0)/10.0);                            
}



 /*  

 * Function Name: get_theta_dot_value 
 * Input: None     
 * Output: Gives value of angular velocity in degrees    
 * Logic: We read gyro sensor data and compute the angular velocity   
 * Example Call: theta_dot = get_theta_dot_value()      

 */

 
double get_theta_dot_value (void)
{
   t_dot = gz/131.0 -2.35 ;  // -2.35 is offset by the gyro sensor
  
  t_dot_l = lowpassfilter ( t_dot,t_dot_l,10);   // f_cut of 10Hz yields better graph

 return ( round(t_dot_l*10.0)/10.0);
}


///////////////////////////////// END MPU6050 FUNCTIONS ///////////////////////////////////////
