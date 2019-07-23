/*
 * File:          u3_exam_Canul_Daniel.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
 #include <webots/robot.h>
 #include <webots/motor.h>
 #include <webots/keyboard.h>
 #include <webots/distance_sensor.h>
 #include <webots/position_sensor.h>

 #include <stdio.h>
/*
 * You may want to add macros here.
 */
 #define TIME_STEP 64

 #define PI 3.1416
 #define DIST_OBSTACLE 20.0
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv){
  /* necessary to initialize webots stuff */
  wb_robot_init();

  wb_keyboard_enable(TIME_STEP);

  int pressed_key;
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   //Motor devices
   WbDeviceTag wheel_right = wb_robot_get_device("wheel1");
   WbDeviceTag wheel_left = wb_robot_get_device("wheel2");
   WbDeviceTag wheel_back = wb_robot_get_device("wheel3");

   wb_motor_set_position(wheel_right, INFINITY);
   wb_motor_set_position(wheel_left, INFINITY);
   wb_motor_set_position(wheel_back, INFINITY);

   //Encoder devices
   WbDeviceTag Encoder1 = wb_robot_get_device("encoder1");
   WbDeviceTag Encoder2 = wb_robot_get_device("encoder2");
   WbDeviceTag Encoder3 = wb_robot_get_device("encoder3");

   wb_position_sensor_enable(Encoder1, TIME_STEP);
   wb_position_sensor_enable(Encoder2, TIME_STEP);
   wb_position_sensor_enable(Encoder3, TIME_STEP);

   float Enco1=0;
   float Enco2=0;
   float Enco3=0;

   //distance sensor devices
   WbDeviceTag dist_sensorR1 = wb_robot_get_device("dist_sensor1");
   WbDeviceTag dist_sensorL2 = wb_robot_get_device("dist_sensor2");

   wb_distance_sensor_enable(dist_sensorR1, TIME_STEP);
   wb_distance_sensor_enable(dist_sensorL2, TIME_STEP);

   float ds_Right1=0;
   float ds_Left2=0;

   //Other variables
   float compare=0;
   float turn_right=0;
   float turn_left=0;
   int w=0;
   int g=1;
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   printf("Left arrow to move linearly to the left\n");
   printf("Right arrow to move linearly to the Right\n");
   printf("UP arrow to move Forward linearly\n");
   printf("DOWN arrow to move Backward linearly\n");
   printf("A key to turn 45° to the left\n");
   printf("S key to turn 45° to the right\n");
   printf("G key to set the automatic mode\n");
   printf("W key to set the manual mode\n");
   
   void manual(){

        //Distance Sensor Read
        ds_Right1 = wb_distance_sensor_get_value(dist_sensorR1);
        ds_Left2 = wb_distance_sensor_get_value(dist_sensorL2);

        Enco1 = wb_position_sensor_get_value(Encoder1);
        Enco2 = wb_position_sensor_get_value(Encoder2);
        Enco3 = wb_position_sensor_get_value(Encoder3);

        //printf("distance_right: %lf\r\n", ds_Right1);
        //printf("distance_left: %lf\r\n", ds_Left2);

        //printf("Encoder1: %lf\r\n", Enco1);
        //printf("Encoder2: %lf\r\n", Enco2);
        //printf("Encoder3: %lf\r\n", Enco3);
        //printf("Comparador: %lf\n",compare );
        // look for obsctacles
        //bool right_obst = dist_right <= OBSTACLE_DIST;
       // bool left_obst = dist_left <= OBSTACLE_DIST;




      if(pressed_key == WB_KEYBOARD_UP){

          wb_motor_set_velocity(wheel_left, 5);
          wb_motor_set_velocity(wheel_right, -5);
          wb_motor_set_velocity(wheel_back, 0);
      }
      else if(pressed_key == WB_KEYBOARD_DOWN){
          wb_motor_set_velocity(wheel_left, -5);
          wb_motor_set_velocity(wheel_right, 5);
          wb_motor_set_velocity(wheel_back, 0);

      }
      else if(pressed_key == WB_KEYBOARD_LEFT){
          wb_motor_set_velocity(wheel_left, 0);
          wb_motor_set_velocity(wheel_right, -5);
          wb_motor_set_velocity(wheel_back, 5);

      }
      else if(pressed_key == WB_KEYBOARD_RIGHT){
          wb_motor_set_velocity(wheel_left, 0);
          wb_motor_set_velocity(wheel_right, 5);
          wb_motor_set_velocity(wheel_back, -5);

      }
      else if(pressed_key == 'S' ){
          compare = Enco1 + 0.785398;
          turn_left = 1;
      }

      else if(turn_left == 1){

          if(Enco1 <= compare){
           wb_motor_set_velocity(wheel_left, 5);
           wb_motor_set_velocity(wheel_right,5);
           wb_motor_set_velocity(wheel_back, 5);
          }
          else{
           wb_motor_set_velocity(wheel_left, 0);
           wb_motor_set_velocity(wheel_right, 0);
           wb_motor_set_velocity(wheel_back, 0);
           turn_left = 0;
          }

      }

      else if(pressed_key == 'A' ){
          compare = Enco1 - 0.785398;
          turn_right = 1;
      }
         
      else if(turn_right == 1){
          if(Enco1 >= compare){
           wb_motor_set_velocity(wheel_left, -5);
           wb_motor_set_velocity(wheel_right,-5);
           wb_motor_set_velocity(wheel_back, -5);
          }
          else{
           wb_motor_set_velocity(wheel_left, 0);
           wb_motor_set_velocity(wheel_right, 0);
           wb_motor_set_velocity(wheel_back, 0);
           turn_right = 0;
          }

      }
      else{
        wb_motor_set_velocity(wheel_left, 0);
        wb_motor_set_velocity(wheel_right, 0);
        wb_motor_set_velocity(wheel_back, 0);
      }
   }

  void automatico(){

    //Distance Sensor Read
    ds_Right1 = wb_distance_sensor_get_value(dist_sensorR1);
    ds_Left2 = wb_distance_sensor_get_value(dist_sensorL2);

    Enco1 = wb_position_sensor_get_value(Encoder1);
    Enco2 = wb_position_sensor_get_value(Encoder2);
    Enco3 = wb_position_sensor_get_value(Encoder3);

    printf("distance_right: %lf\r\n",ds_Right1);
    printf("distance_left: %lf\r\n", ds_Left2);

    printf("Encoder1: %lf\r\n", Enco1);
    printf("Encoder2: %lf\r\n", Enco2);
    printf("Encoder3: %lf\r\n", Enco3);
    // printf("Comparador: %lf\n",compare );
    // Look for obsctacles
    // bool right_obst = dist_right <= OBSTACLE_DIST;
    // bool left_obst = dist_left <= OBSTACLE_DIST;

    wb_motor_set_velocity(wheel_left,  6.66);
    wb_motor_set_velocity(wheel_right, -6.66);
    wb_motor_set_velocity(wheel_back, 0);

    if(ds_Left2 < ds_Right1 && ds_Left2 < 400){

     wb_motor_set_velocity(wheel_left, 6.66);
     wb_motor_set_velocity(wheel_right,6.66);
     wb_motor_set_velocity(wheel_back, 6.66);
    }
    else if(ds_Left2 > ds_Right1 && ds_Right1 < 400){
     wb_motor_set_velocity(wheel_left, -6.66);
     wb_motor_set_velocity(wheel_right, -6.66);
     wb_motor_set_velocity(wheel_back, -6.66);
    }
  }

   wb_motor_set_velocity(wheel_left, 0);
   wb_motor_set_velocity(wheel_right, 0);
   wb_motor_set_velocity(wheel_back, 0);



  while (wb_robot_step(TIME_STEP) != -1){

    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
     //Distance sensor read
    //ds_Right1 = wb_distance_sensor_get_value(dist_sensorR1);
    //printf("First Distance sensor %lf\r\n", ds_Right1);

    //ds_Left2 = wb_distance_sensor_get_value(dist_sensorL2);
    //printf("Second Distance sensor %lf\r\n", ds_Left2);

    //Position Sensor read
    //Enco1 = wb_position_sensor_get_value(Encoder1);
    //printf("Encoder 1 value: %f\r\n", Enco1);

    //Enco2 = wb_position_sensor_get_value(Encoder2);
    //printf("Encoder 2 value: %lf\r\n", Enco2);

    //Enco3 = wb_position_sensor_get_value(Encoder3);
    //printf("Encoder 3 value: %lf\r\n", Enco3);

    /* Process sensor data here */
    pressed_key=wb_keyboard_get_key();
    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
     if(pressed_key == 'W'){
       w = 1;
       g = 0;
     }

     else if (pressed_key == 'G'){
       g = 1;
       w = 0;
     }
     //printf("W: %i   ",w);
    // printf("G: %i\n",g);

     if(w == 1)
     manual();

     if(g == 1)
     automatico();
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
