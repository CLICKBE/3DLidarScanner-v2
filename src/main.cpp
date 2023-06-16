/**
 * 
 * @file main.cpp
 * @author Loïc Reboursière, Maxime Vander Goten, Sami Yunus - UMONS, CLick (loic.reboursiere@umons.ac.be, maxime.vandergoten@umons.ac.be, sami.yunus@umons.ac.be)
 * @brief This code is involved in the development of a 3D motorized LiDAR scanner. It controls two step motors (horizontal and vertical) which moves a LiDAR sensor.
 * When the scan is completed, a 3D cloud point of the scene is obtained which can be used in different ways.
 * 3DLidarScanner-v2 – CLICK - UMONS (Loïc Reboursière, Maxime Vander Goten, Sami Yunus) is free software: you can redistribute it and/or modify it under the terms of the Apache License Version 2.0. This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the Apache License Version 2.0 for more details.
 * You should have received a copy of the Apache License Version 2.0 along with this program.  If not, see http://www.apache.org/licenses/
 * Each use of this software must be attributed to University of MONS – CLICK (Loïc Reboursière, Maxime Vander Goten, Sami Yunus).
 * Any other additional authorizations may be asked to avre@umons.ac.be. 
 * @version 0.1
 * @date 2023-05-25
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
// To use with TFMini-Plus library, https://github.com/budryerson/TFMini-Plus/tree/master
// along with getData( int16_t &dist, int16_t &flux, int16_t &temp) function
#include "TFMPlus.h"
// To use with TFMini Library, https://github.com/opensensinglab/tfmini/blob/master/src/TFMini.h
// Along with getDistance() function
//#include "TFMini.h"
//TFMini tfmini;

TFMPlus tfmini;
SoftwareSerial SerialTFMini(10, 11);  //define software serial port name as Serial1 and define pin2 as RX & pin3 as TX

// Arduino CNC Shield connections
// https://blog.protoneer.co.nz/arduino-cnc-shield/
#define MOTOR_X_STEP 2
#define MOTOR_Y_STEP 3
#define MOTOR_Z_STEP 4
#define MOTOR_X_DIRECTION 5
#define MOTOR_Y_DIRECTION 6
#define MOTOR_Z_DIRECTION 7
#define STEPPER_ENABLE 8

#define TICKS_PER_REVOLUTION 200 // Specification of the NEMA 17 type stepper motor
#define SUBDIVISION  1. / 8. // Chosen through a CNC Shield card jumper

#define SERIAL_SPEED 115200
#define TFMINI_SERIAL_SPEED 115200

float total_nb_of_steps = TICKS_PER_REVOLUTION / SUBDIVISION;
float degree_per_step = 360 / total_nb_of_steps; 

int vertical_nb_of_steps = total_nb_of_steps / 5;// Vertical rotation of 90°
int horizontal_nb_of_steps = total_nb_of_steps / 12; // Horizontal rotation of 30° - factor of 9 for 40°
int vertical_offset = 2;
int horizontal_offset = 10;

int posx=0;
int posy=0;

// Functions declaration
/***
 * Move the horizontal motor by a certain amount of steps
 * Arguments : 
 *  steps (long) : number of steps to move the motor.
 * Return :
 *  void
 */
void xmove( long steps );

/***
 * Move the vertical motor by a certain amount of steps
 * Arguments : 
 *  steps (long) : number of steps to move the motor.
 * Return :
 *  void
 */
void ymove( long steps );

/***
 * Move a specific motor by a certain amount of steps
 * Arguments : 
 *  steps (long) : number of steps to move the motor.
 *  motor_direction_pin (int) : the pin of the Arduino connected to the direction of the stepper motor.
 *  motor_step_pin (int) : the pin of the Arduino connected to the step of the stepper motor.
 * Return :
 *  void
 */
void moveStepper( long steps, int motor_direction_pin, int motor_step_pin );

/***
 * Display info from the scanner setup : (nb of steps (vertical and horizontal), ticks per revolution, total number of steps, subdivision  
 * Arguments : 
 *  None
 * Return :
 *  void
 */ 
void displayInfo();

/***
 * Send throught serial horizontal position, vertical position, distance and their converted XYZ coordinates.
 * The message is as follows : 
 *    scanning_horizontal_position scanning_vertical_position distance X Y Z
 * Arguments : 
 *  None
 * Return :
 *  void
 */
void displayCoordinates();

/***
 * Convert horiz_step_idx, vert_step_idx and distance to (x, y, z) coordinates.
 * Computation comes from : https://github.com/bitluni/3DScannerESP8266
 * And https://www.youtube.com/watch?v=vwUGPjQ_5t4
 * Arguments : 
 *  *x (float) : the float pointer to the x coordinates
 *  *y (float) : the float pointer to the y coordinates
 *  *z (float) : the float pointer to the z coordinates
 *  distance (int16_t) : distance retrieved by LiDAR sensor
 *  hAngleDegree (int) : horizontal angle in degree of the pantilt sweeping
 *  vAngleDegree (int) : vertical angle in degree of the pantilt sweeping
 * Return :
 *  void
 */ 
void computeXYZ( float *x, float *y, float *z, int16_t distance, int hAngleDegree, int vAngleDegree );


/* ----------------------------- ACTUAL PROGRAM ----------------------------- */
void setup() {
  
  //motor X
  pinMode( MOTOR_X_STEP, OUTPUT );
  digitalWrite( MOTOR_X_STEP,LOW );
  
  pinMode( MOTOR_X_DIRECTION, OUTPUT );
  digitalWrite( MOTOR_X_DIRECTION, LOW );

  //motor Y  
  pinMode( MOTOR_Y_STEP, OUTPUT );
  digitalWrite( MOTOR_Y_STEP, LOW );
  
  pinMode( MOTOR_Y_DIRECTION, OUTPUT );
  digitalWrite( MOTOR_Y_DIRECTION, LOW );
  
  // motor Z
  pinMode( MOTOR_Z_STEP, OUTPUT );
  digitalWrite( MOTOR_Z_STEP,LOW );
  
  pinMode( MOTOR_Z_DIRECTION, OUTPUT );
  digitalWrite( MOTOR_Z_DIRECTION, LOW );
  
  //motor enable
  pinMode( STEPPER_ENABLE, OUTPUT );
  digitalWrite( STEPPER_ENABLE, LOW );
  
  Serial.begin( SERIAL_SPEED );         //set bit rate of serial port connecting Arduino with computer

  SerialTFMini.begin( TFMINI_SERIAL_SPEED );    //Initialize the data rate for the SoftwareSerial port
  tfmini.begin( &SerialTFMini );            //Initialize the TF Mini sensor}

  // For now the steppers are manually zeroed. 
  // The zero pointface the ground (vertically) and is paralleled to the back of the structure (horizontal)
  // The total nb of horizontal steps is share out around this manually set horizontal zero 

}

int16_t distance = 0;    // Distance to object in centimeters
int16_t tfFlux = 0;    // Strength or quality of return signal
int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
float x = -1., y = -1., z = -1.;
bool scan = true;

void loop() {
  
  while( Serial.available() == 0 ){}

  char inbyte=Serial.read();
  //Serial.println( "Command " + String( inbyte ) );
  switch ( inbyte ){
  
  //-------------------------------
  // To be remove for final git
  //case 117: //u
  //  zmove(10);
  //  posz=posz+10;
    //Serial.print(posx);
    //Serial.write(9);
    //Serial.println(posy);
  //break;
  
  // To be remove for final git
  //case 100: //d
  //  zmove(-10);
  //  posz=posz-10;
    //Serial.print(posx);
    //Serial.write(9);
    //Serial.println(posy);
  //break;
  //------------------- 
   case 112: //p to allow scan reboot
    scan = true;
    break;

  case 115: //s for 2D scan
    
    // "Init" x axis position
    // Step motor must be positionned manually to the center 
    xmove( -horizontal_nb_of_steps / 2 );

    while(Serial.available()==0){
      //Serial.println( "scanning" );
      int16_t distance = 0;
      //int16_t strength = 0;
      
      // For use of TFMPlus library
      tfmini.getData( distance, tfFlux, tfTemp); // For use of TFMini library : distance = getDistance();

      while ( !distance && scan )
      {
        //Serial.println( "No distance" );

        // For use of TFMPlus library
        tfmini.getData( distance, tfFlux, tfTemp ); // For use of TFMini library : distance = getDistance();
        if (distance){
          ymove( vertical_offset );
          //moveStepper( vertical_offset, )
          posy = posy + vertical_offset;
          computeXYZ( &x, &y, &z, distance, posx * degree_per_step, posy * degree_per_step );
          delay(10);

          displayCoordinates();

          if ( posy >= vertical_nb_of_steps ) { ymove( -vertical_nb_of_steps ); posy = 0; xmove( horizontal_offset ); posx = posx + horizontal_offset;} // déplacement vertical ()
          if ( posx >= horizontal_nb_of_steps ) { xmove( -horizontal_nb_of_steps ); posx = 0; scan = false; Serial.println( "stop" ); break;} // 

        }
      }
    }
  Serial.println( "stop" );
  break;

  case 121: //y for 1D scan
    
    while(Serial.available()==0){
      //Serial.println( "scanning" );
      int16_t distance = 0;
      //int strength = 0;

      // For use of TFMPlus library
      tfmini.getData( distance, tfFlux, tfTemp); // For use of TFMini library : distance = getDistance();

      while ( !distance && scan )
      {

        // For use of TFMPlus library
        tfmini.getData( distance, tfFlux, tfTemp );// For use of TFMini library : distance = getDistance();
        if (distance){
          //Serial.println( distance );
          // ymove( vertical_offset );
          ymove( vertical_offset );
          posy = posy + vertical_offset;
          computeXYZ( &x, &y, &z, distance, posx * degree_per_step, posy * degree_per_step );
          delay(10);

          displayCoordinates();

          if ( posy >= vertical_nb_of_steps ) {
            scan = false;
            Serial.println( "stop" );
            break;
          }

        }
      }
    }
  Serial.println( "end" );
  break;


  case 105 : // i to display info from the scanner setup
    displayInfo();
    Serial.println( "end" );
    break;

  case 118 : // v to set vertical offset (the command must be followed by an int)
    inbyte = Serial.read();
    vertical_offset = Serial.parseInt();
    Serial.println( "---- Modifying vertical offset, new value : " + String( vertical_offset ) );
    Serial.println( "end" );
    //displayInfo();
    break;

  case 104 : // h to set horizontal offset (the command must be followed by an int)
    inbyte = Serial.read();
    horizontal_offset = Serial.parseInt();
    Serial.println( "---- Modifying horizontal offset, new value : " + String( horizontal_offset ) );
    Serial.println( "end" );
    
    break;
  }

}


/* ---------------------------- FUNCTIONS' IMPLEMENTATIONS -----------------------------*/
void xmove( long steps )
{
  moveStepper( steps, MOTOR_X_DIRECTION, MOTOR_X_STEP );
}

void ymove( long steps )
{
   moveStepper( steps, MOTOR_Y_DIRECTION, MOTOR_Y_STEP );
}


void moveStepper( long steps, int motor_direction_pin, int motor_step_pin )
{
  if ( steps < 0 )
  {
    digitalWrite( motor_direction_pin, HIGH );
  }
  else
  {
    digitalWrite( motor_direction_pin, LOW );
  }
  
  for( long i = 0 ; i < abs( steps ) ; i++ ) 
  {
    digitalWrite( motor_step_pin, HIGH );
    delay( 5 );
    digitalWrite( motor_step_pin, LOW );
    delay( 5 );
  } 
}

void displayCoordinates()
{
  Serial.print( posx * degree_per_step );
  Serial.write( 9 );
  Serial.print( posy * degree_per_step );
  Serial.write( 9 );
  Serial.print(distance);
  Serial.write(9);
  Serial.print( x );
  Serial.write( 9 );
  Serial.print( y );
  Serial.write( 9 );
  Serial.println( z );
}

void displayInfo()
{
  Serial.println( "vertical_nb_of_steps " + String( vertical_nb_of_steps ) );
  Serial.println( "horizontal_nb_of_steps " + String( horizontal_nb_of_steps ) );
  Serial.println( "ticks_per_revolution " + String( TICKS_PER_REVOLUTION ) );
  Serial.println( "subdivision " + String( SUBDIVISION ) );
  Serial.println( "total_nb_of_steps " + String( total_nb_of_steps ) );
  Serial.println( "degree_per_step " + String( degree_per_step ) ); 
  Serial.println( "Vertical offset (in steps) : " + String( vertical_offset ) );
  Serial.println( "Horizontal offset (in steps) : " + String( horizontal_offset ) );
  Serial.println( "Vertical precision (in degree) : " + String( degree_per_step * vertical_offset ) );
  Serial.println( "Horizontal precision (in degree) : " + String( degree_per_step * horizontal_offset ) );
}

void computeXYZ( float *x, float *y, float *z, int16_t distance, int hAngleDegree, int vAngleDegree ) 
{

//  float yawf = hAngleDegree * M_PI / 180;
//  float pitchf = vAngleDegree * M_PI / 180;

  float pitchf = hAngleDegree * M_PI / 180;
  float yawf = vAngleDegree * M_PI / 180;


//  float yawf   = hAngleDegree;
//  float pitchf = vAngleDegree;

  //Serial.println(yawf);
  //Serial.println(pitchf);

  // *x = -sin( yawf ) * distance * cos( pitchf );
  // *y = cos( yawf ) * distance * cos( pitchf );
  // *z = distance * sin( pitchf );

  *x = distance * -sin( yawf ) * cos( pitchf );
  *y = distance * sin( yawf ) * sin( pitchf );
  *z = distance * cos( pitchf );

  //Serial.println( " Computing xyz from angles and distances : " 
  //                    + String( *x ) + " " 
  //                    + String( *y ) + " "
  //                    + String( *z ) 
  //                    + " -- point indexes "
  //                    + hAngleDegree + " " 
  //                    + vAngleDegree 
  //              );

}