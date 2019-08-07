
/*
 *    YAAP  - yet another auto pilot
 * 
 *    Using an Arduino UNO and a MPU-6050 board,
 *    simple algorithms and some borrowed MPU-6050 code.
 *    
 *    Current target airframe is a 3 channel slow stick - using rudder to control roll.
 *    Goal is to control rudder, elevator and a servo driven 1 axis camera gimbal ( yaw ).
 *    
 *    Increasing servo pulse width coded to pitch down and roll right. Set TX to work as such.
 *    Servo's should be reversed here and not in the transmitter.
 */

#define DBUG 0    // for serial prints

#define UPRIGHT 0  // mounting orientation, Y axis is always towards the front
#define INVERTED_ 1
#define LEFT_SIDE 2
#define RIGHT_SIDE 3
const char mounting = UPRIGHT;     // pick one of the above

#define WING_ANGLE -2.4            // mounting adjustment and desired trim pitch (-4.4 last flight )
#define ROLL_ANGLE -0.85           // mounting adjustment ( 5.6 when arduino was upright )

#define ELE_REVERSE 0
#define AIL_REVERSE 0
#define GIMBAL_REVERSE 1           // negative numbers for clockwise rotation of the parallax 360 feedback

#include <Wire.h>
#include <Servo.h>
#include <avr/interrupt.h>

Servo aileron;    // or rudder for 3 channel airplane
Servo elevator;
Servo gimbal;

int user_a, user_e, user_g;   // user radio control inputs
int user_pos;                 // feedback servo position

struct PWM {
   uint8_t data;
   unsigned long timer;
};

struct PWM pwm_values[16];    // buffer the received servo signals
uint8_t pwm_in, pwm_out;

// IMU variables
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z;      // accelerometer coordinates
float  fax, fay, faz;             // gyro based accel coordinates.  Less noise.
int16_t temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
unsigned long IMU_timer;

float pitch,roll;
int8_t g_flag;

/********************************************************************************/

void setup(){

  aileron.attach(9);
  elevator.attach(10);
  gimbal.attach(8);
  aileron.writeMicroseconds(1500);
  elevator.writeMicroseconds(1500);
  gimbal.writeMicroseconds(1500);

  Wire.setClock(400000);
  Wire.begin();                                          //Start I2C as master
  if( DBUG ){
     Serial.begin(57600);                                //Use only for debugging
     Serial.println(F("MPU-6050 YAAP V1.0"));
  }
  pinMode(13, OUTPUT);                                   //Set output 13 (LED) as output
  
  delay(2000);                        // unit jiggle delay after battery is connected
  setup_mpu_6050_registers();         //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start
  calibrate_gyro();
  
  fax = acc_x;                        // initial position from the accelerometers
  fay = acc_y;
  faz = acc_z;
 
  IMU_timer = micros();                                   //Reset the loop timer


  // user radio inputs.  Not sure the best way to do pin change interrupts so try this way.
  // Some info about attach_interrupt on internet seems outdated.  This is UNO specific code.  
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);

  noInterrupts();
  PCICR |= 2;
  PCMSK1 |= 15;      // was 7 for just the servo inputs, now added the feedback servo wire
  interrupts();

     //cal360();
     //while(1);
  
}

ISR(PCINT1_vect){    // read pwm servo commands from the radio control receiver wired to A0,A1,A2
                     // save the values for later processing
  pwm_values[pwm_in].data = PINC;
  pwm_values[pwm_in].timer = micros();
  ++pwm_in;
  pwm_in &= 15;
}

void user_process(){    // decode the pwm servo data received, set the user variables to new values
                        // sanity check the data and ignore anything wrong
                        // 4th channel is a special case, feedback servo, and needs some different processing
                        // than the RC pwm signals.
static uint8_t last_bit[4];
static unsigned long last_time[4];
int8_t mask,b,i;
unsigned long dt;
int us;  

  // check each bit for a change and take the appropriate steps needed
  mask = 1;    // start with bit 0
  for(i = 0; i < 4; ++i ){
     b = pwm_values[pwm_out].data & mask;
     if(  b ^ last_bit[i] ){               // this bit changed
         last_bit[i] = b;                  // save the new state of the bit
         if( b ){                          // it went high, just save the timestamp
            last_time[i] = pwm_values[pwm_out].timer;
         }
         else{                             // it went low, so figure out how long it was high
            dt = pwm_values[pwm_out].timer - last_time[i];
            us = dt;                       // make a signed copy
            if( (i < 3 && dt > 800 && dt < 2200) || ( i == 3 && dt < 1200 ) ){   //sanity check
               us -= 1500;                 // sub out the zero point, get -500 to 500 in values
               us = median(i,us);          // discard glitches ( from interrupt latency I think )
               switch(i){
                  case 0:  user_a = us; break;    // ailerons
                  case 1:  user_e = us; break;    // elevator
                  case 2:  user_g = us; break;    // camera gimbal
                  case 3:  user_pos = us+1500; break;  // feedback servo position
               }  
            }  // end sanity check
         }   // end else
     }     // end bit change
     mask <<= 1;   // next bit
  }                // next i
  
  ++pwm_out;                   // done processing this entry
  pwm_out &= 15;
}

// return the median value of the last 3 to ignore outliers in the data stream
// store data for 4 channels
int median( int8_t ch, int val ){
static int vals[4][3];
static int8_t in[4];
int8_t j,i,k;                                  // low, median, high

   vals[ch][in[ch]] = val;                     // save the new value
   ++in[ch];
   if( in[ch] > 2 ) in[ch] = 0;

   j = 0, i = 1, k = 2;                             // pretend they are in the correct order
   if( vals[ch][j] > vals[ch][k] ) k = 0, j = 2;    // swap guess high and low
   if( vals[ch][i] < vals[ch][j] ) i = j;           // is lower than the low guess, pick that one instead
   if( vals[ch][i] > vals[ch][k] ) i = k;           // is higher than the high guess

   return vals[ch][i];
}



void loop(){

     IMU_process();
     while( pwm_in != pwm_out ) user_process();
     if( g_flag ){
        gimbal_process3( gyro_z );
        g_flag = 0;
     }
     servo_process();
}


void calibrate_gyro(){     /* try to find repeatable readings detecting when motionless */
unsigned char trys;
int prev_x, prev_y, prev_z;
int error;

   if( DBUG ) Serial.println(F("Calibrating "));
   
   prev_x = prev_y = prev_x = 0;
   
   for( trys = 0; trys < 20; ++trys ){

     gyro_x_cal = gyro_y_cal = gyro_z_cal = 0;
     
     for (int cal_int = 0; cal_int < 200 ; cal_int ++){ 
       read_mpu_6050_data();                                  //Read the raw acc and gyro data from the MPU-6050
       digitalWrite(13,LOW);                                  // LED will be on after calibration finishes
       gyro_x_cal += gyro_x;                            //Add the gyro x-axis offset to the gyro_x_cal variable
       gyro_y_cal += gyro_y;                            //Add the gyro y-axis offset to the gyro_y_cal variable
       gyro_z_cal += gyro_z;                            //Add the gyro z-axis offset to the gyro_z_cal variable
       delay(3);                                              //Delay 3 ms to simulate the 250Hz program loop
     }
     gyro_x_cal /= 200;                             // Get average values
     gyro_y_cal /= 200;
     gyro_z_cal /= 200;
      
     if( DBUG ){
        Serial.print(gyro_x_cal);  Serial.write(' ');
        Serial.print(gyro_y_cal);  Serial.write(' ');
        Serial.println(gyro_z_cal);
     } 

     error =   abs(gyro_x_cal - prev_x);
     error +=  abs(gyro_y_cal - prev_y);
     error +=  abs(gyro_z_cal - prev_z);
  
     if( error < 5 ) break;     // what is a good value for quiet time? It seems fairly stable.
     
     prev_x = gyro_x_cal;
     prev_y = gyro_y_cal;
     prev_z = gyro_z_cal;   
   } 
}

void change_orientation(){   // loosely tested code
int16_t temp;

    switch( mounting ){    // swap the raw values so the rest of the program thinks the MPU is mounted upright
       case INVERTED_:
          gyro_x = -gyro_x;  gyro_z = -gyro_z;
          acc_x  = -acc_x;   acc_z  = -acc_z;
       break;
       case RIGHT_SIDE:
          temp = gyro_x;  gyro_x = -gyro_z;  gyro_z = temp;
          temp = acc_x;   acc_x  = -acc_z;   acc_z  = temp;
       break;
       case LEFT_SIDE:
          temp = gyro_x;  gyro_x = gyro_z;   gyro_z = -temp;
          temp = acc_x;   acc_x  = acc_z;    acc_z  = -temp;
       break;      
    }
}


void servo_process(){    // write the servo's every 20 ms
static unsigned long timer;
static int yaw_I;    
int a,e;


   if( millis() - timer < 20 ) return;

   timer = millis();

   // combine the auto pilot and the user input using a simple algorithm
   // Set Proportional gain with the servo travel desired for +- 90 degrees tilt
   // 40% is -200,200  100% is -500,500
   // Can't use 100% as that may cancel completely any user inputs.
   a = map((roll-ROLL_ANGLE)*10,-900,900,250,-250);
   e = map((pitch-WING_ANGLE)*10,-900,900,-200,200);

   // accelerometers will see a banked turn as flying upright
   // integrate the yaw rotation to counter this effect
   // apply to aileron.  range 160 / 50hz --> 3 second time constant.  Watch flight for signs of oscillation.
   if( gyro_z < -20 ) --yaw_I;
   if( gyro_z > 20 ) ++yaw_I;
   yaw_I = constrain(yaw_I,-80,80);   //  adjust range for the time constant
   a += (yaw_I >> 2);                 //  and divide to get +- 20 max, a very small surface movement


   // if upside down, zero the elevator, avoid split s into the ground.
   if( faz < 0 ) e = 0;
   
   // add in the user input.
   a += user_a;  e += user_e;

   // reverse the servo's if needed, don't reverse channels in the transmitter.
   if( AIL_REVERSE ) a = -a;
   if( ELE_REVERSE ) e = -e;

   a += 1500;     // add in servo mid position
   e += 1500;

   a = constrain(a,1000,2000);
   e = constrain(e,1000,2000);
   aileron.writeMicroseconds(a);
   elevator.writeMicroseconds(e);
   
   if( DBUG ){
    // Serial.print("E ");  Serial.print(e); Serial.print("  A "); Serial.println(a);
   }
  
}

// we are maintaining a gravity based coordinate system. 
// It is updated/rotated using the gyro data and corrected with the accelerometers.
// The object is to reduce the noise from accelerometers, g-forces from movement, and vibration of the airframe.
void IMU_process(){
long v;
static uint8_t p;
float s,c,t;

  if(micros() - IMU_timer < 5000) return;     // run at 200 hz rate, call as often as wanted
  IMU_timer += 5000;                                           
  
  read_mpu_6050_data();                       // Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                       // Subtract the calibration values
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // rotate the gyro based gravity coordinates using gyro data and small angle approximation.
  //   65.5 is the gyro reading for 1 deg/second.  1 deg/57.3 is angular movement in radians
  //   200 is the sample rate
  // small angle approximations:  sin(a) = a,  cos(a) = 1 - a*a/2  ,angles in radians.
  // The largest errors possible are:
  //   fastest we can measure is 500 deg/second full scale.  That is 2.5 degrees per sample which
  //   is .04363 radians.  Sin is .043616.  Cos is .999048, approximation for cos is 0.999048
  // roll interchanges x and z.
  s = (float)gyro_y * 0.00000128;               // 1 / ( 200 * 65.5 * 57.3 ) = 0.00000133
  c = 1.0 - ( s * s ) / 2.0;                    // empirical value 00000128 works best for roll
  t = fax;
  fax = c * fax - faz * s;
  faz = c * faz + t * s;                                 
  // pitch interchanges y and z.
  s = (float)gyro_x * 0.00000132;
  c = 1.0 - ( s * s ) / 2.0;
  t = fay; 
  fay = c * fay + faz * s;
  faz = c * faz - t * s;
  // yaw interchanges x and y.
  s = (float)gyro_z * 0.00000133;
  c = 1.0 - ( s * s ) / 2.0;
  t = fax;
  fax = c * fax + fay * s;
  fay = c * fay - t * s;

   // set bounds on the values to prevent errors accumlating, should max about 1g on the 8g scale
  fax = constrain( fax,-4500,4500 );
  fay = constrain( fay,-4500,4500 );
  faz = constrain( faz,-4500,4500 );
  
  // leak the accelerometer values into the gyro based ones to counter gyro drift and
  // approximation errors
  fax = 0.995 * fax + 0.005 * acc_x;
  fay = 0.995 * fay + 0.005 * acc_y;
  faz = 0.995 * faz + 0.005 * acc_z;

  // calculate pitch and roll directly from the gyro based axis orientation just as one would with
  // accelerometer values
   v = (long)fax * (long)fax + (long)faz * (long)faz;
   v = sqrt( v );
   if( v != 0 ) pitch = atan( fay/(float)v ) * 57.296;    // get values +- 90
   if( faz != 0 ) roll = atan2( fax,faz ) * -57.296;      // get values +- 180

  // flag to run the gimbal processing
   g_flag = 1;
  
   if( DBUG  ){
      ++p;
      if( ( p & 31 ) == 0 ) {
        //Serial.print(pitch);   Serial.write(' ');  Serial.println(roll);
        //Serial.print((int)fax); Serial.write(' '); Serial.print((int)fay);
        //Serial.write(' '); Serial.println((int)faz);
        }
   }
}

 // yaw servo camera gimbal,  parallax 360 feedback servo. Rotational rate algorithm 2.
void gimbal_process3( int16_t val ){
float rpm;
int us;
int i;
static int8_t p;
static float fpos = 500.0;
static uint8_t mod;

  ++mod;
  mod &= 3;
  if( mod == 0 ){
     val = highpass(val);
     
     // 393 is the gyro reading for 6 deg/sec or 1 rpm
     // calculate what rate we think the servo should turn

     rpm = (float)val / 393.0;                      // match the yaw rotational rate for the gimbal function
     rpm += float(user_g)  * ( 15.0/500.0);         // map user input to max 15 rpm

     // add restoring force toward the zero value
     rpm += (float)( 575.0 - user_pos ) * 0.02;

     // add a correction due to the difference in the position and the calculated future position
     rpm -= 0.05 * ( (float)user_pos - fpos );
     
     // calc the servo command wanted, remove the servo deadband
     us = 0.8 * rpm;                    // 0.575 for high speed rotations
     if( rpm >= 0.5 )      us += 35;    // actual servo deadband found by experiment( 35 )
     else if( rpm < -0.5 ) us -= 18; 
     else                  us = 0;
                                                            
     // avoid 360 degree rotations in case we have a video out cable attached
     //if( pos < 70 ) us = constrain(us, 0, 400 );
     //if( pos > 605 ) us = constrain(us, -400, 0 );

     // calc where we think the position should be for next time, future position.
     // 200 hz rate: 1038 units for a rotation, 12000 samples in a minute --> 0.0865 factor
     //  50 hz rate: 1038 units, 3000 samples per minute --> 0.346 factor
     if( us != 0 ){
        fpos +=  0.346 * rpm ;
     }
     fpos = 0.99*fpos + 0.01*(float)user_pos; // converge the future and actual servo position values
     fpos = constrain(fpos,28,1044);
          
     if( GIMBAL_REVERSE ) us = -us;      // will need reverse as servo moves counter clockwise with increasing PWM period
     us = constrain(us , -220, 220 );    // keep rate in servo active area

     gimbal.writeMicroseconds( 1500 + us ); 

     if( ++p > 32 ){
        p = 0;
        if( DBUG ){
          Serial.print( rpm ); Serial.write(' ');
          Serial.print( us ); Serial.write(' ');
          Serial.print( fpos ); Serial.write(' ');
          Serial.println( user_pos );
        }              
     }
  }
}



int16_t highpass( int16_t val ){
float w0;
int16_t val_out;
//const float a0 = 0.98238544;        // butterworth 0.20hz highpass constants, 50 hz sample rate
//const float a1 = 1.9644605802;
//const float a2 = -0.9650811739;
const float a0 = 0.994461842;        // butterworth 0.25hz highpass constants, 200 hz sample rate
const float a1 = 1.9888929059;
const float a2 = -0.9889542499;
const float b1 = -2.0;
const float b2 = 1.0;
static float w1,w2;
static uint8_t p;
    
    // high pass filter the data so normal turn rates are filtered out leaving just unwanted flight
    // oscillations such as dutch roll to be countered by the camera gimbal
     w0 = a0 * (float)val + a1 * w1 + a2 * w2;
     val_out = w0 + b1 * w1 + b2 * w2;
     w2 = w1;                 // move delay terms
     w1 = w0;

     if( ++p > 32 ){
        p = 0;
        if( DBUG ){
          // Serial.print( val );   Serial.write(' ');
          // Serial.println(val_out);
        }
     }
     return val_out;
}


void cal360(){     // print out some numbers to calibrate the parallax 360 servo

int us;
int rpm;
unsigned long timer;
int pos, oldpos;
int diff;
int mx,mn;

   //for( us = 1400; us <= 1600; ++us ){    // up
   for( us = 1530; us >= 1450; --us ){      // down
      rpm = 0;   mx = 0; mn = 1050;
      timer = millis();
      pos = oldpos = user_pos;
      gimbal.writeMicroseconds(us);
      while( millis() - timer < 60000 ){
        pos = user_pos;
        diff = oldpos - pos;
        diff = abs( diff );
        if( pos > mx ) mx = pos;
        if( pos < mn ) mn = pos;
        oldpos = pos;
        if( diff > 500 ) {
            ++rpm;
            //delay(1);
        }
        delay(1);
        if( pwm_in != pwm_out ) user_process();
      }
      Serial.print(mn); Serial.write(' ');
      Serial.print(mx); Serial.write(' ');
      Serial.print("Servo "); Serial.print(us);
      Serial.print("  RPM "); Serial.println(rpm);
   }
   gimbal.writeMicroseconds(1500);
}

/**************
void ACCEL_process(){    // formulas allow pitch -90 to 90 and roll -180 to 180
long v;                  // from :https://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/

   v  = (long)acc_x * (long)acc_x;
   v += (long)acc_z * (long)acc_z;
   v = sqrt( v );
   if( v != 0 ) angle_pitch_acc = atan( (float)acc_y/(float)v ) * 57.296;
   //v = abs(acc_z); // simplistic model where roll stays in quadrants 1 and 2
                     // the simple model cannot determine if model is upside down.
                     // use v in place of acc_z both places next line.
   if( acc_z != 0 ) angle_roll_acc  = atan2( (float)acc_x,(float)acc_z ) * -57.296;
     
}
**********************/


/*   
   Original MPU code from the below source, and modified where needed.
   
Website: http://www.brokking.net/imu.html
Youtube: https://youtu.be/4BoIE8YQwM8

*/


void read_mpu_6050_data(){                  //Subroutine for reading the raw gyro and accelerometer data

  digitalWrite(13,LOW);                                          // if LED out will show it hangs here
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                     //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                     //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                     //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();               //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the gyro_z variable
  digitalWrite(13,HIGH);                                  // no I2C hangup
  
  if( mounting != UPRIGHT ) change_orientation();
}


void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                          // set device active
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(); 
   // try the built in low pass filters, 100hz bandwidth
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x02);                 
  Wire.endTransmission();
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}


/*  ******************
    

// yaw servo camera gimbal,  parallax 360 feedback servo. Rotational rate algorithm.
void gimbal_process2( int16_t val ){
float rpm;
float r;
float bank;
int us;
static uint8_t mod;
int pos;

static int8_t p;

  // only need to do this at the 50 hz rate
  ++mod;
  mod &= 3;
  if( mod == 0 ){

     // 393 is the gyro reading for 6 deg/sec or 1 rpm
     // calculate what rate we think the servo should turn

     rpm = (float)val / 393.0;                      // match the yaw rotational rate for the gimbal function
     rpm += float(user_g)  * ( 15.0/500.0);         // map user input to max 15 rpm

     // add a small restoring rpm toward zero value ( 338 was aligned straight ahead )
     // the feedback signal is wired via a 1k resistor to pin A3 with 3.3uf to ground
     pos = analogRead(A3);
     rpm += (float)( 338 - pos ) * .015;

     // add a look where you are going based upon the aileron control and current servo rotation
     if( user_a > 20 || user_a < -20 ){              // deadband for trim
        bank = (float)user_a/2.0 + 338;              // where we think we should look
        r = float(user_a) * ( 15.0/500.0 );          // get a rpm from the stick input as variable r
        r = abs(r);                                  // always rotate toward the calculated bank even if backwards to stick
        bank = bank - (float)pos;                    // get the error and sign of the rotation
        r *= bank/100.0;                             // shrink r to zero if we are looking there
        rpm += r;
     }
     
     // calc the servo command wanted, remove the servo deadband     32 -19 for 7.5 volts. 35 -22 for 5 v
     if( rpm >= 1.0 )        us = map( rpm, 0,140, 35,220 );    // replace +-20 with actual deadband found by experiment
     else if( rpm < -1.0 )   us = map( rpm, 0,-140, -22,-220);  // need to calibrate map for voltage applied to the servo
     else                    us = 0;                            // turns too fast at 7.5 volts
                                                            
     
     // avoid 360 degree rotations if we have a video out cable attached
     if( pos < 70 ) us = constrain(us, 0, 400 );
     if( pos > 605 ) us = constrain(us, -400, 0 );
     
     if( GIMBAL_REVERSE ) us = -us;      // will need reverse as servo moves counter clockwise with increasing PWM period
     us = constrain(us , -220, 220 );    // keep rate in servo active area

     gimbal.writeMicroseconds( 1500 + us ); 

     if( ++p > 16 ){
        p = 0;
        //if( DBUG )Serial.print(pos); Serial.write(' '); Serial.print(rpm); Serial.write(' ');\
              Serial.print(-us); Serial.write(' '); Serial.println(roll); 
     }                                         
  }
}


// yaw gimbal 360 servo, with the standard servo algorithm. A combination of gimbal_process and gimbal_process2
void gimbal_process1( int16_t val ){
static float yaw;
static int last;
static int ave_us;
float roll_out;
float angle, angle_at;
int pos, us;

    yaw += val * (0.00000133 * 57.3);         // integrate yaw giving angle in degrees
    roll_out = (roll-ROLL_ANGLE)*0.75;        // leak toward the roll angle. Look where you are going.
    yaw -= 0.006 * (yaw-roll_out);            // leak the integral toward roll.

    // merge the user input
    angle = map( user_g,-500,500,-120,120 );
    angle += yaw;
    angle = constrain(angle,-120,120);
               
    // find the error angle
    pos = analogRead(A3);
    angle_at = (pos-338.0) * ( 180.0/330.0 );     // only about 0.5 degrees resolution, not so good.
    angle = angle - angle_at;

    // PD control
    us = 1.5 * angle;                    // 1.5
    us -= 0.3 * (float)(pos - last);
    last = pos;
    // add a desired deadband and remove servo deadband
    if( us > 0 ) us += 36;
    else if( us < 0 ) us -= 23;
    else us = 0;

    // oscillates at higher P gain, is the feedback out of phase?
    //if( DBUG ) Serial.print(us); Serial.write( ' ' ); Serial.println(pos);
        
    if( GIMBAL_REVERSE ) us = -us;      // will need reverse as servo moves counter clockwise with increasing PWM period
    us = constrain(us , -80, 80 );      // keep rate in a slower servo active area
    
    // rate is 4x 50hz so filter at 3/4 rate.  Or 1/2 rate for less damping.
    ave_us =  ave_us + us;
    ave_us >>= 1;  
    gimbal.writeMicroseconds( 1500 + ave_us ); 

}




// yaw servo gimbal, standard servo
void gimbal_process( int16_t val ){
//float w0;
float val_out;
//const float a0 = 0.894859;        // butterworth 5hz highpass constants, 200 hz sample rate
//const float a1 = 1.778632;
//const float a2 = -0.800803;
//const float b1 = -2.0;
//const float b2 = 1.0;
//static float w1,w2;
//static uint8_t mod;
static float yaw;
float roll_out;
int angle_out;
//static int current_angle;
//static int active;
//static int last_user_g;

 //   ++mod;            // 50 hz rate for writing the servo's
 //   mod &= 3;
    
    // high pass filter the data so normal turn rates are filtered out leaving just unwanted flight
    // oscillations such as dutch roll to be countered by the camera gimbal
    // w0 = a0 * (float)val + a1 * w1 + a2 * w2;
    // val_out = w0 + b1 * w1 + b2 * w2;
    // w2 = w1; w1 = w0;                      // move delay terms 
    // the filter is too agressive or implemented incorrectly.  The integral leak works almost as desired.   

    val_out = val;                            // filter bypass
    yaw += val_out * (0.00000133 * 57.3);     // integrate yaw giving angle in degrees
    roll_out = (roll-ROLL_ANGLE)*0.75;        // leak toward the roll angle. Look where you are going.
    yaw -= 0.006 * (yaw-roll_out);            // leak the integral toward roll.

    // merge the user input

       angle_out = map( user_g,-500,500,-60,60 );
       angle_out += yaw;     
       if( GIMBAL_REVERSE ) angle_out = -angle_out;
       angle_out = constrain(angle_out,-60,60);
       // !!! maybe should process before adding in the 90 offset value
       // PID control or match the rate of change
       // one_shot = angle_out + 90;   change to a +- 500us value
       angle_out = map( angle_out,-90,90,-500,500 );
       gimbal.writeMicroseconds(1500+angle_out);

}


// yaw servo camera gimbal,  parallax 360 feedback servo. Rotational rate algorithm 2.
void gimbal_process3( int16_t val ){
float rpm;
int us;
int pos;
int i;
//static int vals[4];
static int poss[4];
static uint8_t mod;
static int8_t p;
//static float fpos = 338.0;
//static float old_rpm;
float zf;
//static int last_us;
static int8_t last_cnt;

  //vals[mod] = highpass(val);
  poss[mod] = analogRead(A3);
  ++mod;
  mod &= 3;
  if( mod == 0 ){                   // only need to do this at the 50 hz rate
     pos = 2;                       // get average values
     for( i = 0; i < 4; ++i ){
        pos += poss[i];
        //val += vals[i];
     }
     pos >>= 2;
  }
      //val = highpass(val);              // if use the 50hz rate constants
     
     // 393 is the gyro reading for 6 deg/sec or 1 rpm
     // calculate what rate we think the servo should turn

     rpm = (float)val / 393.0;                      // match the yaw rotational rate for the gimbal function
     rpm += float(user_g)  * ( 15.0/500.0);         // map user input to max 15 rpm

     // add a restoring rpm toward zero value ( 338 was aligned straight ahead, now 358 )
     // the feedback signal is wired via a 1k resistor to pin A3 with 3.3uf to ground
     
     i = abs( 358 - pos );                          // deadband this  about 10 degrees
     if( i > 20 ){
        rpm += (float)( 358 - pos ) * 0.05;          // 0.02
     }
     
     zf = (float)(358 - pos ) * 0.05;
     //zf = constrain(zf,-2.0,2.0);
     rpm += zf;

     // add a correction due to the difference in the position and the calculated future position
    // rpm -= 0.03 * ( (float)pos - fpos );  // small factor due to overshoot, filter cap on A3 delays actual pos value
     
     // calc the servo command wanted, remove the servo deadband
     us =  rpm;             // 0.575 *
     if( rpm >= 0.5 ) us += 27;    // actual servo deadband found by experiment (24)
     else if( rpm < -0.5 ) us -= 15; 

     // overcome stiction, modulate 20 rpm value for rpm less than 20
     if( --last_cnt == 0 ) last_cnt = 20;   
     
     if( rpm > 0.5 && rpm < 20.0 && rpm < last_cnt ){
        us += 14 * ( 20 - rpm ) / 20;    // 11
     }
     else if( rpm < -0.5 && rpm > -20.0 && -rpm < last_cnt ){
        us -= 14 * (20 + rpm) / 20;
     }
                                                            
     // avoid 360 degree rotations in case we have a video out cable attached
     if( pos < 70 ) us = constrain(us, 0, 400 );
     if( pos > 605 ) us = constrain(us, -400, 0 );

     // calc where we think the position should be for next time, future position.
     // 660 analog read units for a rotation, 3000 samples in a minute --> 0.22 factor
     
     if( us != 0 ){
        fpos +=  0.11 * rpm ;     // add in half of this one, reduce overshoot
        fpos +=  0.11 * old_rpm;  // add in half of the last one to account for delay due to averaging analog reads
     }
     old_rpm = rpm;
     fpos = 0.99*fpos + 0.01*(float)pos; // converge the future and actual servo position values
     fpos = constrain(fpos,6,660);
     
          
     if( GIMBAL_REVERSE ) us = -us;      // will need reverse as servo moves counter clockwise with increasing PWM period
     us = constrain(us , -220, 220 );    // keep rate in servo active area

     gimbal.writeMicroseconds( 1500 + us ); 

     if( ++p > 16 ){
        p = 0;
        if( DBUG ){
          // Serial.println( fpos - (float)pos );
        }              
     }

}

********************** */
 
