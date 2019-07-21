
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
const char mounting = INVERTED_;     // pick one

#define WING_ANGLE -2.4            // mounting adjustment and desired trim pitch (-4.4 last flight )
#define ROLL_ANGLE  5.6            // mounting adjustment

#define ELE_REVERSE 0
#define AIL_REVERSE 0
#define GIMBAL_REVERSE 0

#include <Wire.h>
#include <Servo.h>
#include <avr/interrupt.h>

Servo aileron;    // or rudder for 3 channel airplane
Servo elevator;
Servo gimbal;

int user_a, user_e, user_g;   // user radio control inputs

struct PWM {
   uint8_t data;
   unsigned long timer;
};

struct PWM pwm_values[8];    // buffer the received servo signals
uint8_t pwm_in, pwm_out;

// IMU variables
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z;      // accelerometer coordinates
float  fax, fay, faz;             // gyro based accel coordinates.  Less noise.
int16_t temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
unsigned long IMU_timer;

float pitch,roll;

/********************************************************************************/

void setup(){
  
  Wire.begin();                                          //Start I2C as master
  if( DBUG ){
     Serial.begin(57600);                                //Use only for debugging
     Serial.println(F("MPU-6050 YAAP V1.0"));
  }
  pinMode(13, OUTPUT);                                   //Set output 13 (LED) as output
  
  delay(2000);                        // unit jiggle delay after battery is connected
  setup_mpu_6050_registers();         //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro
  calibrate_gyro();
  
  fax = acc_x;                        // initial position from the accelerometers
  fay = acc_y;
  faz = acc_z;
 
  IMU_timer = micros();                                   //Reset the loop timer

  aileron.attach(9);
  elevator.attach(10);
  gimbal.attach(8);
  aileron.writeMicroseconds(1500);
  elevator.writeMicroseconds(1500);
  gimbal.writeMicroseconds(1500);

  // user radio inputs.  Not sure the best way to do pin change interrupts so try this way.
  // Some info about attach_interrupt on internet seems outdated.  This is UNO specific code.  
  pinMode(A0,INPUT_PULLUP);
  pinMode(A1,INPUT_PULLUP);
  pinMode(A2,INPUT_PULLUP);

  noInterrupts();
  PCICR |= 2;
  PCMSK1 |= 7;
  interrupts();
  
}

ISR(PCINT1_vect){    // read pwm servo commands from the radio control receiver wired to A0,A1,A2
                     // save the values for later processing
  pwm_values[pwm_in].data = PINC;
  pwm_values[pwm_in].timer = micros();
  ++pwm_in;
  pwm_in &= 7;
}

void user_process(){    // decode the pwm servo data received, set the user variables to new values
                        // sanity check the data and ignore anything wrong
                        // use trend to filter out spurious signals probably caused by variations in
                        // interrupt latency ( algorithm seems to be working )
static uint8_t last_bit[3];
static unsigned long last_time[3];
static int trend_val[3];
static int8_t trend_cnt[3];
int8_t mask,b,i;
unsigned long dt;
int us;  

  // check each bit for a change and take the appropriate steps needed
  mask = 1;    // start with bit 0
  for(i = 0; i < 3; ++i ){
     b = pwm_values[pwm_out].data & mask;
     if(  b ^ last_bit[i] ){               // this bit changed
         last_bit[i] = b;                  // save the new state of the bit
         if( b ){                          // it went high, just save the timestamp
            last_time[i] = pwm_values[pwm_out].timer;
         }
         else{                             // it went low, so figure out how long it was high
            dt = pwm_values[pwm_out].timer - last_time[i];
            us = dt;                       // make a signed copy
            if( dt > 800 && dt < 2200 ){   // test the unsigned long version as a sanity check
               us -= 1500;                                 // sub out the zero point, get -500 to 500 in values
               if( us > trend_val[i]+1 ) ++trend_cnt[i];   // look for a trend, 1us deadband.
               if( us < trend_val[i]-1 ) --trend_cnt[i];
               if( trend_cnt[i] > 2 || trend_cnt[i] < -2 ){    // looks like real user input
                  trend_cnt[i] = 0;
                  trend_val[i] = us;   
                  switch(i){
                     case 0:  user_a = us; break;    // ailerons
                     case 1:  user_e = us; break;    // elevator
                     case 2:  user_g = us; break;    // camera gimbal
                  }
               }  
            }  // end sanity check
         }   // end else
     }     // end bit change
     mask <<= 1;   // next bit
  }                // next i
  
  ++pwm_out;                   // done processing this entry
  pwm_out &= 7;
}

void loop(){

     IMU_process();
     servo_process();
     if( pwm_in != pwm_out ) user_process();

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
int a,e;


   if( millis() - timer < 20 ) return;

   timer = millis();

   // combine the auto pilot and the user input using a simple algorithm
   // Set Proportional gain with the servo travel desired for +- 90 degrees tilt
   // 40% is -200,200  100% is -500,500
   // Can't use 100% as that may cancel completely any user inputs.
   a = map((roll-ROLL_ANGLE)*10,-900,900,250,-250);
   e = map((pitch-WING_ANGLE)*10,-900,900,-200,200);

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

  // run the gimbal processing
  gimbal_process( gyro_z );
  
   if( DBUG  ){
      ++p;
      if( ( p & 31 ) == 0 ) {
        Serial.print(pitch);   Serial.write(' ');  Serial.println(roll);
        //Serial.print((int)fax); Serial.write(' '); Serial.print((int)fay);
        //Serial.write(' '); Serial.println((int)faz);
        }
   }
}


// yaw servo gimbal,  parallax 360 feedback servo
void gimbal_process2( int16_t val ){
int rpm;
int us;
static uint8_t mod;
int zero,pos;
float r;

  // only need to do this at the 50 hz rate
  ++mod;
  mod &= 3;
  if( mod == 0 ){

     // 393 is the gyro reading for 6 deg/sec or 1 rpm
     // match the rotational rate
     rpm = val / 393;                                  // !!! reverse here if just the correction is wrong way
     us  = map( rpm,-150,150,1280-1500,1720-1500 );    // values from servo spec, removing the zero amount of 1500
     // can reduce the deadband by adding or subbing a small value
     if( us > 5 ) us += 15;
     if( us < -5 ) us -= 15;
     
     // vary the zero value depending upon the roll angle
     r = roll - ROLL_ANGLE;                            
     zero = 338 + r * 2.0;                             // gain factor
     // find the error between zero and the feedback signal and apply correction
     pos = analogRead(A3);
     zero -= pos;                                      // get error
     zero = constrain(zero,-60,60);                    // limit slew rate for this ( +-20 will be deadband )
     us += zero;

     
     us  += user_g/4;                                  // add in the user control ( rate )
                                                       // this will be a non-return to zero rotation
                                                           
     // avoid 360 degree rotations
     //if( pos < 60 ) us = constrain(us, 0, 400 );   // !!! testing without this, then check if sign is correct
     //if( pos > 615 ) us = constrain(us, -400, 0 );
     
     if( GIMBAL_REVERSE ) us = -us;
     us = constrain(us , -220, 220 );       // servo control limits from spec, this is a rate command 
                                                  // not a position command like a regular servo

     gimbal.writeMicroseconds( 1500 + us );                                           
  }
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
