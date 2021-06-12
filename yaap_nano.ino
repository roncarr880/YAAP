/*
 *    YAAP  - yet another auto pilot
 * 
 *    Using Arduino Nano/UNO 328p processor boards, a MPU-6050 board and simple algorithms.
 *    
 *    Increasing servo pulse width coded to pitch down and roll right. Set TX to work as such.
 *    Servo's should be reversed here and not in the transmitter.
 *    
 *    This version has non-blocking I2C routines.
 *    Added F.Port decoding as an option for input signals.
 *    
 *    For an airplane setup: set the correct mounting, adjust the roll and pitch angles and
 *    print out the accelerometer zero values, find average error and put them in a #ifdef #endif block.  
 *    ( see read_mpu.. )
 */


//#define PROTO_BOARD_NANO
//#define ULTRALIGHT_UNO   // pick which airplane we are compiling for and select correct board in Tools
//#define SCOUT_GYRO_NANO 
#define AQUASTAR_NANO
//#define WALLY_WING_NANO

#define UPRIGHT 0        // mounting orientation, Y axis is always towards the front, make changes below
#define INVERTED_ 1
#define LEFT_SIDE 2
#define RIGHT_SIDE 3

// airplane specific defines

#ifdef PROTO_BOARD_NANO            // testing on a proto board
 #define DBUG 1                    // controls serial prints, normally should be 0
 #define FPORT
 
 const char mounting = UPRIGHT;    // pick one of the above orientations

 #define WING_ANGLE 0.0           // mounting adjustment and desired trim pitch, reading at desired pitch
 #define ROLL_ANGLE 0.0           // mounting adjustment, reading when wings level

 #define ELE_REVERSE 0
 #define AIL_REVERSE 0
 #define RUD_REVERSE 0
 #define GIMBAL_REVERSE 1           // negative numbers for clockwise rotation of the parallax 360 feedback

 int again[3] = {300,300,300};      // autolevel gain for each mode, 0 for manual mode
 int egain[3] = {300,300,300};
 int ygain[3] = {0,0,0};            // turning detection counter control, applied to aileron channel  
 float adead[3] = {0,0,0};          // deadband if desired
 float edead[3] = {0,0,0};
 float eangle[3] = {0,0,0};         // autolaunch climb angle or other pitch adjustments per mode

#define HAS_RUDDER
// #define HAS_GIMBAL              // rudder channel controls the camera gimbal
#define HAS_INT                    // MPU int pin wired to pin 2

// empirical found values for this MPU
 #define ACC_X_0   180
 #define ACC_Y_0   -50
 #define ACC_Z_0   30
 
#endif

#ifdef WALLY_WING_NANO             // chuck glider wing on Simple Ultimate 
// plane debug
// want more downtrim and/or nose weight
 #define DBUG 0                   // controls serial prints, normally should be 0
 #define FPORT
 
 const char mounting = INVERTED_;    // pick one of the above orientations

// #define WING_ANGLE 1.0           // mounting adjustment(-.5) and desired trim pitch(1.5)
 #define WING_ANGLE 3.0            // 4/11/21  manual mode needs down trim (was 1.0)
 #define ROLL_ANGLE -1.0           // mounting adjustment, reading when wings level ( was 0.0 )

 #define ELE_REVERSE 1
 #define AIL_REVERSE 1
 #define RUD_REVERSE 0              // no rudder channel, 3 channel plane
 #define GIMBAL_REVERSE 1           // negative numbers for clockwise rotation of the parallax 360 feedback

 int again[3] = {800,400,0};        // autolevel gain for each mode, 0 for manual mode
 int egain[3] = {800,400,0};        // have launch, cruise, manual modes
 int ygain[3] = {0,1,0};            // turning detection counter control, applied to aileron channel !!! should be negative when inverted?
 float adead[3] = {0,0,0};          // deadband if desired
 float edead[3] = {0,0,0};
 float eangle[3] = {10,0,0};         // autolaunch climb angle or other pitch adjustments per mode

// #define HAS_RUDDER              // rudder only model, rudder on aileron channel
// #define HAS_GIMBAL              // rudder channel controls the camera gimbal
#define HAS_INT                    // MPU int pin wired to pin 2

// empirical found values for this MPU
 #define ACC_X_0   -250              
 #define ACC_Y_0   -50
 #define ACC_Z_0   155
 
#endif

#ifdef AQUASTAR_NANO
 #define DBUG 0                    // controls serial prints, normally should be 0
// #define FPORT
 
 const char mounting = UPRIGHT;    // pick one of the above orientations

 #define WING_ANGLE -8.0           // -8 mounting adjustment and desired trim pitch, reading at desired pitch
 #define ROLL_ANGLE -5.5           // mounting adjustment, reading when wings level

 #define ELE_REVERSE 0
 #define AIL_REVERSE 1
 #define RUD_REVERSE 0
 #define GIMBAL_REVERSE 1           // negative numbers for clockwise rotation of the parallax 360 feedback

 int again[3] = {300,300,200};        // autolevel gain for each mode, 0 for manual mode
 int egain[3] = {800,300,200};        // higher gain for launch
 int ygain[3] = {0,1,1};             // turning detection counter control, applied to aileron channel. 0,1 or -1.
 int hgain[3] = {0,0,5};             // servo us per degree off course. heading hold.
 float adead[3] = {0,5,5};            // deadband angle in degrees
 float edead[3] = {0,10,10};
 float eangle[3] = {18,0,0};        // autolaunch climb angle 

// #define HAS_RUDDER              // rudder only model, rudder is on the aileron channel
// #define HAS_GIMBAL              // rudder channel controls the camera gimbal

// empirical found values for this MPU
 #define ACC_X_0   -170
 #define ACC_Y_0   0
 #define ACC_Z_0   300    //500
 
#endif

#ifdef SCOUT_GYRO_NANO
 #define DBUG 0                     // controls serial prints, normally should be 0
 // #define FPORT

 const char mounting = INVERTED_;   // pick one of the above orientations

 #define WING_ANGLE  8.0            // mounting adjustment and desired trim pitch ( 8 trimmed too slow ? )
 #define ROLL_ANGLE  2.0            // mounting adjustment
 #define ELE_REVERSE 0
 #define AIL_REVERSE 1
 #define RUD_REVERSE 0
 #define GIMBAL_REVERSE 1           // negative numbers for clockwise rotation of the parallax 360 feedback

 int again[3] = {700,500,300};      // autolevel gain for each mode, 0 for manual mode
 int egain[3] = {600,400,200};
 int ygain[3] = {0,0,0};            // turning detection counter control, applied to aileron channel  
 float adead[3] = {0,5,10};          // deadband if desired
 float edead[3] = {0,5,10};
 float eangle[3] = {0,-1,-2};         // autolaunch climb angle or other pitch adjustments per mode
                                      // !!! faster trim speed test in combination of less self righting

 // pick one of these as they both use the rudder channel for control
 #define HAS_RUDDER
 //#define HAS_GIMBAL  
            
// empirical found values for this MPU, first Nano version

 #define ACC_X_0  0              // this mpu has a large variation in the z direction, defective?
 #define ACC_Y_0  -130
 #define ACC_Z_0  975 
#endif

#ifdef ULTRALIGHT_UNO
 #define DBUG 0                     // controls serial prints, normally should be 0
 //#define FPORT
 
 const char mounting = UPRIGHT;     // pick one of the above orientations

 #define WING_ANGLE -2.0            // mounting adjustment and desired trim pitch, reading at desired pitch
                                    // normally would be positive but mounted slanted down in this case
 #define ROLL_ANGLE -1.54           // mounting adjustment, reading when wings level

 #define ELE_REVERSE 0
 #define AIL_REVERSE 1
 #define RUD_REVERSE 0
 #define GIMBAL_REVERSE 1           // negative numbers for clockwise rotation of the parallax 360 feedback

 int again[3] = {250,250,250};      // autolevel gain for each mode, 0 for manual mode
 int egain[3] = {200,200,200};      // no modes, gimbal is using A3 for position
 int ygain[3] = {1,1,1};            // turning detection counter control, applied to aileron channel  
 float adead[3] = {0,0,0};          // deadband if desired
 float edead[3] = {0,0,0};
 float eangle[3] = {0,0,0};         // autolaunch climb angle or other pitch adjustments per mode

// #define HAS_RUDDER              // rudder is on channel 5 bypassing the UNO, gimbal on 4
 #define HAS_GIMBAL                // rudder channel controls the camera gimbal

// empirical found values for this MPU, UNO version
 #define ACC_X_0   -200
 #define ACC_Y_0    100
 #define ACC_Z_0   -200
 
#endif

#define ALPHA  0.001               // convergence of accelerometer and gyro values.  0.005 is 1 second time constant
#define I2BUFSIZE 16               // must be a power of 2 and at least 16 for this application
#define MPU6050  0x68              // I2C address


#include <Servo.h>

#ifndef FPORT
 #include <avr/interrupt.h>
#endif

Servo aileron;    // or rudder for 3 channel airplane
Servo elevator;
#ifdef HAS_GIMBAL 
  Servo gimbal;
#endif
#ifdef HAS_RUDDER 
  Servo rudder;
#endif
#ifdef FPORT
  Servo throttle;
#endif  

//  I2C buffers and indexes
unsigned int i2buf[I2BUFSIZE];   // writes
uint8_t i2rbuf[I2BUFSIZE];       // reads
uint8_t i2in,i2out;
uint8_t i2rin,i2rout;

// IMU variables
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z;      // accelerometer coordinates
float  fax, fay, faz;             // gyro based accel coordinates.
int16_t temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
unsigned long IMU_timer;
float pitch,roll;

uint8_t setup_done;
uint8_t g_flag;

//  Variables for reading pwm signals from the RC receiver or from FPORT
int user_a, user_e;           // user radio control inputs ( A0, A1 )
int user_g, user_r;           // gimbal control or rudder control ( A2 )
int user_pos, user_m;         // feedback servo position or mode channel ( A3 )
int user_t;                   // throttle input when using FPORT, else ESC wired direct to PWM receiver
int a_trim;                   // value of roll trim found during setup().  Used for gimbal routine.

#ifndef FPORT                 // the way this is written, can't have both FPORT and the gimbal feedback servo 
                              // on the same plane
struct PWM {                  // buffer pin changes and the time they happened
   uint8_t data;
   unsigned long timer;
};
struct PWM pwm_values[16];
uint8_t pwm_in, pwm_out;
#endif

unsigned int frames;          // fport debug message
/******************************************************************************************/

void setup() {

  #ifdef FPORT 
      Serial.begin(115200);
  #else
      if( DBUG ) Serial.begin(115200);
  #endif
  
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);   // LED lit will indicate an error during setup

  aileron.attach(9);
  elevator.attach(10);
  aileron.writeMicroseconds(1500);
  elevator.writeMicroseconds(1500);
  #ifdef HAS_GIMBAL 
     gimbal.attach(8);
     gimbal.writeMicroseconds(1500);
  #endif
  #ifdef HAS_RUDDER
     rudder.attach(11);
     rudder.writeMicroseconds(1500);
  #endif
  #ifdef FPORT
     throttle.attach(12);
     throttle.writeMicroseconds(1000);
     user_t = -500;
  #endif
  
  i2cinit();
  mpu6050_init();
  if( DBUG ) Serial.println(F("MPU-6050 successfully initialized"));
  calibrate_gyro();

  #ifndef FPORT
  // set up the pin change interrupts for reading PWM signals from the receiver
     pinMode(A0,INPUT_PULLUP);        // aileron
     pinMode(A1,INPUT_PULLUP);        // elevator
     pinMode(A2,INPUT_PULLUP);        // rudder or gimbal control
     pinMode(A3,INPUT_PULLUP);        // mode or gimbal position ( parallax 360 servo )

     noInterrupts();    // This is UNO specific code.
     PCICR |= 2;
     PCMSK1 |= 15;
     interrupts();
     
     get_trim();        // save current aileron radio trim position for gimbal algorithm
  #endif

  // set the starting position
  fax = acc_x - ACC_X_0;       // initial position from the accelerometer readings read during gyro calibration
  fay = acc_y - ACC_Y_0;       // the offsets were not applied so apply them here
  faz = acc_z - ACC_Z_0;

  IMU_timer = micros(); 
  setup_done = 1;
  digitalWrite(13,LOW);        // setup completed ok

  #ifdef HAS_INT
     pinMode(2,INPUT);        // polling the state of the MPU data ready on pin 2
  #endif
}


void mpu6050_init(){

  // set MPU active
  i2start( MPU6050 );
  i2send(0x6B);
  i2send(0x03);   // clock source Z gyro
  i2stop(); 
   // try the built in low pass filters, 100hz bandwidth
  i2start(MPU6050);
  #ifdef HAS_INT
    i2send(0x19);
    i2send(0x04);   // sample rate divider for 200 hz, use hardware pin for timing
  #endif
  #ifndef HAS_INT
   i2send(0x1A);  // send address of 1st register, use nano timing, mpu runs at 1 khz
                  // not sure if any difference here, which has the most accurate clock ? nano or mpu
  #endif  
  i2send(0x02);   // lowpass --> 0x1A
  i2send(0x08);   // 0x08 --> 0x1B 500 dps gyros
  i2send(0x10);   // 0x10 --> 0x1C +- 8g accelerometers
  i2stop();
  i2flush();      // clear the i2c write buffer
  
  #ifdef HAS_INT
     // try using the int pin to show when new data is ready. Read on nano pin 2.
    i2start(MPU6050);
    i2send(0x37);   // INT pin config register
    i2send(0x30);   // latch mode and any read clear
    i2send(0x01);   // int enable register, data ready bit
    i2stop();
    i2flush();      // clear the i2c write buffer  
  #endif
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
       i2queue_read( MPU6050, 0x3B, 14 );               // queue read gyro, accel and temp
       while( i2available() < 14 ) i2poll();            // wait for reads during init
       read_mpu_6050_data();                            // Read the raw acc and gyro data from the MPU-6050
       gyro_x_cal += gyro_x;                            // Sum the readings
       gyro_y_cal += gyro_y;
       gyro_z_cal += gyro_z;
       delay(5);                                        // Delay to simulate the 200Hz program loop
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
  
     if( error < 5 ) break;     // Readings seem fairly stable.
     
     prev_x = gyro_x_cal;
     prev_y = gyro_y_cal;
     prev_z = gyro_z_cal;   
   } 
}

#ifndef FPORT
// need to have the interrupts running in order to read the aileron trim position
// trim is used only for the gimbal routine
// don't hang if the TX radio is turned off
void get_trim(){
int i;

   for( i = 0; i < 100; ++i ){                      // 200 ms long
      delay(2);                                     // small delay because the
      while( pwm_in != pwm_out ) user_process();    // feedback servo generates alot of interrupts
      if( i == 50 ) a_trim = user_a;                // capture a value
      if( user_a < -50 || user_a > 50 ){            // someone is moving the sticks, abort
         a_trim = 0;
         break;
      }
      if( i > 50 ){                                 // test if the captured value is stable
         if( user_a < a_trim-5 || user_a > a_trim+5 ){
            a_trim = 0;
            break;
         }
      }
   }  
}
#endif

/********************************************************************/
void loop() {


   i2poll();                                        // keep the I2C bus going
   IMU_process();                                   // find plane orientation

   #ifdef FPORT
      while( Serial.available() ) fport_process();
   #else
      while( pwm_in != pwm_out ) user_process();       // process servo PWM signals from the receiver
   #endif
   
   #ifdef HAS_GIMBAL
   if( g_flag ){
        gimbal_process3( gyro_z );                  // parallax servo gimbal
        g_flag = 0;
   }
   #endif
   
   servo_process();                                 // update the output servo signals
}


#ifdef FPORT

#define FRAME  0x7E               // message is very similar to a SLIP frame
#define FESC   0x7D
uint16_t fchan[16];

void fport_process(){
int dat;
static int shift;
static int len;
static int count;
static uint16_t crc;

   dat = Serial.read();

   // Serial.print(dat,HEX); Serial.write(' ');
   // if( dat == FRAME ) Serial.println();

   switch(dat){
      case FRAME:  shift = len = count = crc = 0;  break;
      case FESC:   shift = 0x20;  break;
      default:
         dat += shift;
         shift = 0;
         crc += dat;
         if( crc > 255 ) crc -= 255;                                     // -256 + 1
         if( count == 0 ) len = dat;
         if( count == 1 && dat != 0 ) len = -2;                          // message type zero wanted else invalid
         if( count == len+1 && crc == 0xff ) fport_process3();           // message completed no errors
            // else if(count == len+1 ){ Serial.print("crc "); Serial.print(crc,HEX); Serial.write(' '); }   
         if( count > 1 && count <= len ) fport_process2( (uint32_t)dat,count-2 );  // process data
         ++count;
      break;
   }
}

void fport_process2( uint32_t data, int indx ){   // fport frame is an imbedded SBUS frame
int i,r;
                                                  // shift up by remainder, down by 11
   if( indx == 0 ){
      for( i = 0; i < 16; ++i ) fchan[i] = 0;                                               
   }
   i = (indx*8)/11;
   r = (indx*8)%11;
   if( i > 15 ) return;
   data <<= r;
   fchan[i] |= data;
   fchan[i] &= 0x7ff;
   data >>= 11;                 // if all the bits shift off the end, zero will OR with the next channel
   if( ++i > 15 ) return;       // and not cause an error.
   fchan[i] |= data;
   fchan[i] &= 0x7ff;
}

void fport_process3( ){     // channels AETRM order

     ++frames;
     
     // avoid junk the rx sends before it connects to the tx. sends all zero's
     if( fchan[0] < 150 ){
        user_a = user_e = user_r = user_m = 0;
        user_t = -500;          // disable throttle
        return;
     }
     
     user_a = map(fchan[0],172,1811,-500,500);
     user_e = map(fchan[1],172,1811,-500,500);
     user_t = map(fchan[2],172,1811,-500,500);
     user_r = map(fchan[3],172,1811,-500,500);
     user_m = map(fchan[4],172,1811,-500,500);  

}

#endif

void IMU_process(){
long v;
static uint8_t p;
static uint8_t state;
float s,c;
float dx,dy,dz;

  switch( state ){
    
  case 0:                                       // wait for timer
    #ifndef  HAS_INT
     if( micros() - IMU_timer >= 5000 ){        // 200 hz rate == 5000us
    #endif
    #ifdef   HAS_INT
     if( digitalRead(2) ){
    #endif    
        if(( p & 127 ) == 127 ){
           Serial.print( micros() - IMU_timer ); Serial.write(' ');
        }
        IMU_timer += 5000;
        #ifdef HAS_INT
           IMU_timer = micros();
        #endif   
        state = 1;
        i2queue_read( MPU6050, 0x3B, 14 );    // queue read gyro, accel and temp
     }                                              
  break;
  case 1:                                     // wait for I2C reads to finish
     if( i2available() >= 14 ) state = 2;     // continue when reads have finished
  break;
  case 2:                                     // process the data
     read_mpu_6050_data();
     state = 0;

  // rotate the gyro based gravity coordinates using gyro data and small angle approximation.
  //   65.5 is the gyro reading for 1 deg/second.  1 deg/57.3 is angular movement in radians
  //   200 is the sample rate
  // small angle approximations:  sin(a) = a,  cos(a) = 1 - a*a/2  ,angles in radians.
  // The largest errors possible are:
  //   fastest we can measure is 500 deg/second full scale.  That is 2.5 degrees per sample which
  //   is .04363 radians.  Sin is .043616.  Cos is .999048, approximation for cos is 0.999048
  // roll interchanges x and z.
     s = (float)gyro_y * 0.00000133;               // 1 / ( 200 * 65.5 * 57.3 ) = 0.00000133
     c = 1.0 - ( s * s ) / 2.0;                    // empirical value 00000128 works best for roll
     dx = c * fax - s * faz;                 // formula are simplified, have - 2 fax for difference
     dz = c * faz + s * fax;                 // and + 1 fax for summation == - 1 fax overall.                
  // pitch interchanges y and z.
     s = (float)gyro_x * 0.00000133;
     c = 1.0 - ( s * s ) / 2.0; 
     dy = c * fay + s * faz;
     dz = dz + c * faz - s * fay - faz;
  // yaw interchanges x and y.
     s = (float)gyro_z * 0.00000133;
     c = 1.0 - ( s * s ) / 2.0;
     dx = dx + c * fax + s * fay - fax;
     dy = dy + c * fay - s * fax - fay;

   // set bounds on the values to prevent errors accumlating, should max about 1g on the 8g scale
     fax = constrain( dx,-4500,4500 );
     fay = constrain( dy,-4500,4500 );
     faz = constrain( dz,-4500,4500 );
  
  // leak the accelerometer values into the gyro based ones to counter gyro drift and
  // approximation errors
     fax = (1.0 - ALPHA) * fax + ALPHA * acc_x;
     fay = (1.0 - ALPHA) * fay + ALPHA * acc_y;
     faz = (1.0 - ALPHA) * faz + ALPHA * acc_z;


  // calculate pitch and roll directly from the gyro based axis orientation just as one would with
  // accelerometer values
     v = (long)fax * (long)fax + (long)faz * (long)faz;
     v = sqrt( v );
     if( v != 0 ) pitch = atan( fay/(float)v ) * 57.296 - WING_ANGLE;    // get values +- 90
     if( faz != 0 ) roll = atan2( fax,faz ) * -57.296 - ROLL_ANGLE;      // get values +- 180

  // flag to run the gimbal processing
     g_flag = 1;
 
   //temp_in_C = (float)temperature/340.0 + 36.53;
  
     if( DBUG  ){
        ++p;
        if( ( p & 127 ) == 0 ) {        // pitch and roll are printed with the compensation values included 
                                       // so zero pitch will look like a small positive wing angle
          Serial.print("Frames "); Serial.print(frames); Serial.write(' ');
          Serial.write('A'); Serial.print( user_a ); Serial.write(' '); 
          Serial.write('E'); Serial.print( user_e ); Serial.write(' ');
          Serial.write('T'); Serial.print( user_t ); Serial.write(' ');
          Serial.write('P'); Serial.print(pitch);   Serial.write(' '); Serial.write('R');  Serial.println(roll);
        }
     }
  break;
  }
}



void change_orientation(){   // loosely tested code
int16_t temp;

    switch( mounting ){    // swap the raw values so the rest of the program thinks the MPU is mounted upright
       case INVERTED_:
          gyro_x = -gyro_x;  gyro_z = -gyro_z;
          acc_x  = -acc_x;   acc_z  = -acc_z;
       break;
       case LEFT_SIDE:
          temp = gyro_x;  gyro_x = -gyro_z;  gyro_z = temp;
          temp = acc_x;   acc_x  = -acc_z;   acc_z  = temp;
       break;
       case RIGHT_SIDE:
          temp = gyro_x;  gyro_x = gyro_z;   gyro_z = -temp;
          temp = acc_x;   acc_x  = acc_z;    acc_z  = -temp;
       break;      
    }
}

void read_mpu_6050_data(){        // on entry read data should have been queued and completed
                                  // all this does is read from the buffer and assign values
static unsigned int p;

  acc_x = i2read_int();
  acc_y = i2read_int();
  acc_z = i2read_int();
  temperature = i2read_int();
  gyro_x = i2read_int();
  gyro_y = i2read_int();
  gyro_z = i2read_int();
  
  if( mounting != UPRIGHT ) change_orientation();


  if( setup_done ){              // adjust the values once the calibration is complete
     gyro_x -= gyro_x_cal;
     gyro_y -= gyro_y_cal;
     gyro_z -= gyro_z_cal;
     acc_x  -= ACC_X_0;
     acc_y  -= ACC_Y_0;
     acc_z  -= ACC_Z_0;
  }

  // determine the accelerometer offsets by moving the mpu to a number of right angles to gravity
  // and noting the values read for +g and -g.  Find an average offset to read approx 4096 both pos and neg G.
  // Do this one time.  Note that this is after the orientation adjustment has been applied.
  if( DBUG ){
     ++p;
     if( ( p & 127 ) == 0){
        Serial.write('X'); Serial.print( acc_x );  Serial.write(' ');    // print out X,Y,Z accel values
        Serial.write('Y'); Serial.print( acc_y );  Serial.write(' ');
        Serial.write('Z'); Serial.print( acc_z );  Serial.write(' ');
     }
  }

}

// 3 modes when not using the parallax servo
void servo_process(){    // write the servo's every 20 ms
static unsigned long timer;    
int a,e,r,m,t,y;
static int yaw_I;
static unsigned int last_frames;
static long heading;


   if( millis() - timer < 20 ) return;

   timer = millis();

   m = 1;                                            // default mode 1, should be a self leveling mode 
   a = e = r = t = y = 0;

   #ifdef FPORT
      if( frames == last_frames ){
         user_t = -500;     // turn off motor if not seeing fport packets
         user_a = user_e = user_r = 0;
         user_m = 0;                                 // default mode 1, should be a self leveling mode
         last_frames = frames;
      }
   #endif

  #ifndef HAS_GYMBOL
    // find the mode
    if( user_m < -200 ) m = 0;
    if( user_m > 200 ) m = 2;
  #endif

  //  gains are servo microseconds per 90 degrees of bank or pitch angle - deadband
  if( roll > adead[m] )  a = map((roll*10), 10*adead[m],900,0,-again[m] );
  if( roll < -adead[m] ) a = map((roll*10),-10*adead[m],-900,0,again[m] );

  if( pitch-eangle[m] > edead[m] ) e = map( 10*(pitch-eangle[m]), 10*edead[m], 900,0, egain[m] );
  if( pitch-eangle[m] < -edead[m]) e = map( 10*(pitch-eangle[m]),-10*edead[m],-900,0,-egain[m] ); 

   // accelerometers will see a banked coordinated turn as flying upright
   // use the yaw rotation to counter this effect.  Leaky integral.
  y = gyro_z / 393;               // yaw rpm 
  y *= ygain[m];                  
  if( y > 0 ) yaw_I += 2;            // about a 1 second lag time here  > yaw_I 
  else if( y < 0 ) yaw_I -= 2;       // bump by 2 as always leaks by one below
  // else if( yaw_I > 0 ) --yaw_I;
  // else if( yaw_I < 0 ) ++yaw_I;
  
  // heading hold
  // if( roll < -10 || roll > 10 ) heading = 0;    // new heading
  // heading change by user when in heading hold mode
  if( hgain[m] != 0 ){
     if( user_a > 200 ) heading += 393;            // 360 deg turn in 1 minute
     if( user_a < -200 ) heading -= 393;
     user_a = a_trim;                              // !!! need fix for when using FPORT
     heading += gyro_z;
  }
  else heading = 0;                 // feature off
  
  y = heading / ( 50 * 65.5 );      // heading degrees off course ( rep rate * gyro reading for 1 degree/sec )
  y *= hgain[m];                   // gain == servo microseconds per 1 degrees off course
  if( yaw_I < y ) ++yaw_I;         // leak to heading, or zero if not using the heading hold
  if( yaw_I > y ) --yaw_I;

  yaw_I = constrain(yaw_I,-100,100);  // limit amount of servo travel for this 
  a += yaw_I;
  
  if( faz < 0 ) e = 0;            // if upside down, zero the elevator, avoid figure 9.
  a = constrain(a,-300,300);      // leave some control movement for user inputs ( max +-500 servo )
  e = constrain(e,-300,300);
  
   // add in the user input.
   a += user_a;  e += user_e; r += user_r;
   t = user_t;

   // reverse the servo's if needed, don't reverse channels in the transmitter.
   if( AIL_REVERSE ) a = -a;
   if( ELE_REVERSE ) e = -e;
   if( RUD_REVERSE ) r = -r;

   a += 1500;     // add in servo mid position
   e += 1500;
   r += 1500;
   t += 1500;

   a = constrain(a,1000,2000);
   e = constrain(e,1000,2000);
   r = constrain(r,1000,2000);
   t = constrain(t,1000,2000);
   aileron.writeMicroseconds(a);
   elevator.writeMicroseconds(e);

   #ifdef HAS_RUDDER
      rudder.writeMicroseconds(r);
   #endif

   #ifdef FPORT
      throttle.writeMicroseconds(t);
   #endif   
   
   //if( DBUG ){
     //Serial.print("  E ");  Serial.print(e); Serial.print("  A "); Serial.println(a);
   //}
  
}

#ifdef HAS_GIMBAL
// pan gimbal using parallax 360 servo

void gimbal_process3( int16_t val ){
float rpm;
float us;
float r;
int i;
static int8_t p;
static float fpos = 547.0;
static uint8_t mod;
static uint8_t right_hold;
static uint8_t left_hold;
//static uint8_t hold;
float c_lim;

  ++mod;             // run at 50 hz rate
  mod &= 3;
  if( mod == 0 ){
     //val = highpass(val);
     
     // 393 is the gyro reading for 6 deg/sec or 1 rpm
     // calculate what rate we think the servo should turn
     rpm = (float)val / 393.0;
     
     // when actively turning, clamp excursions looking away from the turn, hold after stick release
     if( user_a > a_trim+20 )  right_hold = 50;             // at 50 hz rate, value of 50 is 1 second hold
     if( user_a < a_trim-20 )  left_hold = 50;
     if( right_hold ){
         if( user_pos < 556 ) rpm = constrain( rpm,0,60 );  // turning right, allow right of center
         --right_hold;
     }
     if( left_hold ){
         if( user_pos > 556 ) rpm = constrain( rpm,-60,0);  // turning left, allow left of center
         --left_hold;
     }
    
     rpm += float(user_g)  * ( 15.0/500.0);    // map user input to max 15 rpm

     // add a small restoring force toward the zero value ( 547 if servo splines line up correctly )
     r = (float)( 556 - user_pos ) * 0.02;     // .02
     c_lim = 0.8;                              // centering force limit  ( 1.5 )
     r = constrain(r,-c_lim,c_lim);
     rpm += r;
     
     // calc the servo command wanted, remove the servo deadband  6 volt version
     //us = 0.8 * rpm;
     //if( rpm >= 3.0 )      us += 31.8;    // actual servo deadband found by experiment( 34or35 and -18 )
     //else if( rpm < -3.0 ) us -= 15.8;    // slowest rpm is about 4, zero points extrapolated to give
     //else                  us = 0;        // 4 rpm at 35, -19.
     // calc the servo command, 5 volt version
     us = rpm;
     if( rpm > 1.0 )       us += 32.0;
     else if( rpm < -1.0 ) us -= 19.0;
     else                  us = 0.0;
                                                            
     // avoid 360 degree rotations in case we have a video out cable attached
     if( user_pos < 100 ) us = constrain(us, 0, 400 );
     if( user_pos > 1000 ) us = constrain(us, -400, 0 );

     // apply a position feedback correction
     // calc where we think the position should be for next time, future position.
     // 200 hz rate: 1016 units for a rotation, 12000 samples in a minute --> 0.0846 factor
     //  50 hz rate: 1016 units, 3000 samples per minute --> 0.339 factor
     if( us != 0 ){
        us += 0.1 * ( fpos - (float)user_pos );           // 0.2
        fpos +=  0.339 * rpm ;                            // expected position for next time
     }
     fpos = 0.99*fpos + 0.01*(float)user_pos; // converge the future and actual servo position values
     fpos = constrain(fpos,28,1066);

     us = round(us);     
     if( GIMBAL_REVERSE ) us = -us;      // will need reverse as servo moves counter clockwise with increasing PWM period
                                         // or no reverse if mounted upside down
     us = constrain(us , -220, 220 );    // keep rate in servo active area

     gimbal.writeMicroseconds( 1500 + (int)us ); 

    // if( ++p > 32 ){
    //    p = 0;
    //    if( DBUG ){
         // Serial.print( user_a ); Serial.write(' ');    // these seem very stable now.
         // Serial.print( user_e ); Serial.write(' ');
         // Serial.print( user_g ); Serial.write(' ');
         // Serial.println( user_pos );
     //   }              
     // }
  }
}
#endif

/*
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
*/



#ifndef FPORT

// PWM input routines
 /*****************    reading pwm signals  *********************/ 
 ISR(PCINT1_vect){    // read pwm servo commands from the radio control receiver wired to A0,A1,A2
                     // A3 is the parallax 360 feedback signal or a mode channel
                     // save the values for later processing
   pwm_values[pwm_in].data = PINC;
   pwm_values[pwm_in].timer = micros();
   ++pwm_in;
   pwm_in &= 15;
 }

 void user_process(){    // decode the pwm servo data received, set the user variables to new values
                        // sanity check the data and ignore anything wrong
                        // 4th channel is a special case when using the 360 camera gimbal feedback servo
                        // and needs some different processing than the RC pwm signals.
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
            
             #ifdef HAS_GIMBAL
             if( (i < 3 && dt > 800 && dt < 2200) || ( i == 3 && dt < 1200 ) )   //sanity check
             #else
             if( (dt > 800 && dt < 2200) )
             #endif
             {
                us -= 1500;                 // sub out the zero point, get -500 to 500 in values
                us = median(i,us);          // discard glitches ( from interrupt latency I think )
                switch(i){
                  case 0:  user_a = us; break;    // ailerons
                  case 1:  user_e = us; break;    // elevator
                  case 2:  user_r = user_g = us; break;             // camera gimbal control or the rudder
                  case 3:  user_m = us; user_pos = us+1500; break;  // mode or feedback servo position
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

#endif    // end of the PWM routines

/*****  Non-blocking  I2C  functions   ******/

void i2cinit(){
  TWBR = 72;   //  12 400k for 16 meg clock.  72 100k   ((F_CPU/freq)-16)/2
  TWSR = 0;
  TWDR = 0xFF;
  PRR = 0;
}
// use some upper bits in the buffer for control
#define ISTART 0x100
#define ISTOP  0x200

void i2start( unsigned char adr ){
unsigned int dat;
  // shift the address over and add the start flag
  dat = ( adr << 1 ) | ISTART;
  i2send( dat );
}


void i2send( unsigned int data ){   // just save stuff in the buffer
  i2buf[i2in++] = data;
  i2in &= (I2BUFSIZE - 1);
}

void i2stop( ){
   i2send( ISTOP );   // que a stop condition
}


void i2flush(){  // call flush to empty out the buffer when in setup() 

  while( i2poll() ); 
}

// queue a read that will complete later
void i2queue_read( unsigned char adr, unsigned char reg, unsigned char qty ){
unsigned int dat;

   i2start( adr );
   i2send( reg );
   // i2stop();     // or repeated start
   dat = ((unsigned int)qty << 10) | ( adr << 1 ) | ISTART | 1;   // a start with the read bit set
   i2send( dat );
   i2stop();        // stop to complete the transaction
}

int i2read_int(){     // returns 2 values in i2c read queue as a signed integer
int data;

      if( i2rout == i2rin ) return 0;
      data = i2rbuf[i2rout++];
      i2rout &= (I2BUFSIZE-1);
      data <<= 8;
      if( i2rout == i2rin ) return data;
      data |= i2rbuf[i2rout++];
      i2rout &= (I2BUFSIZE-1);
      return data;
}

uint8_t i2available(){
uint8_t qty;

     qty = i2rin - i2rout;
     if( qty > I2BUFSIZE ) qty += I2BUFSIZE;    // some funky unsigned math
     return qty;
}

uint8_t i2poll(){    // everything happens here.  Call this from loop.
static uint8_t state = 0;
static unsigned int data;
static uint8_t delay_counter;
static unsigned int read_qty;
static uint8_t first_read;

   if( delay_counter ){   // the library code has a delay after loading the transmit buffer
     --delay_counter;     //  and before the status bits are tested
     return (16 + delay_counter);
   }
   
   switch( state ){    
      case 0:      // idle state or between characters
        if( i2in != i2out ){   // get next character
           data = i2buf[i2out++];
           i2out &= (I2BUFSIZE - 1 );
         
           if( data & ISTART ){   // start
              if( data & 1 ) read_qty = data >> 10;     // read queued
              data &= 0xff;
              // set start condition
              TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); 
              state = 1; 
           }
           else if( data & ISTOP ){  // stop
              // set stop condition
              TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
              state = 3;
           }
           else{   // just data to send
              TWDR = data;
              TWCR = (1<<TWINT) | (1<<TWEN);
              delay_counter = 5;   // delay for transmit active to come true
              state = 2;
           }
        }
      break; 
      case 1:  // wait for start to clear, send saved data which has the device address
         if( (TWCR & (1<<TWINT)) ){
            state = ( data & 1 ) ? 4 : 2;    // read or write pending?
            first_read = 1;
            TWDR = data;
            TWCR = (1<<TWINT) | (1<<TWEN);
            delay_counter = 5;
         }
      break;
      case 2:  // wait for done
         if( (TWCR & (1<<TWINT)) ){  
            state = 0;
         }
      break;
      case 3:  // wait for stop to clear
         if( (TWCR & (1<<TWSTO)) == 0 ){
            state = 0;
            delay_counter = 5;  // a little delay at the end of a sequence
         }
      break;
      case 4:  // read until count has expired
         if( (TWCR & (1<<TWINT)) ){
            // read data
            if( first_read ) first_read = 0;       // discard the 1st read, need 8 more clocks for real data
            else{
               i2rbuf[i2rin++] = TWDR;
               i2rin &= ( I2BUFSIZE - 1 );
               if( --read_qty == 0 ) state = 0;    // done
            }
            
            if( read_qty ){                        // any left ?
               if( read_qty > 1 ) TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);  // not the last read
               else TWCR = (1<<TWINT) | (1<<TWEN);                            // nack the last read
            }
            delay_counter = 5;
         }
      break;    
   }

   if( i2in != i2out ) return (state + 8);
   else return state;
}
