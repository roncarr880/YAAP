
/*
 *    YAAP  - yet another auto pilot
 * 
 *    Using an Arduino UNO and a MPU-6050 board,
 *    simple algorithms and some borrowed MPU-6050 code.
 *    
 *    Current target airframe is a 3 channel slow stick - using rudder to control roll.
 *    Goal is to control rudder, elevator and a servo driven 1 axis camera gimbal ( yaw ).
 *    
 *    Increasing servo pulse width coded to pitch down and roll right.
 *    Servo's should be reversed here and not in the transmitter.
 */

#define DBUG 1    // for serial prints

#define UPRIGHT 0  // mounting orientation, Y axis is always towards the front
#define INVERTED_ 1
#define LEFT_SIDE 2
#define RIGHT_SIDE 3
const char mounting = UPRIGHT;     // pick one

#define WING_ANGLE 5.0             // desired trim pitch
#define ELE_REVERSE 0
#define AIL_REVERSE 0

#define ACC_MAX_VALUE  4343.0f     // pre-measured value

#include <Wire.h>
#include <Servo.h>
#include <avr/interrupt.h>

Servo aileron;    // actually rudder for the test airplane
Servo elevator;

int user_a, user_e, user_g;   // user radio control inputs

struct PWM {
   uint8_t data;
   unsigned long timer;
};

struct PWM pwm_values[8];    // buffer the received servo signals
uint8_t pwm_in, pwm_out;

// IMU variables
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z;
int16_t temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
unsigned long IMU_timer;
float angle_pitch, angle_roll;
float angle_roll_acc, angle_pitch_acc;

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
  
  ACCEL_process();                    // initial position from the accelerometers, must be upright.
  pitch = angle_pitch = angle_pitch_acc;  
  roll = angle_roll = angle_roll_acc;
 
  IMU_timer = micros();                                               //Reset the loop timer

  aileron.attach(9);
  elevator.attach(10);
  // gimbal.attach(8);
  aileron.writeMicroseconds(1500);
  elevator.writeMicroseconds(1500);

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
  pwm_values[pwm_in].timer = micros();
  pwm_values[pwm_in].data = PINC;
  ++pwm_in;
  pwm_in &= 7;
}

void user_process(){    // decode the pwm servo data received, set the user variables to new values
                        // sanity check the data and ignore anything wrong
static uint8_t last_bit[3];
static unsigned long last_time[3];
int8_t mask,b,i;
unsigned long dt;  

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
            if( dt > 800 && dt < 2200 ){   // could be real
               dt -= 1500;                 // sub out the zero point
               switch(i){
                   case 0:  user_a = dt; break;    // ailerons
                   case 1:  user_e = dt; break;    // elevator
                   case 2:  user_g = dt; break;    // camera gimbal
               }  
            }
         }
     }       // end bit change
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
unsigned long timer;
int a,e;
static int last_a, last_e;

   if( millis() - timer < 20 ) return;

   timer = millis();

   // combine the auto pilot and the user input using a simple algorithm
   a = map(roll*10,-900,900,500,-500);    // auto pilot trys to keep upright and reduce angle to zero
   e = map((pitch-WING_ANGLE)*10,-900,900,-500,500);

   // try 40% auto and 60% user
   a *= 4;
   e *= 4;
   a /= 10;
   e /= 10;

   // add in the user input.  Since the auto is usually opposite the user, put in 100% user to get 60% max
   a += user_a;  e += user_e;

   // reverse the servo's if needed
   if( AIL_REVERSE ) a = -a;
   if( ELE_REVERSE ) e = -e;

   a += 1500;     // add in servo mid position
   e += 1500;

   if( a == last_a && e == last_e ) return;   // save some time?, I don't think so when flying but ok for debug
   last_a = a;  last_e = e;                   // !!! remove this later
   
   aileron.writeMicroseconds(a);
   elevator.writeMicroseconds(e);

   if( DBUG ){
     Serial.print("E ");  Serial.print(e); Serial.print("  A "); Serial.println(a);
   }
  
}


/*   
   MPU code from the below source, and modified where needed for this application
   And perhaps bugs fixed or introduced.
   
*/   

///////////////////////////////////////////////////////////////////////////////////////
/*Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.


///////////////////////////////////////////////////////////////////////////////////////
//Support
///////////////////////////////////////////////////////////////////////////////////////
Website: http://www.brokking.net/imu.html
Youtube: https://youtu.be/4BoIE8YQwM8
Version: 1.0 (May 5, 2016)
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


void IMU_process(){

  if(micros() - IMU_timer < 4000) return;   // run at 250 hz rate, call as often as wanted
  IMU_timer = micros();                                           
  
  read_mpu_6050_data();                              //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_cal;                       //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                       //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                       //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  if( acc_z > 0 ){                              // upside right
     angle_pitch += (float)gyro_x * 0.0000611;  //Calculate the traveled pitch angle
     angle_roll += (float)gyro_y * 0.0000611;   //Calculate the traveled roll angle
  }
  // not technically correct as angles do not grow more than 90 degrees.
  // but when twisted around it integrates the angles reasonably
  else{                                         // upside down
     angle_pitch -= (float)gyro_x * 0.0000611;  //Calculate the traveled pitch angle
     angle_roll -= (float)gyro_y * 0.0000611;   //Calculate the traveled roll angle    
  }
  
  // Yaw corrections
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sine function is in radians
  // angle_pitch += angle_roll * sin(gyro_z * 0.000001066); // transfer the roll angle to the pitch angel
  // angle_roll -= angle_pitch * sin(gyro_z * 0.000001066); // transfer the pitch angle to the roll angel
  
  angle_pitch += sin(angle_roll/57.3) * (float)gyro_z * 0.0000611;
  angle_roll -=  sin(angle_pitch/57.3) * (float)gyro_z * 0.0000611;
  
  
  ACCEL_process();
  
   // Correct the drift of the gyro with ACCEL data, converge a little quicker than the original
   angle_pitch = angle_pitch * 0.995 + angle_pitch_acc * 0.005;    
   angle_roll = angle_roll * 0.995 + angle_roll_acc * 0.005;
  
   // if the airplane loops or rolls, don't make it unwind with another loop or roll
   // I don't think this is needed now since angles to not pass 90 degrees
   /*
    if( angle_pitch > 180 ) angle_pitch -= 360;
    if( angle_pitch < -180 ) angle_pitch += 360;
    if( angle_roll > 180 ) angle_roll -=  360;
    if( angle_roll < -180 ) angle_roll += 360;
    */
  
    if( DBUG ) write_LCD();                               //Write the roll and pitch values to the LCD display
   
   // save a final copy of the working values
   // noInterrupts();           // may or may not need to be guarded depending upon what code is added
   pitch = angle_pitch;         // and how it is configured to work.  If values used in an interrupt 
   roll  = angle_roll;          // then should be guarded.
   // interrupts();
}



// since the accelerometers have noise due to constant movement, a small angle approximation may work
// just as well as the trig functions.  When not moving, the total gravity vector rotated should 
// form a perfect sphere of the max value of the accelerometer, and again variations would be due to noise.
// The max value seems to be about 4343 with noise of +-30.
void ACCEL_process(){

  if( acc_x < acc_z && acc_y < acc_z ){        // more or less upright
     angle_pitch_acc = ((float)acc_y/ACC_MAX_VALUE) * 57.296;       //Calculate the pitch angle 
     angle_roll_acc =  ((float)acc_x/ACC_MAX_VALUE) * -57.296;      //Calculate the roll angle  
  }
  else {                                       // use the current gyro values
     angle_pitch_acc = angle_pitch;   angle_roll_acc = angle_roll;
  }
}

#ifdef NOWAY
  //Accelerometer angle calculations.   Had some problems with NaN's here.
void ACCEL_process(){
long acc_total_vector;
long t;
static uint8_t once;


  t = (long)acc_x * (long)acc_x;     // integer math overflow issues with original program code.
  t+= (long)acc_y * (long)acc_y;     // caused asin to return NaN's
  t+= (long)acc_z * (long)acc_z;
  
  //acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  acc_total_vector = sqrt(t);
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  if( DBUG && once == 0 ){
    Serial.print(acc_x); Serial.write(' ');
    Serial.print(acc_y); Serial.write(' ');
    Serial.print(acc_z); Serial.write(' ');
    Serial.println(acc_total_vector);
    ++once;
  }
 
  angle_pitch_acc = asin((float)acc_y/acc_total_vector) * 57.296;       //Calculate the pitch angle
  if( isnan(angle_pitch_acc)) angle_pitch_acc = angle_pitch; 
  angle_roll_acc = asin((float)acc_x/acc_total_vector) * -57.296;       //Calculate the roll angle
  if( isnan(angle_roll_acc)) angle_roll_acc = angle_roll; 
  
  if( acc_z < 0 ){    // asin repeats and does not find angles in quadrants 3 and 4.
   //  angle_pitch_acc = (angle_pitch_acc >= 0 ) ? 180 - angle_pitch_acc : -180 - angle_pitch_acc;
   //  angle_roll_acc =  (angle_roll_acc >= 0) ? 180 - angle_roll_acc : -180 - angle_roll_acc;
    // discontinuity in the other axis when angles pass 90 degrees, go with the gyro calculation
    angle_pitch_acc = angle_pitch;   angle_roll_acc = angle_roll;
    
  }
  // Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  //  angle_pitch_acc -= 0.0;                              //Accelerometer calibration value for pitch
  //  angle_roll_acc -= 0.0;                               //Accelerometer calibration value for roll

}
#endif

// modified to write to serial instead of a I2C display
void write_LCD(){
static int lcd_loop_counter;
static int angle_pitch_buffer, angle_roll_buffer;


  return;   // defeat this to see other debug printing
  //To get a 250Hz program loop (4us) it's only possible to write one character per loop
  
  if(lcd_loop_counter == 14)lcd_loop_counter = 0;                      //Reset the counter after 14 characters
  lcd_loop_counter ++;                                                 //Increase the counter
  if(lcd_loop_counter == 1){
    angle_pitch_buffer = angle_pitch * 10;                      //Buffer the pitch angle because it will change
    // angle_pitch_buffer = (angle_pitch_acc - angle_pitch) * 10;
    // angle_pitch_buffer = angle_pitch_acc * 10;
    // Serial.write(' '); Serial.println(acc_z);
    Serial.println();
  }
  if(lcd_loop_counter == 2){
    if(angle_pitch_buffer < 0)Serial.print("-");                          //Print - if value is negative
    else Serial.print("+");                                               //Print + if value is negative
  }
  if(lcd_loop_counter == 3)Serial.print(abs(angle_pitch_buffer)/1000);    //Print first number
  if(lcd_loop_counter == 4)Serial.print((abs(angle_pitch_buffer)/100)%10);//Print second number
  if(lcd_loop_counter == 5)Serial.print((abs(angle_pitch_buffer)/10)%10); //Print third number
  if(lcd_loop_counter == 6)Serial.print(".");                             //Print decimal point
  if(lcd_loop_counter == 7)Serial.print(abs(angle_pitch_buffer)%10);      //Print decimal number

  if(lcd_loop_counter == 8){
    angle_roll_buffer = angle_roll * 10; 
     // angle_roll_buffer = (angle_roll_acc - angle_roll) * 10;  // delta error gyro to acc
     // angle_roll_buffer = angle_roll_acc * 10;
    Serial.write(' ');
  }
  if(lcd_loop_counter == 9){
    if(angle_roll_buffer < 0)Serial.print("-");                           //Print - if value is negative
    else Serial.print("+");                                               //Print + if value is negative
  }
  if(lcd_loop_counter == 10)Serial.print(abs(angle_roll_buffer)/1000);    //Print first number
  if(lcd_loop_counter == 11)Serial.print((abs(angle_roll_buffer)/100)%10);//Print second number
  if(lcd_loop_counter == 12)Serial.print((abs(angle_roll_buffer)/10)%10); //Print third number
  if(lcd_loop_counter == 13)Serial.print(".");                            //Print decimal point
  if(lcd_loop_counter == 14)Serial.print(abs(angle_roll_buffer)%10);      //Print decimal number
  
}
