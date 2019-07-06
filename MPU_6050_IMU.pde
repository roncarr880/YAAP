// YAAP   -  yet another auto pilot

#define DBUG 1    // for serial prints

#define UPRIGHT 0  // mounting orientation, Y axis is always towards the front
#define INVERTED 1
#define LEFT_SIDE 2
#define RIGHT_SIDE 3
const char mounting = UPRIGHT;     // pick one

#include <Wire.h>


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
  
  delay(2000);                           // unit jiggle delay after battery is connected
  setup_mpu_6050_registers();            //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro   
  calibrate_gyro();
  
  ACCEL_process();                    // initial position from the accelerometers, must be upright.
  pitch = angle_pitch = angle_pitch_acc;  
  roll = angle_roll = angle_roll_acc;
 
  IMU_timer = micros();                                               //Reset the loop timer
}

void loop(){

     IMU_process();

}


void calibrate_gyro(){     /* try to find repeatable readings for when things are still */
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
       gyro_x_cal += gyro_x;                                  //Add the gyro x-axis offset to the gyro_x_cal variable
       gyro_y_cal += gyro_y;                                  //Add the gyro y-axis offset to the gyro_y_cal variable
       gyro_z_cal += gyro_z;                                  //Add the gyro z-axis offset to the gyro_z_cal variable
       delay(3);                                              //Delay 3us to simulate the 250Hz program loop
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

void change_orientation(){
int16_t temp;

    switch( mounting ){      // swap the raw values so the rest of the program thinks it is mounted upright
       case INVERTED:
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


void read_mpu_6050_data(){                                       //Subroutine for reading the raw gyro and accelerometer data

  digitalWrite(13,LOW);                                          // LED will show if hangs here
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable
  digitalWrite(13,HIGH);
  
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

  gyro_x -= gyro_x_cal;                              //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_cal;                              //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_cal;                              //Subtract the offset calibration value from the raw gyro_z value

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_x * 0.0000611;      //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;       //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  // angle_pitch += angle_roll * sin(gyro_z * 0.000001066); //If the IMU has yawed transfer the roll angle to the pitch angel
  // angle_roll -= angle_pitch * sin(gyro_z * 0.000001066); //If the IMU has yawed transfer the pitch angle to the roll angel
  angle_pitch += sin(angle_roll/57.3) * gyro_z * 0.0000611;
  angle_roll -=  sin(angle_pitch/57.3) * gyro_z * 0.0000611;
  
  ACCEL_process();
  
   // Correct the drift of the gyro with ACCEL data
   angle_pitch = angle_pitch * 0.995 + angle_pitch_acc * 0.005;    
   angle_roll = angle_roll * 0.995 + angle_roll_acc * 0.005;
  
   // if the airplane loops or rolls, don't make it unwind with another loop or roll
    if( angle_pitch > 180 ) angle_pitch -= 360;
    if( angle_pitch < -180 ) angle_pitch += 360;
    if( angle_roll > 180 ) angle_roll -=  360;
    if( angle_roll < -180 ) angle_roll += 360;
  
    if( DBUG ) write_LCD();                                   //Write the roll and pitch values to the LCD display
   
   // save a final copy of the working values
   // noInterrupts();           // may or may not need to be guarded depending upon what code is added
   pitch = angle_pitch;         // and how it is configured to work.  If values used in an interrupt 
   roll  = angle_roll;          // then should be guarded.
   // interrupts();
}


  //Accelerometer angle calculations.
void ACCEL_process(){
long acc_total_vector;
  
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the total accelerometer vector
  
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  if( acc_z < 0 ){
   //  angle_pitch_acc = (angle_pitch_acc >= 0 ) ? 180 - angle_pitch_acc : -180 - angle_pitch_acc;
   //  angle_roll_acc =  (angle_roll_acc >= 0) ? 180 - angle_roll_acc : -180 - angle_roll_acc;
    // discontinuity in the other axis when angles pass 90 degrees, go with the gyro calculation
    angle_pitch_acc = angle_pitch;   angle_roll_acc = angle_roll;
    
  }
  // Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  //  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  //  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

}


// modified to write to serial instead of a I2C display
void write_LCD(){
static int lcd_loop_counter;
static int angle_pitch_buffer, angle_roll_buffer;

  //To get a 250Hz program loop (4us) it's only possible to write one character per loop
  
  if(lcd_loop_counter == 14)lcd_loop_counter = 0;                      //Reset the counter after 14 characters
  lcd_loop_counter ++;                                                 //Increase the counter
  if(lcd_loop_counter == 1){
    angle_pitch_buffer = angle_pitch * 10;                            //Buffer the pitch angle because it will change
    // angle_pitch_buffer = (angle_pitch_acc - angle_pitch) * 10;
    // angle_pitch_buffer = angle_pitch_acc * 10;
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






