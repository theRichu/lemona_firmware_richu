#include "RobotSerialComm.h"
#include "DCDM1210.h"


// ********* Encoder ******** 
#define MAX_ENC_NUM 2
#define MAX_ENC_VALUE 32768
#define MIN_ENC_VALUE -32767
int enc_value[MAX_ENC_NUM]={(int)((MIN_ENC_VALUE)+(MAX_ENC_VALUE))/2};


int r_count=0;
int l_count=0;
int enc1_count=0;
int enc2_count=0;



unsigned int number;
char data[100];
double t_rv,t_lv;
int R_direction=2;
int L_direction=2;
 
 
#define LED_RATE 10000
#define LEAD_A 1
#define LEAD_B 2
#define DISTANCE_MOTER 12.25    // mm

#define ROBOT_ID
 
HardwareTimer timer(2);
DCDM1210 driver;
RobotSerialComm port;
int voltage;
boolean stream=false;

//Variables for motion control
double lin_speed_si=0.0;    //Linear speed in m/s
double ang_speed_si=0.0;    //Rotational speed in rad/s
 
double r_desired_velocity = 0; // 목표 속도 설정
double r_currently_velocity;
double l_desired_velocity = 0; // 목표 속도 설정
double l_currently_velocity;
double r_error,r_error_dot,r_previous_error;
double l_error,l_error_dot,l_previous_error;
double P=100;
double I=50;
double r_input,l_input;
 
int r_way,l_way;

unsigned int reply_arg[5];

/* Encoder Value */
void readEncoder(int enc_id, boolean increment){
 if(enc_id<MAX_ENC_NUM && enc_id>=0){  // INPUT ERROR CHECK
   if((enc_value[enc_id]<MAX_ENC_VALUE) && (enc_value[enc_id]>MIN_ENC_VALUE)){
     if(increment){
       enc_value[enc_id]++;
     }else{
       enc_value[enc_id]--;
     }
   }else if(enc_value[enc_id]==MAX_ENC_VALUE){
     if(increment){
       enc_value[enc_id] = MIN_ENC_VALUE;
     }else{
       enc_value[enc_id]--;
     }
   }else if(enc_value[enc_id]==MIN_ENC_VALUE){
     if(increment){
       enc_value[enc_id]++;
     }else{
       enc_value[enc_id] = MAX_ENC_VALUE;
     }  
   }  
 }
}

void sendRobotInfo(){   
    
    driver.Answer_voltage(&voltage);
    delay(10);
    reply_arg[0] = (int)1;                // Robot ID
    SerialUSB.println(voltage);
    port.reply(ROBOT_INFO, reply_arg, 1);
}

void sendEncodersReads(){    
    reply_arg[0] = enc_value[0];
    reply_arg[1] = enc_value[1];
    port.reply(OMNI_READ_ENCODERS, reply_arg, 2);
}
 
void setup() {
  //Initialize Serial3
  driver.Serial_connect(SERIAL_3,BAUD_RATE_9600);
  //Serial3.begin(9600);
  SerialUSB.begin();
  pinMode(26,INPUT);
  pinMode(27,INPUT);
  pinMode(28,INPUT);
  pinMode(29,INPUT);
  attachInterrupt(26,R_intA,RISING);
  attachInterrupt(27,L_intA,RISING);
//  attachInterrupt(28,R_intB,FALLING);
//  attachInterrupt(29,L_intB,FALLING);
 
  timer.pause();
  timer.setPeriod(LED_RATE); // in microseconds
  timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);
  timer.attachInterrupt(TIMER_CH1, handler_led);
 
  timer.refresh();
  timer.resume();
}
 
int r_control=0,l_control=0;

void loop() {

  int arg[5];

  int action = port.getMsg(arg); 

  if(action==0 && stream==true){
      action=ACTION_START_STREAM;
    }

  switch(action){
          
            case OMNI_CALIBRATION:                 // "@1e", no reply
                //omni.calibrate(1,0,0);
                delay(95);
                break;
            
            case OMNI_SET_PID:                    //@2,"KP","KI","KD"e, no reply
                //omni.set_PID(arg[0], arg[1], arg[2]);
                break;
                
            case OMNI_SET_PRESCALER:              //@3,"enc","value"e, no reply
                //omni.set_prescaler(arg[0], arg[1]);
                break;

            case OMNI_SET_ENC_VALUE:            //@4,"enc","enc_value"e, no reply
                //omni.set_enc_value(arg[0], arg[1]);
                break;

            case ROBOT_INFO:                    //@5e, reply: @5,"temp","firm","bat","r_firm","r_id"e
                sendRobotInfo();
                break;

            case OMNI_READ_ENCODERS:          //@6e,  reply: @6,"enc1(R)","enc2(L)"e
                sendEncodersReads();
                delay(70);
                break;

            case READ_SONARS:                //@7e, reply: @7,"son1(F)","son2(L)","son3(R)"e
                //sendSonarsReads();
                break;    

            case READ_ENCODERS_SONARS:       //@8e, reply: @8,"enc1(R)","enc2(L)","son1(F)","son2(L)","son3(R)"e
                //sendEncodersSonarsReads();
                break;    
                
            case LINEAR_MOVE_PID:            //@9,"speed1","speed3"e, no reply
                //omni.mov_lin3m_pid(arg[0], 0, arg[1]);              
                break;

            case LINEAR_MOVE_NOPID:          //@10,"speed1","speed2"e, no reply
                //omni.mov_lin3m_nopid(arg[0], 0, arg[1]);
                break;

            case MOVE_DIFFERENTIAL_SI:          //@11,"vel_linear","vel_angular"e, no reply
                //SerialUSB.println(arg[0]);
                //SerialUSB.println(arg[1]);
                lin_speed_si= ((double)arg[0]/1000); 
                ang_speed_si= ((double)arg[1]/1000);

                r_desired_velocity = lin_speed_si + DISTANCE_MOTER / (2 * PI) * ang_speed_si;
                l_desired_velocity = lin_speed_si - DISTANCE_MOTER / (2 * PI) * ang_speed_si;
                break;

            case MOVE_POSITIONAL:              //@12,"motor_nr","speed","encoder_Position"e, no reply
                //omni.mov_pos(arg[0], arg[1], arg[2], 1);  // move motor1 at speed1 until encoder count reaches the defined position and then stop with holding torque
                delay(1);                          // wait 1ms for Omni3MD to process information
                break;

            case STOP_MOTORS:                //@13e, no reply
                //omni.stop_motors();
                break;
                
            case ENCODERS_RESET:             //@14e, no reply
                //robot.encodersReset();
                break;                
                              
            case ACTION_GET_DEBUG:           //@15e, reply (to the console): @13,"0/1"e
                //reply_arg[0] = port.getDebug();
                //port.reply(ACTION_GET_DEBUG, reply_arg, 1);
                break;

            case ACTION_SET_DEBUG:           //@16,"0/1"e, no reply
                //port.setDebug(arg[0]);  
                break;

            case ACTION_GET_STREAM:           //@17e, reply @15,"0/1"e
                //reply_arg[0] = stream;
                //port.reply(ACTION_GET_STREAM, reply_arg, 1);
                break;
                
            case ACTION_START_STREAM:      // "@18e,  reply: @6,"enc1(R)","enc2(L)"e (repeatedly)
                stream = true;
                sendEncodersReads();
                //delay(65);                //encoders read update (+- 15Hz)
                break;
                
            case ACTION_STOP_STREAM:        // "@19e,  no reply 
                stream = false;                
                break;
                
            default:
                break;
   } // switch
   
 /*
   int number;
   int line;
   int ang;
   number = SerialUSB.available();
  if(number > 0)
  {
    SerialUSB.read(data,number);
    SerialUSB.print(data);
      //line = data[3] & 0xff;
      //ang = data[5] & 0xff;

      //if(data[2])  line = line * (-1.0);
      //if(data[4])  ang = ang * (-1.0);
      
      line = atol(data);

      r_desired_velocity = line + DISTANCE_MOTER / (2 * PI) * ang;
      r_desired_velocity = r_desired_velocity / 100;
      
      l_desired_velocity = line - DISTANCE_MOTER / (2 * PI) * ang;
      l_desired_velocity = l_desired_velocity / 100;
  }
  */
  /*
  r_currently_velocity = t_rv;
  r_error = r_desired_velocity-r_currently_velocity;
  r_error_dot = r_error-r_previous_error;
  r_input = P * r_error + I * r_error_dot + r_input;
 
  if(r_input < 0)  {
    r_input = r_input*(-1);
    r_way = 0;
  }
  else{
    r_way = 1;
  }
 
  if(r_input >= 999) r_input = 999;
  if(r_input <= 0) r_input = 0;
 
  l_currently_velocity = t_lv;
  l_error = l_desired_velocity-l_currently_velocity;
  l_error_dot = l_error-l_previous_error;
  l_input = P * l_error + I * l_error_dot + l_input;
 
  if(l_input < 0)  {
    l_input = l_input*(-1);
    l_way = 1;
  }
  else{
    l_way = 0;
  }
 
  if(l_input >= 999) l_input = 999;
  if(l_input <= 0) l_input = 0;

  // mistake  L R change
  driver.Moter_control(LEFT, r_way ,(int)r_input,RIGHT, l_way ,(int)l_input);
 
  if(r_way == 0)  r_input = r_input*(-1);
  if(l_way == 1)  l_input = l_input*(-1);
 
  //SerialUSB.write(input*0.252525252525);
  r_previous_error = r_error;
  l_previous_error = l_error;
 
  //delay(1);  
*/
  reset_array();
}
void reset_array()
{
  int i;
  for(i=0; i<100; i++)
    data[i] = 0;
}

void R_intA()                //WHEN R_A RISING
{
  if(digitalRead(28))        //IF(R_B is 1)
  {
    readEncoder(0,false);    //CCW
  }
  else
  {
    readEncoder(0,true);       //CW   
  }
}


void L_intA()                  //WHEN L_A RISING
{
  if(!digitalRead(29))          //IF(R_B is 0)
  {
    readEncoder(1,false);      //CCW
  }
  else
  { 
    readEncoder(1,true);        //CW   
  }
}

 
void handler_led(void) { 
 
  t_rv = 0.135 * PI * r_count / 1155 * 100;
  t_lv = 0.135 * PI * l_count / 1155 * 100;
  r_count = 0;
  l_count = 0;
}
