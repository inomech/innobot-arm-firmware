#include <elapsedMillis.h>
#include "AccelStepper.h"

#define MANUAL 0 
#define AUTOMATIC 1
#define DIRECT 0 
#define REVERSE 1

int controller_direction[6] = {DIRECT,REVERSE,DIRECT,DIRECT,DIRECT,REVERSE}; 

bool in_auto = true; 
AccelStepper *steppers[6] = {NULL, NULL, NULL, NULL, NULL, NULL}; 
float setpoint_angle[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float actual_angle_deg[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float error_sum[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float setpoint_last[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float error_last[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float integral_term[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float u[6];
// int dir[6] = {-1, 1, -1, -1, -1, 1};
float loop_interval[6]; 

elapsedMillis time_change[6]; // loop interval for derivation
elapsedMillis now_time[6]; // loop interval for derivation
elapsedMillis last_time[6]; // loop interval for derivation
elapsedMillis print_time;   // printing period
float actual_angle_rad[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float kp[6] = {40.0, 20.0, 15.0, 10.0, 15.0, 10.0}; 
float ki[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
float kd[6] = {10.0, 7.0, 4.0, 2.0, 4.0, 4.0}; 

int max_speed[6] = {2000, 2000, 2000, 2000, 2000, 2000}; 
int min_speed[6] = {-2000, -2000, -2000, -2000, -2000, -2000};

int ms[6] = {16, 16, 2, 16, 16, 16};
float gear_ratio[6] = {10.32, 5.48, 4.3 * 5, 1 , 4.5, 1};

unsigned int sample_time = 1000; 

const double pivot_length = 0.04;
float object_width = 0.08; 
const int offset_angle = 8; 
const double max_object_width = 0.08; 
const double min_object_width = 0.01; 


void computePid(unsigned int idx){

  if(!in_auto) return; 

  if(time_change[idx] >= sample_time){

    actual_angle_deg[idx] = (360 * (float)steppers[idx]->currentPosition() / (200*ms[idx]))/gear_ratio[idx];     // current angle in [deg]

    if(controller_direction[idx] == REVERSE) actual_angle_deg[idx] = -actual_angle_deg[idx]; 
  
    // compute all the errors 
    
    float error = setpoint_angle[idx] - actual_angle_deg[idx];
    integral_term[idx] += (ki[idx] * error);
    if(integral_term[idx] > max_speed[idx]){
      integral_term[idx] = max_speed[idx];
    }else if(integral_term[idx] < min_speed[idx]){
      integral_term[idx] = min_speed[idx];
    }
    // error_sum[idx] += error;
    // float error_der = error - error_last[idx];

    float input_der = setpoint_angle[idx] - setpoint_last[idx];

    u[idx] = kp[idx] * error + integral_term[idx] +  kd[idx] * input_der;               // control value 

    if(u[idx] > max_speed[idx]){
      u[idx] = max_speed[idx]; 
    }else if(u[idx] < min_speed[idx]){
      u[idx] = min_speed[idx];
    }
    
    if(u[idx] >= max_speed[idx]){

      u[idx] = max_speed[idx];

    }else if(u[idx] <= min_speed[idx]){

      u[idx] = min_speed[idx];
      
    }

    setpoint_last[idx] = setpoint_angle[idx]; 

    error_last[idx] = error; 

    time_change[idx] = 0;   

  }
     
}

void setTunings(float Kp, float Ki, float Kd, int idx){

  if(Kp < 0 || Ki < 0 || Kd < 0) return; 

  double sample_time_in_sec = ((double)sample_time)/1000;
  kp[idx] = Kp; 
  ki[idx] = Ki * sample_time_in_sec; 
  kd[idx] = Kd / sample_time_in_sec;

  if(controller_direction[idx] == REVERSE){
    kp[idx] = -kp[idx];
    ki[idx] = -ki[idx];
    kd[idx] = -kd[idx];
  }

}

void set_controller_direction(int direction, int idx){
  controller_direction[idx] = direction;
}

void setSampleTime(unsigned int new_sample_time){
  
  if(new_sample_time > 0){
    double ratio = (double)new_sample_time / (double)sample_time; 

    for(unsigned int i = 0; i < sizeof(steppers)/sizeof(steppers[0]); i++){
      ki[i] *= ratio; 
      kd[i] /= ratio; 
    }
    sample_time = new_sample_time; 

  }

}

void setOutputLimits(int min, int max, int idx){
  
  if(min > max){
    return; 
  }

  max_speed[idx] = max; 
  min_speed[idx] = min; 

  if(u[idx] > max_speed[idx]){
    u[idx] = max_speed[idx];
  }else if(u[idx] < min_speed[idx]){
    u[idx] = min_speed[idx];
  }

  if(integral_term[idx] > max_speed[idx]){
    integral_term[idx] = max_speed[idx];
  }else if(integral_term[idx] < min_speed[idx]){
    integral_term[idx] = min_speed[idx];
  } 

}

void initialize(){
  
  for(unsigned int i = 0; i < sizeof(steppers)/sizeof(steppers[0]); i++){
  
    setpoint_last[i] = setpoint_angle[i]; 

    integral_term[i] = u[i]; 

    if(integral_term[i] > max_speed[i]) integral_term[i] = max_speed[i];
    else if(integral_term[i] < min_speed[i]) integral_term[i] = min_speed[i]; 

  }
  
}


void set_mode(int mode){
  bool new_auto = (mode == AUTOMATIC);
  if(new_auto && !in_auto){
    initialize(); 
  }
  in_auto = new_auto; 
}


void writeActualPosition(){
    
  print_time = 0;

  char msg[50];
  char buff[8];  

  for(unsigned int idx = 0; idx < sizeof(steppers)/sizeof(steppers[0]); idx++)
  {

    if(idx == 0){
      strcpy(msg,"");
    }

    actual_angle_rad[idx] = M_PI*(actual_angle_deg[idx] / 180);     // current angle 

    dtostrf(actual_angle_rad[idx],8,6,buff);
    strcat(msg,buff);
    
    if(idx != 5){
      strcat(msg,",");
    }

  }

  Serial.println(msg);
     
}


