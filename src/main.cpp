#include "Arduino.h"
#include "math.h"
#include "motor_driver.h"
#include "stdlib.h"

const unsigned int STEP_PIN[6] = {24, 28, 32, 36, 40, 44};
const unsigned int DIR_PIN[6] = {22, 26, 30, 34, 38, 42};
const unsigned int EN_PIN[6] = {23, 25, 27, 29, 31, 33};

const byte numChars = 32; 
char receivedChars[numChars];
char receivedCmdChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing
char tempCmdChars[numChars];  // temporary array for use when parsing
int SIZE = 1000;

const unsigned int FAN_PIN = 10;

// variables to hold the parsed data
char messageFromPC[numChars] = {0};
int integerFromPC = 0; 

unsigned int rcvd_cmd;

char dataStr[100] = "";
char buffer[7];

int i = 0;

boolean newData = false;
boolean newCmdData = false;

void recvWithStartEndMarkers(){

    static boolean recvInProgress = false; 
    static boolean recvCmdInProgress = false;
    static byte ndx = 0;
    char startMarker = '[';
    char startCmdMarker = '{';
    char endMarker = ']';
    char endCmdMarker = '}';
    char rc; 

    while(Serial.available() && newData == false){
        rc = Serial.read();

        if(recvInProgress == true){
            if(rc != endMarker){
                receivedChars[ndx] = rc;
                ndx++;
                if(ndx >= numChars){
                    ndx = numChars - 1;
                }
            }
            else{
                receivedChars[ndx] = '\0';  // terminate string
                recvInProgress = false; 
                ndx = 0;
                newData = true;
            }
        }
        else if(rc == startMarker){
          recvInProgress = true;
        }else if(rc == startCmdMarker){
          recvCmdInProgress = true;
        }else if(recvCmdInProgress == true){
            if(rc != endCmdMarker){
                receivedCmdChars[ndx] = rc;
                ndx++;
                if(ndx >= numChars){
                    ndx = numChars - 1;
                }
            }
            else{
                receivedCmdChars[ndx] = '\0';  // terminate string
                recvCmdInProgress = false; 
                ndx = 0;
                newCmdData = true;
            }
        }
    }

}

void showParsedData(){

  Serial.print("Float 1: ");
  Serial.println(setpoint_angle[0]); 

  Serial.print("Float 2: ");
  Serial.println(setpoint_angle[1]); 

  Serial.print("Float 3: ");
  Serial.println(setpoint_angle[2]);

  Serial.print("Float 4: ");
  Serial.println(setpoint_angle[3]); 

  Serial.print("Float 5: ");
  Serial.println(setpoint_angle[4]);

  Serial.print("Float 6: ");
  Serial.println(setpoint_angle[5]); 

}


void parseData(unsigned int mode){
  
  if(mode == 0){

    char * strtokIndx; 

    strtokIndx = strtok(tempChars, ",");
    setpoint_angle[0] = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    setpoint_angle[1] = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    setpoint_angle[2] = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    setpoint_angle[3] = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    setpoint_angle[4] = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    setpoint_angle[5] = atof(strtokIndx);

    // showParsedData();

  }else if(mode == 1){
    
    char * strtokIndxCmd;

    strtokIndxCmd = strtok(tempCmdChars, " ");
    rcvd_cmd = atoi(strtokIndxCmd);
    
    // Serial.println(rcvd_cmd);

  }
  
}

void setup(){

  Serial.begin(115200);

  for(unsigned int i = 0; i < sizeof(steppers)/sizeof(steppers[0]); i++){
    
    steppers[i] = new AccelStepper(AccelStepper::DRIVER, STEP_PIN[i], DIR_PIN[i]);
    steppers[i]->setMaxSpeed(max_speed[i]);
    steppers[i]->setAcceleration(500.0);
    steppers[i]->setCurrentPosition(0);

    pinMode(STEP_PIN[i],OUTPUT);
    pinMode(DIR_PIN[i],OUTPUT);
    pinMode(EN_PIN[i],OUTPUT);
  
      
    digitalWrite(EN_PIN[i], LOW);

    setTunings(kp[i], ki[i], kd[i], i); 

    
  }
  pinMode (STOP_PIN , INPUT_PULLUP);
  pinMode (E_STOP_PIN , INPUT_PULLUP);
  pinMode (FAN_PIN, OUTPUT);
  
  RUN = true;
  E_STOP_PREV = true;

}

void loop(){
  STOP = digitalRead (STOP_PIN);
  E_STOP = digitalRead (E_STOP_PIN);

  recvWithStartEndMarkers();

  if(newData == true){
    strcpy(tempChars, receivedChars);
        // this temporary copy is necessary to protect the original data
        // because strtok() used in parseData() replaces the commas with \0
    parseData(0);
    // showParsedData();
    newData = false;
  }
  
  if(newCmdData == true){
    
    strcpy(tempCmdChars, receivedCmdChars);

    parseData(1);

    newCmdData = false;

    if(rcvd_cmd == 0){
      writeActualPosition();
    }

  }

  computePid(0);
  steppers[0]->setSpeed(u[0]);
  steppers[0]->runSpeed();
  // implement feedback position control 
  if(STOP == HIGH){
    RUN = false;
  }
  
  if(STOP == LOW & E_STOP == LOW & !E_STOP_PREV){
    RUN = true;
  }

  if (E_STOP == LOW){

    E_STOP_PREV = true;

    digitalWrite(EN_PIN[i], LOW);

    if(RUN == true){
      
      digitalWrite (13, HIGH);

  computePid(1);
  steppers[1]->setSpeed(u[1]);
  steppers[1]->runSpeed();

  computePid(2);
  steppers[2]->setSpeed(u[2]);
  steppers[2]->runSpeed();

  computePid(3);
  steppers[3]->setSpeed(u[3]);
  steppers[3]->runSpeed();

  computePid(4);
  steppers[4]->setSpeed(u[4]);
  steppers[4]->runSpeed();

  computePid(5);
  steppers[5]->setSpeed(u[5]);
  steppers[5]->runSpeed();

  
}
