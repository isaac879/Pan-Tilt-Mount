#include "PanTiltMount.h"
#include <Iibrary.h> //A library I created for Arduino that contains some simple functions I commonly use. Library available at: https://github.com/isaac879/Iibrary
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <EEPROM.h> //To be able to save values when powered off
#include <FastLED.h>

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

//Global scope
CRGB leds[NUM_LEDS];

AccelStepper stepper_pan = AccelStepper(1, PIN_STEP_PAN, PIN_DIRECTION_PAN);
AccelStepper stepper_tilt = AccelStepper(1, PIN_STEP_TILT, PIN_DIRECTION_TILT);
AccelStepper stepper_slider = AccelStepper(1, PIN_STEP_SLIDER, PIN_DIRECTION_SLIDER);

MultiStepper multi_stepper;

ArrayElement program_elements[ARRAY_LENGTH];

int moves_array_elements = 0;
int current_moves_array_index = -1;
char stringText[MAX_STRING_LENGTH + 1];
float pan_steps_per_degree = (200.0 * SIXTEENTH_STEP * PAN_GEAR_RATIO) / 360.0; //Stepper motor has 200 steps per 360 degrees
float tilt_steps_per_degree = (200.0 * SIXTEENTH_STEP * TILT_GEAR_RATIO) / 360.0; //Stepper motor has 200 steps per 360 degrees
float slider_steps_per_millimetre = (200.0 * SIXTEENTH_STEP) / (20 * 2); //Stepper motor has 200 steps per 360 degrees, the timing pully has 20 teeth and the belt has a pitch of 2mm

int step_mode = 1;
bool enable_state = true;
float limit_pan_min = 0;
float limit_pan_max = 0;
float limit_tilt_min = 0;
float limit_tilt_max = 0;
float limit_slider_min = 0;
float limit_slider_max = 0;
byte enable_limits = 0;
float pan_acceleration = 5000;
float tilt_acceleration = 5000;
float slider_acceleration = 5000;
float hall_pan_offset_degrees = 0;
float hall_tilt_offset_degrees = 0;
byte invert_pan = 0;
byte invert_tilt = 0;
byte invert_slider = 0;
byte enable_homing = 0;
float pan_max_speed = 2000;
float tilt_max_speed = 2000;
float slider_max_speed = 2000;
long target_position[3];
float degrees_per_picture = 0.5;
unsigned long delay_ms_between_pictures = 1000;

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void initPanTilt(void){
    Serial.begin(BAUD_RATE);
    pinMode(PIN_MS, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_DIRECTION_PAN, OUTPUT);
    pinMode(PIN_STEP_PAN, OUTPUT);
    pinMode(PIN_DIRECTION_TILT, OUTPUT);
    pinMode(PIN_STEP_TILT, OUTPUT);
    pinMode(PIN_DIRECTION_SLIDER, OUTPUT);
    pinMode(PIN_STEP_SLIDER, OUTPUT);
    pinMode(PIN_PAN_HALL, INPUT_PULLUP);
    pinMode(PIN_TILT_HALL, INPUT_PULLUP);
    pinMode(PIN_SHUTTER_TRIGGER, OUTPUT);
    digitalWrite(PIN_SHUTTER_TRIGGER, LOW);
    FastLED.addLeds<LED_TYPE, PIN_LED_DATA, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    LEDS.showColor(CHSV(160, 255, 255)); //Set led to blue
    setEEPROMVariables();
    stepper_pan.setMinPulseWidth(20);
    stepper_tilt.setMinPulseWidth(20);
    stepper_slider.setMinPulseWidth(20);
    setStepMode(step_mode); //steping mode
    stepper_pan.setMaxSpeed(pan_max_speed);
    stepper_pan.setAcceleration(pan_acceleration);
    stepper_tilt.setMaxSpeed(tilt_max_speed);
    stepper_tilt.setAcceleration(tilt_acceleration);
    stepper_slider.setMaxSpeed(slider_max_speed);
    stepper_slider.setAcceleration(slider_acceleration);
    invertPanDirection(invert_pan);
    invertTiltDirection(invert_tilt);
    invertSliderDirection(invert_slider);
    multi_stepper.addStepper(stepper_pan);
    multi_stepper.addStepper(stepper_tilt);
    multi_stepper.addStepper(stepper_slider);
    enableSteppers(true);
    LEDS.showColor(CHSV(96 , 255, 255));
    printi(F("Setup complete.\n"));
    if(enable_homing == 1){
        printi(F("Beginning homing...\n"));
        if(findHome()){
            printi(F("Homing complete.\n"));
            LEDS.showColor(CHSV(0 , 0, 255)); //White
            delay(200);
            LEDS.showColor(CHSV(0 , 0, 0)); //off
            delay(200);
            LEDS.showColor(CHSV(0 , 0, 255)); //White
            delay(200);
            LEDS.showColor(CHSV(0 , 0, 0)); //off
            delay(200);
            LEDS.showColor(CHSV(0 , 0, 255)); //White
            delay(200);
        }
        else{
            stepper_pan.setCurrentPosition(0);
            stepper_tilt.setCurrentPosition(0);
            printi(F("Error finding home position... Current position has been set as home.\n"));
            LEDS.showColor(CHSV(240 , 255, 255)); //pink
            delay(200);
            LEDS.showColor(CHSV(0 , 0, 0)); //off
            delay(200);
            LEDS.showColor(CHSV(240 , 255, 255)); //pink
            delay(200);
            LEDS.showColor(CHSV(0 , 0, 0)); //off
            delay(200);
            LEDS.showColor(CHSV(240 , 255, 255)); //pink
            delay(200);
        }
    }
    ledBatteryLevel(getBatteryPercentage()); 
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float boundFloat(float value, float lower, float upper){
    if(value < lower){
        value = lower;
    }
    else if(value > upper){
        value = upper;
    }
    return value;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void serialFlush(void){
    while(Serial.available() > 0){
        char c = Serial.read();
    }
} 

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void enableSteppers(bool state){
    if(state == true){
        digitalWrite(PIN_ENABLE, LOW); //Enable the stepper drivers
        printi(F("Motors enabled.\n"));
    }
    else{
        digitalWrite(PIN_ENABLE, HIGH); //Disabe the stepper drivers
        printi(F("Motors disabled.\n"));
    }
    enable_state = state;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setStepMode(int mode){
    
    if(mode == 0){//Full step mode
        if(step_mode == 1){//was in sixteenth step mode
            stepper_pan.setCurrentPosition(stepper_pan.currentPosition() / SIXTEENTH_STEP);
            stepper_tilt.setCurrentPosition(stepper_tilt.currentPosition() / SIXTEENTH_STEP);
            stepper_slider.setCurrentPosition(stepper_slider.currentPosition() / SIXTEENTH_STEP);
        }
        digitalWrite(PIN_MS, LOW);
        pan_steps_per_degree = (200.0 * FULL_STEP * PAN_GEAR_RATIO) / 360.0; //Stepper motor has 200 steps per 360 degrees
        tilt_steps_per_degree = (200.0 * FULL_STEP * TILT_GEAR_RATIO) / 360.0; //Stepper motor has 200 steps per 360 degrees
        slider_steps_per_millimetre = (200.0 * FULL_STEP) / (20 * 2);
        step_mode = 0;
        printi(F("Set to Full Step mode.\n"));
    }
    else if(mode == 1){//Sixteenth step mode
        if(step_mode == 0){//was in full step mode
            stepper_pan.setCurrentPosition(stepper_pan.currentPosition() * SIXTEENTH_STEP);
            stepper_tilt.setCurrentPosition(stepper_tilt.currentPosition() * SIXTEENTH_STEP);
            stepper_slider.setCurrentPosition(stepper_slider.currentPosition() * SIXTEENTH_STEP);
        }
        digitalWrite(PIN_MS, HIGH);
        pan_steps_per_degree = (200.0 * SIXTEENTH_STEP * PAN_GEAR_RATIO) / 360.0; //Stepper motor has 200 steps per 360 degrees
        tilt_steps_per_degree = (200.0 * SIXTEENTH_STEP * TILT_GEAR_RATIO) / 360.0; //Stepper motor has 200 steps per 360 degrees
        slider_steps_per_millimetre = (200.0 * SIXTEENTH_STEP) / (20 * 2);
        step_mode = 1;
        printi(F("Set to Sixteenth Step mode.\n"));
    }
    else{
        printi(F("Invalid step mode... Enter a 0 for full step or 1 for sixteenth step mode.\n"));
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void panJogDegrees(float jogAngle){
    target_position[0] = panDegreesToSteps(jogAngle);
    multi_stepper.moveTo(target_position);
}
/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void tiltJogDegrees(float jogAngle){
    target_position[1] = tiltDegreesToSteps(jogAngle);
    multi_stepper.moveTo(target_position);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float panDegreesToSteps(float angle){
    return pan_steps_per_degree * angle;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float tiltDegreesToSteps(float angle){
    return tilt_steps_per_degree * angle;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float sliderMillimetresToSteps(float mm){
    return mm * slider_steps_per_millimetre ;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float sliderStepsToMillimetres(long steps){
    return steps / slider_steps_per_millimetre;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void sliderMoveTo(float mm){
    target_position[2] = sliderMillimetresToSteps(mm);
    multi_stepper.moveTo(target_position);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void printProgramElements(void){
    printi(F("---Program Elements---\n"));
    for(int row = 0; row < moves_array_elements; row++){
        printi(F(""), row, F("\t|"));
        printi(F(" Pan: "), panStepsToDegrees(program_elements[row].panStepCount), 3, F("º\t"));
        printi(F("Tilt: "), tiltStepsToDegrees(program_elements[row].tiltStepCount), 3, F("º\t"));
        printi(F("Slider: "), sliderStepsToMillimetres(program_elements[row].sliderStepCount), 3, F("mm\t"));
        printi(F("Pan Speed: "), panStepsToDegrees(program_elements[row].panSpeed), 3, F(" º/s\t"));
        printi(F("Tilt Speed: "), tiltStepsToDegrees(program_elements[row].tiltSpeed), 3, F(" º/s\t"));  
        printi(F("Slider Speed: "), sliderStepsToMillimetres(program_elements[row].sliderSpeed), 3, F(" mm/s\t"));      
        printi(F("Delay: "), program_elements[row].msDelay, F("ms |\n"));    
    }
    printi(F("\n"));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void debugReport(void){
    printi(F("---Debug Report---\n"));
    printi(F("Motor enable state: "), enable_state);
//    printi(F("Step Mode: "), step_mode);
//    printi(F("Pan Hall sensor state: "), digitalRead(PIN_PAN_HALL));
//    printi(F("Tilt Hall sensor state: "), digitalRead(PIN_TILT_HALL));
//    printi(F("Pan step count: "), stepper_pan.currentPosition());
//    printi("Tilt step count: ", stepper_tilt.currentPosition());
//    printi("Slider step count: ", stepper_slider.currentPosition());
    printi(F("Pan angle: "), panStepsToDegrees(stepper_pan.currentPosition()), 3, F("º\n"));
    printi(F("Tilt angle: "), tiltStepsToDegrees(stepper_tilt.currentPosition()), 3, F("º\n")); 
    printi(F("Slider position: "), sliderStepsToMillimetres(stepper_slider.currentPosition()), 3, F("mm\n")); 
//    printi(F("Pan steps per º: "), pan_steps_per_degree);
//    printi(F("Tilt steps per º: "), tilt_steps_per_degree);
//    printi(F("Slider steps per mm: "), slider_steps_per_millimetre);
//    printi(F("Pan current steps/s: "), stepper_pan.speed());
//    printi(F("Tilt current steps/s: "), stepper_tilt.speed());
//    printi(F("Slider current steps/s: "), stepper_slider.speed());
//    printi(F("Pan current º/s: "), panStepsToDegrees(stepper_pan.speed()));
//    printi(F("Tilt current º/s: "), tiltStepsToDegrees(stepper_tilt.speed()));
//    printi(F("Slider current mm/s: "), sliderStepsToMillimetres(stepper_slider.speed()));  
//    printi(F("Pan maximum steps/s: "), stepper_pan.maxSpeed());
//    printi(F("Tilt maximum steps/s: "), stepper_tilt.maxSpeed());
//    printi(F("Slider maximum steps/s: "), stepper_slider.maxSpeed());
    printi(F("Pan max speed: "), panStepsToDegrees(stepper_pan.maxSpeed()), 3, F("º/s\n"));
    printi(F("Tilt max speed: "), tiltStepsToDegrees(stepper_tilt.maxSpeed()), 3, F("º/s\n"));
    printi(F("Slider max speed: "), sliderStepsToMillimetres(stepper_slider.maxSpeed()), 3, F("mm/s\n"));    
//    printi(F("Pan acceleration steps/s/s: "), pan_acceleration);
//    printi(F("Tilt acceleration steps/s/s: "), tilt_acceleration);
//    printi(F("Slider acceleration steps/s/s: "), slider_acceleration);   
//    printi(F("Pan acceleration º/s/s: "), panStepsToDegrees(pan_acceleration));
//    printi(F("Tilt acceleration º/s/s: "), tiltStepsToDegrees(tilt_acceleration));
//    printi(F("Slider acceleration mm/s/s: "), sliderStepsToMillimetres(slider_acceleration));    
//    printi(F("Pan min limit: "), limit_pan_min, 3, F("º\n"));
//    printi(F("Pan max limit: "), limit_pan_max, 3, F("º\n"));    
//    printi(F("Tilt min limit: "), limit_tilt_min, 3, F("º\n"));
//    printi(F("Tilt max limit: "), limit_tilt_max, 3, F("º\n"));
//    printi(F("Slider min limit: "), limit_slider_min, 3, F("mm\n"));
//    printi(F("Slider max limit: "), limit_slider_max, 3, F("mm\n"));
//    printi(F("Enable limits: "), enable_limits);    
//    printi(F("Pan invert direction: "), invert_pan);
//    printi(F("Tilt invert direction: "), invert_tilt);
//    printi(F("Slider invert direction: "), invert_slider);
//    printi(F("Pan Hall offset: "), hall_pan_offset_degrees, 3, F("º\n"));
//    printi(F("Tilt Hall offset: "), hall_tilt_offset_degrees, 3, F("º\n"));  
    printi(F("Battery voltage: "), getBatteryVoltage(), 3, F("V\n"));
    printi(F("Battery percentage: "), getBatteryPercentage(), 3, F("%\n"));
    printi(F("Homing on start-up: "), enable_homing);    
    printi(F("Angle between pictures: "), degrees_per_picture, 3, F("º\n"));
    printi(F("Panoramiclapse delay between pictures: "), delay_ms_between_pictures, F("ms\n"));   
    printi(F("Version: "));
    printi(F(VERSION_NUMBER));
    printi(F("\n"));
    printEEPROM();
    printProgramElements();
    printi(F("-----------\n"));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

int setTargetPositions(float panDeg, float tiltDeg){
    if(enable_limits == 1 && !((panDeg >= limit_pan_min && panDeg <= limit_pan_max) && (tiltDeg >= limit_tilt_min && tiltDeg <= limit_tilt_max))){
        return -1;
    }
    target_position[0] = panDegreesToSteps(panDeg);
    target_position[1] = tiltDegreesToSteps(tiltDeg);
    multi_stepper.moveTo(target_position); 
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

int setTargetPositions(float panDeg, float tiltDeg, float sliderMillimetre){
    if(enable_limits == 1 && !((panDeg >= limit_pan_min && panDeg <= limit_pan_max) && (tiltDeg >= limit_tilt_min && tiltDeg <= limit_tilt_max) 
    && (sliderMillimetre >= limit_slider_min && sliderMillimetre <= limit_slider_max))){
        return -1;
    }
    target_position[0] = panDegreesToSteps(panDeg);
    target_position[1] = tiltDegreesToSteps(tiltDeg);
    target_position[2] = sliderMillimetresToSteps(sliderMillimetre);
    multi_stepper.moveTo(target_position); 
}

///*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
//
//int setTargetPositionsSteps(long panSteps, long tiltSteps){//TODO: limits
//    target_position[0] = panSteps;
//    target_position[1] = tiltSteps;
//    multi_stepper.moveTo(target_position); 
//}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool findHome(void){
    if(enable_limits == 1){
        printi(F("Homing not available with limits enabled\n"));
        return false;
    }
    bool panHomeFlag = false;
    bool tiltHomeFlag = false;
    int panHomingDir = -1;
    int tiltHomingDir = -1; 
    
    setTargetPositions(0, 0);
    stepper_pan.setCurrentPosition(0);//set step count to 0
    stepper_tilt.setCurrentPosition(0);//set step count to 0


    while(digitalRead(PIN_PAN_HALL) == 0 || digitalRead(PIN_TILT_HALL) == 0){//If already on a Hall sensor move off
        target_position[0] = target_position[0] + panDegreesToSteps(!digitalRead(PIN_PAN_HALL));//increment by 1 degree
        target_position[1] = target_position[1] + tiltDegreesToSteps(!digitalRead(PIN_TILT_HALL));//increment by 1 degree
        if(target_position[0] > panDegreesToSteps(360) && target_position[1] > tiltDegreesToSteps(360)){//If both axis have done more than a full rotation there must be an issue...
            return false;
        }
        multi_stepper.moveTo(target_position); 
        multi_stepper.runSpeedToPosition();
    }
    stepper_pan.setCurrentPosition(0);//set step count to 0
    stepper_tilt.setCurrentPosition(0);//set step count to 0

    setTargetPositions(-45, -45);
    while(multi_stepper.run()){
        if(digitalRead(PIN_PAN_HALL) == 0){
            stepper_pan.setCurrentPosition(0);//set step count to 0
            setTargetPositions(0, -45 * !tiltHomeFlag);
            panHomeFlag = true;
            panHomingDir = 1;
        }
        if(digitalRead(PIN_TILT_HALL) == 0){
            stepper_tilt.setCurrentPosition(0);
            setTargetPositions(-45 * !panHomeFlag, 0);
            tiltHomeFlag = true;
            tiltHomingDir = 1;
        }
    }     

    setTargetPositions(45 * !panHomeFlag, 45 * !tiltHomeFlag);//set angle to 0 for an axis if it's home
    while(multi_stepper.run()){
        if(digitalRead(PIN_PAN_HALL) == 0){
            stepper_pan.setCurrentPosition(0);//set step count to 0
            setTargetPositions(0, 45);
            panHomeFlag = true;
        }
        if(digitalRead(PIN_TILT_HALL) == 0){
            stepper_tilt.setCurrentPosition(0);
            setTargetPositions(45 * !panHomeFlag, 0);
            tiltHomeFlag = true;
        }
    } 
    
    setTargetPositions(360 * !panHomeFlag, 360 * !tiltHomeFlag);//full rotation on both axis so it must pass the home position
    while(multi_stepper.run()){
        if(digitalRead(PIN_PAN_HALL) == 0){
            stepper_pan.setCurrentPosition(0);//set step count to 0
            setTargetPositions(0, 360 * !tiltHomeFlag);
            panHomeFlag = true;
        }
        if(digitalRead(PIN_TILT_HALL)  == 0){
            stepper_tilt.setCurrentPosition(0);
            setTargetPositions(360 * !panHomeFlag, 0);
            tiltHomeFlag = true;
        }
    } 
    if(panHomeFlag && tiltHomeFlag){
        setTargetPositions(hall_pan_offset_degrees * panHomingDir, hall_tilt_offset_degrees * tiltHomingDir);
        multi_stepper.runSpeedToPosition();
        stepper_pan.setCurrentPosition(0);//set step count to 0
        stepper_tilt.setCurrentPosition(0);//set step count to 0
        setTargetPositions(0, 0);
        return true;
    }
    else{
        return false;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float getBatteryVoltage(void){ //TODO: Calibrate the values for your battery
    return mapNumber(analogRead(PIN_INPUT_VOLTAGE), 0, 1007, 0, 12.6);//1007 = 12.6V
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float getBatteryPercentage(void){ //TODO: Calibrate the values for your battery
    return boundFloat(mapNumber(getBatteryVoltage(), 9, 12.6, 0, 100), 0, 100); //780 = 9V = 0%, 1023 = 12.6V = 100%
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float panStepsToDegrees(long steps){
    return steps / pan_steps_per_degree;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float panStepsToDegrees(float steps){
    return steps / pan_steps_per_degree;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float tiltStepsToDegrees(long steps){
    return steps / tilt_steps_per_degree;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float tiltStepsToDegrees(float steps){
    return steps / tilt_steps_per_degree;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

int addPosition(void){
    if(moves_array_elements >= 0 && moves_array_elements < ARRAY_LENGTH){
        program_elements[moves_array_elements].panStepCount = stepper_pan.currentPosition();
        program_elements[moves_array_elements].tiltStepCount = stepper_tilt.currentPosition();    
        program_elements[moves_array_elements].sliderStepCount = stepper_slider.currentPosition();    
        program_elements[moves_array_elements].panSpeed = stepper_pan.maxSpeed();
        program_elements[moves_array_elements].tiltSpeed = stepper_tilt.maxSpeed();    
        program_elements[moves_array_elements].sliderSpeed = stepper_slider.maxSpeed();            
        current_moves_array_index = moves_array_elements;
        moves_array_elements++;//increment the index
        printi(F("Position added at index: "), current_moves_array_index);
        return 0;
    }
    else{
        printi(F("Maximum number of position reached\n"));
    }
    return -1;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void clearArray(void){
    for(int row = 0; row < ARRAY_LENGTH; row++){
        program_elements[row].panStepCount = 0;
        program_elements[row].tiltStepCount = 0;
        program_elements[row].sliderStepCount = 0;
        program_elements[row].panSpeed = 0;
        program_elements[row].tiltSpeed = 0;
        program_elements[row].sliderSpeed = 0;
        program_elements[row].msDelay = 0;
    }
    moves_array_elements = 0;
    current_moves_array_index = -1;
    printi(F("All positions cleared."));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void moveToIndex(int index){
    if(index < moves_array_elements && index >= 0){
        target_position[0] = program_elements[index].panStepCount;
        target_position[1] = program_elements[index].tiltStepCount;
        target_position[2] = program_elements[index].sliderStepCount;
        stepper_pan.setMaxSpeed(program_elements[index].panSpeed);
        stepper_tilt.setMaxSpeed(program_elements[index].tiltSpeed);
        stepper_slider.setMaxSpeed(program_elements[index].sliderSpeed);
        multi_stepper.moveTo(target_position); //Sets new target positions
        multi_stepper.runSpeedToPosition(); //Moves and blocks until complete
        delay(program_elements[index].msDelay);
        current_moves_array_index = index;
//        printi(F("Moved to index: "), current_moves_array_index);
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void executeMoves(int repeat){
    for(int i = 0; i < repeat; i++){
        for(int row = 0; row < moves_array_elements; row++){
            moveToIndex(row);
        }
        ledBatteryLevel(getBatteryPercentage()); 
        if(getBatteryVoltage() < 9.5){//9.5V is used as the cut off to allow for inaccuracies and be on the safe side.
            delay(200);
            if(getBatteryVoltage() < 9.5){//Check voltage is still low and the first wasn't a miscellaneous reading
                printi(F("Battery low! Turn the power off."));
                while(1){}//loop and do nothing
            }
        }
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void gotoMovesArrayStart(void){
    moveToIndex(0);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void gotoMovesArrayEnd(void){
    moveToIndex(moves_array_elements - 1);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void editMovesArrayIndex(void){
    program_elements[current_moves_array_index].panStepCount = stepper_pan.currentPosition();
    program_elements[current_moves_array_index].tiltStepCount = stepper_tilt.currentPosition(); 
    program_elements[current_moves_array_index].sliderStepCount = stepper_slider.currentPosition(); 
    program_elements[current_moves_array_index].panSpeed = stepper_pan.maxSpeed();
    program_elements[current_moves_array_index].tiltSpeed = stepper_tilt.maxSpeed();
    program_elements[current_moves_array_index].sliderSpeed = stepper_slider.maxSpeed();
    
    printi(F("Edited at index: "), current_moves_array_index);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void addDelay(unsigned int ms){
    program_elements[current_moves_array_index].msDelay = ms;
    printi(ms, F(""));
    printi(F("ms delay added at index: "), current_moves_array_index);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void scaleMovesArrayPanMaxSpeed(float newMax){
    float currentMax = 0;
    for(int row = 0; row < moves_array_elements; row++){ //Find the maximum pan speed
        if(program_elements[row].panSpeed > currentMax){
            currentMax = program_elements[row].panSpeed;
        }  
    }
    float speedRatio = newMax / currentMax;
    for(int row = 0; row < moves_array_elements; row++){ //Scale all the pan speeds      
        program_elements[row].panSpeed = program_elements[row].panSpeed * speedRatio;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void scaleMovesArrayTiltMaxSpeed(float newMax){
    float currentMax = 0;
    for(int row = 0; row < moves_array_elements; row++){ //Find the maximum pan speed
        if(program_elements[row].tiltSpeed > currentMax){
            currentMax = program_elements[row].tiltSpeed;
        }  
    }
    float speedRatio = newMax / currentMax;
    for(int row = 0; row < moves_array_elements; row++){ //Scale all the pan speeds      
        program_elements[row].tiltSpeed = program_elements[row].tiltSpeed * speedRatio;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void scaleMovesArraySliderMaxSpeed(float newMax){
    float currentMax = 0;
    for(int row = 0; row < moves_array_elements; row++){ //Find the maximum pan speed
        if(program_elements[row].sliderSpeed > currentMax){
            currentMax = program_elements[row].sliderSpeed;
        }  
    }
    float speedRatio = newMax / currentMax;
    for(int row = 0; row < moves_array_elements; row++){ //Scale all the pan speeds      
        program_elements[row].sliderSpeed = program_elements[row].sliderSpeed * speedRatio;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void invertPanDirection(bool invert){
    printi(F("Pan inversion set to: "), invert);
    invert_pan = invert;
    stepper_pan.setPinsInverted(invert, false, false);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void invertTiltDirection(bool invert){
    printi(F("Tilt inversion set to: "), invert);
    invert_tilt = invert;
    stepper_tilt.setPinsInverted(invert, false, false);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void invertSliderDirection(bool invert){
    printi(F("Slider inversion set to: "), invert);
    invert_slider = invert;
    stepper_slider.setPinsInverted(invert, false, false);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void saveEEPROM(void){
    EEPROM.put(EEPROM_ADDRESS_ENABLE_HOMING, enable_homing);
    EEPROM.put(EEPROM_ADDRESS_MODE, step_mode);
//    EEPROM.put(EEPROM_ADDRESS_LIMIT_PAN_MIN, limit_pan_min);
//    EEPROM.put(EEPROM_ADDRESS_LIMIT_PAN_MAX, limit_pan_max);
//    EEPROM.put(EEPROM_ADDRESS_LIMIT_TILT_MIN, limit_tilt_min);
//    EEPROM.put(EEPROM_ADDRESS_LIMIT_TILT_MAX, limit_tilt_max);
    EEPROM.put(EEPROM_ADDRESS_PAN_MAX_SPEED, pan_max_speed);
    EEPROM.put(EEPROM_ADDRESS_TILT_MAX_SPEED, tilt_max_speed);
    EEPROM.put(EEPROM_ADDRESS_SLIDER_MAX_SPEED, slider_max_speed);
    EEPROM.put(EEPROM_ADDRESS_PAN_ACCELERATION, pan_acceleration);
    EEPROM.put(EEPROM_ADDRESS_TILT_ACCELERATION, tilt_acceleration);
    EEPROM.put(EEPROM_ADDRESS_SLIDER_ACCELERATION, slider_acceleration);  
    EEPROM.put(EEPROM_ADDRESS_HALL_PAN_OFFSET, hall_pan_offset_degrees);
    EEPROM.put(EEPROM_ADDRESS_HALL_TILT_OFFSET, hall_tilt_offset_degrees);
    EEPROM.put(EEPROM_ADDRESS_INVERT_PAN, invert_pan);
    EEPROM.put(EEPROM_ADDRESS_INVERT_TILT, invert_tilt);    
    EEPROM.put(EEPROM_ADDRESS_INVERT_SLIDER, invert_slider);      
    EEPROM.put(EEPROM_ADDRESS_DEGREES_PER_PICTURE, degrees_per_picture);
    EEPROM.put(EEPROM_ADDRESS_PANORAMICLAPSE_DELAY, delay_ms_between_pictures);
    EEPROM.put(EEPROM_ADDRESS_ENABLE_LIMITS, enable_limits);
    EEPROM.put(EEPROM_ADDRESS_PAN_MIN_LIMIT, limit_pan_min);
    EEPROM.put(EEPROM_ADDRESS_PAN_MAX_LIMIT, limit_pan_max);
    EEPROM.put(EEPROM_ADDRESS_TILT_MIN_LIMIT, limit_tilt_min);
    EEPROM.put(EEPROM_ADDRESS_TILT_MAX_LIMIT, limit_tilt_max);    
    EEPROM.put(EEPROM_ADDRESS_SLIDER_MIN_LIMIT, limit_slider_min);
    EEPROM.put(EEPROM_ADDRESS_SLIDER_MAX_LIMIT, limit_slider_max);    
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void printEEPROM(void){
    int itemp;
    float ftemp;
    long ltemp;
    printi(F("---Saved Values---\n"));
    EEPROM.get(EEPROM_ADDRESS_MODE, itemp);
    if(itemp == 0){
        printi(F("Step mode: Full step\n"));
    }
    else if(itemp == 1){
        printi(F("Step mode: Sixteenth step\n"));
    }
//    EEPROM.get(EEPROM_ADDRESS_LIMIT_PAN_MIN, ltemp);
//    printi(F("Pan min limit: "), ltemp);
//    EEPROM.get(EEPROM_ADDRESS_LIMIT_PAN_MAX, ltemp);
//    printi(F("Pan max limit: "), ltemp);
//    EEPROM.get(EEPROM_ADDRESS_LIMIT_TILT_MIN, ltemp);
//    printi(F("Tilt min limit: "), ltemp);
//    EEPROM.get(EEPROM_ADDRESS_LIMIT_TILT_MAX, ltemp);
//    printi(F("Tilt max limit: "), ltemp);
    EEPROM.get(EEPROM_ADDRESS_PAN_MAX_SPEED, ftemp);
    printi(F("Pan max speed: "), panStepsToDegrees(ftemp), 3, F("º/s\n"));
    EEPROM.get(EEPROM_ADDRESS_TILT_MAX_SPEED, ftemp);
    printi(F("Tilt max speed: "), tiltStepsToDegrees(ftemp), 3, F("º/s\n"));
    EEPROM.get(EEPROM_ADDRESS_SLIDER_MAX_SPEED, ftemp);
    printi(F("Slider max speed: "), sliderStepsToMillimetres(ftemp), 3, F("mm/s\n"));
//    EEPROM.get(EEPROM_ADDRESS_PAN_ACCELERATION, ftemp);
//    printi(F("Pan max acceleration: "), ftemp);
//    EEPROM.get(EEPROM_ADDRESS_TILT_ACCELERATION, ftemp);
//    printi(F("Tilt max acceleration: "), ftemp);
//    EEPROM.get(EEPROM_ADDRESS_SLIDER_ACCELERATION, ftemp);
//    printi(F("Slider max acceleration: "), ftemp); 
    EEPROM.get(EEPROM_ADDRESS_HALL_PAN_OFFSET, ftemp);
    printi(F("Pan Hall offset: "), ftemp, 3, F("º\n"));
    EEPROM.get(EEPROM_ADDRESS_HALL_TILT_OFFSET, ftemp);
    printi(F("Tilt Hall offset: "), ftemp, 3, F("º\n"));
    EEPROM.get(EEPROM_ADDRESS_DEGREES_PER_PICTURE, ftemp);
    printi(F("Angle between pictures: "), ftemp, 3, F(" º\n"));
    EEPROM.get(EEPROM_ADDRESS_PANORAMICLAPSE_DELAY, ltemp);
    printi(F("Delay between pictures: "), ltemp, F("ms\n"));   
    printi(F("Pan invert: "), EEPROM.read(EEPROM_ADDRESS_INVERT_PAN));
    printi(F("Tilt invert: "), EEPROM.read(EEPROM_ADDRESS_INVERT_TILT));
    printi(F("Slider invert: "), EEPROM.read(EEPROM_ADDRESS_INVERT_SLIDER)); 
    printi(F("Homing on start-up: "), EEPROM.read(EEPROM_ADDRESS_ENABLE_HOMING));
    printi(F("Enable limits: "), EEPROM.read(EEPROM_ADDRESS_ENABLE_LIMITS));
    EEPROM.get(EEPROM_ADDRESS_PAN_MIN_LIMIT, ftemp);
    printi(F("Pan min limit: "), ftemp, 3, F("º\n"));   
    EEPROM.get(EEPROM_ADDRESS_PAN_MAX_LIMIT, ftemp);
    printi(F("Pan max limit: "), ftemp, 3, F("º\n"));
    EEPROM.get(EEPROM_ADDRESS_TILT_MIN_LIMIT, ftemp);
    printi(F("Tilt min limit: "), ftemp, 3, F("º\n"));
    EEPROM.get(EEPROM_ADDRESS_TILT_MAX_LIMIT, ftemp); 
    printi(F("Tilt max limit: "), ftemp, 3, F("º\n")); 
    EEPROM.get(EEPROM_ADDRESS_SLIDER_MIN_LIMIT, ftemp);
    printi(F("Slider min limit: "), ftemp, 3, F("mm\n"));
    EEPROM.get(EEPROM_ADDRESS_SLIDER_MAX_LIMIT, ftemp); 
    printi(F("Slider max limit: "), ftemp, 3, F("mm\n"));  
    printi(F("\n"));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setEEPROMVariables(void){
    printi(F("Setting values from EEPROM...\n"));
    EEPROM.get(EEPROM_ADDRESS_MODE, step_mode);
//    EEPROM.get(EEPROM_ADDRESS_LIMIT_PAN_MIN, limit_pan_min);
//    EEPROM.get(EEPROM_ADDRESS_LIMIT_PAN_MAX, limit_pan_max);
//    EEPROM.get(EEPROM_ADDRESS_LIMIT_TILT_MIN, limit_tilt_min);
//    EEPROM.get(EEPROM_ADDRESS_LIMIT_TILT_MAX, limit_tilt_max);
    EEPROM.get(EEPROM_ADDRESS_PAN_MAX_SPEED, pan_max_speed);
    EEPROM.get(EEPROM_ADDRESS_TILT_MAX_SPEED, tilt_max_speed);
    EEPROM.get(EEPROM_ADDRESS_SLIDER_MAX_SPEED, slider_max_speed);
    EEPROM.get(EEPROM_ADDRESS_PAN_ACCELERATION, pan_acceleration);
    EEPROM.get(EEPROM_ADDRESS_TILT_ACCELERATION, tilt_acceleration);
    EEPROM.get(EEPROM_ADDRESS_SLIDER_ACCELERATION, slider_acceleration);
    EEPROM.get(EEPROM_ADDRESS_HALL_PAN_OFFSET, hall_pan_offset_degrees);
    EEPROM.get(EEPROM_ADDRESS_HALL_TILT_OFFSET, hall_tilt_offset_degrees);
    EEPROM.get(EEPROM_ADDRESS_DEGREES_PER_PICTURE, degrees_per_picture);
    EEPROM.get(EEPROM_ADDRESS_PANORAMICLAPSE_DELAY, delay_ms_between_pictures);
    EEPROM.get(EEPROM_ADDRESS_ENABLE_LIMITS, enable_limits);
    EEPROM.get(EEPROM_ADDRESS_PAN_MIN_LIMIT, limit_pan_min);
    EEPROM.get(EEPROM_ADDRESS_PAN_MAX_LIMIT, limit_pan_max);
    EEPROM.get(EEPROM_ADDRESS_TILT_MIN_LIMIT, limit_tilt_min);
    EEPROM.get(EEPROM_ADDRESS_TILT_MAX_LIMIT, limit_tilt_max);   
    EEPROM.get(EEPROM_ADDRESS_SLIDER_MIN_LIMIT, limit_slider_min);
    EEPROM.get(EEPROM_ADDRESS_SLIDER_MAX_LIMIT, limit_slider_max);       
    invert_pan = EEPROM.read(EEPROM_ADDRESS_INVERT_PAN);
    invert_tilt = EEPROM.read(EEPROM_ADDRESS_INVERT_TILT);
    invert_slider = EEPROM.read(EEPROM_ADDRESS_INVERT_SLIDER);
    enable_homing = EEPROM.read(EEPROM_ADDRESS_ENABLE_HOMING);
    printi(F("Values set.\n"));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void toggleAutoHoming(void){
    if(enable_homing == 0){
        enable_homing = 1;
        printi(F("Homing enabled.\n"));
    }
    else{
        enable_homing = 0;
        printi(F("Homing disabled.\n"));
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void toggleEnableLimits(void){
    if(enable_limits == 1){
        enable_limits = 0;
        printi(F("Limits disabled.\n"));
    }
    else{
        enable_limits = 1;
        printi(F("Limits enabled.\n"));
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void triggerCameraShutter(void){
    digitalWrite(PIN_SHUTTER_TRIGGER, HIGH);
    delay(SHUTTER_DELAY);
    digitalWrite(PIN_SHUTTER_TRIGGER, LOW);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void ledBatteryLevel(float batteryPercentage){
    byte hue = mapNumber(batteryPercentage, 0, 100, 0, 96);
    LEDS.showColor(CHSV(hue , 255, 255));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void panoramiclapseInterpolation(float panStartAngle, float tiltStartAngle, float sliderStartPos, float panStopAngle, float tiltStopAngle, float sliderStopPos, float degPerPic, unsigned long msDelay){
    if(degPerPic == 0) return;
    if(msDelay > SHUTTER_DELAY){
        msDelay = msDelay - SHUTTER_DELAY;
    }
    float panAngle = panStopAngle - panStartAngle;
    float tiltAngle = tiltStopAngle - tiltStartAngle;
    float sliderDistance = sliderStopPos - sliderStartPos;
    float largestAngle = (abs(panAngle) > abs(tiltAngle)) ? panAngle : tiltAngle;
    unsigned int numberOfIncrements = abs(largestAngle) / degPerPic;
    if(numberOfIncrements == 0) return;
    float panInc = panAngle / numberOfIncrements;
    float tiltInc = tiltAngle / numberOfIncrements;
    float sliderInc = sliderDistance / numberOfIncrements;
    
    for(int i = 0; i <= numberOfIncrements; i++){
        setTargetPositions(panStartAngle + (panInc * i), tiltStartAngle + (tiltInc * i), sliderStartPos + (sliderInc * i));
        multi_stepper.runSpeedToPosition();//blocking move to the next position
        delay(msDelay);
        LEDS.showColor(CHSV(160 , 255, 255)); //blue
        triggerCameraShutter();//capture the picture
        LEDS.showColor(CHSV(160 , 255, 0)); //off
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void panoramiclapse(float degPerPic, unsigned long msDelay, int repeat){   
    if(moves_array_elements < 2){ 
        printi(F("Not enough positions recorded\n"));
        return; //check there are posions to move to
    }
    for(int i = 0; i < repeat; i++){
        for(int index = 0; index < moves_array_elements - 1; index++){
            panoramiclapseInterpolation(panStepsToDegrees(program_elements[index].panStepCount), tiltStepsToDegrees(program_elements[index].tiltStepCount), sliderStepsToMillimetres(program_elements[index].sliderStepCount),
            panStepsToDegrees(program_elements[index + 1].panStepCount), tiltStepsToDegrees(program_elements[index + 1].tiltStepCount), sliderStepsToMillimetres(program_elements[index + 1].sliderStepCount), degPerPic, msDelay);
        }
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void timelapse(unsigned int numberOfPictures, unsigned long msDelay){
    if(msDelay > SHUTTER_DELAY){
        msDelay = msDelay - SHUTTER_DELAY;
    }
   
    float panAngle = 0;
    float tiltAngle = 0;
    float sliderTravel = 0;
    
    if(moves_array_elements >= 2){ 
        panAngle = panStepsToDegrees(program_elements[1].panStepCount) - panStepsToDegrees(program_elements[0].panStepCount);
        tiltAngle = tiltStepsToDegrees(program_elements[1].tiltStepCount) - tiltStepsToDegrees(program_elements[0].tiltStepCount);
        sliderTravel = sliderStepsToMillimetres(program_elements[1].sliderStepCount) - sliderStepsToMillimetres(program_elements[0].sliderStepCount);
    }
    
    float sliderInc = sliderTravel / numberOfPictures;
    float panInc = panAngle / numberOfPictures;
    float tiltInc = tiltAngle / numberOfPictures;
    
    for(int i = 0; i <= numberOfPictures; i++){
        setTargetPositions(panStepsToDegrees(stepper_pan.currentPosition()) + (panInc * i), tiltStepsToDegrees(stepper_tilt.currentPosition()) + (tiltInc * i), sliderStepsToMillimetres(stepper_slider.currentPosition()) + (sliderInc * i));
        multi_stepper.runSpeedToPosition();//blocking move to the next position
        delay(msDelay);
        LEDS.showColor(CHSV(160 , 255, 255)); //blue
        triggerCameraShutter();//capture the picture
        LEDS.showColor(CHSV(160 , 255, 0)); //off
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void serialData(void){
    char instruction = Serial.read();
    if(instruction == INSTRUCTION_BYTES_PAN_SPEED){
        int count = 0;
        while(Serial.available() < 2){//Wait for six bytes to be available. Breaks after ~200ms if bytes are not received.
                delayMicroseconds(200); 
                count++;
                if(count > 1000){
                    serialFlush();//Clear the serial buffer
                    break;   
                }
            }
            int panStepSpeed = (Serial.read() << 8) + Serial.read(); 
            stepper_pan.setSpeed(panStepSpeed);
            stepper_pan.runSpeed();
    }
    
    delay(2); //wait to make sure all data in the serial message has arived
    ledBatteryLevel(getBatteryPercentage()); 
    memset(&stringText[0], 0, sizeof(stringText)); //clear the array
    while(Serial.available()){//set elemetns of stringText to the serial values sent
        char digit = Serial.read(); //read in a char
        strncat(stringText, &digit, 1); //add digit to the end of the array
    }
    serialFlush();//Clear any excess data in the serial buffer
    int serialCommandValueInt = atoi(stringText); //converts stringText to an int
    float serialCommandValueFloat = atof(stringText); //converts stringText to a float
    if(instruction == 'G' && stringText[0] == 0) return;//For the bluetooth issue of it sending data when it connects...
    switch(instruction){
        case INSTRUCTION_SLIDER_MIN_LIMIT:{
            limit_slider_min = serialCommandValueFloat;
            printi(F("Slider min limit set to: "), limit_slider_min, 3, F("mm\n"));
        }
        break;
        case INSTRUCTION_SLIDER_MAX_LIMIT:{
            limit_slider_max = serialCommandValueFloat;
            printi(F("Slider max limit set to: "), limit_slider_max, 3, F("mm\n"));
        }
        break;
        
        case INSTRUCTION_SLIDER_MILLIMETRES:{
            sliderMoveTo(serialCommandValueFloat);
        }
        break;
        case INSTRUCTION_DELAY_BETWEEN_PICTURES:{
            delay_ms_between_pictures = serialCommandValueFloat;
            printi(F("Delay between pictures: "), delay_ms_between_pictures, F("ms\n"));
        }
        break;
        case INSTRUCTION_ANGLE_BETWEEN_PICTURES:{
            degrees_per_picture = serialCommandValueFloat;
            printi(F("Degrees per picture: "), degrees_per_picture, 3, F("º\n"));
        }
        break;     
        case INSTRUCTION_ENABLE_LIMITS:{
            toggleEnableLimits();
        }
        break;
        case INSTRUCTION_PAN_MIN_LIMIT:{
            limit_pan_min = serialCommandValueFloat;
            printi(F("Pan min limit set to: "), limit_pan_min, 3, F("º\n"));
        }
        break;
        case INSTRUCTION_PAN_MAX_LIMIT:{
            limit_pan_max = serialCommandValueFloat;
            printi(F("Pan max limit set to: "), limit_pan_max, 3, F("º\n"));
        }
        break;
        case INSTRUCTION_TILT_MIN_LIMIT:{
            limit_tilt_min = serialCommandValueFloat;
            printi(F("Tilt min limit set to: "), limit_tilt_min, 3, F("º\n"));
        }
        break;
        case INSTRUCTION_TILT_MAX_LIMIT:{
            limit_tilt_max = serialCommandValueFloat;
            printi(F("Tilt max limit set to: "), limit_tilt_max, 3, F("º\n"));
        }
        break;
        case INSTRUCTION_PANORAMICLAPSE:{
            printi(F("Starting panorama.\n"));
            panoramiclapse(degrees_per_picture, delay_ms_between_pictures, 1);
            printi(F("Finished\n"));
        }
        break;
        case INSTRUCTION_TIMELAPSE:{
            printi(F("Starting timelapse with "), serialCommandValueInt, F(" pics\n"));
            printi(F(""), delay_ms_between_pictures, F("ms between pics\n"));
            timelapse(serialCommandValueInt, delay_ms_between_pictures);
            printi(F("Finished\n"));
        }
        break;
        case INSTRUCTION_SCALE_PAN_SPEED:{
            scaleMovesArrayPanMaxSpeed(panDegreesToSteps(serialCommandValueFloat));
        }
        break;
        case INSTRUCTION_SCALE_TILT_SPEED:{
            scaleMovesArrayTiltMaxSpeed(tiltDegreesToSteps(serialCommandValueFloat));
        }
        break;
        case INSTRUCTION_TRIGGER_SHUTTER:{
            triggerCameraShutter();
        }
        break;
        case INSTRUCTION_AUTO_HOME:{
            printi(F("Beginning homing.\n"));
            if(findHome()){
                printi(F("Homing complete.\n"));
            }
            else{
                stepper_pan.setCurrentPosition(0);
                stepper_tilt.setCurrentPosition(0);
                setTargetPositions(0, 0);
                printi(F("Error homing. Current position has been set as home.\n"));
            }
        }
        break;
        case INSTRUCTION_TOGGLE_HOMING:{
            toggleAutoHoming();
        }
        break;
        case INSTRUCTION_SET_PAN_HALL_OFFSET:{
            hall_pan_offset_degrees = serialCommandValueFloat;
        }
        break;
        case INSTRUCTION_SET_TILT_HALL_OFFSET:{
            hall_tilt_offset_degrees = serialCommandValueFloat;
        }
        break;
         case INSTRUCTION_INVERT_SLIDER:{
            invertSliderDirection(serialCommandValueInt);
        }
        break;
        case INSTRUCTION_INVERT_TILT:{
            invertTiltDirection(serialCommandValueInt);
        }
        break;
        case INSTRUCTION_INVERT_PAN:{
            invertPanDirection(serialCommandValueInt);
        }
        break;
        case INSTRUCTION_SAVE_TO_EEPROM:{
            printi(F("Saving values to EEPROM...\n"));
            saveEEPROM();
//            printi(F("Saved.\n"));
        }
        break;
        case INSTRUCTION_ADD_POSITION:{
            addPosition();
        }
        break;
        case INSTRUCTION_STEP_FORWARD:{
            moveToIndex(current_moves_array_index + 1);
        }
        break;
        case INSTRUCTION_STEP_BACKWARD:{
            moveToIndex(current_moves_array_index - 1);
        }
        break;
        case INSTRUCTION_JUMP_TO_START:{
            gotoMovesArrayStart();
        }
        break;
        case INSTRUCTION_JUMP_TO_END:{
            gotoMovesArrayEnd();
        }
        break;
        case INSTRUCTION_EDIT_ARRAY:{
            editMovesArrayIndex();
        }
        break;
        case INSTRUCTION_ADD_DELAY:{
            addDelay(serialCommandValueInt);
        }
        break;
        case INSTRUCTION_CLEAR_ARRAY:{
            clearArray();
        }
        break;
        case INSTRUCTION_EXECUTE_MOVES:{
            executeMoves(serialCommandValueInt);
        }
        break;      
        case INSTRUCTION_PAN_RUN_SPEED:{
            stepper_pan.setSpeed(panDegreesToSteps(serialCommandValueFloat));
            stepper_pan.runSpeed();
        }
        break;
//        case INSTRUCTION_TILT_RUN_SPEED:{
//            stepper_tilt.setSpeed(tiltDegreesToSteps(serialCommandValueFloat));
//            stepper_tilt.runSpeed();
//        }
//        break;
        case INSTRUCTION_DEBUG_STATUS:{
            debugReport();
        }
        break;
        case INSTRUCTION_PAN_STEPS:{
            target_position[0] = serialCommandValueInt;
            multi_stepper.moveTo(target_position);
        }
        break;
        case INSTRUCTION_PAN_DEGREES:{
            panJogDegrees(serialCommandValueFloat);
        }
        break;  
        case INSTRUCTION_TILT_STEPS:{
            target_position[1] = serialCommandValueInt;
            multi_stepper.moveTo(target_position);
        }
        break; 
        case INSTRUCTION_TILT_DEGREES:{
            tiltJogDegrees(serialCommandValueFloat);
        }
        break; 
        case INSTRUCTION_SET_HOME:{
            stepper_pan.setCurrentPosition(0);
            stepper_tilt.setCurrentPosition(0);
            stepper_slider.setCurrentPosition(0);
        }
        break; 
        case INSTRUCTION_ENABLE:{
            enableSteppers(serialCommandValueInt);
        }
        break; 
        case INSTRUCTION_STEP_MODE:{
            setStepMode(serialCommandValueInt);
        }
        break; 
        case INSTRUCTION_SET_ACCELLERATION:{
            printi("Setting acceleration to ", serialCommandValueFloat, 1, " steps/s/s.\n");
            pan_acceleration = serialCommandValueFloat;
            tilt_acceleration = serialCommandValueFloat;
            slider_acceleration = serialCommandValueFloat;
            stepper_pan.setAcceleration(pan_acceleration);
            stepper_tilt.setAcceleration(tilt_acceleration);
            stepper_slider.setAcceleration(slider_acceleration);
        }
        break; 
        case INSTRUCTION_SET_PAN_SPEED:{
            printi("Setting maximum pan speed to ", serialCommandValueFloat, 1, " º/s.\n");
            pan_max_speed = panDegreesToSteps(serialCommandValueFloat);
            stepper_pan.setMaxSpeed(pan_max_speed);
        }
        break; 
        case INSTRUCTION_SET_TILT_SPEED:{
            printi("Setting maximum tilt speed to ", serialCommandValueFloat, 1, " º/s.\n");
            tilt_max_speed = tiltDegreesToSteps(serialCommandValueFloat);
            stepper_tilt.setMaxSpeed(tilt_max_speed);
        }
        break;
        case INSTRUCTION_SET_SLIDER_SPEED:{
            printi("Setting maximum slider speed to ", serialCommandValueFloat, 1, " mm/s.\n");
            slider_max_speed = sliderMillimetresToSteps(serialCommandValueFloat);
            stepper_slider.setMaxSpeed(slider_max_speed);
        }
        break; 
//        case INSTRUCTION_MULTISTEPPER_TEST_2:{            
//            target_position[0] = panDegreesToSteps(45);
//            target_position[1] = tiltDegreesToSteps(180);
//            multi_stepper.moveTo(target_position);
//        }
//        break; 
//        case INSTRUCTION_MULTISTEPPER_TEST_3:{            
//            target_position[0] = panDegreesToSteps(-45);
//            target_position[1] = tiltDegreesToSteps(-180);
//            multi_stepper.moveTo(target_position);
//        }
//        break; 
//        case COMMAND_CARTESIAN:{//Moves relitive to the current position
//            while(Serial.available() != 6){//Wait for six bytes to be available. Breaks after ~200ms if bytes are not received.
//                delayMicroseconds(200); 
//                count++;
//                if(count > 1000){
//                    serialFlush();//Clear the serial buffer
//                    break;   
//                }
//            }
//            int xTarget = (Serial.read() << 8) + Serial.read();  
//            int yTarget = (Serial.read() << 8) + Serial.read(); 
//            int zTarget = (Serial.read() << 8) + Serial.read(); 
//         
//            inverse_kinematics(end_effector.x + xTarget, end_effector.y + yTarget, end_effector.z + zTarget);//Calculates servo positions 
//            move_servos();//Moves the robot's servos
//        }
//        break;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void mainLoop(void){
    while(1){
        if(Serial.available()) serialData();
        multi_stepper.run();
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
