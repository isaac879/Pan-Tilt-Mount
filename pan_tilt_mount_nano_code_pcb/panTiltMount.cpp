#include "PanTiltMount.h"
#include <Iibrary.h> //A library I created for Arduino that contains some simple functions I commonly use. Library available at: https://github.com/isaac879/Iibrary
#include <AccelStepper.h> //Library to control the stepper motors http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
#include <MultiStepper.h> //Library to control multiple coordinated stepper motors http://www.airspayce.com/mikem/arduino/AccelStepper/classMultiStepper.html#details
#include <EEPROM.h> //To be able to save values when powered off
#include <FastLED.h> //Controls the WS2812B Addressable status LED

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

//Global scope
CRGB leds[NUM_LEDS];

AccelStepper stepper_pan = AccelStepper(1, PIN_STEP_PAN, PIN_DIRECTION_PAN);
AccelStepper stepper_tilt = AccelStepper(1, PIN_STEP_TILT, PIN_DIRECTION_TILT);
AccelStepper stepper_slider = AccelStepper(1, PIN_STEP_SLIDER, PIN_DIRECTION_SLIDER);

MultiStepper multi_stepper;

KeyframeElement keyframe_array[KEYFRAME_ARRAY_LENGTH];

int keyframe_elements = 0;
int current_keyframe_index = -1;
char stringText[MAX_STRING_LENGTH + 1];
float pan_steps_per_degree = (200.0 * SIXTEENTH_STEP * PAN_GEAR_RATIO) / 360.0; //Stepper motor has 200 steps per 360 degrees
float tilt_steps_per_degree = (200.0 * SIXTEENTH_STEP * TILT_GEAR_RATIO) / 360.0; //Stepper motor has 200 steps per 360 degrees
float slider_steps_per_millimetre = (200.0 * SIXTEENTH_STEP) / (20 * 2); //Stepper motor has 200 steps per 360 degrees, the timing pully has 20 teeth and the belt has a pitch of 2mm

int step_mode = SIXTEENTH_STEP;
bool enable_state = true;
float hall_pan_offset_degrees = 0;
float hall_tilt_offset_degrees = 0;
byte invert_pan = 0;
byte invert_tilt = 0;
byte invert_slider = 0;
byte enable_homing = 0;
float pan_max_speed = 15; //degrees/second
float tilt_max_speed = 45; //degrees/second
float slider_max_speed = 15; //mm/second
long target_position[3];
float degrees_per_picture = 0.5;
unsigned long delay_ms_between_pictures = 1000;
FloatCoordinate intercept;

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void initPanTilt(void){
    Serial.begin(BAUD_RATE);
    pinMode(PIN_MS1, OUTPUT);
    pinMode(PIN_MS2, OUTPUT);
    pinMode(PIN_MS3, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);
    pinMode(PIN_DIRECTION_PAN, OUTPUT);
    pinMode(PIN_STEP_PAN, OUTPUT);
    pinMode(PIN_DIRECTION_TILT, OUTPUT);
    pinMode(PIN_STEP_TILT, OUTPUT);
    pinMode(PIN_DIRECTION_SLIDER, OUTPUT);
    pinMode(PIN_STEP_SLIDER, OUTPUT);
    pinMode(PIN_PAN_HALL, INPUT_PULLUP);
    pinMode(PIN_TILT_HALL, INPUT_PULLUP);
    pinMode(PIN_SLIDER_HALL, INPUT_PULLUP);
    pinMode(PIN_SHUTTER_TRIGGER, OUTPUT);
    digitalWrite(PIN_SHUTTER_TRIGGER, LOW);
    FastLED.addLeds<LED_TYPE, PIN_LED_DATA, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    LEDS.showColor(CHSV(160, 255, 255)); //Set led to blue
    setEEPROMVariables();
    setStepMode(step_mode); //steping mode
    stepper_pan.setMaxSpeed(panDegreesToSteps(pan_max_speed));
    stepper_tilt.setMaxSpeed(tiltDegreesToSteps(tilt_max_speed));
    stepper_slider.setMaxSpeed(sliderMillimetresToSteps(slider_max_speed));
    invertPanDirection(invert_pan);
    invertTiltDirection(invert_tilt);
    invertSliderDirection(invert_slider);
    multi_stepper.addStepper(stepper_pan);
    multi_stepper.addStepper(stepper_tilt);
    multi_stepper.addStepper(stepper_slider);
    digitalWrite(PIN_ENABLE, LOW); //Enable the stepper drivers
    enable_state = true;
    //printi(F("Setup complete.\n"));
    if(enable_homing == 1){
        printi(F("Beginning homing...\n"));
        if(findHome()){
            printi(F("Homing complete.\n"));
        }
        else{
            stepper_pan.setCurrentPosition(0);
            stepper_tilt.setCurrentPosition(0);
            printi(F("Error finding home position. Current position has been set as home.\n"));
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

void enableSteppers(void){
    if(enable_state == false){
        digitalWrite(PIN_ENABLE, LOW); //Enable the stepper drivers
        enable_state = true;
        printi(F("Motors enabled.\n"));
    }
    else{
        digitalWrite(PIN_ENABLE, HIGH); //Disabe the stepper drivers
        enable_state = false;
        printi(F("Motors disabled.\n"));
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setStepMode(int newMode){
    float stepRatio = newMode / step_mode; //Ratio between the new step mode and the previously set one. 
    if(newMode == FULL_STEP){
        digitalWrite(PIN_MS1, LOW);
        digitalWrite(PIN_MS2, LOW);
        digitalWrite(PIN_MS3, LOW);    
    }
    else if(newMode == HALF_STEP){
        digitalWrite(PIN_MS1, HIGH);
        digitalWrite(PIN_MS2, LOW);
        digitalWrite(PIN_MS3, LOW);    
    }
    else if(newMode == QUARTER_STEP){
        digitalWrite(PIN_MS1, LOW);
        digitalWrite(PIN_MS2, HIGH);
        digitalWrite(PIN_MS3, LOW); 
    }
    else if(newMode == EIGHTH_STEP){
        digitalWrite(PIN_MS1, HIGH);
        digitalWrite(PIN_MS2, HIGH);
        digitalWrite(PIN_MS3, LOW); 
    }
    else if(newMode == SIXTEENTH_STEP){
        digitalWrite(PIN_MS1, HIGH);
        digitalWrite(PIN_MS2, HIGH);
        digitalWrite(PIN_MS3, HIGH); 
    }
    else{ //If an invalid step mode was entered.
        printi(F("Invalid step mode... Enter a 1, 2, 4, 8, 16 for corresponding step mode.\n"));
        return;
    }
    //Scale current step to match the new step mode
    stepper_pan.setCurrentPosition(stepper_pan.currentPosition() * stepRatio);
    stepper_tilt.setCurrentPosition(stepper_tilt.currentPosition() * stepRatio);
    stepper_slider.setCurrentPosition(stepper_slider.currentPosition() * stepRatio);

    pan_steps_per_degree = (200.0 * newMode * PAN_GEAR_RATIO) / 360.0; //Stepper motor has 200 steps per 360 degrees
    tilt_steps_per_degree = (200.0 * newMode * TILT_GEAR_RATIO) / 360.0; //Stepper motor has 200 steps per 360 degrees
    slider_steps_per_millimetre = (200.0 * newMode) / (20 * 2); //Stepper motor has 200 steps per 360 degrees, the timing pully has 20 teeth and the belt has a pitch of 2mm

    stepper_pan.setMaxSpeed(panDegreesToSteps(pan_max_speed));
    stepper_tilt.setMaxSpeed(tiltDegreesToSteps(tilt_max_speed));
    stepper_slider.setMaxSpeed(sliderMillimetresToSteps(slider_max_speed));
    step_mode = newMode;
    printi(F("Set to "), step_mode, F(" step mode.\n"));
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

long sliderMillimetresToSteps(float mm){
    return mm * slider_steps_per_millimetre;
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

void printKeyframeElements(void){
    printi(F("---Keyframes---\n"));
    printi(F("Keyframe index: "), current_keyframe_index, F("\n"));
    for(int row = 0; row < keyframe_elements; row++){
        printi(F(""), row, F("\t|"));
        printi(F(" Pan: "), panStepsToDegrees(keyframe_array[row].panStepCount), 3, F("º\t"));
        printi(F("Tilt: "), tiltStepsToDegrees(keyframe_array[row].tiltStepCount), 3, F("º\t"));
        printi(F("Slider: "), sliderStepsToMillimetres(keyframe_array[row].sliderStepCount), 3, F("mm\t"));
        printi(F("Pan Speed: "), panStepsToDegrees(keyframe_array[row].panSpeed), 3, F(" º/s\t"));
        printi(F("Tilt Speed: "), tiltStepsToDegrees(keyframe_array[row].tiltSpeed), 3, F(" º/s\t"));  
        printi(F("Slider Speed: "), sliderStepsToMillimetres(keyframe_array[row].sliderSpeed), 3, F(" mm/s\t"));      
        printi(F("Delay: "), keyframe_array[row].msDelay, F("ms |\n"));    
    }
    printi(F("\n"));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void debugReport(void){
    printi(F("---Status---\n"));
    printi(F("Enable state: "), enable_state);
    printi(F("Step Mode: "), step_mode);
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
//    printi(F("Battery voltage: "), getBatteryVoltage(), 3, F("V\n"));
    printi(F("Battery percentage: "), getBatteryPercentage(), 3, F("%\n"));
    printi(F("Homing on start-up: "), enable_homing);    
    printi(F("Angle between pictures: "), degrees_per_picture, 3, F("º\n"));
    printi(F("Panoramiclapse delay between pictures: "), delay_ms_between_pictures, F("ms\n"));   
    printi(F("Version: "));
    printi(F(VERSION_NUMBER));
    printi(F("\n"));
    printEEPROM();
    printKeyframeElements();
    printi(F("-------\n"));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

int setTargetPositions(float panDeg, float tiltDeg){
    target_position[0] = panDegreesToSteps(panDeg);
    target_position[1] = tiltDegreesToSteps(tiltDeg);
    multi_stepper.moveTo(target_position); 
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

int setTargetPositions(float panDeg, float tiltDeg, float sliderMillimetre){
    target_position[0] = panDegreesToSteps(panDeg);
    target_position[1] = tiltDegreesToSteps(tiltDeg);
    target_position[2] = sliderMillimetresToSteps(sliderMillimetre);
    multi_stepper.moveTo(target_position); 
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool findHome(void){
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
    if(keyframe_elements >= 0 && keyframe_elements < KEYFRAME_ARRAY_LENGTH){
        keyframe_array[keyframe_elements].panStepCount = stepper_pan.currentPosition();
        keyframe_array[keyframe_elements].tiltStepCount = stepper_tilt.currentPosition();    
        keyframe_array[keyframe_elements].sliderStepCount = stepper_slider.currentPosition();    
        keyframe_array[keyframe_elements].panSpeed = stepper_pan.maxSpeed();
        keyframe_array[keyframe_elements].tiltSpeed = stepper_tilt.maxSpeed();    
        keyframe_array[keyframe_elements].sliderSpeed = stepper_slider.maxSpeed();            
        current_keyframe_index = keyframe_elements;
        keyframe_elements++;//increment the index
        printi(F("Position added at index: "), current_keyframe_index);
        return 0;
    }
    else{
        printi(F("Max number of position reached\n"));
    }
    return -1;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void clearKeyframes(void){
    for(int row = 0; row < KEYFRAME_ARRAY_LENGTH; row++){
        keyframe_array[row].panStepCount = 0;
        keyframe_array[row].tiltStepCount = 0;
        keyframe_array[row].sliderStepCount = 0;
        keyframe_array[row].panSpeed = 0;
        keyframe_array[row].tiltSpeed = 0;
        keyframe_array[row].sliderSpeed = 0;
        keyframe_array[row].msDelay = 0;
    }
    keyframe_elements = 0;
    current_keyframe_index = -1;
    printi(F("Keyframes cleared.\n"));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void moveToIndex(int index){
    if(index < keyframe_elements && index >= 0){
        target_position[0] = keyframe_array[index].panStepCount;
        target_position[1] = keyframe_array[index].tiltStepCount;
        target_position[2] = keyframe_array[index].sliderStepCount;
        stepper_pan.setMaxSpeed(keyframe_array[index].panSpeed);
        stepper_tilt.setMaxSpeed(keyframe_array[index].tiltSpeed);
        stepper_slider.setMaxSpeed(keyframe_array[index].sliderSpeed);
        multi_stepper.moveTo(target_position); //Sets new target positions
        multi_stepper.runSpeedToPosition(); //Moves and blocks until complete
        delay(keyframe_array[index].msDelay);
        current_keyframe_index = index;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void executeMoves(int repeat){
    for(int i = 0; i < repeat; i++){
        for(int row = 0; row < keyframe_elements; row++){
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

void gotoFirstKeyframe(void){
    moveToIndex(0);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void gotoLastKeyframe(void){
    moveToIndex(keyframe_elements - 1);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void editKeyframe(void){
    keyframe_array[current_keyframe_index].panStepCount = stepper_pan.currentPosition();
    keyframe_array[current_keyframe_index].tiltStepCount = stepper_tilt.currentPosition(); 
    keyframe_array[current_keyframe_index].sliderStepCount = stepper_slider.currentPosition(); 
    keyframe_array[current_keyframe_index].panSpeed = stepper_pan.maxSpeed();
    keyframe_array[current_keyframe_index].tiltSpeed = stepper_tilt.maxSpeed();
    keyframe_array[current_keyframe_index].sliderSpeed = stepper_slider.maxSpeed();
    
    printi(F("Edited index: "), current_keyframe_index);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void addDelay(unsigned int ms){
    keyframe_array[current_keyframe_index].msDelay = ms;
    printi(ms, F(""));
    printi(F("ms delay added at index: "), current_keyframe_index);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

//void scaleMovesArrayPanMaxSpeed(float newMax){
//    float currentMax = 0;
//    for(int row = 0; row < moves_array_elements; row++){ //Find the maximum pan speed
//        if(program_elements[row].panSpeed > currentMax){
//            currentMax = program_elements[row].panSpeed;
//        }  
//    }
//    float speedRatio = newMax / currentMax;
//    for(int row = 0; row < moves_array_elements; row++){ //Scale all the pan speeds      
//        program_elements[row].panSpeed = program_elements[row].panSpeed * speedRatio;
//    }
//}
//
///*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
//
//void scaleMovesArrayTiltMaxSpeed(float newMax){
//    float currentMax = 0;
//    for(int row = 0; row < moves_array_elements; row++){ //Find the maximum pan speed
//        if(program_elements[row].tiltSpeed > currentMax){
//            currentMax = program_elements[row].tiltSpeed;
//        }  
//    }
//    float speedRatio = newMax / currentMax;
//    for(int row = 0; row < moves_array_elements; row++){ //Scale all the pan speeds      
//        program_elements[row].tiltSpeed = program_elements[row].tiltSpeed * speedRatio;
//    }
//}
//
///*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
//
//void scaleMovesArraySliderMaxSpeed(float newMax){
//    float currentMax = 0;
//    for(int row = 0; row < moves_array_elements; row++){ //Find the maximum pan speed
//        if(program_elements[row].sliderSpeed > currentMax){
//            currentMax = program_elements[row].sliderSpeed;
//        }  
//    }
//    float speedRatio = newMax / currentMax;
//    for(int row = 0; row < moves_array_elements; row++){ //Scale all the pan speeds      
//        program_elements[row].sliderSpeed = program_elements[row].sliderSpeed * speedRatio;
//    }
//}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void invertPanDirection(bool invert){
    printi(F("Pan inversion: "), invert);
    invert_pan = invert;
    stepper_pan.setPinsInverted(invert, false, false);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void invertTiltDirection(bool invert){
    printi(F("Tilt inversion: "), invert);
    invert_tilt = invert;
    stepper_tilt.setPinsInverted(invert, false, false);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void invertSliderDirection(bool invert){
    printi(F("Slider inversion: "), invert);
    invert_slider = invert;
    stepper_slider.setPinsInverted(invert, false, false);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void saveEEPROM(void){
    EEPROM.put(EEPROM_ADDRESS_ENABLE_HOMING, enable_homing);
    EEPROM.put(EEPROM_ADDRESS_MODE, step_mode);
    EEPROM.put(EEPROM_ADDRESS_PAN_MAX_SPEED, pan_max_speed);
    EEPROM.put(EEPROM_ADDRESS_TILT_MAX_SPEED, tilt_max_speed);
    EEPROM.put(EEPROM_ADDRESS_SLIDER_MAX_SPEED, slider_max_speed);
    EEPROM.put(EEPROM_ADDRESS_HALL_PAN_OFFSET, hall_pan_offset_degrees);
    EEPROM.put(EEPROM_ADDRESS_HALL_TILT_OFFSET, hall_tilt_offset_degrees);
    EEPROM.put(EEPROM_ADDRESS_INVERT_PAN, invert_pan);
    EEPROM.put(EEPROM_ADDRESS_INVERT_TILT, invert_tilt);    
    EEPROM.put(EEPROM_ADDRESS_INVERT_SLIDER, invert_slider);      
    EEPROM.put(EEPROM_ADDRESS_DEGREES_PER_PICTURE, degrees_per_picture);
    EEPROM.put(EEPROM_ADDRESS_PANORAMICLAPSE_DELAY, delay_ms_between_pictures);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void printEEPROM(void){
    int itemp;
    float ftemp;
    long ltemp;
    printi(F("---EEPROM---\n"));
    EEPROM.get(EEPROM_ADDRESS_MODE, itemp);
    printi(F("Step mode: "), itemp, F("\n"));
    EEPROM.get(EEPROM_ADDRESS_PAN_MAX_SPEED, ftemp);
    printi(F("Pan max speed: "), ftemp, 3, F("º/s\n"));
    EEPROM.get(EEPROM_ADDRESS_TILT_MAX_SPEED, ftemp);
    printi(F("Tilt max speed: "), ftemp, 3, F("º/s\n"));
    EEPROM.get(EEPROM_ADDRESS_SLIDER_MAX_SPEED, ftemp);
    printi(F("Slider max speed: "), ftemp, 3, F("mm/s\n")); 
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
    printi(F("\n"));
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setEEPROMVariables(void){
    EEPROM.get(EEPROM_ADDRESS_MODE, step_mode);
    EEPROM.get(EEPROM_ADDRESS_PAN_MAX_SPEED, pan_max_speed);
    EEPROM.get(EEPROM_ADDRESS_TILT_MAX_SPEED, tilt_max_speed);
    EEPROM.get(EEPROM_ADDRESS_SLIDER_MAX_SPEED, slider_max_speed);
    EEPROM.get(EEPROM_ADDRESS_HALL_PAN_OFFSET, hall_pan_offset_degrees);
    EEPROM.get(EEPROM_ADDRESS_HALL_TILT_OFFSET, hall_tilt_offset_degrees);
    EEPROM.get(EEPROM_ADDRESS_DEGREES_PER_PICTURE, degrees_per_picture);
    EEPROM.get(EEPROM_ADDRESS_PANORAMICLAPSE_DELAY, delay_ms_between_pictures);    
    invert_pan = EEPROM.read(EEPROM_ADDRESS_INVERT_PAN);
    invert_tilt = EEPROM.read(EEPROM_ADDRESS_INVERT_TILT);
    invert_slider = EEPROM.read(EEPROM_ADDRESS_INVERT_SLIDER);
    enable_homing = EEPROM.read(EEPROM_ADDRESS_ENABLE_HOMING);
    printi(F("EEPROM values set.\n"));
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
    if(keyframe_elements < 2){ 
        printi(F("Not enough keyframes recorded\n"));
        return; //check there are posions to move to
    }
    for(int i = 0; i < repeat; i++){
        for(int index = 0; index < keyframe_elements - 1; index++){
            panoramiclapseInterpolation(panStepsToDegrees(keyframe_array[index].panStepCount), tiltStepsToDegrees(keyframe_array[index].tiltStepCount), sliderStepsToMillimetres(keyframe_array[index].sliderStepCount),
            panStepsToDegrees(keyframe_array[index + 1].panStepCount), tiltStepsToDegrees(keyframe_array[index + 1].tiltStepCount), sliderStepsToMillimetres(keyframe_array[index + 1].sliderStepCount), degPerPic, msDelay);
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
    
    if(keyframe_elements >= 2){ 
        panAngle = panStepsToDegrees(keyframe_array[1].panStepCount) - panStepsToDegrees(keyframe_array[0].panStepCount);
        tiltAngle = tiltStepsToDegrees(keyframe_array[1].tiltStepCount) - tiltStepsToDegrees(keyframe_array[0].tiltStepCount);
        sliderTravel = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount) - sliderStepsToMillimetres(keyframe_array[0].sliderStepCount);
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

bool calculateTargetCoordinate(void){
    float m1, c1, m2, c2;
    
    LinePoints line0;
    line0.x0 = sliderStepsToMillimetres(keyframe_array[0].sliderStepCount);
    line0.y0 = 0;
    line0.x1 = line0.x0 + cos(degToRads(panStepsToDegrees(keyframe_array[0].panStepCount)));
    line0.y1 = sin(degToRads(panStepsToDegrees(keyframe_array[0].panStepCount)));
    
    LinePoints line1;
    line1.x0 = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount);
    line1.y0 = 0;
    line1.x1 = line1.x0 + cos(degToRads(panStepsToDegrees(keyframe_array[1].panStepCount)));
    line1.y1 = sin(degToRads(panStepsToDegrees(keyframe_array[1].panStepCount)));
    
    if((line0.x1 - line0.x0) != 0){
        m1 = (line0.y1 - line0.y0) / (line0.x1 - line0.x0);
        c1 = line0.y1 - m1 * line0.x1;
    }

    if((line1.x1 - line1.x0) != 0){
        m2 = (line1.y1 - line1.y0) / (line1.x1 - line1.x0);
        c2 = line1.y1 - m2 * line1.x1;
    }

    if((line0.x1 - line0.x0) == 0){
        intercept.x = line0.x0;
        intercept.y = m2 * intercept.x + c2;    
    }
    else if((line1.x1 - line1.x0) == 0){
        intercept.x = line1.x0;
        intercept.y = m1 * intercept.x + c1;
    }
    else{
        if(m1 == m2){
            printi(F("Positions do not intersect."));
            return false;
        }
        intercept.x = (c2 - c1) / (m1 - m2);
        intercept.y = m1 * intercept.x + c1;
    }
    intercept.z = tan(degToRads(tiltStepsToDegrees(keyframe_array[0].tiltStepCount))) * sqrt(pow(intercept.x - sliderStepsToMillimetres(keyframe_array[0].sliderStepCount), 2) + pow(intercept.y, 2));
    if(((panStepsToDegrees(keyframe_array[0].panStepCount) > 0 && panStepsToDegrees(keyframe_array[1].panStepCount) > 0) && intercept.y < 0)
    || ((panStepsToDegrees(keyframe_array[0].panStepCount) < 0 && panStepsToDegrees(keyframe_array[1].panStepCount) < 0) && intercept.y > 0) || intercept.y == 0){
        printi(F("Invalid intercept.\n"));
        return false;
    }
    return true;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void interpolateTargetPoint(FloatCoordinate targetPoint, int repeat){
    if(keyframe_elements < 2){ 
        printi(F("Not enough keyframes recorded\n"));
        return; //check there are posions to move to
    }
    
    float sliderStartPos = sliderStepsToMillimetres(keyframe_array[0].sliderStepCount); //slider start position
    float sliderEndPos = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount);
    float panAngle = 0;
    float tiltAngle = 0;
    float x = targetPoint.x - sliderStepsToMillimetres(keyframe_array[0].sliderStepCount);
    float ySqared = pow(targetPoint.y, 2);    
    float sliderTravel = sliderStepsToMillimetres(keyframe_array[1].sliderStepCount) - sliderStepsToMillimetres(keyframe_array[0].sliderStepCount);
    int numberOfIncrements = abs(sliderTravel);
    float increment = sliderTravel / numberOfIncrements;//size of interpolation increments in mm
    
    for(int j = 0; (j < repeat || (repeat == 0 && j == 0)); j++){
        for(int i = 0; i <= numberOfIncrements; i++){
            x = targetPoint.x - (sliderStartPos + increment * i);
            panAngle = radsToDeg(atan2(targetPoint.y, x));
            tiltAngle = radsToDeg(atan2(targetPoint.z, sqrt(pow(x, 2) + ySqared)));
            setTargetPositions(panAngle, tiltAngle, sliderStartPos + increment * i);
            multi_stepper.runSpeedToPosition();//blocking move to the next position
        }
        x = targetPoint.x - sliderEndPos;
        panAngle = radsToDeg(atan2(targetPoint.y, x));
        tiltAngle = radsToDeg(atan2(targetPoint.z, sqrt(pow(x, 2) + ySqared)));
        setTargetPositions(panAngle, tiltAngle, sliderEndPos);
        multi_stepper.runSpeedToPosition();//blocking move to the next position

        for(int i = numberOfIncrements; (i >= 0 && repeat > 0); i--){
            x = targetPoint.x - (sliderStartPos + increment * i);
            panAngle = radsToDeg(atan2(targetPoint.y, x));
            tiltAngle = radsToDeg(atan2(targetPoint.z, sqrt(pow(x, 2) + ySqared)));
            setTargetPositions(panAngle, tiltAngle, sliderStartPos + increment * i);
            multi_stepper.runSpeedToPosition();//blocking move to the next position
        }
        if(repeat > 0){
            setTargetPositions(panAngle, tiltAngle, sliderStartPos);
            multi_stepper.runSpeedToPosition();//blocking move to the next position 
        }     
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool sliderHoming(void){
    while(digitalRead(PIN_SLIDER_HALL) == 0){ //Move off the hall
        target_position[2] = target_position[2] + sliderMillimetresToSteps(1); 
        multi_stepper.moveTo(target_position); 
        multi_stepper.runSpeedToPosition();        
    }
    setTargetPositions(panStepsToDegrees(target_position[0]), tiltStepsToDegrees(target_position[1]), -1200);//1200 is the length of the slider
    while(multi_stepper.run()){
        if(digitalRead(PIN_SLIDER_HALL) == 0){
            stepper_slider.setCurrentPosition(0);//set step count to 0
            setTargetPositions(panStepsToDegrees(target_position[0]), tiltStepsToDegrees(target_position[1]), 0);
            return true;
        }
    }
    return false;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void serialData(void){
    char instruction = Serial.read();
    if(instruction == INSTRUCTION_BYTES_SLIDER_PAN_TILT_SPEED){
        int count = 0;
        while(Serial.available() < 6){//Wait for 6 bytes to be available. Breaks after ~20ms if bytes are not received.
                delayMicroseconds(200); 
                count++;
                if(count > 100){
                    serialFlush();//Clear the serial buffer
                    break;   
                }
            }
            int sliderStepSpeed = (Serial.read() << 8) + Serial.read(); 
            int panStepSpeed = (Serial.read() << 8) + Serial.read(); 
            int tiltStepSpeed = (Serial.read() << 8) + Serial.read(); 

            stepper_slider.setSpeed(sliderStepSpeed);
            stepper_pan.setSpeed(panStepSpeed);
            stepper_tilt.setSpeed(tiltStepSpeed);
            stepper_slider.runSpeed();
            stepper_pan.runSpeed();
            stepper_tilt.runSpeed();
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
        case INSTRUCTION_SLIDER_HOME:{
            sliderHoming();
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
//        case INSTRUCTION_SCALE_PAN_SPEED:{
//            scaleMovesArrayPanMaxSpeed(panDegreesToSteps(serialCommandValueFloat));
//        }
//        break;
//        case INSTRUCTION_SCALE_TILT_SPEED:{
//            scaleMovesArrayTiltMaxSpeed(tiltDegreesToSteps(serialCommandValueFloat));
//        }
//        break;
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
                stepper_slider.setCurrentPosition(0);
                setTargetPositions(0, 0, 0);
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
            printi(F("Pan Hall offset: "), hall_pan_offset_degrees, 3, F("º\n"));
        }
        break;
        case INSTRUCTION_SET_TILT_HALL_OFFSET:{
            hall_tilt_offset_degrees = serialCommandValueFloat;
            printi(F("Tilt Hall offset: "), hall_tilt_offset_degrees, 3, F("º\n"));
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
            saveEEPROM();
            printi(F("Saved to EEPROM.\n"));
        }
        break;
        case INSTRUCTION_ADD_POSITION:{
            addPosition();
        }
        break;
        case INSTRUCTION_STEP_FORWARD:{
            moveToIndex(current_keyframe_index + 1);
            printi(F("Index: "), current_keyframe_index, F("\n"));
        }
        break;
        case INSTRUCTION_STEP_BACKWARD:{
            moveToIndex(current_keyframe_index - 1);
            printi(F("Index: "), current_keyframe_index, F("\n"));
        }
        break;
        case INSTRUCTION_JUMP_TO_START:{
            gotoFirstKeyframe();
            printi(F("Index: "), current_keyframe_index, F("\n"));
        }
        break;
        case INSTRUCTION_JUMP_TO_END:{
            gotoLastKeyframe();
            printi(F("Index: "), current_keyframe_index, F("\n"));
        }
        break;
        case INSTRUCTION_EDIT_ARRAY:{
            editKeyframe();
        }
        break;
        case INSTRUCTION_ADD_DELAY:{
            addDelay(serialCommandValueInt);
        }
        break;
        case INSTRUCTION_CLEAR_ARRAY:{
            clearKeyframes();
        }
        break;
        case INSTRUCTION_EXECUTE_MOVES:{
            executeMoves(serialCommandValueInt);
        }
        break;      
//        case INSTRUCTION_PAN_RUN_SPEED:{
//            stepper_pan.setSpeed(panDegreesToSteps(serialCommandValueFloat));
//            stepper_pan.runSpeed();
//        }
//        break;
//        case INSTRUCTION_TILT_RUN_SPEED:{
//            stepper_tilt.setSpeed(tiltDegreesToSteps(serialCommandValueFloat));
//            stepper_tilt.runSpeed();
//        }
//        break;
        case INSTRUCTION_DEBUG_STATUS:{
            debugReport();
        }
        break;
        case INSTRUCTION_PAN_DEGREES:{
            panJogDegrees(serialCommandValueFloat);
        }
        break;  
        case INSTRUCTION_TILT_DEGREES:{
            tiltJogDegrees(serialCommandValueFloat);
        }
        break; 
        case INSTRUCTION_SET_HOME:{
            setTargetPositions(0, 0, 0);
            stepper_pan.setCurrentPosition(0);
            stepper_tilt.setCurrentPosition(0);
            stepper_slider.setCurrentPosition(0);           
        }
        break; 
        case INSTRUCTION_ENABLE:{
            enableSteppers();
        }
        break; 
        case INSTRUCTION_STEP_MODE:{
            setStepMode(serialCommandValueInt);
        }
        break; 
        case INSTRUCTION_SET_PAN_SPEED:{
            printi("Max pan speed: ", serialCommandValueFloat, 1, " º/s.\n");
            pan_max_speed = serialCommandValueFloat;
            stepper_pan.setMaxSpeed(panDegreesToSteps(pan_max_speed));
        }
        break; 
        case INSTRUCTION_SET_TILT_SPEED:{
            printi("Max tilt speed: ", serialCommandValueFloat, 1, " º/s.\n");
            tilt_max_speed = serialCommandValueFloat;
            stepper_tilt.setMaxSpeed(tiltDegreesToSteps(tilt_max_speed));
        }
        break;
        case INSTRUCTION_SET_SLIDER_SPEED:{
            printi("Max slider speed: ", serialCommandValueFloat, 1, " mm/s.\n");
            slider_max_speed = serialCommandValueFloat;
            stepper_slider.setMaxSpeed(sliderMillimetresToSteps(slider_max_speed));
        }
        break;
        case INSTRUCTION_ORIBIT_POINT:{            
            if(calculateTargetCoordinate()){
                printi("Target coordinates: \nx: ", intercept.x, 3, " mm\n");
                printi("y: ", intercept.y, 3, " mm\n");
                printi("z: ", intercept.z, 3, " mm\n");
                interpolateTargetPoint(intercept, serialCommandValueInt);
            }
        }
        break;  
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
