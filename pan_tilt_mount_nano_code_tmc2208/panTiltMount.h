#ifndef PANTILTMOUNT_H
#define PANTILTMOUNT_H

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#define BAUD_RATE 57600

#define PIN_SHUTTER_TRIGGER A1
#define PIN_PAN_HALL A3
#define PIN_TILT_HALL A4
#define PIN_SLIDER_HALL 2
#define PIN_INPUT_VOLTAGE A5
#define PIN_ENABLE 12
#define PIN_MS1 11
#define PIN_MS2 10
#define PIN_STEP_SLIDER 4
#define PIN_DIRECTION_SLIDER 3
#define PIN_STEP_TILT 6
#define PIN_DIRECTION_TILT 5
#define PIN_STEP_PAN 8
#define PIN_DIRECTION_PAN 7

#define HALF_STEP 2
#define QUARTER_STEP 4
#define EIGHTH_STEP 8
#define SIXTEENTH_STEP 16

#define SLIDER_PULLEY_TEETH 36.0
#define PAN_GEAR_RATIO 8.4705882352941176470588235294118 //144/17 teeth
//Used with the belt driven tilt axis.
#define TILT_GEAR_RATIO 7.6875 //123/16 teeth
//Used with herringbone gears
//#define TILT_GEAR_RATIO 3.047619047619047619047619047619 //64/21 teeth

#define MAX_STRING_LENGTH 10
#define KEYFRAME_ARRAY_LENGTH 35

#define SHUTTER_DELAY 200

#define INSTRUCTION_BYTES_SLIDER_PAN_TILT_SPEED 4
#define INSTRUCTION_STEP_MODE 'm'
#define INSTRUCTION_PAN_DEGREES 'p'
#define INSTRUCTION_TILT_DEGREES 't'
#define INSTRUCTION_ENABLE 'e'
#define INSTRUCTION_SET_PAN_SPEED 's'
#define INSTRUCTION_SET_TILT_SPEED 'S'
#define INSTRUCTION_INVERT_PAN 'i'
#define INSTRUCTION_INVERT_TILT 'I'
#define INSTRUCTION_SET_PAN_HALL_OFFSET 'o'
#define INSTRUCTION_SET_TILT_HALL_OFFSET 'O'
#define INSTRUCTION_SET_HOMING 'H'
#define INSTRUCTION_TRIGGER_SHUTTER 'c'
#define INSTRUCTION_AUTO_HOME 'A'
#define INSTRUCTION_DEBUG_STATUS 'R'
#define INSTRUCTION_EXECUTE_MOVES ';'
#define INSTRUCTION_ADD_POSITION '#'
#define INSTRUCTION_STEP_FORWARD '>'
#define INSTRUCTION_STEP_BACKWARD '<'
#define INSTRUCTION_JUMP_TO_START '['
#define INSTRUCTION_JUMP_TO_END ']'
#define INSTRUCTION_EDIT_ARRAY 'E'
#define INSTRUCTION_ADD_DELAY 'd'
#define INSTRUCTION_EDIT_DELAY 'D'
#define INSTRUCTION_CLEAR_ARRAY 'C'
#define INSTRUCTION_SAVE_TO_EEPROM 'U'
#define INSTRUCTION_PANORAMICLAPSE 'L'
#define INSTRUCTION_ANGLE_BETWEEN_PICTURES 'b'
#define INSTRUCTION_DELAY_BETWEEN_PICTURES 'B'
#define INSTRUCTION_TIMELAPSE 'l'
#define INSTRUCTION_SLIDER_MILLIMETRES 'x'
#define INSTRUCTION_INVERT_SLIDER 'j'
#define INSTRUCTION_SET_SLIDER_SPEED 'X'
#define INSTRUCTION_ORIBIT_POINT '@'
#define INSTRUCTION_CALCULATE_TARGET_POINT 'T'
#define INSTRUCTION_ACCEL_ENABLE 'a'
#define INSTRUCTION_PAN_ACCEL_INCREMENT_DELAY 'q'
#define INSTRUCTION_TILT_ACCEL_INCREMENT_DELAY 'Q'
#define INSTRUCTION_SLIDER_ACCEL_INCREMENT_DELAY 'w'
#define INSTRUCTION_SCALE_SPEED 'W'

#define EEPROM_ADDRESS_HOMING_MODE 0
#define EEPROM_ADDRESS_PAN_MAX_SPEED 17
#define EEPROM_ADDRESS_TILT_MAX_SPEED 21
#define EEPROM_ADDRESS_HALL_PAN_OFFSET 33
#define EEPROM_ADDRESS_HALL_TILT_OFFSET 37
#define EEPROM_ADDRESS_INVERT_PAN 41
#define EEPROM_ADDRESS_INVERT_TILT 42
#define EEPROM_ADDRESS_MODE 43
#define EEPROM_ADDRESS_DEGREES_PER_PICTURE 45
#define EEPROM_ADDRESS_PANORAMICLAPSE_DELAY 49
#define EEPROM_ADDRESS_SLIDER_MAX_SPEED 70
#define EEPROM_ADDRESS_SLIDER_ACCELERATION 74
#define EEPROM_ADDRESS_INVERT_SLIDER 78
#define EEPROM_ADDRESS_ACCELERATION_ENABLE 79
#define EEPROM_ADDRESS_PAN_ACCEL_INCREMENT_DELAY 80
#define EEPROM_ADDRESS_TILT_ACCEL_INCREMENT_DELAY 82
#define EEPROM_ADDRESS_SLIDER_ACCEL_INCREMENT_DELAY 84

//#define VERSION_NUMBER "Version: 3.11.2\n"
#define VERSION_NUMBER "Version: Hall 1.0.0\n"

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

struct KeyframeElement {
    long panStepCount = 0;
    float panSpeed = 0;
    long tiltStepCount = 0;
    float tiltSpeed = 0;
    long sliderStepCount = 0;
    float sliderSpeed = 0;
    int msDelay = 0;
};

struct FloatCoordinate {
    float x;
    float y;
    float z;
};

struct LinePoints {
    float x0;
    float y0;
    float x1;
    float y1;
};

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

void initPanTilt(void);
void serialFlush(void);
void enableSteppers(void);
void setStepMode(int);
void serialData(void);
void mainLoop(void);
void panDegrees(float);
void tiltDegrees(float);
void debugReport(void);
bool findHome(void);
float getBatteryVoltage(void);
float getBatteryPercentage(void);
float boundFloat(float, float, float);
float panDegreesToSteps(float);
float tiltDegreesToSteps(float);
float panStepsToDegrees(long);
float panStepsToDegrees(float);
float tiltStepsToDegrees(long);
float tiltStepsToDegrees(float);
int addPosition(void);
void clearKeyframes(void);
void executeMoves(int);
void moveToIndex(int);
void gotoFirstKeyframe(void);
void gotoLastKeyframe(void);
void editKeyframe(void);
void addDelay(unsigned int ms);
void editDelay(unsigned int ms);
void printKeyframeElements(void);
void saveEEPROM(void);
void printEEPROM(void);
void setEEPROMVariables(void);
void invertPanDirection(bool);
void invertTiltDirection(bool);
int setTargetPositions(float, float, float);
void toggleAutoHoming(void);
void triggerCameraShutter(void);
void panoramiclapseInterpolation(float, float, float, float, float, float, float, unsigned long);
void panoramiclapse(float, unsigned long, int);
long sliderMillimetresToSteps(float);
float sliderStepsToMillimetres(long);
void sliderMoveTo(float);
void invertSliderDirection(bool);
void timelapse(unsigned int, unsigned long);
bool calculateTargetCoordinate(void);
void interpolateTargetPoint(FloatCoordinate);
void toggleAcceleration(void);
void scaleKeyframeSpeed(float);

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

#endif
