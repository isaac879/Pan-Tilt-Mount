/*--------------------------------------------------------------------------------------------------------------------------------------------------------
 * 
 * CHECK THE CODE FOR "TODO:" AND EDIT APPROPRIATELY 
 * 
 * The code is developed for a 3D printed pan/tilt/slider mount for a Canon EOS 250D DSLR. It is controlled by an Arduino Nano, JDY-31 Bluetooth module, 
 * TMC2208 stepper motor drivers and some custom circuitry.
 * 
 * Pan Tilt Mount STL files: https://www.thingiverse.com/thing:4547074
 * 
 * Project video: https://youtu.be/1FfB7cLkUyQ
 *                 
 * All measurements are in SI units unless otherwise specified.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 * IN THE SOFTWARE
 * 
 * Code written by isaac879
 * 
 * Last modified: 03/11/2020
 *
 *--------------------------------------------------------------------------------------------------------------------------------------------------------*/

//TODOs:
//EEPROM values will need to be set after uploading the code and then a reset is required.

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

//Comment code/more documentation
//Efficiency improvements

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include "panTiltMount.h"

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup(){
    initPanTilt();
}

void loop(){
    mainLoop();
}
