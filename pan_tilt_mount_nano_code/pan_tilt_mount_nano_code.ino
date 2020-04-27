 /*--------------------------------------------------------------------------------------------------------------------------------------------------------
 * 
 * CHECK THE CODE FOR "TODO:" AND EDIT APPROPRIATELY 
 * 
 * The code is developed for a Delta robot. The robot is controlled by an Arduino Nano.
 * 
 * Pan Tilt Mount STL files: 
 * 
 * Project video: 
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
 * Last modified: 07/04/2020
 *
 *--------------------------------------------------------------------------------------------------------------------------------------------------------*/
//TODOs:
//change defines for pins to match pcb design
//Change step counts for other step modes
//function to keep focus on a point


//can probably remove most acelleration stuff
//program array in deg
//point to point in x time
//Make an AT+ command set
//report commands
//Rename arrays to Keyframes

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

#include "panTiltMount.h"

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup(){
    initPanTilt();
}

void loop(){
    mainLoop();
}
