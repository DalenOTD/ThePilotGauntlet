/*
  ================================================================
  ONE-HANDED ADAPTIVE GAME CONTROLLER
  Teensy 4.1 USB Game Controller
  ================================================================

  HARDWARE:
  - Teensy 4.1
  - Left analog flight stick (2 potentiometers)
  - Right thumb joystick module (X, Y, push switch)
  - 10 pushbuttons:
      A, B, X, Y
      LB, RB
      LT, RT
      Start, Select
  - 4 pushbuttons for D-pad

  ------------------------------------------------
  ARDUINO / TEENSY SETUP
  ------------------------------------------------

  Board:
      Tools -> Board -> Teensy 4.1

  USB Type:
      Tools -> USB Type ->
      Serial + Keyboard + Mouse + Joystick

  After uploading the code, the controller can be tested in
  Windows by opening:

      joy.cpl

  ------------------------------------------------
  IMPORTANT ELECTRICAL INFORMATION
  ------------------------------------------------

  Teensy 4.1 uses 3.3V logic.

  DO NOT connect 5V directly to an analog or digital input.

  All buttons in this project use INPUT_PULLUP.

  This means:

      NOT PRESSED = HIGH
      PRESSED     = LOW

  Therefore, every pushbutton should be wired:

      TEENSY DIGITAL PIN
             |
          [BUTTON]
             |
            GND

  You do NOT need a separate resistor for each button because
  INPUT_PULLUP enables the Teensy's internal pull-up resistor.

  All controls must share a COMMON GROUND with the Teensy.

  ------------------------------------------------
  ANALOG JOYSTICK WIRING
  ------------------------------------------------

  RIGHT THUMB JOYSTICK MODULE:

      Module      Teensy 4.1
      -------------------------
      VCC    ->   3.3V
      GND    ->   GND
      VRX    ->   A0 / Pin 14
      VRY    ->   A1 / Pin 15
      SW     ->   Pin 2

  NOTE:
  Wire colors vary between joystick modules.
  Follow the terminal labels (VCC, GND, VRX, VRY, SW), not
  wire color alone.

  ------------------------------------------------
  LEFT FLIGHT STICK WIRING
  ------------------------------------------------

  The flight stick contains two potentiometers.

      Function        Teensy 4.1
      ---------------------------
      Power       ->  3.3V
      Ground      ->  GND
      X signal    ->  A2 / Pin 16
      Y signal    ->  A3 / Pin 17

  On the prototype used for this project:

      Red    = 3.3V
      Black  = GND
      White  = A2 / Pin 16
      Green  = A3 / Pin 17

  IMPORTANT:
  These wire colors apply to the specific flight-stick assembly
  used in this prototype. Verify the wiring if using a different
  joystick or potentiometer assembly.

  ------------------------------------------------
  DIGITAL BUTTON WIRING
  ------------------------------------------------

      Control       Teensy Pin
      ------------------------
      Right Stick   2
      A             3
      B             4
      X             5
      Y             6
      LB            7
      RB            8
      RT            9
      LT            10
      Start         11
      Select        12

  Each button connects between its assigned pin and GND.

  ------------------------------------------------
  D-PAD WIRING
  ------------------------------------------------

      Direction     Teensy Pin
      ------------------------
      Up            20
      Down          21
      Left          22
      Right         23

  Each D-pad switch connects between its assigned pin and GND.

  ------------------------------------------------
  WINDOWS joy.cpl BUTTON MAPPING
  ------------------------------------------------

      Controller      joy.cpl
      ------------------------
      X            -> Button 1
      A            -> Button 2
      B            -> Button 3
      Y            -> Button 4
      LB           -> Button 5
      RB           -> Button 6
      LT           -> Button 7
      RT           -> Button 8
      Select       -> Button 9
      Start        -> Button 10
      Right Stick  -> Button 12

  The D-pad is sent as a POV / hat switch rather than four
  separate joystick buttons.

  ------------------------------------------------
  STARTUP CALIBRATION
  ------------------------------------------------

  When the controller is powered on:

      DO NOT MOVE EITHER JOYSTICK.

  The controller waits one second and measures the resting
  position of all four analog axes.

  These measurements become the center positions.

  ================================================================
*/


// ================================================================
// CONFIGURATION
// ================================================================

// Amount of movement around center that will be ignored.
// Increase this if a joystick drifts while untouched.
// Decrease it if small movements are not being detected.
const int DEADZONE = 25;

// Number of readings averaged during startup calibration.
const int CALIBRATION_SAMPLES = 32;

// Time to allow the user to release the sticks after power-on.
const int CALIBRATION_DELAY_MS = 1000;

// Delay between controller updates.
// 5 ms = approximately 200 updates per second.
const int LOOP_DELAY_MS = 5;


// ================================================================
// RIGHT THUMB JOYSTICK
// ================================================================

const int PIN_RIGHT_X  = A0;   // Teensy Pin 14
const int PIN_RIGHT_Y  = A1;   // Teensy Pin 15
const int PIN_RIGHT_SW = 2;    // Push joystick down for R3


// ================================================================
// FACE / SHOULDER / MENU BUTTONS
// ================================================================

const int PIN_A_BTN      = 3;
const int PIN_B_BTN      = 4;
const int PIN_X_BTN      = 5;
const int PIN_Y_BTN      = 6;

const int PIN_LB_BTN     = 7;
const int PIN_RB_BTN     = 8;

const int PIN_RT_BTN     = 9;
const int PIN_LT_BTN     = 10;

const int PIN_START_BTN  = 11;
const int PIN_SELECT_BTN = 12;


// ================================================================
// D-PAD
// ================================================================

const int PIN_DPAD_UP    = 20;
const int PIN_DPAD_DOWN  = 21;
const int PIN_DPAD_LEFT  = 22;
const int PIN_DPAD_RIGHT = 23;


// ================================================================
// LEFT FLIGHT STICK
// ================================================================

const int PIN_LEFT_X = A2;    // Teensy Pin 16
const int PIN_LEFT_Y = A3;    // Teensy Pin 17


// ================================================================
// CALIBRATED CENTER VALUES
// ================================================================
//
// A perfect 10-bit analog center would be 512.
//
// Real potentiometers rarely rest at exactly 512, so these values
// are measured automatically when the controller starts.
//

int centerRightX = 512;
int centerRightY = 512;
int centerLeftX  = 512;
int centerLeftY  = 512;


// ================================================================
// READ AND AVERAGE AN ANALOG INPUT
// ================================================================
//
// Takes several readings from one analog pin and averages them.
//
// This reduces the effect of small electrical fluctuations during
// startup calibration.
//

int readAverage(int pin, int samples = CALIBRATION_SAMPLES) {

  long total = 0;

  for (int i = 0; i < samples; i++) {
    total += analogRead(pin);
    delay(2);
  }

  return total / samples;
}


// ================================================================
// CONVERT ANALOG READING TO USB JOYSTICK POSITION
// ================================================================
//
// Teensy reads each analog axis from:
//
//      0 -------- 512 -------- 1023
//      MIN       CENTER         MAX
//
// However, the physical joystick may not actually center at 512.
//
// "center" contains the measured center value from startup.
//
// This function remaps BOTH sides of the physical center so that
// the computer still receives:
//
//      0 -------- 512 -------- 1023
//
// A deadzone is then placed around 512 to prevent small amounts
// of potentiometer noise from causing joystick drift.
//

int axisToJoystick(int raw, int center) {

  int output;

  if (raw >= center) {

    // Upper half of joystick movement
    output = map(
      raw,
      center, 1023,
      512,    1023
    );

  } else {

    // Lower half of joystick movement
    output = map(
      raw,
      0,      center,
      0,      512
    );
  }

  // Prevent values outside the valid USB joystick range.
  output = constrain(output, 0, 1023);

  // Force small movements around center back to exact center.
  if (abs(output - 512) < DEADZONE) {
    output = 512;
  }

  return output;
}


// ================================================================
// READ A PUSHBUTTON
// ================================================================
//
// All buttons use INPUT_PULLUP.
//
// Because of this:
//      HIGH = button released
//      LOW  = button pressed
//
// Returning true makes the rest of the program easier to read.
//

bool pressed(int pin) {
  return digitalRead(pin) == LOW;
}


// ================================================================
// READ D-PAD AND CONVERT TO POV HAT
// ================================================================
//
// Teensy's USB joystick uses angles for the POV / hat:
//
//       315     0      45
//          \    |    /
//           \   |   /
//      270 --- CENTER --- 90
//           /   |   \
//          /    |    \
//       225    180    135
//
// -1 means that no D-pad direction is pressed.
//

int readDpadHat() {

  bool up    = pressed(PIN_DPAD_UP);
  bool down  = pressed(PIN_DPAD_DOWN);
  bool left  = pressed(PIN_DPAD_LEFT);
  bool right = pressed(PIN_DPAD_RIGHT);

  // Diagonal directions
  if (up && right)   return 45;
  if (right && down) return 135;
  if (down && left)  return 225;
  if (left && up)    return 315;

  // Cardinal directions
  if (up)    return 0;
  if (right) return 90;
  if (down)  return 180;
  if (left)  return 270;

  // D-pad released
  return -1;
}


// ================================================================
// SETUP
// ================================================================

void setup() {

  // --------------------------------------------------------------
  // Configure every pushbutton as INPUT_PULLUP.
  //
  // Each button therefore needs only:
  //
  //     PIN -> BUTTON -> GND
  // --------------------------------------------------------------

  pinMode(PIN_RIGHT_SW, INPUT_PULLUP);

  pinMode(PIN_A_BTN, INPUT_PULLUP);
  pinMode(PIN_B_BTN, INPUT_PULLUP);
  pinMode(PIN_X_BTN, INPUT_PULLUP);
  pinMode(PIN_Y_BTN, INPUT_PULLUP);

  pinMode(PIN_LB_BTN, INPUT_PULLUP);
  pinMode(PIN_RB_BTN, INPUT_PULLUP);
  pinMode(PIN_LT_BTN, INPUT_PULLUP);
  pinMode(PIN_RT_BTN, INPUT_PULLUP);

  pinMode(PIN_START_BTN, INPUT_PULLUP);
  pinMode(PIN_SELECT_BTN, INPUT_PULLUP);

  pinMode(PIN_DPAD_UP, INPUT_PULLUP);
  pinMode(PIN_DPAD_DOWN, INPUT_PULLUP);
  pinMode(PIN_DPAD_LEFT, INPUT_PULLUP);
  pinMode(PIN_DPAD_RIGHT, INPUT_PULLUP);


  // --------------------------------------------------------------
  // ANALOG RESOLUTION
  // --------------------------------------------------------------
  //
  // Use 10-bit analog readings:
  //
  //     Minimum = 0
  //     Center  = approximately 512
  //     Maximum = 1023
  //

  analogReadResolution(10);


  // --------------------------------------------------------------
  // STARTUP CALIBRATION
  // --------------------------------------------------------------
  //
  // IMPORTANT:
  // Both joysticks should remain untouched during this period.
  //

  delay(CALIBRATION_DELAY_MS);

  centerRightX = readAverage(PIN_RIGHT_X);
  centerRightY = readAverage(PIN_RIGHT_Y);

  centerLeftX = readAverage(PIN_LEFT_X);
  centerLeftY = readAverage(PIN_LEFT_Y);


  // --------------------------------------------------------------
  // MANUAL USB UPDATE MODE
  // --------------------------------------------------------------
  //
  // Changes are stored until Joystick.send_now() is called.
  // This allows all axes and buttons to be sent together.
  //

  Joystick.useManualSend(true);
}


// ================================================================
// MAIN CONTROLLER LOOP
// ================================================================

void loop() {

  // --------------------------------------------------------------
  // READ RAW ANALOG VALUES
  // --------------------------------------------------------------

  int rawRightX = analogRead(PIN_RIGHT_X);
  int rawRightY = analogRead(PIN_RIGHT_Y);

  int rawLeftX = analogRead(PIN_LEFT_X);
  int rawLeftY = analogRead(PIN_LEFT_Y);


  // --------------------------------------------------------------
  // PROCESS RIGHT THUMB JOYSTICK
  // --------------------------------------------------------------

  int rightX = axisToJoystick(rawRightX, centerRightX);
  int rightY = axisToJoystick(rawRightY, centerRightY);


  // --------------------------------------------------------------
  // PROCESS LEFT FLIGHT STICK
  // --------------------------------------------------------------

  int baseLeftX = axisToJoystick(rawLeftX, centerLeftX);
  int baseLeftY = axisToJoystick(rawLeftY, centerLeftY);


  /*
     FLIGHT-STICK ORIENTATION

     In this controller, the flight-stick mechanism is physically
     rotated relative to the orientation expected by the computer.

     Therefore, its X and Y axes are swapped:

         Physical Y -> Computer X
         Physical X -> Computer Y

     Neither axis requires inversion in this particular build.

     If you mount the flight stick differently, this is the section
     that may need to be changed.
  */

  int leftX = baseLeftY;
  int leftY = baseLeftX;


  // --------------------------------------------------------------
  // SEND ANALOG STICKS
  // --------------------------------------------------------------

  // LEFT STICK
  Joystick.X(leftX);
  Joystick.Y(leftY);

  // RIGHT STICK
  Joystick.Z(rightX);
  Joystick.Zrotate(rightY);


  // --------------------------------------------------------------
  // FACE BUTTONS
  // --------------------------------------------------------------

  Joystick.button(1, pressed(PIN_X_BTN));   // X
  Joystick.button(2, pressed(PIN_A_BTN));   // A
  Joystick.button(3, pressed(PIN_B_BTN));   // B
  Joystick.button(4, pressed(PIN_Y_BTN));   // Y


  // --------------------------------------------------------------
  // SHOULDER BUTTONS / TRIGGERS
  // --------------------------------------------------------------

  Joystick.button(5, pressed(PIN_LB_BTN));  // LB
  Joystick.button(6, pressed(PIN_RB_BTN));  // RB
  Joystick.button(7, pressed(PIN_LT_BTN));  // LT
  Joystick.button(8, pressed(PIN_RT_BTN));  // RT


  // --------------------------------------------------------------
  // MENU BUTTONS
  // --------------------------------------------------------------

  Joystick.button(9,  pressed(PIN_SELECT_BTN));  // Select
  Joystick.button(10, pressed(PIN_START_BTN));   // Start


  // --------------------------------------------------------------
  // RIGHT STICK CLICK
  // --------------------------------------------------------------

  Joystick.button(12, pressed(PIN_RIGHT_SW));    // R3


  // --------------------------------------------------------------
  // D-PAD
  // --------------------------------------------------------------

  Joystick.hat(readDpadHat());


  // --------------------------------------------------------------
  // SEND COMPLETE CONTROLLER STATE TO COMPUTER
  // --------------------------------------------------------------

  Joystick.send_now();


  // Approximately 200 controller updates per second.
  delay(LOOP_DELAY_MS);
}