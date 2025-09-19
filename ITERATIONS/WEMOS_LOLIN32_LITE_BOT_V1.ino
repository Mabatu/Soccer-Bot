#include <Bluepad32.h>

#define rightWheels 33
#define leftWheels 32

#define RF 22
#define RB 19
#define LF 23
#define LB 18

#define NEUTRAL_POINT 4
#define DEAD_ZONE 4
#define MAX_SPEED 1023
#define MIN_SPEED 200

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processGamepad(ControllerPtr ctl) {
  //All motors off -> direction control.
  digitalWrite(RF, LOW);
  digitalWrite(RB, LOW);
  digitalWrite(LF, LOW);
  digitalWrite(LB, LOW);

  analogWrite(leftWheels, 0);
  analogWrite(rightWheels, 0);

  int LY = ctl->axisY();
  int RY = ctl->axisRY();
  int L2 = ctl->throttle();
  int R2 = ctl->brake();
  int buttons = ctl->buttons();

  //Controls for right wheels.
  if (RY < NEUTRAL_POINT - DEAD_ZONE) {
    //Set direction.
    digitalWrite(RF, HIGH);
    digitalWrite(RB, LOW);

    //Set speed.
    int dutyCycle = map(RY, (NEUTRAL_POINT - DEAD_ZONE - 1), -508, MIN_SPEED, MAX_SPEED);
    analogWrite(rightWheels, dutyCycle);
  }

  else if (RY > NEUTRAL_POINT + DEAD_ZONE) {
    //Set direction.
    digitalWrite(RF, LOW);
    digitalWrite(RB, HIGH);

    //Set speed.
    int dutyCycle = map(RY, (NEUTRAL_POINT + DEAD_ZONE + 1), 512, MIN_SPEED, MAX_SPEED);
    analogWrite(rightWheels, dutyCycle);
  }

  //Controls for left wheels.
  if (LY < NEUTRAL_POINT - DEAD_ZONE) {
    //Set direction.
    digitalWrite(LF, HIGH);
    digitalWrite(LB, LOW);

    //Set speed.
    int dutyCycle = map(LY, (NEUTRAL_POINT - DEAD_ZONE - 1), -508, MIN_SPEED, MAX_SPEED);
    analogWrite(leftWheels, dutyCycle);
  }

  else if (LY > NEUTRAL_POINT + DEAD_ZONE) {
    //Set direction.
    digitalWrite(LF, LOW);
    digitalWrite(LB, HIGH);

    //Set speed.
    int dutyCycle = map(LY, (NEUTRAL_POINT + DEAD_ZONE + 1), 512, MIN_SPEED, MAX_SPEED);
    analogWrite(leftWheels, dutyCycle);
  }

  if (L2 > 4) {
    digitalWrite(RF, HIGH);
    digitalWrite(RB, LOW);
    digitalWrite(LF, HIGH);
    digitalWrite(LB, LOW);
    
    analogWrite(rightWheels, L2);
    analogWrite(leftWheels, L2);
  }
  if (R2 > 4) {
    digitalWrite(RF, LOW);
    digitalWrite(RB, HIGH);
    digitalWrite(LF, LOW);
    digitalWrite(LB, HIGH);

    analogWrite(rightWheels, R2);
    analogWrite(leftWheels, R2);
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void setup() {
  //direction control for motors, for use with L298 motor driver
  pinMode(RF, OUTPUT);
  pinMode(RB, OUTPUT);
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);

  //PWM pins to control speed of wheels
  pinMode(rightWheels, OUTPUT);
  pinMode(leftWheels, OUTPUT);


  analogWriteFrequency(25000);
  analogWriteResolution(10);

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
}
