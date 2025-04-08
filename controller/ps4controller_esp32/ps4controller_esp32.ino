#include <Bluepad32.h>
#include <HardwareSerial.h>
#include <Arduino.h>

#define L_AXISX_OFFSET 4
#define L_AXISY_OFFSET 4
#define R_AXISX_OFFSET 4
#define R_AXISY_OFFSET 4

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
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

void handleAxisData(int32_t input, char* axisData) {
  if (input < -512)
    input = -512;
  else if (input > 508)
    input = 508;

  int abs_num = input;
  char temp[4];
  if (abs_num < 0) {
    abs_num = -abs_num / 512.0 * 100;
    axisData[0] = '-';
  } else {
    abs_num = abs_num / 508.0 * 100;
    axisData[0] = '+';
  }
  sprintf(temp, "%d", abs_num);
  int len = strlen(temp);

  for (int i = 0; i < 3 - len; i++) 
    axisData[i + 1] = '0';

  strcpy(axisData + 1 + (3 - len), temp);
  strcat(axisData, "\0");
}

void processGamepad(ControllerPtr ctl) {
  char temp[4][5] = {""};
  handleAxisData(ctl->axisX() - L_AXISX_OFFSET, temp[0]);
  handleAxisData(ctl->axisY() - L_AXISY_OFFSET, temp[1]);
  handleAxisData(ctl->axisRX() - R_AXISX_OFFSET, temp[2]);
  handleAxisData(ctl->axisRY() - R_AXISY_OFFSET, temp[3]);
  char output[41] = "";
  sprintf(output, "c:%1x,%03x,%s,%s,%s,%s,%04d,%04d,%1x",
    ctl->dpad(),
    ctl->buttons(),
    temp[0],
    temp[1],
    temp[2],
    temp[3],
    ctl->brake(),
    ctl->throttle(),
    ctl->miscButtons());

  Serial.println(output);
  Serial.flush();
}

void processControllers() {
  if (myControllers[0] && myControllers[0]->isConnected() && myControllers[0]->hasData()) 
    if (myControllers[0]->isGamepad()) 
      processGamepad(myControllers[0]);
    else 
      Serial.println("Unsupported controller");
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  delay(1);
}