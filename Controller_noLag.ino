#include <Bluepad32.h>
#include "motors.hpp"
#include <uni.h>

ControllerPtr myController;
static const char * controller_addr_string = "4E:BA:9B:A9:1D:75";   //Meu controle PS4
// static const char * controller_addr_string = "E4:17:D8:58:86:BE";   //8BitDo Controller Palma

void onConnectedController(ControllerPtr ctl) {
    Serial.printf("CALLBACK: Controller is connected\n");
    ControllerProperties properties = ctl->getProperties();
    Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
    myController = ctl;
}

void onDisconnectedController(ControllerPtr ctl) {
            Serial.printf("CALLBACK: Controller disconnected\n");
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // D-pad
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmask of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}

int16_t apply_deadZone(int16_t value){
    if(value > -5 && value < 5) value = 0;
    return value;
}

enum Controller_mode{
  PRO_MODE,
  EASY_MODE
};

Controller_mode mode = PRO_MODE;
int power = 255;

void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->y()) { // Triangle
        mode = EASY_MODE;
        ctl->setColorLED(0, 255, 0);  // Green
    }

    if (ctl->b()) { // Circle
        mode = PRO_MODE;
        ctl->setColorLED(255, 0, 0);  // Red
    }
    
    int16_t TH, ST;
    if(mode == EASY_MODE){
        ST = ST * 0.7;
        power = 128;
    }else{
        ST = ST * 0.5;
        power = 255;
    }
    
    TH = myController->throttle() - myController->brake();    // (-1020 - 1020) R2 - L2
    ST = myController->axisRX();                              // (-508 - 512) right X axis

    TH =  apply_deadZone(map(TH, -1020, 1020, -power, power));
    ST =  apply_deadZone(map(ST, -508, 512, -power, power));

    int16_t left, right;
    left  = constrain(TH + ST, -power, power);
    right = constrain(TH - ST, -power, power);

    Serial.printf("  %d\t%d\n", left, right);

    motor_control(LEFT, left);
    motor_control(RIGHT, right);
}

void processControllers() {
    if (myController && myController->isConnected() && myController->hasData()) {
        processGamepad(myController);
    }
    
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    motor_setup();
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    BP32.setup(&onConnectedController, &onDisconnectedController);
    bd_addr_t controller_addr;
    BP32.forgetBluetoothKeys();
    uni_bt_allowlist_remove_all();
    sscanf_bd_addr(controller_addr_string, controller_addr);
    uni_bt_allowlist_add_addr(controller_addr);
    uni_bt_allowlist_set_enabled(true);
    BP32.enableNewBluetoothConnections(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
}
