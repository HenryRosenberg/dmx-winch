#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPDash.h>
#include <AccelStepper.h>
#include <esp_dmx.h>
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>

// WiFi Credentials <------------------------------------------------------- IMPORTANT: WiFi Credentials! <---------
const char* ssid = ""; // Network name
const char* password = ""; // Password
const char* hostname = "DMXWinch";

// DMX Dropout Protection: How long DMX must stay at 0 before acting on it (in milliseconds)
const unsigned long DMX_ZERO_DEBOUNCE_MS = 50; // Adjust this value to filter out momentary dropouts

// Hardware pinouts
const int PIN_STEPPER_PUL = 42;
const int PIN_STEPPER_DIR = 2;
const int PIN_STEPPER_ENA = 1;
const int PIN_STEPPER_BRAKE = 4;
const int PIN_LIMIT_SWITCH = 5;

const int PIN_DMX_RX = 11;  // RO on MAX485
const int PIN_DMX_TX = 14;  // DI on MAX485
const int PIN_DMX_EN = 12;  // DE and RE tied together on MAX485

const int TICKS_PER_REV = 3200;
const int GEAR_RATIO = 10;
const float DRUM_DIA_INCHES = 4.3;
const float INCH_TO_TICKS_RATIO = (GEAR_RATIO * TICKS_PER_REV) / (3.14159 * DRUM_DIA_INCHES);

int winchStrokeLength = 60; // in, Max extension of the winch
float dropDownAfterHoming = 2.0; // in, how much to lower the winch after hitting the endstop
bool autoHomeOnStartup = false; // Auto-home when powered on
bool autoHomeOnDMXZero = false; // Auto-home when DMX value is 0

bool enableDMXControl = true; // Can be disabled via webserver
bool brakeWhenNoMovement = false; // Can be enabled via webserver
bool brakeWhenAtZero = true; // Brake when at home position, can be disabled via webserver
bool isHoming = false; // Flag for non-blocking homing routine
bool homingDropping = false; // Track if we're in the drop-down phase of homing
bool eStopActive = false; // Emergency stop flag
bool dmxInitialized = false; // Track if DMX was successfully initialized

// DMX configuration
int dmxAddress = 1; // DMX start address (1-512)
const dmx_port_t dmxPort = DMX_NUM_1; // Port 0 on ESP32 is used for USB serial
unsigned long lastDmxPacket = 0;
bool dmxEverReceived = false;
const TickType_t DMX_POLL_TICKS = 0; // Non-blocking poll so loop stays responsive

// DMX dropout protection
unsigned long dmxZeroStartTime = 0; // When DMX first went to 0
bool dmxZeroConfirmed = false; // Has DMX been at 0 long enough?

// Persistent storage
Preferences preferences;

// NeoPixel setup
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// NeoPixel status colors
#define COLOR_OFF             pixel.Color(0,   0,   0)
#define COLOR_WIFI_CONNECTING pixel.Color(0,   0,   100) // Blue
#define COLOR_WIFI_CONNECTED  pixel.Color(0,   100, 100) // Cyan
#define COLOR_DMX_ACTIVE      pixel.Color(0,   100, 0)   // Green
#define COLOR_MANUAL_MODE     pixel.Color(100, 50,  0)   // Orange
#define COLOR_ESTOP           pixel.Color(100, 0,   0)   // Red
#define COLOR_HOMING          pixel.Color(50,  0,   50)  // Purple
#define COLOR_DMX_LOST        pixel.Color(100, 100, 0)   // Yellow

// Use DRIVER mode for PUL/DIR stepper drivers
AccelStepper winchMotor(AccelStepper::DRIVER, PIN_STEPPER_PUL, PIN_STEPPER_DIR);

AsyncWebServer server(80); // Start Webserver
ESPDash dashboard(server); // Attach ESP-DASH to AsyncWebServer

// Webserver widgets
dash::SeparatorCard paramsSeparator(dashboard,              "Parameters:");
dash::SliderCard strokeLengthSlider(dashboard,              "Stroke Length (in)", 0, 240);
dash::SliderCard velocitySlider(dashboard,                  "Max Velocity (in/s)", 0, 20);
dash::SliderCard accelSlider(dashboard,                     "Acceleration (in/s^2)", 0, 50);
dash::ToggleButtonCard brakeWhenNoMovementButton(dashboard, "Brake when stopped?");
dash::ToggleButtonCard brakeWhenAtZeroButton(dashboard,     "Brake when DMX=0?");

dash::SeparatorCard homingSeparator(dashboard,         "Homing Settings:");
dash::SliderCard dropDownSlider(dashboard,             "Drop after homing (in)", 0, 12);
dash::ToggleButtonCard homeButton(dashboard,           "Home now");
dash::ToggleButtonCard autoHomeButton(dashboard,       "Home at startup?");
dash::ToggleButtonCard autoHomeOnZeroButton(dashboard, "Home when DMX=0?");

dash::SeparatorCard dmxSeparator(dashboard,       "DMX Settings:");
dash::ToggleButtonCard enableDMXButton(dashboard, "Enable DMX?");
dash::SliderCard dmxAddressSlider(dashboard,      "DMX Address", 1, 512);
dash::GenericCard dmxStatusCard(dashboard,        "DMX Status:");

dash::SeparatorCard toolsSeparator(dashboard, "Utilities:");
dash::ToggleButtonCard eStopButton(dashboard, "🛑 E-STOP 🛑");
dash::ToggleButtonCard resetButton(dashboard, "Hard Reset");

dash::SeparatorCard manualControlsSeparator(dashboard, "Manual Controls (DMX must be disabled):");
dash::ToggleButtonCard raiseButton(dashboard,          "Raise to 0");
dash::ToggleButtonCard lowerButton(dashboard,          "Lower to max");
dash::SliderCard goToPositionSlider(dashboard,         "Go to position", 0, 240);


void updateNeoPixel() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  
  // Blink every 500ms for certain states
  if(millis() - lastBlink > 500) {
    blinkState = !blinkState;
    lastBlink = millis();
  }
  
  // Priority order of status (highest priority first)
  if(eStopActive) {
    // ESTOP: Blinking red
    pixel.setPixelColor(0, blinkState ? COLOR_ESTOP : COLOR_OFF);
  }
  else if(isHoming) {
    // Homing: Solid purple
    pixel.setPixelColor(0, COLOR_HOMING);
  }
  else if(enableDMXControl && dmxEverReceived && (millis() - lastDmxPacket < 2000)) {
    // DMX Active: Solid green
    pixel.setPixelColor(0, COLOR_DMX_ACTIVE);
  }
  else if(enableDMXControl && dmxEverReceived && (millis() - lastDmxPacket >= 2000)) {
    // DMX Signal Lost: Blinking yellow
    pixel.setPixelColor(0, blinkState ? COLOR_DMX_LOST : COLOR_OFF);
  }
  else if(!enableDMXControl) {
    // Manual Mode: Solid orange
    pixel.setPixelColor(0, COLOR_MANUAL_MODE);
  }
  else if(!dmxInitialized) {
    // DMX init failed: Solid red (dim)
    pixel.setPixelColor(0, COLOR_ESTOP);
  }
  else {
    // WiFi Connected, waiting for DMX: Solid cyan
    pixel.setPixelColor(0, COLOR_WIFI_CONNECTED);
  }
  
  pixel.show();
}

// Motor task - runs on Core 0 for maximum smoothness
void motorTask(void* parameter) {
  // Disable watchdog for this task
  disableCore0WDT();
  
  while(true) {
    winchMotor.run();
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n=== DMX Winch Controller Starting ===");

  // Initialize NeoPixel
  pixel.begin();
  pixel.setBrightness(50); // 0-255
  pixel.setPixelColor(0, COLOR_WIFI_CONNECTING); // Blue while connecting
  pixel.show();

  pinMode(PIN_STEPPER_PUL,   OUTPUT);
  pinMode(PIN_STEPPER_DIR,   OUTPUT);
  pinMode(PIN_STEPPER_ENA,   OUTPUT);
  pinMode(PIN_STEPPER_BRAKE, OUTPUT);
  pinMode(PIN_LIMIT_SWITCH,  INPUT_PULLDOWN);
  
  digitalWrite(PIN_STEPPER_ENA, HIGH); // Disable the stepper driver on startup
  digitalWrite(PIN_STEPPER_BRAKE, LOW); // Engage brake on startup

  // Load persistent settings
  preferences.begin("winch", false); // Open in read/write mode
  winchStrokeLength = preferences.getInt("strokeLength", 60);
  brakeWhenNoMovement = preferences.getBool("brakeWhenStop", false);
  brakeWhenAtZero = preferences.getBool("brakeAtZero", true);
  dmxAddress = preferences.getInt("dmxAddress", 1);
  enableDMXControl = preferences.getBool("enableDMX", true);
  dropDownAfterHoming = preferences.getFloat("dropDown", 2.0);
  autoHomeOnStartup = preferences.getBool("autoHome", false);
  autoHomeOnDMXZero = preferences.getBool("autoHomeDMX0", false);
  
  Serial.println("Loaded persistent settings:");
  Serial.print("  Stroke Length: "); Serial.println(winchStrokeLength);
  Serial.print("  Drop After Homing: "); Serial.println(dropDownAfterHoming);
  Serial.print("  Auto Home (Startup): "); Serial.println(autoHomeOnStartup ? "Yes" : "No");
  Serial.print("  Auto Home (DMX=0): "); Serial.println(autoHomeOnDMXZero ? "Yes" : "No");
  Serial.print("  DMX Zero Debounce: "); Serial.print(DMX_ZERO_DEBOUNCE_MS); Serial.println(" ms");
  Serial.print("  DMX Address: "); Serial.println(dmxAddress);
  Serial.print("  Enable DMX: "); Serial.println(enableDMXControl ? "Yes" : "No");
  Serial.print("  INCH_TO_TICKS_RATIO: "); Serial.println(INCH_TO_TICKS_RATIO);

  // Configure DMX (esp_dmx returns bool: true = success)
  Serial.println("Initializing DMX...");
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_personality_t personalities[] = {
    {1, "Winch Position"} // 1 channel personality
  };
  int personality_count = 1;

  if (dmx_driver_install(dmxPort, &config, personalities, personality_count)) {
    Serial.println("DMX driver installed");
    if (dmx_set_pin(dmxPort, PIN_DMX_TX, PIN_DMX_RX, PIN_DMX_EN)) {
      Serial.println("DMX pins configured");
      dmxInitialized = true;
      Serial.println("DMX initialized on port 1");
      Serial.print("Listening on DMX address: ");
      Serial.println(dmxAddress);
    } else {
      Serial.println("ERROR: Failed to set DMX pins");
    }
  } else {
    Serial.println("ERROR: Failed to install DMX driver");
  }

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.printf("WiFi Failed!\n");
      return;
  }
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());

  // mDNS allows the webpage to appear on http://DMXWinch.local
  if (!MDNS.begin(hostname)) {
    Serial.println("Error setting up MDNS");
  } else {
    Serial.print("mDNS started. Connect to: http://");
    Serial.print(hostname);
    Serial.println(".local");
  }

  WiFi.setAutoReconnect(true); // Automatically reconenct to WiFi if disconnected

  // Webserver callback configuration ---------------------------------------------
  enableDMXButton.onChange([](bool state){
    Serial.println(String("Enable DMX: ")+(state?"true":"false"));
    
    enableDMXControl = state;
    preferences.putBool("enableDMX", state);
    
    if (state) {
      dmxEverReceived = false;
      lastDmxPacket = 0;
      Serial.println("DMX state reset - ready for first packet");
    }

    enableDMXButton.setValue(state);
    dashboard.sendUpdates();
  });
  
  brakeWhenNoMovementButton.onChange([](bool state){
    Serial.println(String("Brake When No Movement: ")+(state?"true":"false"));
    
    brakeWhenNoMovement = state;
    preferences.putBool("brakeWhenStop", state);

    brakeWhenNoMovementButton.setValue(state);
    dashboard.sendUpdates();
  });

  brakeWhenAtZeroButton.onChange([](bool state){
    Serial.println(String("Brake When At Zero: ")+(state?"true":"false"));
    
    brakeWhenAtZero = state;
    preferences.putBool("brakeAtZero", state);

    brakeWhenAtZeroButton.setValue(state);
    dashboard.sendUpdates();
  });

  dropDownSlider.onChange([](int value){
    Serial.println("Drop Down Slider Triggered: "+String(value));
    
    dropDownAfterHoming = (float)value;
    preferences.putFloat("dropDown", dropDownAfterHoming);
    
    dropDownSlider.setValue(value);
    dashboard.sendUpdates();
  });

  autoHomeButton.onChange([](bool state){
    Serial.println(String("Auto Home on Startup: ")+(state?"true":"false"));
    
    autoHomeOnStartup = state;
    preferences.putBool("autoHome", state);

    autoHomeButton.setValue(state);
    dashboard.sendUpdates();
  });

  autoHomeOnZeroButton.onChange([](bool state){
    Serial.println(String("Auto Home on DMX=0: ")+(state?"true":"false"));
    
    autoHomeOnDMXZero = state;
    preferences.putBool("autoHomeDMX0", state);

    autoHomeOnZeroButton.setValue(state);
    dashboard.sendUpdates();
  });

  homeButton.onChange([](bool state){
    if(eStopActive) {
      Serial.println("ESTOP ACTIVE - Cannot home");
      return;
    }
    
    Serial.println("Home Button Triggered - Starting homing routine");
    
    if(state) {
      homingDropping = false; // Reset the homing state
      isHoming = true; // Start non-blocking homing routine
    }
  });

  eStopButton.onChange([](bool state){
    if(state) {
      Serial.println("🛑 EMERGENCY STOP ACTIVATED! 🛑");
      eStopActive = true;
      
      // Stop motor immediately
      winchMotor.stop();
      winchMotor.setCurrentPosition(winchMotor.currentPosition()); // Hold position
      
      // Engage brake and disable motor
      digitalWrite(PIN_STEPPER_BRAKE, LOW);
      digitalWrite(PIN_STEPPER_ENA, HIGH);
      
      // Disable DMX
      enableDMXControl = false;
      
      // Stop homing if active
      isHoming = false;
      
      eStopButton.setValue(true);
      dashboard.sendUpdates();
    }
  });

  resetButton.onChange([](bool state){
    Serial.println("Hard Reset Button Triggered");
    
    if(state) {
      // Engage brake and enable stepper driver during reset
      digitalWrite(PIN_STEPPER_BRAKE, LOW); // Enable motor brake
      digitalWrite(PIN_STEPPER_ENA, LOW); // Enable stepper driver
      
      delay(50); // Brief delay to ensure brake engages
      
      ESP.restart(); // Restart the MCU
    }
  });

  raiseButton.onChange([](bool state){
    if(eStopActive) {
      Serial.println("ESTOP ACTIVE! Cannot raise");
      return;
    }
    
    Serial.println("Raise Button Triggered - Moving to 0\"");

    if(!enableDMXControl && state) {
      winchMotor.moveTo(0); // Send winch to the top
    }
  });

  lowerButton.onChange([](bool state){
    if(eStopActive) {
      Serial.println("ESTOP ACTIVE! Cannot lower");
      return;
    }
    
    Serial.print("Lower Button Triggered - Moving to max (");
    Serial.print(winchStrokeLength);
    Serial.println("\")");

    if(!enableDMXControl && state) {
      winchMotor.moveTo(winchStrokeLength * INCH_TO_TICKS_RATIO); // Send winch down to its max
    }
  });

  strokeLengthSlider.onChange([](int value){
    Serial.println("Stroke Length Slider Triggered: "+String(value));
    
    winchStrokeLength = value; // Update global variable
    preferences.putInt("strokeLength", value);
    goToPositionSlider.setMax(winchStrokeLength); // Update other slider
    
    strokeLengthSlider.setValue(value);
    dashboard.sendUpdates();
  });

  goToPositionSlider.onChange([](int value){
    if(eStopActive) {
      Serial.println("ESTOP ACTIVE! Cannot move");
      return;
    }
    
    Serial.print("Go to Position Slider Triggered - Moving to ");
    Serial.print(value);
    Serial.println("\"");
    
    if(!enableDMXControl) {
      winchMotor.moveTo(value * INCH_TO_TICKS_RATIO); // Update motor position directly
    }
    
    goToPositionSlider.setValue(value);
    dashboard.sendUpdates();
  });

  velocitySlider.onChange([](int value){
    Serial.println("Velocity Slider Triggered: "+String(value));
    Serial.print("  Setting motor speed to: ");
    Serial.println(value * INCH_TO_TICKS_RATIO);
    
    preferences.putInt("maxVelocity", value);
    winchMotor.setMaxSpeed(value * INCH_TO_TICKS_RATIO); // Update motor directly
    
    velocitySlider.setValue(value);
    dashboard.sendUpdates();
  });

  accelSlider.onChange([](int value){
    Serial.println("Acceleration Slider Triggered: "+String(value));
    Serial.print("  Setting motor accel to: ");
    Serial.println(value * INCH_TO_TICKS_RATIO);
    
    preferences.putInt("maxAccel", value);
    winchMotor.setAcceleration(value * INCH_TO_TICKS_RATIO); // Update motor directly
    
    accelSlider.setValue(value);
    dashboard.sendUpdates();
  });

  dmxAddressSlider.onChange([](int value){
    Serial.println("DMX Address Slider Triggered: "+String(value));
    
    dmxAddress = value; // Update DMX address
    preferences.putInt("dmxAddress", value);
    
    dmxAddressSlider.setValue(value);
    dashboard.sendUpdates();
  });

  server.begin(); // Start AsyncWebServer

  // Sync widgets to loaded values and set initial motor parameters
  int initialVelocity = preferences.getInt("maxVelocity", 8);
  int initialAccel = preferences.getInt("maxAccel", 8);
  
  winchMotor.setMaxSpeed(initialVelocity * INCH_TO_TICKS_RATIO);
  winchMotor.setAcceleration(initialAccel * INCH_TO_TICKS_RATIO);
  winchMotor.setCurrentPosition(0);
  
  goToPositionSlider.setMax(winchStrokeLength);
  accelSlider.setValue(initialAccel);
  velocitySlider.setValue(initialVelocity);
  strokeLengthSlider.setValue(winchStrokeLength);
  goToPositionSlider.setValue(0);
  dmxAddressSlider.setValue(dmxAddress);
  dropDownSlider.setValue((int)dropDownAfterHoming);
  enableDMXButton.setValue(enableDMXControl);
  brakeWhenNoMovementButton.setValue(brakeWhenNoMovement);
  brakeWhenAtZeroButton.setValue(brakeWhenAtZero);
  autoHomeButton.setValue(autoHomeOnStartup);
  autoHomeOnZeroButton.setValue(autoHomeOnDMXZero);
  eStopButton.setValue(false);
  dashboard.sendUpdates();

  // End Webserver configuration -----------------------------------------------------

  Serial.print("Motor configured - Max Speed: ");
  Serial.print(initialVelocity * INCH_TO_TICKS_RATIO);
  Serial.print(" ticks/s, Max Accel: ");
  Serial.print(initialAccel * INCH_TO_TICKS_RATIO);
  Serial.println(" ticks/s^2");
  
  Serial.println("Setup complete!\n");

  // Start motor task on Core 0 for maximum smoothness
  xTaskCreatePinnedToCore(
    motorTask,        // Function to run
    "MotorTask",      // Task name
    4096,             // Stack size (bytes)
    NULL,             // Parameters
    1,                // Priority (1 = low, higher than idle)
    NULL,             // Task handle
    0                 // Core 0 (Core 1 runs WiFi/dashboard by default)
  );
  
  Serial.println("Motor task started on Core 0\n");

  // Auto-home on startup if enabled
  if(autoHomeOnStartup) {
    Serial.println("Auto-home enabled - starting homing sequence");
    delay(1000); // Brief delay to let everything settle
    isHoming = true;
  }
}

void loop() {
  // Update NeoPixel status (quick, non-blocking)
  updateNeoPixel();
  
  // ESTOP overrides everything
  if(eStopActive) {
    digitalWrite(PIN_STEPPER_BRAKE, LOW);  // Brake engaged
    digitalWrite(PIN_STEPPER_ENA, HIGH);   // Motor disabled
    
    static unsigned long lastEStopMsg = 0;
    if(millis() - lastEStopMsg > 5000) {
      Serial.println("🛑 ESTOP ACTIVE 🛑 Cycle power to continue");
      if(WiFi.isConnected()) {
        dmxStatusCard.setValue("🛑 ESTOP ACTIVE 🛑");
        dashboard.sendUpdates();
      }
      lastEStopMsg = millis();
    }
    return;
  }

  // Limit switch: reset position if at home and motor thinks it's below zero
  bool limitSwitchTriggered = (analogReadMilliVolts(PIN_LIMIT_SWITCH) > 2500);
  if (limitSwitchTriggered && winchMotor.currentPosition() < 0 && !isHoming) {
    Serial.println("SAFETY: Limit switch hit while below zero - resetting to 0");
    winchMotor.setCurrentPosition(0);
    winchMotor.moveTo(0);
  }

  // Non-blocking homing routine
  if (isHoming) {
    digitalWrite(PIN_STEPPER_BRAKE, HIGH);
    digitalWrite(PIN_STEPPER_ENA, LOW);
    
    if (!homingDropping && limitSwitchTriggered) {
      // Step 1: Hit limit switch, now drop down
      Serial.print("Limit switch hit - dropping down ");
      Serial.print(dropDownAfterHoming);
      Serial.println("\"...");
      winchMotor.setCurrentPosition(0); // Current position is now zero
      winchMotor.moveTo(dropDownAfterHoming * INCH_TO_TICKS_RATIO); // Move down by dropDown amount
      homingDropping = true;
    } 
    else if (homingDropping && !winchMotor.isRunning()) {
      // Step 2: Finished dropping, check if still on limit switch
      if(limitSwitchTriggered) {
        // Still on limit switch after dropping - need to drop more!
        Serial.println("Still on limit switch after drop - dropping another " + String(dropDownAfterHoming) + "\"");
        winchMotor.setCurrentPosition(0);
        winchMotor.moveTo(dropDownAfterHoming * INCH_TO_TICKS_RATIO);
        // Stay in homingDropping mode
      } else {
        // Successfully dropped off limit switch - complete homing
        Serial.println("Drop complete - setting new zero");
        winchMotor.setCurrentPosition(0); // This dropped position is now the new zero
        winchMotor.moveTo(0); // Target is zero (stay here)
        isHoming = false;
        homingDropping = false;
        Serial.println("*** HOMING COMPLETE ***");
      }
    } 
    else if (!homingDropping) {
      // Step 0: Still searching for limit switch
      winchMotor.moveTo(-100000);
      
      // Safety check: if motor has moved too far, abort homing
      if(winchMotor.currentPosition() <= -1.5 * winchStrokeLength * INCH_TO_TICKS_RATIO) {
        isHoming = false;
        homingDropping = false;
        Serial.println("HOMING ABORTED - Extension limit reached without hitting limit switch");
        winchMotor.setCurrentPosition(0);
        winchMotor.moveTo(0);
      }
    }
    return;
  }

  // DMX Control - FAST, non-blocking poll
  static uint8_t lastDmxValue = 255;  // Track last value to avoid redundant moveTo calls
  if (enableDMXControl && dmxInitialized) {
    dmx_packet_t packet;
    size_t slotsNeeded = (size_t)(dmxAddress + 1);
    size_t received = dmx_receive_num(dmxPort, &packet, slotsNeeded, DMX_POLL_TICKS);

    if (received && !packet.err && packet.size > (size_t)dmxAddress) {
      int slotValue = dmx_read_slot(dmxPort, dmxAddress);
      if (slotValue >= 0) {
        uint8_t dmxValue = (uint8_t)slotValue;
        
        // DROPOUT PROTECTION: Special handling for DMX = 0
        if(dmxValue == 0) {
          if(dmxZeroStartTime == 0) {
            // First time we've seen 0 - start the timer
            dmxZeroStartTime = millis();
            dmxZeroConfirmed = false;
          } else if(!dmxZeroConfirmed && (millis() - dmxZeroStartTime >= DMX_ZERO_DEBOUNCE_MS)) {
            // DMX has been at 0 for long enough - confirm it
            dmxZeroConfirmed = true;
            Serial.print("DMX=0 confirmed after ");
            Serial.print(DMX_ZERO_DEBOUNCE_MS);
            Serial.println("ms");
          }
        } else {
          // DMX is not 0 - reset the zero detection
          dmxZeroStartTime = 0;
          dmxZeroConfirmed = false;
        }
        
        // Only update position if DMX value changed AND if it's 0, it must be confirmed
        if(dmxValue != lastDmxValue) {
          if (!dmxEverReceived) {
            Serial.println("*** FIRST DMX PACKET RECEIVED! ***");
            dmxEverReceived = true;
          }
          
          // DMX = 0: Only act on it if confirmed (not a momentary dropout)
          if(dmxValue == 0) {
            if(dmxZeroConfirmed && autoHomeOnDMXZero) {
              Serial.println("DMX: Ch" + String(dmxAddress) + " = 0 (confirmed) → Starting auto-home");
              homingDropping = false; // Reset the homing state
              isHoming = true;
              lastDmxValue = dmxValue; // Update so we don't repeat
            }
            // If not confirmed yet, don't update lastDmxValue - wait for confirmation
          }
          else {
            // DMX 1-255: Map to stroke length (1 goes to top, 255 goes to max)
            int targetPosition = map(dmxValue, 1, 255, 0, winchStrokeLength);
            Serial.print("DMX: Ch");
            Serial.print(dmxAddress);
            Serial.print(" = ");
            Serial.print(dmxValue);
            Serial.print(" → Moving to ");
            Serial.print(targetPosition);
            Serial.println("\"");
            
            winchMotor.moveTo(targetPosition * INCH_TO_TICKS_RATIO);
            lastDmxValue = dmxValue;
          }
        }
        
        lastDmxPacket = millis();
      }
    }
  }


  // Brake and motor enable control  
  if(winchMotor.isRunning()) {
    digitalWrite(PIN_STEPPER_BRAKE, HIGH);
    digitalWrite(PIN_STEPPER_ENA, LOW);
  }
  else if(brakeWhenAtZero && winchMotor.currentPosition() == 0) {
    digitalWrite(PIN_STEPPER_BRAKE, LOW);
    digitalWrite(PIN_STEPPER_ENA, HIGH);
  }
  else if(brakeWhenNoMovement) {
    digitalWrite(PIN_STEPPER_BRAKE, LOW);
    digitalWrite(PIN_STEPPER_ENA, HIGH);
  }
  else {
    digitalWrite(PIN_STEPPER_BRAKE, HIGH);
    digitalWrite(PIN_STEPPER_ENA, LOW);
  }

  // Limit switch safety - CRITICAL: Detects problems!
  // Only check if NOT homing (homing handles its own limit switch logic)
  if (limitSwitchTriggered && !isHoming) {
    int currentPos = winchMotor.currentPosition();
    int targetPos = winchMotor.targetPosition();
    
    // Moving upward (target < current): Normal - trigger homing with drop
    if (targetPos < currentPos) {
      Serial.println("SAFETY: Limit switch hit during upward movement - starting homing with drop");
      winchMotor.stop();
      homingDropping = false; // Reset state for fresh homing
      isHoming = true; // This will trigger the full homing routine with drop
    }
    // Moving downward (target > current): PROBLEM - rope may be looped!
    else if (targetPos > currentPos) {
      Serial.println("🚨 CRITICAL ERROR: Limit switch hit during DOWNWARD movement!");
      Serial.println("🚨 This likely means the rope has looped around the drum!");
      Serial.println("🚨 Triggering E-STOP for safety");
      
      eStopActive = true;
      winchMotor.stop();
      digitalWrite(PIN_STEPPER_BRAKE, LOW);
      digitalWrite(PIN_STEPPER_ENA, HIGH);
      enableDMXControl = false;
      
      if(WiFi.isConnected()) {
        dmxStatusCard.setValue("🚨 ROPE LOOP ERROR 🚨");
        eStopButton.setValue(true);
        dashboard.sendUpdates();
      }
    }
  }

  // PERIODIC UPDATES (non-critical, every 1/4 second), only do if WiFi is connected
  static unsigned long lastPeriodicUpdate = 0;
  if((millis() - lastPeriodicUpdate > 250) && WiFi.isConnected()) {
    lastPeriodicUpdate = millis();
    
    // Update dashboard based on current mode
    if(enableDMXControl && dmxInitialized) {
      if(dmxEverReceived && (millis() - lastDmxPacket < 2000)) {
        int currentPos = winchMotor.targetPosition() / INCH_TO_TICKS_RATIO;
        dmxStatusCard.setValue(String("DMX: ") + String(lastDmxValue) + " → " + String(currentPos) + "\"");
      } else if(dmxEverReceived) {
        dmxStatusCard.setValue("DMX Signal Lost");
      } else {
        dmxStatusCard.setValue("Waiting for DMX...");
      }
      dashboard.sendUpdates();
    } else if(!enableDMXControl) {
      dmxStatusCard.setValue("Manual Control");
      dashboard.sendUpdates();
    } else {
      dmxStatusCard.setValue("DMX Init Failed");
      dashboard.sendUpdates();
    }
  }
}