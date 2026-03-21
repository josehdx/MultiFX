Disable PBS

change from
// Movement Threshold: Only act if movement > 8 units (avoids micro-jitter)
bool movedA = abs((int)calibratedA - (int)lastMidiA) > 8;
bool movedB = abs((int)calibratedB - (int)lastMidiB) > 8;

to
// Movement Threshold: OVERRIDDEN FOR SLEEP TEST
bool movedA = false; // abs((int)calibratedA - (int)lastMidiA) > 8;
bool movedB = false; // abs((int)calibratedB - (int)lastMidiB) > 8;

void setup() {
    // ... (keep existing backlight/screen init code) ...

    btmidi.setName("Whammy_S3");
    pinMode(CAROUSEL_BUTTON_PIN, INPUT_PULLUP);
    pinMode(FREEZE_BUTTON_PIN, INPUT_PULLUP); 
    pinMode(0, INPUT_PULLUP); // Boot button

    // --- PLACE THE OVERRIDES HERE FOR YOUR TEST ---
    pinMode(pinPB, INPUT_PULLUP);  // Ties A0 to High to stop noise
    pinMode(pinPB2, INPUT_PULLUP); // Ties A2 to High to stop noise
    
    FilteredAnalog<>::setupADC();
    // ... (rest of setup) ...
}






Find these lines in MidiTask (around line 431):

C++
bool movedA = false; // abs((int)calibratedA - (int)lastMidiA) > 8;
bool movedB = false; // abs((int)calibratedB - (int)lastMidiB) > 8;


Change them back to the actual math:

C++
// Movement Threshold: Only act if movement > 8 units (avoids micro-jitter)
bool movedA = abs((int)calibratedA - (int)lastMidiA) > 8;
bool movedB = abs((int)calibratedB - (int)lastMidiB) > 8; 



2. Remove the INPUT_PULLUP Overrides in setup()
During the test, we digitally tied the analog pins to 3.3V (HIGH) so they wouldn't float and generate noise. If you leave these on, your pedals won't be able to sweep the voltage down to 0V.

Find these lines in setup() (around line 586):

C++
    // --- PLACE THE OVERRIDES HERE FOR YOUR TEST ---
    pinMode(pinPB, INPUT_PULLUP);  // Ties A0 to High to stop noise
    pinMode(pinPB2, INPUT_PULLUP); // Ties A2 to High to stop noise