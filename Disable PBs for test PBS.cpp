Change this:

C++
        // RESTORED: Actually check if the pedals moved more than 8 units
        bool movedA = abs((int)calibratedA - (int)lastMidiA) > 8;
        bool movedB = abs((int)calibratedB - (int)lastMidiB) > 8;     
To this:

C++
        // OVERRIDDEN FOR TESTING: Force pedals to never "move"
        bool movedA = false;
        bool movedB = false;    
2. Re-apply the Hardware Clamps
If nothing is plugged into the analog pins, they act like little antennas and pick up electromagnetic noise from the room. We need to digitally tie them to 3.3V so they sit perfectly still.

Scroll down to your setup() function (around line 730) and uncomment those two lines we previously turned off:

Change this:

C++
    // --- PLACE THE OVERRIDES HERE FOR YOUR TEST ---
        //pinMode(pinPB, INPUT);
        //pinMode(pinPB2, INPUT);
    
    FilteredAnalog<>::setupADC();
To this:

C++
    // --- PLACE THE OVERRIDES HERE FOR YOUR TEST ---
    pinMode(pinPB, INPUT_PULLUP);
    pinMode(pinPB2, INPUT_PULLUP);
    
    FilteredAnalog<>::setupADC();