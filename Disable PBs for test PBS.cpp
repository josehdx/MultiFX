            int diffA = abs((int)calibratedA - (int)lastMidiA);
            int diffB = abs((int)calibratedB - (int)lastMidiB);
            
            // THRESHOLD DETECTION: This logic resets ONLY Screen Timers
            if (diffA > 256 || diffB > 256) {
                if (isScreenOff) turnScreenOn();
                lastScreenActivityTime = millis();
            }

            // ACTIVE MODE
            bool movedA = diffA > 8;
            bool movedB = diffB > 8;



to this

            int diffA = abs((int)calibratedA - (int)lastMidiA);
            int diffB = abs((int)calibratedB - (int)lastMidiB);
            
            // THRESHOLD DETECTION: Only PB1 (diffA) can wake the screen now
            if (diffA > 256) {
                if (isScreenOff) turnScreenOn();
                lastScreenActivityTime = millis();
            }

            // ACTIVE MODE
            bool movedA = diffA > 8;
            bool movedB = false; // PB2 is forced OFF and will be completely ignored