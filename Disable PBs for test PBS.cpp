     int diffA = abs((int)calibratedA - (int)lastMidiA);
            int diffB = abs((int)calibratedB - (int)lastMidiB);
            
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
            
            if (diffA > 256) {
                if (isScreenOff) turnScreenOn();
                lastScreenActivityTime = millis();
            }

            // ACTIVE MODE
            bool movedA = diffA > 8;
            bool movedB = false; // PB2 is forced OFF and will be completely ignored



            or this


            bool movedA = false; // PB1 is forced OFF and will be completely ignored
            bool movedB = false;