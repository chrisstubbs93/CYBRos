/**
    \file Hoverboard.ino
    Handles comms and control of hoverboards.
*/

#include "Settings.h"
#include "Globals.h"


void powerOnHB(){
        digitalWrite(FHBPwrCommandPin, HIGH);
        delay(500);
        digitalWrite(FHBPwrCommandPin, LOW);

        digitalWrite(RHB1PwrCommandPin, HIGH);
        delay(500);
        digitalWrite(RHB1PwrCommandPin, LOW);
        
        digitalWrite(RHB2PwrCommandPin, HIGH);
        delay(500);
        digitalWrite(RHB2PwrCommandPin, LOW);
}

void Send(int16_t uSteer, int16_t uSpeed, int16_t brake, int16_t driveMode)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.brake    = (int16_t)brake;
  Command.driveMode = (int16_t)driveMode;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed ^ Command.brake ^ Command.driveMode);

  // Write to all hoverboards
  Serial1.write((uint8_t *) &Command, sizeof(Command));
  Serial2.write((uint8_t *) &Command, sizeof(Command));
  Serial3.write((uint8_t *) &Command, sizeof(Command));
}


void Receive(Stream &port, SerialFeedback &Feedback, SerialFeedback &NewFeedback)
{
    // Check for new data availability in the Serial buffer
    if (port.available()) {
        incomingByte 	  = port.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}