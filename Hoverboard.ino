/**
    \file Hoverboard.ino
    Handles comms and control of hoverboards.
*/

#include "Settings.h"
#include "Globals.h"

void Send(int16_t uSteer, int16_t uSpeed, int16_t brake, int16_t driveMode)
{
  int16_t uSpeed0, uSpeed1, uSpeed2;

  if(currentDriveMode == TRQ_MODE){
    //uSpeed0 = map(uSpeed,0,maxthrottleTRQ,0,FHBpowerSplit);
    //uSpeed1 = map(uSpeed,0,maxthrottleTRQ,0,RHB1powerSplit);
    //uSpeed2 = map(uSpeed,0,maxthrottleTRQ,0,RHB2powerSplit);
    uSpeed0 = uSpeed;
    uSpeed1 = uSpeed;
    uSpeed2 = uSpeed;
  } else {
    //Do not apply powersplit in speed mode because the axles will tug-of-war the chassis in half
    uSpeed0 = uSpeed;
    uSpeed1 = uSpeed;
    uSpeed2 = uSpeed;
  }
  // Create commands
  Hoverboard[0].Command.start    = (uint16_t)START_FRAME;
  Hoverboard[0].Command.steer    = (int16_t)uSteer;
  Hoverboard[0].Command.speed    = (int16_t)uSpeed0;
  Hoverboard[0].Command.brake    = (int16_t)brake;
  Hoverboard[0].Command.driveMode = (int16_t)driveMode;
  Hoverboard[0].Command.checksum = (uint16_t)(Hoverboard[0].Command.start ^ Hoverboard[0].Command.steer ^ Hoverboard[0].Command.speed ^ Hoverboard[0].Command.brake ^ Hoverboard[0].Command.driveMode);

  Hoverboard[1].Command.start    = (uint16_t)START_FRAME;
  Hoverboard[1].Command.steer    = (int16_t)uSteer;
  Hoverboard[1].Command.speed    = (int16_t)uSpeed1;
  Hoverboard[1].Command.brake    = (int16_t)brake;
  Hoverboard[1].Command.driveMode = (int16_t)driveMode;
  Hoverboard[1].Command.checksum = (uint16_t)(Hoverboard[1].Command.start ^ Hoverboard[1].Command.steer ^ Hoverboard[1].Command.speed ^ Hoverboard[1].Command.brake ^ Hoverboard[1].Command.driveMode);

  Hoverboard[2].Command.start    = (uint16_t)START_FRAME;
  Hoverboard[2].Command.steer    = (int16_t)uSteer;
  Hoverboard[2].Command.speed    = (int16_t)uSpeed2;
  Hoverboard[2].Command.brake    = (int16_t)brake;
  Hoverboard[2].Command.driveMode = (int16_t)driveMode;
  Hoverboard[2].Command.checksum = (uint16_t)(Hoverboard[2].Command.start ^ Hoverboard[2].Command.steer ^ Hoverboard[2].Command.speed ^ Hoverboard[2].Command.brake ^ Hoverboard[2].Command.driveMode);

  // Write to all hoverboards
  Serial1.write((uint8_t *) &Hoverboard[0].Command, sizeof(Hoverboard[0].Command));
  Serial2.write((uint8_t *) &Hoverboard[1].Command, sizeof(Hoverboard[1].Command));
  Serial3.write((uint8_t *) &Hoverboard[2].Command, sizeof(Hoverboard[2].Command));
}


void Receive(Stream &port, SerialFeedback &Feedback, SerialFeedback &NewFeedback, unsigned long &lastTimestamp)
{
  while(port.available()>0){
      // Check for new data availability in the Serial buffer
      if (port.available()) {
        for (int x = 0; x < sizeof(telemBuffer)-1; x++) {
          telemBuffer[x] = telemBuffer[x+1]; //cycle the buffer
        }
        telemBuffer[sizeof(telemBuffer)-1] = port.read();
        bufStartFrame	= ((uint16_t)(telemBuffer[1]) << 8) | (uint16_t)telemBuffer[0];       // Construct the start frame
      }
      else {
          return;
      }
      NewFeedback.start = ((uint16_t)(telemBuffer[1]) << 8) | (uint16_t)telemBuffer[0];
      NewFeedback.cmd1 = ((uint16_t)(telemBuffer[3]) << 8) | (uint16_t)telemBuffer[2];
      NewFeedback.cmd2 = ((uint16_t)(telemBuffer[5]) << 8) | (uint16_t)telemBuffer[4];
      NewFeedback.speedR_meas = ((uint16_t)(telemBuffer[7]) << 8) | (uint16_t)telemBuffer[6];
      NewFeedback.speedL_meas = ((uint16_t)(telemBuffer[9]) << 8) | (uint16_t)telemBuffer[8];
      NewFeedback.batVoltage = ((uint16_t)(telemBuffer[11]) << 8) | (uint16_t)telemBuffer[10];
      NewFeedback.dcCurrent = ((uint16_t)(telemBuffer[13]) << 8) | (uint16_t)telemBuffer[12];
      NewFeedback.boardTemp = ((uint16_t)(telemBuffer[15]) << 8) | (uint16_t)telemBuffer[14];
      NewFeedback.cmdLed = ((uint16_t)(telemBuffer[17]) << 8) | (uint16_t)telemBuffer[16];
      NewFeedback.checksum = ((uint16_t)(telemBuffer[19]) << 8) | (uint16_t)telemBuffer[18];
      uint16_t checksum;
      checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.dcCurrent ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
      // Check validity of the new data
      if (bufStartFrame == START_FRAME && checksum == NewFeedback.checksum) {
          // Copy the new data
          memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
          lastTimestamp = millis();
      } else {
        //decode fail
      }
  }
}

bool isHoverboardConnected(int hb){

  if (millis() - Hoverboard[hb].lastTimestamp > HB_TIMEOUT){
    char disconnectWarn[128];
    sprintf(disconnectWarn, "Hoverboard %i disconnected for %lu", hb, (millis() - Hoverboard[hb].lastTimestamp));
    sendError(disconnectWarn);

    if (millis() - Hoverboard[hb].lastTimestamp > HB_TIMEOUT_START){
      if (millis() - Hoverboard[hb].restartTimestamp > HB_RESTART_WAIT){
        char restartWarn[128];
        sprintf(restartWarn, "Hoverboard %i restarting", hb);
        sendError(restartWarn);
        if(hb == 0) {
          digitalWrite(FHBPwrCommandPin, HIGH);
          delay(100);
          digitalWrite(FHBPwrCommandPin, LOW);
          Hoverboard[hb].restartTimestamp = millis();
        }
        if(hb == 1) {
          digitalWrite(RHB1PwrCommandPin, HIGH);
          delay(100);
          digitalWrite(RHB1PwrCommandPin, LOW);
          Hoverboard[hb].restartTimestamp = millis();
        }
        if(hb == 2) {
          digitalWrite(RHB2PwrCommandPin, HIGH);
          delay(100);
          digitalWrite(RHB2PwrCommandPin, LOW);
          Hoverboard[hb].restartTimestamp = millis();
        }
      }
    }
    return false;
  } else {
    return true;
  }

}

void Receiveold(Stream &port, SerialFeedback &Feedback, SerialFeedback &NewFeedback)
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
        Serial.println("Start byte");
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
                            ^ NewFeedback.batVoltage ^ NewFeedback.dcCurrent ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

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
    } else {
      Serial.print("idx:");
      Serial.println(idx);
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

void serialFlush(Stream &port){
  while(port.available() > 0) {
    char t = port.read();
  }
}