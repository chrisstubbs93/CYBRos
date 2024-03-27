/**
    \file Hoverboard.ino
    Handles comms and control of hoverboards.
*/



// Global variables for hoverboard
int16_t currentDriveMode = TRQ_MODE;
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  int16_t  brake;
  int16_t  driveMode;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

void Send(int16_t uSteer, int16_t uSpeed, int16_t brake, int16_t driveMode)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.brake    = (int16_t)brake;
  Command.driveMode = (int16_t)driveMode;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed ^ Command.brake ^ Command.driveMode);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command));
}

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