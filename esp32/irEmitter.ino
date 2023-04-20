#include <IRremote.h> // >v3.0.0
                                            
#define PIN_SEND 5

void setup()  
{  
  IrSender.begin(PIN_SEND); // Initializes IR sender
}  
                               
void loop()  
{  
  IrSender.sendNEC(0x0102, 0x34); // the address 0x0102 with the command 0x34 is sent edited!!
  delay(500); // wait for half a second
}  
