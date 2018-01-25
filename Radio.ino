#include <RFM69.h>

#define NETWORKID     0   // This should really stay 0.  DONT CHANGE THIS
#define MYNODEID      147   // Easy to just keep this one and have a dest of two ALWAYS so we can easily determine generated syncword.
#define TONODEID      147   // NOTE - THIS MAY CHANGE TO ENABLE FREQUENCY SHARING
#define FREQUENCY     RF69_915MHZ //This translates directly to 915.000 MHz  - WARNING: THIS WILL CHANGE IN THE FUTURE
#define USEACK        false // Request ACKs or not - NOTE: we will never use ACK's in the competition

void setup_radio()
{    
    radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
    radio.setHighPower();
    radio.setPowerLevel(20);

    Serial.print("Radio Setup as node: ");
    Serial.print(MYNODEID,DEC);
    Serial.println();
}

void radio_send() {
    static float sendbuffer[14];
    static int i;
    i = 0;

    sendbuffer[i] = millis(); i++;
    sendbuffer[i] = myIMU.ax; i++;
    sendbuffer[i] = myIMU.ay; i++;
    sendbuffer[i] = myIMU.az; i++;
    sendbuffer[i] = myIMU.gx; i++;
    sendbuffer[i] = myIMU.gy; i++;
    sendbuffer[i] = myIMU.gz; i++;
    sendbuffer[i] = myIMU.mx; i++;
    sendbuffer[i] = myIMU.my; i++;
    sendbuffer[i] = myIMU.mz; i++;

    sendbuffer[i] = myIMU.temperature; i++;

    #if (ARHS)
        sendbuffer[i] = myIMU.yaw;   i++;
        sendbuffer[i] = myIMU.pitch; i++;
        sendbuffer[i] = myIMU.roll;  i++;
    #endif

    radio.send(TONODEID, sendbuffer, i * sizeof(*sendbuffer));
}
