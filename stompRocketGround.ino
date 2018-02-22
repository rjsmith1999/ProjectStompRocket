// RFM69HCW ground receiver, based on Sparkfun's example
// Include the RFM69 and SPI libraries:

#include <RFM69.h>
#include <SPI.h>

// Addresses for this node. CHANGE THESE FOR EACH NODE!

#define NETWORKID     0   // Must be the same for all nodes
#define MYNODEID      2   // My node ID
#define TONODEID      1   // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:

//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

// AES encryption (or not):

#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):

#define USEACK        true // Request ACKs or not

// Packet sent/received indicator LED (optional):

#define LED           3 // LED positive pin

// Toggle debug printing

//#define DEBUG_PRINT

// Create a library object for our RFM69HCW module:

RFM69 radio;

void setup()
{
  // Open a serial port so we can send keystrokes to the module:

  Serial.begin(115200);
  #ifdef DEBUG_PRINT
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  
  #endif
  
  // Set up the indicator LED (optional):

  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

  // Initialize the RFM69HCW:

  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW

  // Turn on encryption if desired:

  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);

#ifdef DEBUG_PRINT
  Serial.print("Stomp Rocket Ground Receiver Started\n");
#endif
}

void loop()
{
  digitalWrite(LED, HIGH);

  // RECEIVING

  // In this section, we'll check with the RFM69HCW to see
  // if it has received any packets:

  if (radio.receiveDone()) // Got one!
  {
    digitalWrite(LED, LOW);
    // Print out the information:
    #ifdef DEBUG_PRINT
    Serial.print("received from node ");
    Serial.print(radio.SENDERID, DEC);
    Serial.print("\n");
    #endif

    // The actual message is contained in the DATA array,
    // and is DATALEN bytes in size:

    int before = micros();
    int* recvData;
    recvData = (int*)radio.DATA;


    for (int i = 0; i < 7; i++){
      Serial.print(recvData[i]);
      Serial.print(" ");
    }

    Serial.print("\n");
    int after = micros();

   // Serial.println(after - before);
    
   // for (byte i = 0; i < radio.DATALEN; i++)
     // Serial.print((char)radio.DATA[i]);

    // RSSI is the "Receive Signal Strength Indicator",
    // smaller numbers mean higher power.

    #ifdef DEBUG_PRINT
    Serial.print("\n RSSI ");
    Serial.println(radio.RSSI);
    #endif

    // Send an ACK if requested.
    // (You don't need this code if you're not using ACKs.)

    if (radio.ACKRequested())
    {
      radio.sendACK();
      #ifdef DEBUG_PRINT
      Serial.println("ACK sent");
      #endif
    }
   // Blink(LED,10);
  }
}

void Blink(byte PIN, int DELAY_MS)
// Blink an LED for a given number of ms
{
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}
