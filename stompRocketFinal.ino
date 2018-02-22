#include <RFM69.h>
#include <SPI.h>

#include "quaternionFilters.h"
#include "MPU9250.h"

#include <stdint.h>
#include "SparkFunBME280.h"

#include "Wire.h"
#include "SPI.h"



// Addresses for this node. CHANGE THESE FOR EACH NODE!

#define NETWORKID     0   // Must be the same for all nodes
#define MYNODEID      2   // My node ID
#define TONODEID      255   // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:

//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

// AES encryption (or not):

#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):

#define USEACK        false // Request ACKs or not


//#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging

#define LED 3

// Create a library object for our RFM69HCW module:

#define SCALING   1000

RFM69 radio;
MPU9250 myIMU;
BME280 mySensor;

int oldTime = 0;

// Use this to translate between floats and integers
// A union uses the same amount of data for storing ints or floats, so it can translate between the two
// without loosing the data in a float.
/*union floatInt_t{
  int i;
  float f;
};
*/
void setup()
{
  // Open a serial port so we can send keystrokes to the module:

  Serial.begin(9600);
  if(SerialDebug)
  {
    Serial.print("Node ");
    Serial.print(MYNODEID,DEC);
    Serial.println(" ready");  
  }

  // Set up the indicator LED (optional):
  // Initialize the RFM69HCW:
  if(SerialDebug)
  {
    Serial.println(radio.initialize(FREQUENCY, MYNODEID, NETWORKID) );
  }
  else
  {
    radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  }
  radio.setHighPower(); // Always use this for RFM69HCW

  
  radio.promiscuous(true);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  if(SerialDebug)
  {
    Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
    Serial.print(" I should be "); Serial.println(0x71, HEX);
  }

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    if(SerialDebug)
    {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
      
      Serial.print("x-axis self test: acceleration trim within : ");
      Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
      Serial.print("y-axis self test: acceleration trim within : ");
      Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
      Serial.print("z-axis self test: acceleration trim within : ");
      Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
      Serial.print("x-axis self test: gyration trim within : ");
      Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
      Serial.print("y-axis self test: gyration trim within : ");
      Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
      Serial.print("z-axis self test: gyration trim within : ");
      Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");
    }

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    
     myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    if(SerialDebug)
    {
      Serial.println("MPU9250 initialized for active data mode....");
    }

    
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }



  //***Driver settings********************************//
    //commInterface can be I2C_MODE or SPI_MODE
    //specify chipSelectPin using arduino pin names
    //specify I2C address.  Can be 0x77(default) or 0x76

    //For I2C, enable the following and disable the SPI section
    mySensor.settings.commInterface = I2C_MODE;
    mySensor.settings.I2CAddress = 0x77;

    //For SPI enable the following and dissable the I2C section
    //mySensor.settings.commInterface = SPI_MODE;
    //mySensor.settings.chipSelectPin = 10;


    //***Operation settings*****************************//

    //runMode can be:
    //  0, Sleep mode
    //  1 or 2, Forced mode
    //  3, Normal mode
    mySensor.settings.runMode = 3; //Forced mode

    //tStandby can be:
    //  0, 0.5ms
    //  1, 62.5ms
    //  2, 125ms
    //  3, 250ms
    //  4, 500ms
    //  5, 1000ms
    //  6, 10ms
    //  7, 20ms
    mySensor.settings.tStandby = 0;

    //filter can be off or number of FIR coefficients to use:
    //  0, filter off
    //  1, coefficients = 2
    //  2, coefficients = 4
    //  3, coefficients = 8
    //  4, coefficients = 16
    mySensor.settings.filter = 0;

    
    
    //tempOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.tempOverSample = 1;

    //pressOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.pressOverSample = 1;

    //humidOverSample can be:
    //  0, skipped
    //  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    mySensor.settings.humidOverSample = 1;
    
    delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.         Serial.begin(57600);
    if(SerialDebug)
    {
      Serial.print("Starting BME280... result of .begin(): 0x");
      //Calling .begin() causes the settings to be loaded
    }
    Serial.println(mySensor.begin(), HEX);
    
    oldTime = millis();

    pinMode(LED, OUTPUT);
}

void loop()
{
  // Set up a "buffer" for characters that we'll send:

  static int sendbuffer[7];
  static int sendlength = 4*7;
  static int newTime = 0;
  static int counter = 0;

  sendbuffer[0] = counter;
  ++counter;
 
  getIMUData(sendbuffer);

  sendbuffer[5] = getHeight();

  sendbuffer[6] = 96; //This can be used as a test to confirm proper transmission;

  newTime = millis();

  sendbuffer[1] = newTime - oldTime;
  oldTime = newTime;
  
  

  // SENDING

  if(SerialDebug)
  {
    Serial.println("Sending: ");
    printSendbuffer(sendbuffer);
  }
  digitalWrite(LED,HIGH);
  if (USEACK)
  {
     if (radio.sendWithRetry(TONODEID, sendbuffer, sendlength))
       if(SerialDebug)
       {
          Serial.println("ACK received!");
       }
     else
        if(SerialDebug)
        {
          Serial.println("no ACK received");
        }
  }

      // If you don't need acknowledgements, just use send():

 else // don't use ACK
 {
    radio.send(TONODEID, sendbuffer, sendlength);
    if(SerialDebug)
    {
      Serial.println("Message Sent");
    }
 }
 digitalWrite(LED, LOW);

  if(SerialDebug)
  {  
    delay(1000);
  }
  delay(5);
}


void getIMUData(int* sendbuffer)
{

  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    sendbuffer[2] = myIMU.accelCount[0]*myIMU.aRes * SCALING; // - accelBias[0];
    sendbuffer[3] = myIMU.accelCount[1]*myIMU.aRes * SCALING; // - accelBias[1];
    sendbuffer[4] = myIMU.accelCount[2]*myIMU.aRes * SCALING; // - accelBias[2];
  }
  
}

float getHeight()
{
  return mySensor.readFloatAltitudeFeet();
}

void printSendbuffer(int* sendbuffer){
  //union floatInt_t conv;
  Serial.print("Count = ");
  Serial.println(sendbuffer[0]);

  Serial.print("dTime = ");
  Serial.println(sendbuffer[1]);

  Serial.print("Accel X = ");
  //conv.i = sendbuffer[2];
  Serial.println(sendbuffer[2]);

  Serial.print("Accel Y = ");
  //conv.i = sendbuffer[3];
  Serial.println(sendbuffer[3]);

  Serial.print("Accel Z = ");
  //conv.i = sendbuffer[4];
  Serial.println(sendbuffer[4]);

  Serial.print("Height = ");
  //conv.i = sendbuffer[5];
  Serial.println(sendbuffer[5]);

  Serial.print("Test = ");
  //conv.i = sendbuffer[6];
  Serial.println(sendbuffer[6]);
}


