#include <driverlib.h>
//#include <Wire.h>

#define TMP102_ADDRESS 0x48
#define TMP102_ADDRESS_RECEIVE 0x49

static uint8_t receiveBuffer[2] = { 0x01, 0x01};
static uint8_t *receiveBufferPointer;
volatile unsigned char receiveCount = 0;
volatile uint8_t interruptCount = 0;


#define USE_B0

void setup()
{
  Serial.begin(9600);  

  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);  

  Serial.println("ready");
  delay(1000);
  digitalWrite(RED_LED, HIGH);  

  Serial.println("set");

  delay(1000);
  digitalWrite(RED_LED, LOW);  
  Serial.println("go");
  delay(500);
}

void loop() {
  #ifdef USE_B0
    //Assign I2C pins to USCI_B0
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN0 + GPIO_PIN1);
  #else
    //Assign I2C pins to USCI_B1
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2);
  #endif

  //Initialize transmit data packet
  uint8_t transmitData = 0b00000000;


  #ifdef USE_B0
    //Initialize Master
    USCI_B_I2C_masterInit(USCI_B0_BASE, USCI_B_I2C_CLOCKSOURCE_SMCLK, UCS_getSMCLK(), USCI_B_I2C_SET_DATA_RATE_100KBPS);

    //Specify slave address
    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE, TMP102_ADDRESS);

    //Set Transmit mode
    USCI_B_I2C_setMode(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_MODE);

    //Enable I2C Module to start operations
    USCI_B_I2C_enable(USCI_B0_BASE);

    //Enable TX interrupt
    USCI_B_I2C_clearInterruptFlag(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT);
    USCI_B_I2C_enableInterrupt(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT);
  #else
    //Initialize Master
    USCI_B_I2C_masterInit(USCI_B1_BASE, USCI_B_I2C_CLOCKSOURCE_SMCLK, UCS_getSMCLK(), USCI_B_I2C_SET_DATA_RATE_100KBPS);

    //Specify slave address
    USCI_B_I2C_setSlaveAddress(USCI_B1_BASE, TMP102_ADDRESS);

    //Set Transmit mode
    USCI_B_I2C_setMode(USCI_B1_BASE, USCI_B_I2C_TRANSMIT_MODE);

    //Enable I2C Module to start operations
    USCI_B_I2C_enable(USCI_B1_BASE);

    //Enable TX interrupt
    USCI_B_I2C_clearInterruptFlag(USCI_B1_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT);
    USCI_B_I2C_enableInterrupt(USCI_B1_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT);
  #endif

//  while (1) {
    Serial.println("test1");
  #ifdef USE_B0
    //Send single byte data.    
    USCI_B_I2C_masterSendSingleByte(USCI_B0_BASE, transmitData);
  #else
    //Send single byte data.    
    USCI_B_I2C_masterSendSingleByte(USCI_B1_BASE, transmitData);
  #endif
  
    Serial.println("test2");
    //Delay until transmission completes
    
  #ifdef USE_B0
    while (USCI_B_I2C_isBusBusy(USCI_B0_BASE)) ;
  #else
    while (USCI_B_I2C_isBusBusy(USCI_B1_BASE)) ;
  #endif
    Serial.println("test3");

    Serial.println(interruptCount);
    delay(1000);
    Serial.println("done transmit");
    

  // Now start receiving data from TMP102


    //Specify slave address, use 1 as the last bit to indicate read
//    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE, TMP102_ADDRESS);
    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE, TMP102_ADDRESS_RECEIVE);


  //Set receive mode
  USCI_B_I2C_setMode(USCI_B0_BASE, USCI_B_I2C_RECEIVE_MODE);
  Serial.println("test0");
  delay(500);

    //Enable I2C Module to start operations
    //USCI_B_I2C_enable(USCI_B0_BASE);

  //Enable master Receive interrupt
  USCI_B_I2C_clearInterruptFlag(USCI_B0_BASE, USCI_B_I2C_RECEIVE_INTERRUPT | USCI_B_I2C_TRANSMIT_INTERRUPT);
  USCI_B_I2C_enableInterrupt(USCI_B0_BASE, USCI_B_I2C_RECEIVE_INTERRUPT);
  Serial.println("test1");
  delay(500);

  //wait for bus to be free
  while (USCI_B_I2C_isBusBusy(USCI_B0_BASE )) ;
  Serial.println("test2");
  delay(500);

    receiveBufferPointer = (unsigned char*)receiveBuffer;
    receiveCount = 2;
    //Initialize multi reception
    USCI_B_I2C_masterMultiByteReceiveStart(USCI_B0_BASE);
    
//      while (USCI_B_I2C_isBusy(USCI_B0_BASE )) ;
//      *receiveBufferPointer++ = USCI_B_I2C_masterMultiByteReceiveNext(USCI_B0_BASE);
//      while (USCI_B_I2C_isBusy(USCI_B0_BASE )) ;
//      *receiveBufferPointer++ = USCI_B_I2C_masterMultiByteReceiveFinish(USCI_B0_BASE);
    
    
    Serial.println("test3");
//  delay(500);

    //Enter low power mode 0 with interrupts enabled.
    __bis_SR_register(LPM0_bits + GIE);
    __no_operation();
    Serial.println("test4");    
    Serial.println(interruptCount);

  Serial.println("done receive");
  delay(500);

//  int16_t t=(Wire.read() << 8 | Wire.read()) >> 4;
  Serial.print(receiveBuffer[0]);
  Serial.print(" ");
  Serial.print(receiveBuffer[1]);
  Serial.println("");
  
  Serial.println("start calculate temp");
  
  int16_t t=(receiveBuffer[0] << 8 | receiveBuffer[1]) >> 4;

  if (t>0b100000000000) t -= 4096;

  // Integer
  t *= 10; // one decimal place
  t += 8; // rounding
  Serial.print("tmp102 temperature ");
  Serial.print(t/160, DEC);
  Serial.print(".");
  Serial.print((t%160)/16, DEC);
  Serial.print("\n");



delay(3000);
}

__attribute__((interrupt(USCI_B0_VECTOR)))
void USCI_B0_ISR(void)
{
  
  interruptCount++;
    Serial.println("interrupt1");
  switch (UCB0IV) {
    //Vector 12: Transmit buffer empty - TXIF
    case USCI_I2C_UCTXIFG:
    {
      __no_operation();
      break;
    }
    case USCI_I2C_UCRXIFG:
    {
      //Decrement RX byte counter
      receiveCount--;
      if (receiveCount) {
        if (receiveCount == 1) {
          //Initiate end of reception -> Receive byte with NAK
          *receiveBufferPointer++ = USCI_B_I2C_masterMultiByteReceiveFinish(USCI_B0_BASE);
        }else  {
          //Keep receiving one byte at a time
          *receiveBufferPointer++ = USCI_B_I2C_masterMultiByteReceiveNext(USCI_B0_BASE);
        }
      }else  {
        //Receive last byte
        *receiveBufferPointer = USCI_B_I2C_masterMultiByteReceiveNext(USCI_B0_BASE);
        __bic_SR_register_on_exit(LPM0_bits);
      }
      break;
    }    
    default:  break;
  }
}


__attribute__((interrupt(USCI_B1_VECTOR)))
void USCI_B1_ISR(void)
{
    Serial.println("interrupt1");
  switch (UCB1IV) {
    //Vector 12: Transmit buffer empty - TXIF
    case USCI_I2C_UCTXIFG:
    {
      __no_operation();
      break;
    }
    case USCI_I2C_UCRXIFG:
    {
      //Decrement RX byte counter
      receiveCount--;
      if (receiveCount) {
        if (receiveCount == 1) {
          //Initiate end of reception -> Receive byte with NAK
          *receiveBufferPointer++ = USCI_B_I2C_masterMultiByteReceiveFinish(USCI_B1_BASE);
        }else  {
          //Keep receiving one byte at a time
          *receiveBufferPointer++ = USCI_B_I2C_masterMultiByteReceiveNext(USCI_B1_BASE);
        }
      }else  {
        //Receive last byte
        *receiveBufferPointer = USCI_B_I2C_masterMultiByteReceiveNext(USCI_B1_BASE);
        __bic_SR_register_on_exit(LPM0_bits);
      }
      break;
    }    
    default:  break;
  }
}


  
#ifdef NOT_IN_USE
void oldloop()
{
  // put your main code here, to run repeatedly:
  Wire.begin();
  Wire.beginTransmission(TMP102_ADDRESS);
    Serial.println("test1");
  Wire.write(0b00000000);
    Serial.println("test2");
  Wire.endTransmission();
    Serial.println("test3");

  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.requestFrom(TMP102_ADDRESS, 2);
  int16_t t=(Wire.read() << 8 | Wire.read()) >> 4;

  Wire.endTransmission();
  if (t>0b100000000000) t -= 4096;

  // Float adds 6 kB
  // Serial.print(t*0.0625, DEC);
  // Serial.print("\n");

  // Integer
  t *= 10; // one decimal place
  t += 8; // rounding
  Serial.print("tmp102 temperature ");
  Serial.print(t/160, DEC);
  Serial.print(".");
  Serial.print((t%160)/16, DEC);
  Serial.print("\n");

  delay(3000);  
}
#endif
