#include <stdio.h>
#include "SPI.h"
#include "cmd_strobes.h"
#include "registers.h"

#define DEVICE_ADDRESS 0xA5
//define SPI pins
#define pin_SS 8
#define pin_MOSI 11
#define pin_MISO 12
#define pin_SCK 13
#define pin_RESETN 7

#define pin_greenLED 2
#define pin_redLED 3
#define pin_yellowLED 4

//define crystal oscillator frequency to 32MHz
#define f_xosc 32000000;

//declare variables to print status on time intervals
unsigned long previousTime = 0;
unsigned long currentTime = millis();
const int interval = 1000;
const long int ack_timeout = 2000;
long int lastTransmit = 0;
bool tx_fifo_err = false;
bool tx_mode = false;
bool rx_mode = true;
String message;
uint8_t address;

//************************************************************ SETUP ********************************************************//
void setup() {                
  // initialize the digital pin as an output.
  Serial.begin(9600); 

  //reset chip
  pinMode(pin_RESETN, OUTPUT);
  digitalWrite(pin_RESETN, HIGH);
  delay(50);
  digitalWrite(pin_RESETN, LOW);
  delay(50);
  digitalWrite(pin_RESETN, HIGH);
  
  //Configure SPI
  pinMode(pin_SS, OUTPUT);
  set_CSn(0);
  delay(50);
  SPI.begin();
  
  //This is the correct setting for the CC1120
  SPI.setBitOrder(MSBFIRST);
  
  //**********CONFIGURE CHIP***************//
      byte test;
      byte value;
      
      //Set SS
      set_CSn(0);
      
      while(digitalRead(pin_MISO))
        delay(100);
     
      //RESET 
      cmd_str(SRES, 0);             //SRES                  reset chip
      delay(100);
      Serial.print("\n*** Setup Chip Status***\n");
      p_chip_status();
      Serial.println();
      
      //Reset RX FIFO
      cmd_str(SFRX, 1);             //SFRX                  flush RX FIFO
      
      //Reset TX FIFO
      cmd_str(SFTX, 1);             //SFTX                  flush TX FIFO
  //********** END CONFIGURE CHIP**********//
      
      reg_setting();
  
   //**************SET UP TX****************//
  
  //************************************************************ MAIN PROGRAM *****************************************************//
   //turn on green LED when main code starts running
   Serial.println("\n*** Beginning of Main Program ***\n");
   
   //turn on green LED on breadboard and setup other LED pins as outputs
   pinMode(pin_greenLED, OUTPUT);
   pinMode(pin_redLED, OUTPUT);
   pinMode(pin_yellowLED, OUTPUT);
   digitalWrite(pin_greenLED, HIGH);
   
   //strobe commands to start RX
   cmd_str(SCAL, 1);                   //calibrate frequency synthesizer
   delay(250);
   cmd_str(SAFC, 1);
   delay(250);
    tx_fifo_err = false;
    rx_mode = true;
    message = "ACK";
    address = 0xFF;
    tx_mode = false;
    // Reset FIFO registers
    reg_write2F(TXFIRST, 0x00);
    // Put the ACK Packet in the FIFO
    dir_FIFO_write(0,(uint8_t)message.length()+1);
    dir_FIFO_write(1,address);
    for(uint8_t i=0; i<message.length(); i++) dir_FIFO_write(i+2, message[i]);
    reg_write2F(TXLAST, (uint8_t) message.length() + 2);
    reg_write2F(RXFIRST, 0x00);
    reg_write2F(RXLAST, 0x00);   
    cmd_str(SRX, 1);  // put in RX mode
}
   
//************************************************************ MAIN LOOP ********************************************************//
void loop() {
  //monitor_LEDs();
  //input_cmd_str();
  p_status_interval(); // print chip status in defined interval
  usr_serial_cmd(); // wait for user input to send a string over. Default in RX mode

  // Get the current state
  uint8_t state;
  bool CHIP_RDYn;
  get_status(&CHIP_RDYn, &state);

  // Waiting for acknowledge didnt get it yet
  if (tx_mode && state == STATERX){
    uint8_t rxFirst = reg_read2F(RXFIRST);
    uint8_t rxLast = reg_read2F(RXLAST);
    if (rxFirst){
      lastTransmit = millis();
      reg_write2F(RXFIRST,0);
      reg_write2F(RXLAST,0);
    }
    // Waited too long, resend
    if (millis()-lastTransmit>= ack_timeout){
      Serial.print("***ACK Wait Timeout, Retransmitting Message***\n");
      reg_write2F(TXFIRST, 0x00);         //set TX FIRST to 0
      cmd_str(STX, 1);                    //put in TX mode
      lastTransmit = millis();
    }
  }
  // Still sending the data that we have
  else if (tx_mode && state == STATETX){
    uint8_t rxFirst = reg_read2F(RXFIRST);
    uint8_t rxLast = reg_read2F(RXLAST);
    if (rxFirst){
      reg_write2F(RXFIRST,0);
      reg_write2F(RXLAST,0);
      lastTransmit = millis();
    }
  }
  // Waiting for data to come
  else if (rx_mode){
    // Get the data from the FIFO
    uint8_t rxFirst = reg_read2F(RXFIRST);
    uint8_t rxLast = reg_read2F(RXLAST);
    uint8_t resetRegs = 1;
    // Got some data
    if (rxFirst <= rxLast){
      uint8_t fifo[128] = {0};
      uint8_t j = 0;
      for (uint8_t i = rxFirst; i < rxLast; i++){
        fifo[j++] = dir_FIFO_read(i);
      }
      // We have a packet
      if (fifo[0] <= (rxLast - rxFirst - 1)){
        Serial.print("Got packet for address: ");
        Serial.print(fifo[1]);
        Serial.print(" msg: \"");
        for (j = 1; j < fifo[0]; j++){
          Serial.print((char) fifo[j+1]);
        }
        Serial.print("\"\n");
        reg_write2F(RXFIRST, fifo[0]);
        rxFirst += fifo[0];
      }
      // The packet doesn't seem to be done
      else if (fifo[0] >= (rxLast - rxFirst - 1)){
      }
    }
    if (rxFirst - rxLast == 0 && rxFirst){
      reg_write2F(RXFIRST, 0x00);
      reg_write2F(RXLAST, 0x00);
    }
    reg_write2F(TXFIRST, 0); // So we can send another ACK
  }
  // Something went wrong
  else {
  }


  /*
  // NOTE: need to find a way to update tx_success based on input from received side
  if(tx_success){
    cmd_str(SFTX,1); // flush tx fifo
    cmd_str(SRX, 1); //put in RX mode
  }*/
}



//************************************************************NEW READ AND WRITE FUNCTIONS***************************************//




//************************************************************ FUNCTION DECLARATION**********************************************//


//write to register address addr with data
byte reg_read(byte addr){
  
    addr = addr + B10000000; //add the read bit
    SPI.transfer(addr); //send desired address
    return SPI.transfer(0); //read back

}

//reads register in extended memory space
byte reg_read2F(byte addr){
    SPI.transfer(B10101111); //address extension command
    SPI.transfer(addr); //send the desired address
    return SPI.transfer(0); //read back
}

//write to register address addr with data
byte reg_write(byte addr, byte data){
    SPI.transfer(addr); //send desired address
    return SPI.transfer(data); //send desired address}
  
}

//rwrites to register in extended memory space
byte reg_write2F(byte addr, byte data){
    SPI.transfer(B00101111); //address extension command
    SPI.transfer(addr); //send desired address
    return SPI.transfer(data); //send desired address}
}

//writes status information to variables in main loop\
//Stephen: ????
void get_status(bool *CHIP_RDYn, byte *state){
  
  byte status_b = cmd_str(SNOP, 0);  
  *CHIP_RDYn = (status_b/128)%2; //7th bit (reading backwards)
  *state = ((status_b/64)%2)*4 + ((status_b/32)%2)*2 + (status_b/16)%2; //6th-4th bits (reading backwards)
  return;
}

//parses chip status byte
void p_chip_status(){
  byte status_b = cmd_str(SNOP, 0);
  byte CHIP_RDYn = (status_b/128)%2; //7th bit (reading backwards)
  byte state = ((status_b/64)%2)*4 + ((status_b/32)%2)*2 + (status_b/16)%2; //6th-4th bits (reading backwards)
  
  Serial.print("Chip status: ");
  
  if(CHIP_RDYn)
    Serial.print("NOT READY (1), ");
  else
    Serial.print("READY (0), ");
  
  //parses state  
  if(state == B000)
    Serial.println("IDLE (000)");
  else if(state == B001)
    Serial.println("RX (001)");
  else if(state == B010)
    Serial.println("TX (010)");
  else if(state == B011)
    Serial.println("FSTXON (011)");
  else if(state == B100)
    Serial.println("CALIBRATE (100)");
  else if(state == B101)
    Serial.println("SETTLING (101)");
  else if(state == B110)
    Serial.println("RX FIFO ERROR (110)");
  else if(state == B111){
    Serial.println("TX FIFO ERROR (111)");
    tx_fifo_err = true;
  }
  
  return;
}

//send command strobe and print info on command strobe if Print bit is true
byte cmd_str(byte addr, bool Print){
  
  //prints command strobe executed if applicable
  if(Print){
    Serial.print("Command strobe(");
    if(addr == SRES)
      Serial.println("0x30): SRES");
    else if(addr == SFSTXON)  
      Serial.println("0x31): SFSTXON");
    else if(addr == SXOFF)
      Serial.println("0x32): SXOFF");
    else if(addr == SCAL)
      Serial.println("0x33): SCAL");
    else if(addr == SRX)
      Serial.println("0x34): SRX");
    else if(addr == STX)
      Serial.println("0x35): STX");
    else if(addr == SIDLE)
      Serial.println("0x36): SIDLE");
    else if(addr == SAFC)
      Serial.println("0x37): SAFC");
    else if(addr == SWOR)
      Serial.println("0x38): SWOR");
    else if(addr == SPWD)
      Serial.println("0x39): SPWD");
    else if(addr == SFRX)
      Serial.println("0x3A): SFRX");
    else if(addr == SFTX)
      Serial.println("0x3B): SFTX");
    else if(addr == SWORRST)
      Serial.println("0x3C): SWORRST");
    else if(addr == SNOP)
      Serial.println("0x3C): SNOP");
  }
    
  return SPI.transfer(addr);
}

//reads FIFO using direct access
byte dir_FIFO_read(byte addr){
  SPI.transfer(B10111110); //direct FIFO read address
  SPI.transfer(addr); //send desired address
  return SPI.transfer(0); //read back
}

//writes in FIFO using direct access
byte dir_FIFO_write(byte addr, byte data){
  SPI.transfer(B00111110); //direct FIFO write address
  SPI.transfer(addr); //send desired FIFO address
  return SPI.transfer(data); //send desired data
}

//sets chip select to either LOW or HIGH
//STEPHEN: needs a check
void set_CSn(bool state){
  if(state)
    digitalWrite(pin_SS, HIGH);
  else
    digitalWrite(pin_SS, LOW);
}

//prints the value of each register
void p_all_reg(){
  
  for (int i = 0x00; i<=0x2E; i++){
   Serial.print("0x");
   if(reg_read(i)<0x10) {Serial.print("0");}
   Serial.println(reg_read(i), HEX);
 }
 
 for (int i = 0x00; i<=0x39; i++){
   Serial.print("0x");
   if(reg_read2F(i)<0x10) {Serial.print("0");}
   Serial.println(reg_read2F(i), HEX);
 }
 
 for (int i = 0x64; i<=0xA0; i++){
   Serial.print("0x");
   if(reg_read2F(i)<0x10) {Serial.print("0");}
   Serial.println(reg_read2F(i), HEX);
 }
 
 for (int i = 0xD2; i<=0xD9; i++){
   Serial.print("0x");
   if(reg_read2F(i)<0x10) {Serial.print("0");}
   Serial.println(reg_read2F(i), HEX);
 }
 
 return;
}


void usr_serial_cmd(){
  //inputs command from serial monitor
  if(Serial.available()){
    int count = 0;
    char cmd[128] = {0};
    char cD1[128] = {0} ;
    char cD2[1] = {0} ;
    delay(100); //delay so that the serial input doesn't get erased

    String usr_serial_str_in = "";
    char letter;
    
    do{  //continue reading input until nothing is input  
    letter = Serial.read();  //set as char
    usr_serial_str_in += letter;
    }while(Serial.available());
    // Got the reset command
    if(usr_serial_str_in == "RESET"){
      tx_fifo_err = false;
      rx_mode = true;
      message = "ACK";
      address = 0xFF;
      tx_mode = false;
      //strobe commands to reset
      cmd_str(SRES, 1);
      cmd_str(SFTX, 1); // flush TX FIFO
      cmd_str(SFRX, 1); // flush RX FIFO
      reg_setting(); // setup register setting
      // Reset FIFO registers
      reg_write2F(TXFIRST, 0x00);
      // Put the ACK Packet in the FIFO
      dir_FIFO_write(0,(uint8_t)message.length()+1);
      dir_FIFO_write(1,address);
      for(uint8_t i=0; i<message.length(); i++) dir_FIFO_write(i+2, message[i]);
      reg_write2F(TXLAST, (uint8_t) message.length() + 2);
      reg_write2F(RXFIRST, 0x00);
      reg_write2F(RXLAST, 0x00);   
      cmd_str(SRX, 1);  // put in RX mode
    }
    else if ((count = sscanf(usr_serial_str_in.c_str(), "%[^:]:%*c%[^:]%*c%*c%c", cmd, cD1, cD2)) >= 2){
      message = "";
      // If we just have a send we will broadcast
      if (strcmp(cmd,"SEND")== 0 && count == 2){
        message = cD1;
        address = 0xFF; // Broadcast
      }
      // Otherwise get the address and send to that
      else if (strcmp(cmd, "SEND") ==  0 && cD1[strlen(cD1)-2] == 'T' && cD1[strlen(cD1)-1] == 'O'){
        cD1[strlen(cD1)-3] = '\0';
        message = cD1;
        address = cD2[0];
      }
      if (message.length() > 0){
        Serial.print("\n*** Sending \"");
        Serial.print(message);
        Serial.print("\" to address: ");
        Serial.print((int) address);
        Serial.print('\n');
        if(message.length()<0x80){
          // The first byte is the length of the packet (message + 1 for the address)
          dir_FIFO_write(0,(uint8_t)message.length()+1);
          // The second byte is the address
          dir_FIFO_write(1,address);
          // The rest is the actual data
          for(int i=0x00; i<message.length(); i++) dir_FIFO_write(i+2, message[i]);
          //set up TX FIFO pointers
          reg_write2F(TXFIRST, 0x00);            //set TX FIRST to 0
          reg_write2F(TXLAST, message.length()+2); //set TX LAST (maximum OF 0X7F)
          reg_write2F(RXFIRST, 0x00);              //set TX FIRST to 0
          reg_write2F(RXLAST, 0x00); //set TX LAST (maximum OF 0X7F)
          //strobe commands to start TX
          cmd_str(STX, 1);                    //put in TX mode
          tx_mode = true;
          lastTransmit = millis();
        }
      }
      else {
        Serial.print("Error processing user command\n");
      }
    }
  }
}

//uses the p_chip_status() function to print status on regular time intervals
void p_status_interval(){
//print status on time intervals
 currentTime=millis();
 if(currentTime-previousTime>=interval){
   p_chip_status();
   byte TX_FIRST = reg_read2F(0xD3);
   byte TX_LAST = reg_read2F(0xD5);
   Serial.print("TX_FIRST: ");
   Serial.println(TX_FIRST, HEX);
   Serial.print("TX_LAST: ");
   Serial.println(TX_LAST, HEX);
   previousTime=currentTime;
 }
}


//changes the nth bit in register 'reg' to data
void reg_write_bit(int reg, int n, int data){
  int old_value = reg_read(reg);
  int power = pow(2, n);
  int new_value = old_value-(old_value%(2*power)) + data*power + old_value%power;
  reg_write(reg, new_value);
  return;
}

//changes the nth bit in register 'reg' to data (extended register space)
void reg_write_bit2F(int reg, int n, int data){
  int old_value = reg_read2F(reg);
  int power = pow(2, n);
  int new_value = old_value-(old_value%(power)) + data*power + old_value%power;
  reg_write2F(reg, new_value);
  return;
}

// setup register settings
void reg_setting(){
   //NOTE: This is based on SmartRF settings
  // 1. Open the device in smartRF
  // 2. 'Register Reset'
  // 3. High performance mode
  // 4. The regs should match below 

  //high performance settings
  reg_write2F(0x12, 0x00);          //FS_DIG1: 0x00         Frequency Synthesizer Digital Reg. 1
  reg_write2F(0x13, 0x5F);          //FS_DIG0: 0x5F         Frequency Synthesizer Digital Reg. 0
  reg_write2F(0x16, 0x40);          //FS_CAL1: 0x40         Frequency Synthesizer Calibration Reg. 1
  reg_write2F(0x17, 0x0E);          //FS_CAL0: 0x0E         Frequency Synthesizer Calibration Reg. 0
  reg_write2F(0x19, 0x03);          //FS_DIVTWO: 0x03       Frequency Synthesizer Divide by 2
  reg_write2F(0x1B, 0x33);          //FS_DSM0: 0x33         FS Digital Synthesizer Module Configuration Reg. 0
  reg_write2F(0x1D, 0x17);          //FS_DVCO: 0x17         Frequency Synthesizer Divider Chain Configuration ..
  reg_write2F(0x1F, 0x50);          //FS_PFD: 0x50          Frequency Synthesizer Phase Frequency Detector Con..
  reg_write2F(0x20, 0x6E);          //FS_PRE: 0x6E          Frequency Synthesizer Prescaler Configuration
  reg_write2F(0x21, 0x14);          //FS_REG_DIV_CML: 0x14  Frequency Synthesizer Divider Regulator Configurat..
  reg_write2F(0x22, 0xAC);          //FS_SPARE: 0xAC        Set up Frequency Synthesizer Spare
  reg_write2F(0x27, 0xB4);          //FS_VCO0: 0xB4         FS Voltage Controlled Oscillator Configuration Reg..
  reg_write2F(0x32, 0x0E);          //XOSC5: 0x0E           Crystal Oscillator Configuration Reg. 5
  reg_write2F(0x36, 0x03);          //XOSC1: 0x03           Crystal Oscillator Configuration Reg. 0
  
  //modulation and freq deviation settings
  reg_write(0x0A, B01001000);       //DEVIATION_M: 0x48      set DEV_M to 72 which sets freq deviation to 20.019531kHz (with DEV_M=5)
  reg_write(0x0B, B00000101);       //MODCFG_DEV_E: 0x05     set up modulation mode and DEV_E to 5 (see DEV_M register)
  reg_write(0x21, B00000100);       //FS_CFG: B00010100      set up LO divider to 8 (410.0 - 480.0 MHz band), out of lock detector disabled
  
  //set preamble
  reg_write(0x0D, 0x00);            //PREAMBLE_CFG1: 0x00    No preamble
  reg_write_bit(0x0E, 5, 0);        //PQT_EN: 0x00           Preamble detection disabled
  
  //TOC_LIMIT
  reg_write_bit2F(0x02, 7, 0);        //TOC_LIMIT: 0x00      Using the low tolerance setting (TOC_LIMIT = 0) greatly reduces system settling times and system power consumption as no preamble bits are needed for bit synchronization or frequency offset compensation (4 bits preamble needed for AGC settling).
  reg_write_bit2F(0x02, 6, 0);        //TOC_LIMIT: 0x00      Using the low tolerance setting (TOC_LIMIT = 0) greatly reduces system settling times and system power consumption as no preamble bits are needed for bit synchronization or frequency offset compensation (4 bits preamble needed for AGC settling).
  
  //set SYNC word
  reg_write_bit(0x08, 6, 0);        //PQT_GATING_EN: 0       PQT gating disabled (preamble not required)
  reg_write(0x09, B00010111);       //SYNC_CFG0: B00010111   32 bit SYNC word. Bit error qualifier disabled. No check on bit errors
  reg_write(0x04, 0x93);            //SYNC3: 0x93            Set SYNC word bits 31:24
  reg_write(0x05, 0x0B);            //SYNC2: 0x0B            Set SYNC word bits 23:16
  reg_write(0x06, 0x51);            //SYNC1: 0x51            Set SYNC word bits 15:8
  reg_write(0x07, 0xDE);            //SYNC0: 0xDE            Set SYNC word bits 7:0
  
  //set packets
  reg_write_bit(0x12, 6, 1);        //FIFO_EN: 0             FIFO enable set to true
  reg_write_bit(0x13, 6, 0);        //TRANSPARENT_MODE_EN: 0 Disable transparent mode
  reg_write(0x26, 0x00);            //PKT_CFG2: 0x00         set FIFO mode
  reg_write(0x27, B00110000);       //PKT_CFG1: 0x30         set address check and 0xFF broadcast
  reg_write(0x28, B00100000);       //PKT_CFG0: 0x30         set variable packet length
  reg_write(0x2E, 0x7F);            //PKT_LEN: 0xFF          set packet max packet length to 0x7F
  reg_write(0x1F, DEVICE_ADDRESS);  //DEV_ADDR register is set to DEVICE_ADDRESS
  reg_write(0x29, B01001111);       //RFEND_CFG1: 0x80       go to TX after a good packet
  reg_write(0x2A, B00110000);       //RFEND_CFG0: 0x30       go to RX after transmitting a packet
  
  //set power level
  reg_write(0x2B, B01111111);       //PA_CFG2: 0x7F          set POWER_RAMP to 64 (output power to 14.5dBm, equation 21)
  
  //frequency offset setting
  reg_write2F(0x0A, 0);             //FREQOFF1: 0x00         set frequency offset to 0
  reg_write2F(0x0B, 0);             //FREQOFF0: 0x00
  
  //Frequency setting
  reg_write2F(0x0C, 0x6C);          //FREQ2: 0x6C            set frequency to 434MHz (sets Vco, see equation from FREQ2 section of user guide)
  reg_write2F(0x0D, 0x80);          //FREQ1: 0x80
  reg_write2F(0x0E, 0x00);          //FREQ0: 0x00
}



