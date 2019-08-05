// correction of word "Folks"...
// correction of 000~111 colors

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
//sign & c are initialized inside inline asm code
//register uint8_t sign, c;
#pragma GCC diagnostic pop

//#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "stdint.h"
#include <Arduino.h>

#include "SoftwareI2C.h" // This library is only to run with find slaves on BUS of SCL&SCA.
SoftwareI2C softwarei2c;  

#define sda 7
#define scl 8

#define addressWR  0x70
#define addressRD  0x71

// add this to the top of your sketch
#define NOP __asm__ __volatile__ ("nop\n\t")

// and then use it in code as follows
// NOP; // delay 62.5ns on a 16MHz AtMega

int flag = 0;
int ack;

char value=0;  // Note: This need be only char, because need support the value of "-1" to compare the decrement of "value" variable!!!
unsigned char positionArray=0;

void nop(void);

unsigned char reead, write2, readed;
unsigned char rd,wrt,wrt2,i,j;
unsigned int clkHigh750nSec=750; // must be greater than 600 nano Sec //  to debug the timers of SDC & SCL // 1.3 uSecs until start new communication at least!
unsigned int sclLow1300nSec=1300;  // minimo 1.3uSeconds
unsigned int sdaLow110nSec=110; // time between data I'm let the time of uP communication set
unsigned int sdaHigh110nSec=110;
unsigned char counter=0;

// This is the construct of array of chars, please check if is compatible to ASCII table!!!
// I'm only create the chars to the message I want sent, construct the other symbols!!!
 unsigned char letterB[]{0x00,0xA5,0x30};
 unsigned char letterL[]{0x05,0x80,0x00};
 unsigned char letterA[]{0x07,0x05,0x30};
 unsigned char letterU[]{0x05,0x80,0x30};
 unsigned char letterP[]{0x07,0x05,0x10};
//            Letter U already done
 unsigned char letterN[]{0x05,0x18,0x30};
 unsigned char letterK[]{0x07,0x0A,0x00};
 unsigned char letterT[]{0x00,0x21,0x00};

 unsigned char letterR[]{0x07,0x0D,0x10};
 unsigned char letterO[]{0x05,0x81,0x30};
 unsigned char letterY[]{0x00,0x52,0x00};
 unsigned char letterM[]{0x05,0x12,0x30};

 unsigned char letterF[]{0x07,0x05,0x00};
 unsigned char letterS[]{0x03,0x85,0x20};
 unsigned char letterH[]{0x07,0x04,0x30};
 unsigned char letterI[]{0x00,0xA1,0x00};
 unsigned char letter_[]{0x00,0x00,0x00};
 
 unsigned char text[30]; //Array used to keep the sequence of bytes to send to PCF8576DH, when send it by array metod.
 // Bellow it the operation OR between the last and first byte of each group of 3 bytes
 // Note the first group of 3 dont have operation OR on the firt 8bits, ok
 // Look to the third byt of 1º group: (0x30 | 0x05) Stay 0x35 to send!!!

//{0x00,0xA5,0x30};
//           {0x05,0x80,0x00};
//                     {0x07,0x05,0x30};
//                               {0x05,0x80,0x30};
//                                          {0x07,0x05,0x10};
//                                                    {0x05,0x80,0x30};
//                                                              {0x05,0x18,0x30};
//                                                                         {0x07,0x0A,0x00};
// Last of word, Make a OR to the firts of next byte                                 {0x00,0x21,0x00};

/*********************************************************/
void noOperand(){
  asm volatile (
    "sbi %0, %1 \n" //pinMode(13, OUTPUT);
    : : "I" (_SFR_IO_ADDR(DDRB)), "I" (DDB5)
  );
}

void nop(unsigned int multiple){
 // and then use it in code as follows
 for (int i =0; i< multiple; i++){
  NOP; // delay 62.5ns on a 16MHz AtMega
 }
}
//*********************myDelay****************************/
void myDelay(unsigned int count)
{
  int i,j;
  for(i=0;i<count;i++)
    for(j=0;j<1275;j++);
}
//******************** ZONE of I2C communicaations *********************//
//*********************START****************************/
void start(){    //start condition
  // Time need waite until start new communication. ( 1.3 uSec) but the delay cant be done here,I'm do it on stp
   // The start condition is: Downing SDA while SCL is HIGH
   // The status of SCL is HIGH because the Stop let it in this value!
  digitalWrite(scl, LOW);
  nop(10);
  digitalWrite(sda, HIGH);
  nop(10);
  digitalWrite(scl, HIGH);
  nop(10);
  digitalWrite(sda,LOW);
  nop(10);
}
//*********************STOP****************************/
void stp(){     //stp condition
  digitalWrite(sda, LOW);
  nop(10);
  digitalWrite(scl, HIGH);  // The SCL stay every time on upper value, this mean must finish allways upper!
  nop(10);
  digitalWrite(sda, HIGH); // The stop condition is: Rise SDA while SCL is HIGH
  nop(10);
  delayMicroseconds(2);  // time of free bus before a new start communication(1.3 uSec)
}
//*********************Slave AKNOWLEDGE****************************/
void readAcknowledge(){    //acknowledge condition
  digitalWrite(scl, LOW);
  nop(10);
  pinMode(sda, INPUT);
  nop(10);
  digitalWrite(scl, HIGH);
  nop(20);
  digitalWrite(scl, LOW);
  nop(10);
  digitalWrite(sda, HIGH);
  nop(10);
  pinMode(sda, OUTPUT);
  nop(10);
}
//*********************Master AKNOWLEDGE****************************/
void sendAcknowledge(){    //acknowledge condition
  // This is used when the Master is reading and need send a ACK by each byte received drom Slave!
  // SDA must be as a output pin to force bus sda go down!
  digitalWrite(scl, LOW);
  nop(10);
  digitalWrite(sda, LOW); // Here is the master wich send a ack to the slave!!!
  nop(10);
  digitalWrite(scl, HIGH);
  nop(10);
  digitalWrite(scl, LOW);
  nop(10);
  digitalWrite(sda, HIGH);
  nop(10);
}
void noAcknowledge(){    //acknowledge condition
  digitalWrite(scl, LOW);
  nop(10);
  digitalWrite(sda, HIGH);
  nop(10);
  digitalWrite(scl, HIGH);
  nop(10);
  ack=digitalRead(PD7);         //reading acknowledge
  digitalWrite(scl, LOW);
  nop(10);
}
//******************** END ZONE of I2C communicaations *********************//


//******************* ZONE of communication to the Slave Sent mode ****************************//
void send_byte(unsigned char Bits){  //send byte serially
// On the send Byte, the SDA must be High to allow the 
//receiver send a low pulse in sda(like a pull-up status)
unsigned char data = 170; //value to transmit, binary 10101010
unsigned char mask = 1; //our bitmask
data=Bits;
digitalWrite(scl, LOW);   // Must stay low value after 8 bits
nop(10);
pinMode(sda, OUTPUT);
// The start let the SCL in low value
          for (mask = 10000000; mask>0; mask >>= 1) { //iterate through bit mask
                  if (data & mask){ // if bitwise AND resolves to true
                    digitalWrite(sda, HIGH);
                    nop(10);
                    //Serial.print("1");
                  }
                  else{ //if bitwise and resolves to false
                    digitalWrite(sda, LOW);
                    nop(10);
                    //Serial.print("0");
                  }
                  
            //Note: The change of data must occurr while the SCL is LOW, only after the pulse of SCL take place!
            digitalWrite(scl, HIGH);  // Generate a pulse to validation of data on bus.
            nop(10);
            digitalWrite(scl, LOW);   // Must stay low value after 8 bits
            nop(10);
          }
        pinMode(sda, INPUT);
}
//
unsigned char read_byte(){     //reading from EEPROM serially
 unsigned int i;
int val = 0;      // variable to store the read value 
 reead=0;
 digitalWrite(scl, LOW);
 pinMode(sda, INPUT);
 nop(30);
        for(i=0;i<8;i++){
          reead=reead<<1;
          digitalWrite(scl, HIGH);
          nop(50);
          val = digitalRead(PD7);
            if(val == 1){
            reead++;}
       digitalWrite(scl, LOW);
       nop(50);   
        }
 pinMode(sda, OUTPUT);
 nop(30);
  //Serial.print(" reead: ");  // This print lines do a delay to big... use it only to debug!
  //Serial.println(reead, BIN);// Only to debug
  return reead;       //Returns 8 bit data here
}
//
void readBIT(void){
  String variable = "PORTD";

  if ( variable == "PORTD" )
  {
    Serial.println(bitRead(PORTD,3));
  }
}
void save(){   //save in EEPROM
  start();
  send_byte(0xA0);            //device address
  readAcknowledge();
  send_byte(0x00);            //word address
  readAcknowledge();
  send_byte(5);               //send data
  readAcknowledge();
  send_byte(65);
  readAcknowledge();
  stp();
  if(ack=0)
  {
    digitalWrite(13, HIGH);
    myDelay(100);
    digitalWrite(13, LOW);
    myDelay(100);
  }
  else
  digitalWrite(13, HIGH);
  readAcknowledge();
}
/************************ Test bit by bit display *****************************/
void testBitPFC8576DH(){
  // Congigure te time and day to the module DS3231 
    start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0b11001100);   //  C 1 0 X E B M1 M0   // E= Enable 1
      readAcknowledge();
      send_byte(0b10000000);   //  C 0 A A A A A A // 6 bits of address of 40 seg.
      readAcknowledge();
      send_byte(0b11100000);   //  C 1 1 0 0 A2 A1 A0 //8 of Subaddress
      readAcknowledge();
      send_byte(0b11111000);   //  C 1 1 1 1 0 1 1 // Bank 0 or 1 of arriving data
      readAcknowledge();
      send_byte(0b01110000);   //  C 1 1 1 0 A BF1 BF0 // Blink and Hz  BF1 & BF0 Display blinking at 0,5 HZ
      readAcknowledge();
    //start();
    //Note: The write mode finish use allways the Acknowledge sended by the SLAVE!
     // send_byte(addressWR);// 
     // readAcknowledge();
    
      send_byte(0xFF);// 0
      readAcknowledge();
      send_byte(0xFF);// 1
      readAcknowledge();
      send_byte(0xFF);// 2
      readAcknowledge();
      send_byte(0xFF);// 3
      readAcknowledge(); 
      send_byte(0xFF);// 4
      readAcknowledge();
      send_byte(0xFF);// 5
      readAcknowledge();
      send_byte(0xFF);// 6
      readAcknowledge();
      send_byte(0xFF);// 7
      readAcknowledge();
      send_byte(0xFF);// 8
      readAcknowledge();
      send_byte(0xFF);// 9
      readAcknowledge();
      send_byte(0xFF);// A
      readAcknowledge();
      send_byte(0xFF);// B
      readAcknowledge();
      send_byte(0xFF);// C
      readAcknowledge();
      send_byte(0xFF);// D
      readAcknowledge();
      send_byte(0xFF);// E
      readAcknowledge(); 
      send_byte(0xFF);// F
      readAcknowledge();
      send_byte(0xFF);// G
      readAcknowledge();
      send_byte(0xFF);// H
      readAcknowledge();
      send_byte(0xFF);// I
      readAcknowledge();
      send_byte(0xFF);// J
      readAcknowledge();
        
    stp();
}
/***************************** HI FOLKS *********************************/
void HiFolksArray(void){
  unsigned char letter3;
  unsigned char letter1;
  int v =0;
  text[0]=letterH[0];                   text[1]= letterH[1];             // 1xx, 2xx, 
  text[2]=(letterI[0]  | letterH[2]);   text[3]= letterI[1];             // 3xx, 4xx,
  text[4]=(letter_[0]  | letterI[2]);   text[5]= letter_[1];             // 5xx, 6xx,
  text[6]=(letter_[0]  | letter_[2]);   text[7]= letter_[1];             // 5xx, 6xx, 
  text[8]=(letterF[0]  | letter_[2]);   text[9]= letterF[1];             // 7xx, 8xx,
  text[10]=(letterO[0]  | letterF[2]);   text[11]= letterO[1];             // 9xx, Axx, 
  text[12]=(letterL[0] | letterO[2]);   text[13]=letterL[1];             // Bxx, Cxx,
  text[14]=(letterK[0] | letterL[2]);   text[15]=letterK[1];            // Dxx, Exx,
  text[16]=(letterS[0] | letterK[2]);   text[17]=letterS[1]; text[18]=letterS[2];  // Fxx, Gxx, Hxx
  
  //
  start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0b11001000);   //  C 1 0 X E B M1 M0   // E= Enable 1
      readAcknowledge();
      send_byte(0b10000000);   //  C 0 A A A A A A // 6 bits of address of 40 seg.
      readAcknowledge();
      send_byte(0b11100000);   //  C 1 1 0 0 A2 A1 A0 //8 of Subaddress
      readAcknowledge();
      send_byte(0b11111000);   //  C 1 1 1 1 0 1 1 // Bank 0 or 1 of arriving data
      readAcknowledge();
      send_byte(0b01110000);   //  C 1 1 1 0 A BF1 BF0 // Blink and Hz  BF1 & BF0 Display blinking at 0,5 HZ
      readAcknowledge();
      //
      send_byte(0x00); // This byte is to avoid turn on the symbols at first address!
      readAcknowledge();
      //
      for(i=0;i<27;i++){
        v++;
            if (v==3){
              //Serial.println(text[i], HEX);
              v=0;
            }
              else{
               // Serial.print(text[i], HEX); Serial.print(" ");
              }
              
            send_byte(text[i]);
            readAcknowledge();
            }
      stp();
}
/*********************************** BLAUPUNKT By Array ******************************/
void BlaupunktArray(void){
  unsigned char letter3;
  unsigned char letter1;
  int v =0;
  text[0]=letterB[0];                   text[1]= letterB[1];             // 1xx, 2xx, 
  text[2]=(letterL[0]  | letterB[2]);   text[3]= letterL[1];             // 3xx, 4xx,
  text[4]=(letterA[0]  | letterL[2]);   text[5]= letterA[1];             // 5xx, 6xx, 
  text[6]=(letterU[0]  | letterA[2]);   text[7]= letterU[1];             // 7xx, 8xx,
  text[8]=(letterP[0]  | letterU[2]);   text[9]= letterP[1];             // 9xx, Axx, 
  text[10]=(letterU[0] | letterP[2]);   text[11]=letterU[1];             // Bxx, Cxx, 
  text[12]=(letterN[0] | letterU[2]);   text[13]=letterN[1];             // Dxx, Exx,
  text[14]=(letterK[0] | letterN[2]);   text[15]=letterK[1];            // Fxx, Gxx,
  text[16]=(letterT[0] | letterK[2]);   text[17]=letterT[1]; text[18]=letterT[2];  // Hxx, Ixx, Jxx
  
  //
  start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0b11001000);   //  C 1 0 X E B M1 M0   // E= Enable 1
      readAcknowledge();
      send_byte(0b10000000);   //  C 0 A A A A A A // 6 bits of address of 40 seg.
      readAcknowledge();
      send_byte(0b11100000);   //  C 1 1 0 0 A2 A1 A0 //8 of Subaddress
      readAcknowledge();
      send_byte(0b11111000);   //  C 1 1 1 1 0 1 1 // Bank 0 or 1 of arriving data
      readAcknowledge();
      send_byte(0b01110000);   //  C 1 1 1 0 A BF1 BF0 // Blink and Hz  BF1 & BF0 Display blinking at 0,5 HZ
      readAcknowledge();
      //
      send_byte(0x00); // This byte is to avoid turn on the symbols at first address!
      readAcknowledge();
      //
      for(i=0;i<27;i++){
        v++;
            if (v==3){
              //Serial.println(text[i], HEX);
              v=0;
            }
              else{
                //Serial.print(text[i], HEX); Serial.print(" ");
              }
              
            send_byte(text[i]);
            readAcknowledge();
            }
      stp();
}
/******************************** BROOKLYN *****************************/
void BrooklynArray(void){
  unsigned char letter3;
  unsigned char letter1;
  int v =0;
  text[0]=letterB[0];                   text[1]= letterB[1];             // 1xx, 2xx, 
  text[2]=(letterR[0]  | letterB[2]);   text[3]= letterR[1];             // 3xx, 4xx,
  text[4]=(letterO[0]  | letterR[2]);   text[5]= letterO[1];             // 5xx, 6xx, 
  text[6]=(letterO[0]  | letterO[2]);   text[7]= letterO[1];             // 7xx, 8xx,
  text[8]=(letterK[0]  | letterO[2]);   text[9]= letterK[1];             // 9xx, Axx, 
  text[10]=(letterL[0] | letterK[2]);   text[11]=letterL[1];             // Bxx, Cxx, 
  text[12]=(letterY[0] | letterK[2]);   text[13]=letterY[1];             // Dxx, Exx,
  text[14]=(letterN[0] | letterY[2]);   text[15]=letterN[1];            // Fxx, Gxx,
  text[16]=((0x00) | letterN[2]);       text[17]=0x00; text[18]=0x00;  // Hxx, Ixx, Jxx
  
  //
  start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0b11001000);   //  C 1 0 X E B M1 M0   // E= Enable 1
      readAcknowledge();
      send_byte(0b10000000);   //  C 0 A A A A A A // 6 bits of address of 40 seg.
      readAcknowledge();
      send_byte(0b11100000);   //  C 1 1 0 0 A2 A1 A0 //8 of Subaddress
      readAcknowledge();
      send_byte(0b11111000);   //  C 1 1 1 1 0 1 1 // Bank 0 or 1 of arriving data
      readAcknowledge();
      send_byte(0b01110000);   //  C 1 1 1 0 A BF1 BF0 // Blink and Hz  BF1 & BF0 Display blinking at 0,5 HZ
      readAcknowledge();
      //
      send_byte(0x00); // This byte is to avoid turn on the symbols at first address!
      readAcknowledge();
      //
      for(i=0;i<27;i++){
        v++;
            if (v==3){
              //Serial.println(text[i], HEX);
              v=0;
            }
              else{
                //Serial.print(text[i], HEX); Serial.print(" ");
              }
              
            send_byte(text[i]);
            readAcknowledge();
            }
      stp();
}

/******************************* BLAUPUNKT Byte by Byte  *****************************/
void Blaupunkt_PFC8576DH(){
  // Congigure te time and day to the module DS3231 
    start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0b11001100);   //  C 1 0 X E B M1 M0   // E= Enable 1
      readAcknowledge();
      send_byte(0b10000000);   //  C 0 A A A A A A // 6 bits of address of 40 seg.
      readAcknowledge();
      send_byte(0b11100000);   //  C 1 1 0 0 A2 A1 A0 //8 of Subaddress
      readAcknowledge();
      send_byte(0b11111000);   //  C 1 1 1 1 0 1 1 // Bank 0 or 1 of arriving data
      readAcknowledge();
      send_byte(0b01110000);   //  C 1 1 1 0 A BF1 BF0 // Blink and Hz  BF1 & BF0 Display blinking at 0,5 HZ
      readAcknowledge();
    
      send_byte(0x00);// 0  First Byte will be sent to PCF8576DH, Note: belongs to symbols but also to the first display of 13 segm(3 bits)
      readAcknowledge();
      send_byte(0x00);// 1  Second byte, 
      readAcknowledge();
      send_byte(0xA5);// 2
      readAcknowledge();
      send_byte(0x35);// 3
      readAcknowledge(); 
      send_byte(0x80);// 4
      readAcknowledge();
      send_byte(0x07);// 5
      readAcknowledge();
      send_byte(0x05);// 6
      readAcknowledge();
      send_byte(0x35);// 7
      readAcknowledge();
      send_byte(0x80);// 8
      readAcknowledge();
      send_byte(0x37);// 9
      readAcknowledge();
      send_byte(0x05);// A
      readAcknowledge();
      send_byte(0x15);// B
      readAcknowledge();
      send_byte(0x80);// C
      readAcknowledge();
      send_byte(0x35);// D
      readAcknowledge();
      send_byte(0x18);// E
      readAcknowledge(); 
      send_byte(0x37);// F
      readAcknowledge();
      send_byte(0x0A);// G
      readAcknowledge();
      send_byte(0x00);// H
      readAcknowledge();
      send_byte(0x21);// I
      readAcknowledge();
      send_byte(0x00);// J
      readAcknowledge();
        
    stp();
}
/************************************ Clear of display *********************************/
void clearPFC8576DH ( void){
  start();
      send_byte(addressWR);  // slave with write active 
      readAcknowledge();
      send_byte(0b11001100);   //  C 1 0 X E B M1 M0   // E= Enable 1
      readAcknowledge();
      send_byte(0b10000000);   //  C 0 A A A A A A // 6 bits of address of 40 seg.
      readAcknowledge();
      send_byte(0b11100000);   //  C 1 1 0 0 A2 A1 A0 //8 of Subaddress
      readAcknowledge();
      send_byte(0b11111000);   //  C 1 1 1 1 0 1 1 // Bank 0 or 1 of arriving data
      readAcknowledge();
      send_byte(0b01110000);   //  C 1 1 1 0 A BF1 BF0 // Blink and Hz  BF1 & BF0 Display blinking at 0,5 HZ
      readAcknowledge();
    
     for (int i = 0; i < 20; i++){
      send_byte(0x00);
      readAcknowledge();
     }
     stp();
}
/*********************************  Test seg by seg   *******************************/
void testSegPFC8576DH (void){
    //Note: The write mode finish use allways the Acknowledge sended by the SLAVE!
     for (int n = 0; n < 8; n++){ // to write a bit of the byte segments to test
          for (int i = 0; i < 20; i++){ // Number of bytes wich compound the display segments
              start();
              send_byte(addressWR);  // slave with write active 
              readAcknowledge();
              send_byte(0b11001100);   //  C 1 0 X E B M1 M0   // E= Enable 1
              readAcknowledge();
              send_byte(0b10000000);   //  C 0 A A A A A A // 6 bits of address of 40 seg.
              readAcknowledge();
              send_byte(0b11100000);   //  C 1 1 0 0 A2 A1 A0 //8 of Subaddress
              readAcknowledge();
              send_byte(0b11111000);   //  C 1 1 1 1 0 1 1 // Bank 0 or 1 of arriving data
              readAcknowledge();
              send_byte(0b01110000);   //  C 1 1 1 0 A BF1 BF0 // Blink and Hz  BF1 & BF0 Display blinking at 0,5 HZ
              readAcknowledge();
                  for (int m = 0; m < i; m++){  // Define the byte number of segments to test
                    send_byte(0x00); 
                    readAcknowledge();
                  }
              send_byte(0x01 << n);//
              readAcknowledge();
              stp();
              delay(300);    
           }
     }
}

/*************** SCAN BUS (Find slaves connecte to the bus SCL & SCA ******************/
void scanBUS(void){
  // This function help on debut to determine the answer of slave.
  Serial.begin(9600);
  softwarei2c.begin(7, 8);       // sda, scl is the pins I'm use to communicate with BUS
    
  Serial.println("begin to scan...");
    for(unsigned char i=1; i<=127; i++)
    {
        if(softwarei2c.beginTransmission(i))
        {
            Serial.print("0x");
            Serial.println(i, HEX);

            //while(1);
        }
        softwarei2c.endTransmission();
    }
    Serial.println("find nothing more");
    //while(1);
}

/********************************  SETUP  *****************************************/
void setup() {
// put your setup code here, to run once:
Serial.begin(9600);
pinMode(sda, OUTPUT);
pinMode(scl, OUTPUT);
pinMode(10, INPUT);

//pinMode(4, INPUT_PULLUP);
//pinMode(5, INPUT_PULLUP);
//pinMode(6, INPUT_PULLUP);
pinMode(4, OUTPUT);
pinMode(5, OUTPUT);
pinMode(6, OUTPUT);

pinMode(13, OUTPUT);

//Make a delay to avoid wrong configuration of PCF8576DH during power up
//Depending of flags status, maybe is necessary do a second reset to the clock be active!
delayMicroseconds(200);

//scanBUS();

}
//*********************************  LOOP   ******************************************//
void loop() {
 // put your main code here, to run repeatedly:
 int val = 0;      // variable to store the read value 
 unsigned char mask = 1; //our bitmask
 int cycle=0;
 int cor=-1;
 
  //scanBUS();
 
  clearPFC8576DH();
        for(int s=0; s<4; s++){
                  testBitPFC8576DH();
                  delay(800);
                  clearPFC8576DH();
                  delay(400);
                }
//delay(200);
//Blaupunkt_PFC8576DH(); // Write Blaupunkt byte by byte
//testSegPFC8576DH();
//testBitPFC8576DH();
//Blaupunkt_PFC8576DH();

         while(1){
          cycle++;  // This is only to do a delay of chars on display. Is note better way! Only to test, OK!!!
              if ((cycle>150) & (cycle <300)){
                HiFolksArray();
              }
              if ((cycle>300) & (cycle < 450)){
                BlaupunktArray(); // Use array of characters
              }
               if((cycle>450) & (cycle <600)){
                BrooklynArray();
                if(cycle>600){
                  cycle=0;
                }
               }
                   if (val==digitalRead(10)){
                            if (val & mask){ // if bitwise AND resolves to true
                               // Used only to debug // Change it!!!      
                            }
                   }
                  else{  // This is only to play with the color set.
            if(cor==0){
            digitalWrite(4, LOW);
            digitalWrite(5, LOW);
            digitalWrite(6, LOW);
            }
            if(cor==1){
            digitalWrite(4, HIGH);
            digitalWrite(5, LOW);
            digitalWrite(6, LOW);
            }
            if(cor==2){
            digitalWrite(4, LOW);
            digitalWrite(5, HIGH);
            digitalWrite(6, LOW);
            }
            if(cor==3){
            digitalWrite(4, HIGH);
            digitalWrite(5, HIGH);
            digitalWrite(6, LOW);
            }
            if(cor==4){
            digitalWrite(4, LOW);
            digitalWrite(5, LOW);
            digitalWrite(6, HIGH);
            }
            if(cor==5){
            digitalWrite(4, HIGH);
            digitalWrite(5, LOW);
            digitalWrite(6, HIGH);
            }
            if(cor==6){
            digitalWrite(4, LOW);
            digitalWrite(5, HIGH);
            digitalWrite(6, HIGH);
            }
            if(cor==7){
            digitalWrite(4, HIGH);
            digitalWrite(5, HIGH);
            digitalWrite(6, HIGH);
            }
                if(cor>7){
                  cor=-1;
                }  
                else{
                  cor++; 
                }
        }
     val = digitalRead(10);
 } // While
}
