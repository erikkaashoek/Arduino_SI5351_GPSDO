/*
 * A Time Base using a GPS controlled SI5351A Adafruit board to generate 10 MHz, 26 MHz or any other frequency up to 110 MHz.
Permission is granted to use, copy, modify, and distribute this software and documentation for non-commercial purposes. 
(F2DC 17 April 2017)

I was greatly helped by the works of Gene Marcus W3PM, Jason Milldrum NT7S-Dana H. Myers K6JQ, Igor Gonzales Martin and many others.
Thanks to all of them. 

==> This new version V.5.0 (May2020) can be used with the present version of the NT7S SI5351 library. I tested it with the 2.1.4 NT7S revision.
==> Version V.5.2 (June 2020) : The LCD shows now error E as "Rel.err=-70e-8"

SW modified by Erik Kaashoek to allow for longer measurement times enabling more accuracy.

*/

// include the library code:
#include <LiquidCrystal.h>
#include <string.h>
#include <ctype.h>
#include <avr/interrupt.h>  
#include <avr/io.h>
#include <Wire.h>

// Set up MCU pins
#define ppsPin                   2 // from GPS 
#define FreqAlarm                3 // LED Alarm XtalFreq
#define RS                       7 // LCD RS
#define E                        8 // LCD Enable
#define DB4                      9 // LCD DB4
#define DB5                     10 // LCD DB5
#define DB6                     11 // LCD DB6
#define DB7                     12 // LCD DB7
#define FreqSelect               4 // Choice between 10MHZ (or another frequency) and F=26 MHz

// Define SI5351A register addresses
#define CLK_ENABLE_CONTROL       3
#define CLK0_CONTROL            16 
#define CLK1_CONTROL            17
#define CLK2_CONTROL            18
#define SYNTH_PLL_A             26
#define SYNTH_PLL_B             34
#define SYNTH_MS_0              42
#define SYNTH_MS_1              50
#define SYNTH_MS_2              58
#define PLL_RESET              177
#define XTAL_LOAD_CAP          183
uint64_t SI5351aSetFreq(int synth, uint64_t freq_x100, int r_div);
uint64_t SI5351aActualFreq(uint64_t freq_x100);
uint64_t SI5351aSetPLLFreq(uint32_t a,uint32_t b,uint32_t c);
uint64_t SI5351aActualPLLFreq(uint32_t a,uint32_t b,uint32_t c);

uint64_t PLLFReq_x100 = 90000000000;

#define CAL_FREQ  2500000      // In Hz, maximum is 4.5MHz
#define CALFACT_START 687
#define MIN_PULSES  4

#define MAX_TIME  (int)(1000000000ULL / CAL_FREQ )    // Time to count 1 billion counts

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

// Variables 
byte res,threshold,Epos;
byte CptInit=1;
char StartCommand2[7] = "$GPRMC",buffer[300] = "";
int IndiceCount=0,StartCount=0,counter=0,indices[13];
int validGPSflag = 1,Chlength;
int byteGPS=-1,second=0,minute=0,hour=0;
unsigned long mult=0;         // Count of overflows of 16 bit counter
int alarm = 0;
unsigned int tcount=0;        // counts the seconds in a measurement cycle 
int start = 1;                // Wait one second after first pps 
int duration = 2;             // Initial measurement duration
int target_duration = 1;         // Value of duration for next measurement cycle
int available = 0;            // Flag set to 1 after pulse counting finished
int stable_count = 0;         // Count of consecutive measurements without correction needed.

int32_t measdif,              // Measured difference in 1/100 Hz
  old_measdif = 0,
  measdif_integrated = 0,
  calfact =0;                 // Current correction factor in 1/100 Hz
int64_t target_count,             // Target count of pulses
  measured_count;                 // Actual count of pulses
int64_t target_freq1;         // Additional output frequency

/* Clock - interrupt routine for counting the CLK0 2.5 MHz signal
Called every second by GPS 1PPS on Arduino Nano D2
*/
void PPSinterrupt()
{
  tcount++;// Increment the seconds counter
  if (tcount == start) // Start counting the 2.5 MHz signal from SI5351A CLK0
  {
    TCCR1B = 7;    //Clock on rising edge of pin 5
  } else if (tcount == target_duration + start)  //Stop the counter : the 40 second gate time is elapsed
  {     
    TCCR1B = 0;      //Turn off counter
    measured_count = mult * 0x10000ULL + TCNT1 - MIN_PULSES;   //measured_count is the number of pulses counted during duration PPS.
    duration = tcount - start;
    target_count=CAL_FREQ*duration;          //Calculate the target count     
ready:
    TCNT1 = 0;       //Reset counter to zero
    mult = 0;
    tcount = 0;      //Reset the seconds counter
    available = 1;
  }
#if 0
  else if (tcount > start+10) {
    
    measured_count = mult * 0x10000ULL + TCNT1 - MIN_PULSES;   //measured_count is the number of pulses counted during duration PPS.
    duration = tcount - start;
    target_count=CAL_FREQ*duration;          //Calculate the target count
    if(abs(measured_count -target_count) > 4) // Within threshold, increase duration
      TCCR1B = 0;      //Turn off counter
      goto ready;
  }
#endif 
 
  if(validGPSflag == 1) //Start the UTC timekeeping process
  {
    second++;  
    if (second == 60)   //Set time using GPS NMEA data 
    {
      minute++ ;
      second=0 ;
    }
    if (minute == 60) 
    {
      hour++;
      minute=0 ;
    }
    
    if (hour == 24) hour=0;
    lcd.setCursor(7,0); // LCD cursor on the right part of Line 0
    if (hour < 10) lcd.print ("0");
    lcd.print (hour);
    lcd.print (":");
    if (minute < 10) lcd.print ("0");
    lcd.print (minute);
    lcd.print (":");
    if (second < 10) lcd.print ("0");
    lcd.print (second);
    lcd.print ("Z");  // UTC Time Indicator
  }
}

// Timer 1 overflow intrrupt vector.
ISR(TIMER1_OVF_vect) 
{
  mult++;        //Increment overflow multiplier
  TIFR1 = (1<<TOV1);  //Clear overlow flag by shifting left 
}

String ToString(uint64_t x)
{
     boolean flag = false; // For preventing string return like this 0000123, with a lot of zeros in front.
     String str = "";      // Start with an empty string.
     uint64_t y = 10000000000000000000;
     int res;
     if (x == 0)  // if x = 0 and this is not testet, then function return a empty string.
     {
           str = "0";
           return str;  // or return "0";
     }    
     while (y > 0)
     {                
            res = (int)(x / y);
            if (res > 0)  // Wait for res > 0, then start adding to string.
                flag = true;
            if (flag == true)
                str = str + String(res);
            x = x - (y * (uint64_t)res);  // Subtract res times * y from x
            y = y / 10;                   // Reducer y with 10    
     }
     return str;
}  

void setup()
{
  Serial.begin(9600);  // Define the GPS port speed
  Wire.begin(1);    // I2C bus address = 1
  SI5351aStart();  
  SI5351aSetFreq(SYNTH_MS_0,1000000000ULL,2); 
  SI5351aSetFreq(SYNTH_MS_1, 1000000000ULL,0);
//  SI5351_write(CLK_ENABLE_CONTROL,0b00000100); // Turn OFF CLK2
//  SI5351_write(CLK_ENABLE_CONTROL,0b00000000); // Turn ON CLK2

//Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;     //Disable Timer during setup
  TCCR1A = 0;     //Reset
  TCNT1  = 0;     //Reset counter to zero
  TIFR1  = 1;     //Reset overflow
  TIMSK1 = 1;     //Turn on overflow flag

  pinMode(FreqAlarm, OUTPUT); // Alarm LED for weird measured_count
 // Set up the LCD's number of columns and rows 
  lcd.begin(16,2); 

  // GPS 1pps input
  pinMode(ppsPin, INPUT);

  lcd.display();           // initialize LCD
  lcd.setCursor(0,1);
  lcd.print(" F2DC V.5.2"); // display version number on the LCD
  delay(1000);
  
   // Set up IO switches
  pinMode(FreqSelect, INPUT);  // Initialize the frequency select pin
  digitalWrite(FreqSelect, HIGH); // internal pull-up enabled 

  // Set Arduino D2 for external interrupt input on the rising edge of GPS 1PPS
  attachInterrupt(0, PPSinterrupt, RISING);  


//Read the frequency setting pin
  res=digitalRead(FreqSelect);
// When testing with the frequency beat method, add or substract 800 Hz (or your choice). 
  if (res==HIGH)
    {
   target_freq1 = 1000000000ULL; // Freq_1=100MHz
    }
    else
    {
    target_freq1 = 1000000000LL; //   Freq_1=10 MHz in 1/100th Hz
    }
   
  lcd.setCursor(0,1);
  lcd.print("Waiting for GPS");

  TCCR1B = 0;    //Turn off Counter

 // Turn OFF CLK2 
//  SI5351.output_enable(SI5351_CLK2,0); 

// Set up parameters
  calfact = CALFACT_START; // Determined experimentally for my SI5351A 25 MHz crystal. Run the 'SI5351_calibration.ino' program 
  // proposed by NT7S in his examples software package to get the calfact value associated with your SI5351A card.
  // Then use this value as 'your calfact and your calfact_old' instead of -2800.

  target_count=CAL_FREQ*duration;
  dispfreq1();
  
}
//******************************************************************
// Loop 
void loop()
{
  int lock = 0; 
  String str = "";
  uint64_t target_freq,actual_freq;
  if (validGPSflag == 0) GPSprocess( ); //If GPS is selected, wait for valid NMEA data
  else
  {
    if(available) // Frequency calculation data available                                   
    {
       available = 0;              
// Compute calfactor (and update if needed)
        measdif =(int32_t)((measured_count -target_count) * MAX_TIME  / duration); // PPB Error calculation           
        if(measdif<-50000 || measdif>+50000) // Impossible error, alarm
        {
          digitalWrite(FreqAlarm,LOW);   // measured_count OK : turn the LED OFF 
          alarm = 1;
          LCDmeasdif(false); // display E (measdif) on the LCD
          target_duration = 1;
        }
        else  
        {
          digitalWrite(FreqAlarm,LOW);   // measured_count OK : turn the LED OFF 
          alarm = 0;
          threshold = 1 * MAX_TIME  / duration;    // More than 3 count difference
          if(abs(measured_count -target_count) < 4) // Within threshold, increase duration
          {
            LCDmeasdif(true); // display E (measdif) on the LCD
            lock = 1;
            target_duration = duration * 2;
            if (target_duration > MAX_TIME)
              target_duration = MAX_TIME;
          }
#if 0
          if (measdif > 0)
            measdif += MAX_TIME / duration /2;
          else if (measdif < 0)  
            measdif -= MAX_TIME / duration /2;
#endif
#if 0
          if (old_measdif != 0 ) {
            int32_t ratio = measdif * 100 / old_measdif;
            if (ratio > 0 && ratio > 20 && ratio < 500) {   // constant drift
              measdif += measdif;                
            }
          }
          old_measdif = measdif;
#endif
         // if (duration > 100)
         //   measdif = measdif / 2;
          calfact=calfact - measdif; // compute the new calfact
          LCDmeasdif(false); // Call the display Error E (measdif) routine
          if(abs(measured_count -target_count) > 10) // Too large, increase speed
          {
            target_duration = duration / 2;
            if (target_duration == 0)
              target_duration = 1;
          }
          target_freq = 1000000000+calfact;
          
          uint32_t a,b,c, f_a,f_b,f_c;
          f_c = c = 1048575UL - 10000UL;
          uint32_t delta = 100000000;
          
          for (a = 24; a<=36; a++) {
            for (b = 524287UL; b <524303UL; b++) {
              SI5351aActualPLLFreq(a,b,c);
              actual_freq = SI5351aActualFreq(target_freq);
              if (abs(actual_freq - target_freq) < delta) {
                delta = abs(actual_freq - target_freq);
                f_a = a;
                f_b = b;
                f_c = c;
              }
            }
          }
          SI5351aActualPLLFreq(f_a,f_b,f_c);
          SI5351aSetPLLFreq(f_a,f_b,f_c);
#if 0
          Serial.print("a,b,c=");
          Serial.print(f_a);
          Serial.print(",");
          Serial.print(f_b);
          Serial.print(",");
          Serial.print(f_c);
          Serial.println();
#endif
          SI5351aSetFreq(SYNTH_MS_0,target_freq,2);  // 2.5MHz
          actual_freq=SI5351aSetFreq(SYNTH_MS_1,target_freq,0);    // 10MHz
        }

        Serial.print(hour);
        Serial.print(":");
        Serial.print(minute);
        Serial.print(":");
        Serial.print(second);
        
        Serial.print(" dur=");
        Serial.print(duration); 
        String str;
        Serial.print(" dcount=");
        Serial.print((int)(measured_count -target_count));
        Serial.print(" freq=");
        str = ToString(actual_freq);
        Serial.print(str);
        if (target_freq!=actual_freq) {
          Serial.print(" Freq_Error = ");
          Serial.print((int)(target_freq-actual_freq));
        }
        if (lock) 
          Serial.println(" Lock");
        else
          Serial.println(" ");
          
      }
  } 
}



//***********************************
// Display the SI5351A CLK1 frequency on the LCD
void dispfreq1()
{
  
  // Display frequency on the LCD
  int Freq_Display=target_freq1/10000000;
  float Freq_Disp_Fl=float(Freq_Display);
  Freq_Disp_Fl=Freq_Disp_Fl/10;
  lcd.setCursor(0,0);
  lcd.print(Freq_Disp_Fl,1);
  lcd.print("M   ");    
}

//************************************
// Display on the LCD the difference E between measured CLK0 and theoretical 100e6 
void LCDmeasdif(int good)
{
          lcd.setCursor(0,1);
          lcd.print("                ");
          lcd.setCursor(0,1);
          if (alarm)
            lcd.print("> "); 
          if (measdif == 0)
            lcd.print("lock");
          else
            lcd.print(measdif);
          lcd.setCursor(6,1);
          lcd.print(" "); 
          lcd.print(duration); 
          lcd.print("s "); 
          lcd.print(calfact);
}          

//***************************************
//  GPS NMEA processing starts here

void GPSprocess(void)
{
  byte temp,i;
  byteGPS=Serial.read();      // Read a byte coming from the GPS serial port 
  if (byteGPS == -1) // See if the port is empty yet
  { 
    delay(100);  
  } 
  else // NMEA string data begins here
  { 
    buffer[counter]=byteGPS;     // If there is serial port data, it is put in the buffer
    counter++;                  
    if (byteGPS==13) // If the received byte is = to 13, end of transmission
    {      
        IndiceCount=0;
        StartCount=0;
        
//      NMEA $GPRMC data begins here        
        for (int i=1;i<7;i++) // Check if the received command starts with $GPRMC
          { 
          if (buffer[i]==StartCommand2[i-1])
            {
            StartCount++;
            }
          }                

        if(StartCount==6) // If yes, continue and process the GPS data
          {
          for (int i=0;i<300;i++)
            {
            if (buffer[i]==',') // check for the position of the  "," separator
              {
              indices[IndiceCount]=i;
              IndiceCount++;
              }
            if (buffer[i]=='*') // ... and the "*"
              {
              indices[12]=i;
              IndiceCount++;
              }
            }
          // Load time data
          temp = indices[0];
          hour = (buffer[temp+1]-48)*10 + buffer[temp+2]-48;
          minute = (buffer[temp+3]-48)*10 + buffer[temp+4]-48;
          second = (buffer[temp+5]-48)*10 + buffer[temp+6]-48;
          temp = indices[1]; 
          
          if (buffer[temp+1] == 65) // Check for "A", ie GPS data valid
            {
            validGPSflag = 1; 
            lcd.setCursor(0,1);
              lcd.print("               ");
            }             
          else
            {
            validGPSflag = 0;
            }        
        } 
      counter=0;   // Reset the buffer
      for (int i=0;i<300;i++)
      { 
        buffer[i]=' '; 
      }

    } 
  } 
}

//------------------------------------- SI5351 -------------------------------------
// Set SI5351A I2C address
#define SI5351A_addr          0x60 


#define XtalFreq  26000000ULL

//  SI5351aStart();
//  SI5351aSetFreq(SYNTH_MS_0,2500000); 
//  SI5351aSetFreq(SYNTH_MS_1, Freq_1);
//  SI5351_write(CLK_ENABLE_CONTROL,0b00000100); // Turn OFF CLK2
//  SI5351_write(CLK_ENABLE_CONTROL,0b00000000); // Turn ON CLK2
//  SI5351aSetFreq(SYNTH_MS_2, Freq_2);          // Set CLK2 frequency

uint64_t SI5351aActualFreq(uint64_t freq_x100)
{
  unsigned long  a, b, c;
  c = 0xFFFFF;  // Denominator derived from max bits 2^20
  a = PLLFReq_x100 / freq_x100; // 36 is derived from 900/25 MHz
  unsigned long long rest =  PLLFReq_x100 - a*freq_x100;
  b = (rest * c) / freq_x100;
  return( PLLFReq_x100 * c /(a*c + b) );
}
//******************************************************************
//  SI5351 Multisynch processing
//******************************************************************
uint64_t SI5351aSetFreq(int synth, uint64_t freq_x100, int r_div)
{
  unsigned long long CalcTemp;
  unsigned long  a, b, c, p1, p2, p3;
  uint64_t actual_freq;
  
  c = 0xFFFFF;  // Denominator derived from max bits 2^20

  a = PLLFReq_x100 / freq_x100; // 36 is derived from 900/25 MHz
  unsigned long long rest =  PLLFReq_x100 - a*freq_x100;
  b = (rest * c) / freq_x100;

  actual_freq = PLLFReq_x100 * c /(a*c + b);

//  CalcTemp = round(XtalFreq * 36) % freq;
//  CalcTemp *= c;
//  CalcTemp /= freq ; 
//  b = CalcTemp;  // Calculated numerator


#if 0
    Serial.print("ab=");
    Serial.print(a);
    Serial.print(",");
    Serial.print(b);
    Serial.print(", rest=");
    Serial.print((unsigned long)rest);
    String str;
        Serial.print(" target=");
        str = ToString(freq_x100); Serial.print(str);
        Serial.print(" actual=");
        str = ToString(actual_freq); Serial.print(str);
        Serial.println();
#endif

  // Refer to SI5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to multisynth registers
  SI5351_write(synth + 0, (p3 & 0xFF00)>>8);  
  SI5351_write(synth + 1, (p3 & 0xFF));
  SI5351_write(synth + 2, ((p1 & 0x00030000) >> 16) | ((0x03 & r_div)<<4) );
  SI5351_write(synth + 3, (p1 & 0x0000FF00) >> 8);
  SI5351_write(synth + 4, (p1 & 0x000000FF));
  SI5351_write(synth + 5, ((p3 & 0xF0000) >> 12) | ((p2 & 0x000F0000) >> 16));
  SI5351_write(synth + 6, (p2 & 0x0000FF00) >> 8);
  SI5351_write(synth + 7, (p2 & 0x000000FF));
  return actual_freq;
}

uint64_t SI5351aActualPLLFreq(uint32_t a,uint32_t b,uint32_t c)
{
  PLLFReq_x100 = ((((uint64_t)XtalFreq) * 100) * (a * c + b) / c);
  return PLLFReq_x100;
}

uint64_t SI5351aSetPLLFreq(uint32_t a,uint32_t b,uint32_t c)
{
  PLLFReq_x100 = ((((uint64_t)XtalFreq) * 100) * (a * c + b) / c);

#if 0
    Serial.print("abc=");
    Serial.print(a);
    Serial.print(",");
    Serial.print(b);
    Serial.print(",");
    Serial.print(c);

    String str;
        Serial.print(" PLLFreq=");
        str = ToString(PLLFReq_x100); Serial.println(str);
#endif

  unsigned long p1, p2, p3;
  // Refer to SI5351 Register Map AN619 for following formula
  p3  = c;
  p2  = (128 * b) % c;
  p1  = 128 * a;
  p1 += (128 * b / c);
  p1 -= 512;

  // Write data to PLL registers
  SI5351_write(SYNTH_PLL_A + 0, (p3 & 0xFF00)>>8);
  SI5351_write(SYNTH_PLL_A + 1, (p3 & 0xFF));
  SI5351_write(SYNTH_PLL_A + 2, (p1 & 0x00030000) >> 16);
  SI5351_write(SYNTH_PLL_A + 3, (p1 & 0x0000FF00) >> 8);
  SI5351_write(SYNTH_PLL_A + 4, (p1 & 0x000000FF));
  SI5351_write(SYNTH_PLL_A + 5, ((p3 & 0xF0000) >> 12) | ((p2 & 0x000F0000) >> 16));
  SI5351_write(SYNTH_PLL_A + 6, (p2 & 0x0000FF00) >> 8);
  SI5351_write(SYNTH_PLL_A + 7, (p2 & 0x000000FF));

  SI5351_write(SYNTH_PLL_B + 0, (p3 & 0xFF00)>>8);
  SI5351_write(SYNTH_PLL_B + 1, (p3 & 0xFF));
  SI5351_write(SYNTH_PLL_B + 2, (p1 & 0x00030000) >> 16);
  SI5351_write(SYNTH_PLL_B + 3, (p1 & 0x0000FF00) >> 8);
  SI5351_write(SYNTH_PLL_B + 4, (p1 & 0x000000FF));
  SI5351_write(SYNTH_PLL_B + 5, ((p3 & 0xF0000) >> 12) | ((p2 & 0x000F0000) >> 16));
  SI5351_write(SYNTH_PLL_B + 6, (p2 & 0x0000FF00) >> 8);
  SI5351_write(SYNTH_PLL_B + 7, (p2 & 0x000000FF));
}

//******************************************************************
//  SI5351 initialization routines
//******************************************************************
void SI5351aStart()
{
  // Initialize SI5351A
  SI5351_write(XTAL_LOAD_CAP,0b01000000);      // Set crystal load to 6pF
  SI5351_write(CLK_ENABLE_CONTROL,0b00000000); // Enable all outputs
  SI5351_write(CLK0_CONTROL,0b00001111);       // Set PLLA to CLK0, 8 mA output
  SI5351_write(CLK1_CONTROL,0b00001111);       // Set PLLA to CLK1, 8 mA output
  SI5351_write(CLK2_CONTROL,0b00101111);       // Set PLLB to CLK2, 8 mA output
  SI5351_write(PLL_RESET,0b10100000);          // Reset PLLA and PLLB

  SI5351aSetPLLFreq((uint32_t)36,(uint32_t)0,(uint32_t)0xFFFFFUL);
}

//******************************************************************
//Write I2C data routine
//******************************************************************
uint8_t SI5351_write(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(SI5351A_addr);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}
//------------------------------------------------------------------------------------------------------
