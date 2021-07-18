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

#define USE_PHASE_DETECTOR  true        // Set to false if the XTAL is too unstable to phase lock
#define USE_XTAL            false       // Set to false if a stable TCXO is used for the SI5351
#define PLL_START_DURATION  30          // Start using phase when duration exceeds this.

int64_t PLLFReq_x1000 = 900000000000LL;  // In 1/1000 Hz, will be update to actual frequency
int64_t XtalFreq_x1000 = 26000000000LL;   // In 1/1000 Hz
#define INITIAL_XTAL 26000000000LL

#define CAL_FREQ  2500000UL      // In Hz, maximum is 4.5MHz
#define CALFACT_START 0       // Can be set to pre-load the correction to speed up locking.
#define MIN_PULSES  4       // Heuristic number of pulses that are missed due to the code structure.

#define MAX_TIME  (int)(10000000000LL / CAL_FREQ )    // Time to count 1 billion counts

#define TEST  false

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);

// Variables 
byte res,threshold,Epos;
byte CptInit=1;
char StartCommand2[7] = "$GPRMC",buffer[300] = "";
int IndiceCount=0,StartCount=0,counter=0,indices[13];
int validGPSflag = 1;     // Set to 1 to avoid waiting for a valid GPS string in case no connection between GPS and Arduino
int Chlength;
int byteGPS=-1,second=0,minute=0,hour=0;
int64_t mult=0;         // Count of overflows of 16 bit counter
int alarm = 0;
unsigned int tcount=0;        // counts the seconds in a measurement cycle 
int start = 1;                // Wait one second after first pps 
int duration = 2;             // Initial measurement duration
int target_duration = 1;         // Value of duration for next measurement cycle
int available = 0;            // Flag set to 1 after pulse counting finished
int stable_count = 0;         // Count of consecutive measurements without correction needed.

int32_t measdif,              // Measured difference in 1/100 Hz
  calfact =0;                 // Current correction factor in 1/100 Hz
  int32_t prev_calfact = 0;
int64_t target_count,             // Target count of pulses
  measured_count;                 // Actual count of pulses

volatile int phase = 0;
int prev_phase = 0;
int p_delta = 0;
volatile int tick = 0;
float p_delta_average = 0;
int p_delta_count = 0;
int p_delta_max = 10;
int p_delta_sum = 0;


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
int64_t SI5351aSetFreq(int synth, int64_t freq_x1000, int r_div);
int64_t SI5351aActualFreq(int64_t freq_x1000);
int64_t SI5351aSetPLLFreq(uint32_t a,uint32_t b,uint32_t c);
int64_t SI5351aActualPLLFreq(uint32_t a,uint32_t b,uint32_t c);


/* Clock - interrupt routine for counting the CLK0 2.5 MHz signal
Called every second by GPS 1PPS on Arduino Nano D2
*/
void PPSinterrupt()
{
  tcount++;// Increment the seconds counter
  if (tcount == start) // Start counting the 2.5 MHz signal from SI5351A CLK0
  {
    TCCR1B = 7;    //Clock on rising edge of pin 5
  } else if (tcount == target_duration + start || (tcount > start  && TEST) ) //Stop the counter : the 40 second gate time is elapsed
  {     
    TCCR1B = 0;      //Turn off counter
    measured_count = TCNT1;   //measured_count is the number of pulses counted during duration PPS.
    measured_count += ((int64_t)mult) * 0x10000LL - (int64_t)MIN_PULSES;   //measured_count is the number of pulses counted during duration PPS.
    duration = target_duration;
    target_count=((int64_t)CAL_FREQ)*duration;          //Calculate the target count     
    if (TEST) measured_count = target_count + 1;
ready:
    TCNT1 = 0;       //Reset counter to zero
    mult = 0;
    tcount = 0;      //Reset the seconds counter
    available = 1;
  }
#if 0     // Experiment to see if you can stop earlier then target_duration when enough count error detected. Not working!!!
  else if (tcount > start+10) {
    
    measured_count = mult * 0x10000LL + TCNT1 - MIN_PULSES;   //measured_count is the number of pulses counted during duration PPS.
    duration = tcount - start;
    target_count=CAL_FREQ*duration;          //Calculate the target count
    if(abs(measured_count -target_count) > 4) // Within threshold, increase duration
      TCCR1B = 0;      //Turn off counter
      goto ready;
  }
#endif 
  if (USE_PHASE_DETECTOR) {
    phase = analogRead(A0);
    p_delta = phase - prev_phase;
    prev_phase = phase;

    if (p_delta_count < p_delta_max) {
      if (abs(p_delta)<50) {
        if (p_delta_count == 0)
          p_delta_sum = 0;
          p_delta_sum += p_delta;
        p_delta_count++;
      }
    }
  }
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
    lcd.setCursor(0,0);
#if 0
    if (phase < 100) lcd.print (" ");
    if (phase < 10) lcd.print (" ");
    lcd.print(phase);
#endif
    if (p_delta >= 0)
      lcd.print(" ");
    if (abs(p_delta) < 100) lcd.print (" ");
    if (abs(p_delta) < 10) lcd.print (" ");
    lcd.print(p_delta);
//    lcd.print("     ");    
    lcd.setCursor(7,0); // LCD cursor on the right part of Line 0
    if (hour < 10) lcd.print ("0");
    lcd.print (hour);
    lcd.print (":");
    if (minute < 10) lcd.print ("0");
    lcd.print (minute);
    lcd.print (":");
    if (second < 10) lcd.print ("0");
    lcd.print (second);
 //   lcd.print ("Z");  // UTC Time Indicator
  }
}

// Timer 1 overflow intrrupt vector.
ISR(TIMER1_OVF_vect) 
{
  mult++;        //Increment overflow multiplier
  TIFR1 = (1<<TOV1);  //Clear overlow flag by shifting left 
}

String ToString(int64_t x)    // Very clumsy conversion of 64 bit numbers
{
     boolean flag = false; // For preventing string return like this 0000123, with a lot of zeros in front.
     String str = "";      // Start with an empty string.
     uint64_t y = 10000000000000000000;
     int res;
    if (x<0) {
      x = -x;
     str = "-";
    }
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
            x = x - (y * (int64_t)res);  // Subtract res times * y from x
            y = y / 10;                   // Reducer y with 10    
     }
     return str;
}  

void setup()
{
  Serial.begin(9600);  // Define the GPS port speed
  Wire.begin(1);    // I2C bus address = 1
  SI5351aStart();  
  SI5351aSetFreq(SYNTH_MS_0,10000000000LL,2);     // 10MHz, divide by 4
  SI5351aSetFreq(SYNTH_MS_1, 10000000000LL,0);    // 10MHz
//  SI5351_write(CLK_ENABLE_CONTROL,0b00000100); // Turn OFF CLK2, not used
//  SI5351_write(CLK_ENABLE_CONTROL,0b00000000); // Turn ON CLK2, not used

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
  digitalWrite(FreqSelect, HIGH); // internal pLL-up enabled 

  // Set Arduino D2 for external interrupt input on the rising edge of GPS 1PPS
  attachInterrupt(0, PPSinterrupt, RISING);  
   
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
}
//******************************************************************
// Loop 
void loop()
{
  int phase_locked = false;
  int lock = 0; 
  String str = "";
  int64_t target_freq,actual_freq;
  if (validGPSflag == 0) GPSprocess( ); //If GPS is selected, wait for valid NMEA data
  else
  {    
    if (USE_PHASE_DETECTOR && target_duration >= PLL_START_DURATION) {
      if (p_delta_count == p_delta_max /* || (p_delta_count > 10 && fabs((float)p_delta_sum / (float)p_delta_count ) > 10)*/) {
        p_delta_average = (float)p_delta_sum / (float)p_delta_count;
        Serial.print(hour);
        Serial.print(":");
        Serial.print(minute);
        Serial.print(":");
        Serial.print(second);
        Serial.print(" dur=");
        Serial.print(p_delta_max); 

        Serial.print(" p_average=");
        Serial.print(p_delta_average); 
        p_delta_count = 0;
        if (p_delta_max > 10 && p_delta_average > 4.0) {
          p_delta_max /= 2;
        }
        if (p_delta_max < 40 && p_delta_average < 1.0) {
          p_delta_max *= 2;
          while (target_duration < p_delta_max)
            target_duration *= 2;
        }
        if (fabs(p_delta_average) < 2.0)
          measdif = p_delta_average*9;      // Half speed to avoid overcompensations
        else
          measdif = p_delta_average*18;
        calfact += measdif;
        LCDmeasdif(true); // display E (measdif) on the LCD
        tcount = 0;
        phase_locked = true;
        goto update;
      }
    } else
      p_delta_count = 0;
    if(available) // Frequency calculation data available                                   
    {
       available = 0;              
// Compute calfactor (and update if needed)
        measdif =(int32_t)((measured_count -target_count) * MAX_TIME  / duration); // PPB Error calculation           
#if 1
        if(measdif<-50000 || measdif>+50000) // Impossible error, alarm, not used
        {
          digitalWrite(FreqAlarm,LOW);   // measured_count OK : turn the LED OFF 
          alarm = 1;
          LCDmeasdif(false); // display E (measdif) on the LCD
          target_duration /= 2;
          if (target_duration == 0)
              target_duration = 1;

        }
        else  
#endif
        {
          digitalWrite(FreqAlarm,LOW);   // measured_count OK : turn the LED OFF 
          alarm = 0;
          threshold = 1 * MAX_TIME  / duration;    // More than 3 count difference, no longer used
          if(abs(measured_count -target_count) < 4) // Within threshold, increase duration
          {
            LCDmeasdif(true); // display E (measdif) on the LCD
            lock = 1;
            target_duration = duration * 2;
            if (target_duration > MAX_TIME)
              target_duration = MAX_TIME;
          }

          if (target_duration < PLL_START_DURATION || !phase_locked) {   // Not yet in phase detection mode
            calfact=calfact - measdif; // compute the new calfact
            LCDmeasdif(false); // Call the display Error E (measdif) routine
            if(abs(measured_count -target_count) > 10) // Too large, increase speed
            {
              target_duration = duration / 2;
              if (target_duration == 0)
                target_duration = 1;
            }
          }
    update:
          target_freq = 10000000000;
          XtalFreq_x1000 = INITIAL_XTAL - calfact*2 - calfact/2;

          // 
          // This routine searches the best combination of PLL and fractional divider settings to minimize the frequency error
          uint32_t a,b,c, f_a=36,f_b=0,f_c;
          f_c = c = 1048575UL - 10000UL;
          uint32_t delta = 100000000;
        
          if (USE_XTAL)         // XTAL drifts when changing VCO PLL settings so skip
            goto fixed_CVO ;         
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
   fixed_CVO:
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
          p_delta_count = 0;
          p_delta_sum = 0;
          p_delta_average = 0.0;
        }

        String str;
        if (!phase_locked) {
          Serial.print(hour);
          Serial.print(":");
          Serial.print(minute);
          Serial.print(":");
          Serial.print(second);
        
          Serial.print(" dur=");
          Serial.print(duration); 
//          Serial.print(" tcount="); str = ToString(target_count);  Serial.print(str);
          Serial.print(" acount="); str = ToString(measured_count);  Serial.print(str);
          Serial.print(" dcount=");
          Serial.print((int)(measured_count -target_count));
        }
#if 1
        Serial.print(" calfact=");
        Serial.print(calfact);
        Serial.print(" freq=");
        str = ToString(10000000000ULL + (int64_t)calfact);
        Serial.print(str);

        Serial.print(" corr=");
        int e = 0;
        float f = ((float)(calfact - prev_calfact))/10000000000.0;
        while (f != 0.0 && fabs(f) < 1.0) {
          f *= 10.0;
          e--;
        }
        Serial.print(f,1);
        Serial.print("e");
        Serial.print(e);

        prev_calfact = calfact;
#endif
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

//************************************
// Display on the LCD the difference E between measured CLK0 and theoretical 100e6 
void LCDmeasdif(int good)
{
          lcd.setCursor(0,1);
          lcd.print("                ");
          lcd.setCursor(0,1);
          if (alarm)
            lcd.print("> "); 
          if (measdif > 0) lcd.print(" "); 
          if (abs(measdif) < 100) lcd.print(" "); 
          if (abs(measdif) < 10) lcd.print(" "); 
          lcd.print(measdif);
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



//  SI5351aStart();
//  SI5351aSetFreq(SYNTH_MS_0,2500000); 
//  SI5351aSetFreq(SYNTH_MS_1, Freq_1);
//  SI5351_write(CLK_ENABLE_CONTROL,0b00000100); // Turn OFF CLK2
//  SI5351_write(CLK_ENABLE_CONTROL,0b00000000); // Turn ON CLK2
//  SI5351aSetFreq(SYNTH_MS_2, Freq_2);          // Set CLK2 frequency

int64_t SI5351aActualFreq(int64_t freq_x1000)
{
  unsigned long  a, b, c;
  c = 0xFFFFF;  // Denominator derived from max bits 2^20
  a = PLLFReq_x1000 / freq_x1000; // 36 is derived from 900/25 MHz
  int64_t rest =  PLLFReq_x1000 - a*freq_x1000;
  b = (rest * c) / freq_x1000;
  return( PLLFReq_x1000 * c /(a*c + b) );
}
//******************************************************************
//  SI5351 Multisynch processing
//******************************************************************
int64_t SI5351aSetFreq(int synth, int64_t freq_x1000, int r_div)
{
  int64_t CalcTemp;
  unsigned long  a, b, c, p1, p2, p3;
  int64_t actual_freq;
  
  c = 0xFFFFF;  // Denominator derived from max bits 2^20

  a = PLLFReq_x1000 / freq_x1000; // 36 is derived from 900/25 MHz
  int64_t rest =  PLLFReq_x1000 - a*freq_x1000;
  b = (rest * c) / freq_x1000;

  actual_freq = PLLFReq_x1000 * c /(a*c + b);

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
    Serial.print((long) rest);
    String str;
        Serial.print(" target=");
        str = ToString(freq_x1000); Serial.print(str);
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

int64_t SI5351aActualPLLFreq(uint32_t a,uint32_t b,uint32_t c)
{
  PLLFReq_x1000 = (XtalFreq_x1000 * (a * c + b) / c);
  return PLLFReq_x1000;
}

int64_t SI5351aSetPLLFreq(uint32_t a,uint32_t b,uint32_t c)
{
  PLLFReq_x1000 = (XtalFreq_x1000 * (a * c + b) / c);

#if 0
    Serial.print("abc=");
    Serial.print(a);
    Serial.print(",");
    Serial.print(b);
    Serial.print(",");
    Serial.print(c);

    String str;
        Serial.print(" PLLFreq=");
        str = ToString(PLLFReq_x1000); Serial.println(str);
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
