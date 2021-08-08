/*Permission is granted to use, copy, modify, and distribute this software and documentation for non-commercial purposes. 
(F2DC 17 April 2017)

I was greatly helped by the works of Gene Marcus W3PM, Jason Milldrum NT7S-Dana H. Myers K6JQ, Igor Gonzales Martin and many others.
Thanks to all of them. 

==> This new version V.5.0 (May2020) can be used with the present version of the NT7S SI5351 library. I tested it with the 2.1.4 NT7S revision.
==> Version V.5.2 (June 2020) : The LCD shows now error E as "Rel.err=-70e-8"

SW modified by Erik Kaashoek to allow for longer measurement times enabling more accuracy.

 */

// include the library code:
#include <string.h>
#include <ctype.h>
#include <avr/interrupt.h>  
#include <avr/io.h>
#include <Wire.h>

#include <EEPROM.h>

//#define OLED
//#define LCD
#define DAC

#ifdef DAC
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;
#endif

#ifdef LCD
#include <LiquidCrystal.h>
#endif
#ifdef OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif

#ifdef LCD
#define displ_print lcd.print
#define displ_setCursor(X,Y) lcd.setCursor(X, Y)
#else
#ifdef OLED
#define displ_print display.print
#define displ_setCursor(X,Y) displ.setCursor(X*10, Y*32)
#else
#endif
#define displ_print(X)
#define displ_setCursor(X,Y)
#endif

// Set up MCU pins
#define ppsPin                   2 // from GPS 
#define ALARM_LED                3 // LED Alarm XtalFreq
#define LOCK_LED                 4
#define OPEN_LOOP                8
#define COUNT_PIN                5
#define DAC_HIGH                6
#define DAC_LOW                 7

#ifdef lcd
#define RS                       7 // LCD RS
#define E                        8 // LCD Enable
#define DB4                      9 // LCD DB4
#define DB5                     10 // LCD DB5
#define DB6                     11 // LCD DB6
#define DB7                     12 // LCD DB7
#endif

#define USE_PHASE_DETECTOR  true        // Set to false if the XTAL is too unstable to phase lock
#define SEARCH_OPTIMUM      false       // Set to true if a stable TCXO is used for the SI5351, should be false when not using the phase detector

#define MIN_DURATION        5
#define PLL_START_DURATION  40       // Start using phase when duration exceeds this.
int phase_start_duration = PLL_START_DURATION;
#define  INITIAL_PLL 900000000000LL
int64_t PLLFReq_x1000 = INITIAL_PLL;  // In 1/1000 Hz, will be update to actual frequency
//#define INITIAL_XTAL 26000000000LL      // In 1/1000 Hz
#define INITIAL_XTAL 10000000000LL      // In 1/1000 Hz
int64_t XtalFreq_x1000 = INITIAL_XTAL;   

#define A_START (600000000000LL / INITIAL_XTAL)
#define A_STOP  (900000000000LL / INITIAL_XTAL)

#define CAL_FREQ  2500000UL      // In Hz, maximum is 4.5MHz
#define CALFACT_START 0       // Can be set to pre-load the correction to speed up locking.
#define MIN_PULSES  0       // Heuristic number of pulses that are missed due to the code structure.

#define MAX_TIME  (int)(10000000000LL / CAL_FREQ )    // Time to count 10 billion counts    with 2.5MHz this is 4000 seconds
//#define MAX_TIME  (int)(1000000000LL / CAL_FREQ )    // Time to count 1 billion counts, with 2.5MHz this is  400 seconds
//#define MAX_TIME  (int)(100000000LL / CAL_FREQ )    // Time to count 100 million counts, with 2.5MHz this is  40 seconds
//#define MAX_TIME  (int)(10000000LL / CAL_FREQ )    // Time to count 10 million billion counts, with 2.5MHz this is  4 seconds

#define SMALL_STEP    10      // 10e-10
//#define SMALL_STEP    100      // 10e-9
//#define SMALL_STEP    1000      // 10e-8

#define TEST  false

#ifdef LCD
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(RS, E, DB4, DB5, DB6, DB7);
#endif
// Variables 
byte res,Epos;
byte CptInit=1;
char StartCommand2[7] = "$GPRMC",buffer[300] = "";
int IndiceCount=0,StartCount=0,counter=0,indices[13];
int validGPSflag = 1;     // Set to 1 to avoid waiting for a valid GPS string in case no connection between GPS and Arduino
int Chlength;
int byteGPS=-1,second=0,minute=0,hour=0;
int64_t mult=0;         // Count of overflows of 16 bit counter
int alarm = 0;
unsigned int pulse_count=0;        // counts the seconds in a measurement cycle 
int duration = 2;             // Initial measurement duration
int target_duration = 4;         // Value of duration for next measurement cycle
int count_available = 0;            // Flag set to 1 after pulse counting finished
int speed_available = 0;
int phase_available = 0;
int stable_count = 0;         // Count of consecutive measurements without correction needed.


long dac_out = 0;
long  dac_manual = 0;
#if 0
long dac_base = 30257;
float dac_mult = 0.025;
#else
#define DAC_RANGE 4096
long dac_base = DAC_RANGE/2 - 1;
float dac_mult = 1;
#endif

#define MAX_CALFACT (DAC_RANGE/2)
#define MIN_CALFACT (-DAC_RANGE/2)


int32_t calfact_x10 =0;                 // Current correction factor in 1/100 Hz
int32_t prev_calfact_x10 = 0;
int64_t target_count,             // Target count of pulses
measured_count;                 // Actual count of pulses

volatile int phase = 0;
float speed = 0;
float phase_average = 0;
float prev_phase_average = 0;
int prev_phase = 0;
int phase_wrap = false;
int p_delta = 0;
volatile int tick = 0;
float p_delta_average = 0;
int phase_count = 0;
int phase_count_max = PLL_START_DURATION;
int p_delta_sum = 0;

float high_gain =3.0e-7 / (float)256; // Minimum correction step of high DAC
float gain = (float)3.0/(float)200000000.0 / (float)4096;  // Minimum correction step of low DAC
float corr = 0.0;
float min_corr = 0.0;


int dump_phase = false;

int alarm_led = true;
int lock_led = false;
int blink_alarm = false;
int blink_lock = false;


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


int force_duration = 0;
unsigned int low_count = 0;
int64_t high_count = 0;
unsigned int prev_low_count = 0;
int64_t prev_high_count = 0;
int64_t prev_count = 0;

int64_t delta_count = 0;

// Timer 1 overflow interrupt vector.
ISR(TIMER1_OVF_vect) 
{
  mult++;        //Increment overflow multiplier
  high_count++;
  TIFR1 = (1<<TOV1);  //Clear overflow flag by shifting left 
}


/* Clock - interrupt routine for counting the CLK0 2.5 MHz signal
Called every second by GPS 1PPS on Arduino Nano D2
 */
void PPSinterrupt()
{
  
  low_count = TCNT1;
  delta_count = (int64_t)low_count  - (int64_t)prev_low_count + (high_count - prev_high_count) * 0x10000LL;
  prev_high_count = high_count;
  prev_low_count = low_count;
//  String str = ToString(delta_count);  Serial.println(str);
  if (!count_available) {
    if (pulse_count == 0)
      measured_count = 0;  // Add pulses
    pulse_count++;              // Increment the seconds counter
    measured_count += delta_count;  // Add pulses
    if (pulse_count >= target_duration) //Stop the counter : the target_duration gate time is elapsed
    {     
      duration = target_duration;
      target_count=((int64_t)CAL_FREQ)*duration;          //Calculate the target count     
      count_available = 1;
    }
  } 
  if (USE_PHASE_DETECTOR && !phase_available) {
    phase = analogRead(A0);
    p_delta = phase - prev_phase;
    prev_phase = phase;

    if (phase_count < phase_count_max) {          // Result count_available when phase_count == phase_count_max
      if (abs(p_delta)<250) {                   // Valid phase measurement, this to prevent phase wrapping
        if (phase_count == 0) {                // Reset sum 
          p_delta_sum = 0;
          prev_phase_average = phase_average;
          phase_average = 0;
          phase_wrap = false;
        }
        p_delta_sum += p_delta;
        phase_count++;
        phase_average += phase;
      } else {
        phase_wrap = true;
      }
      if (phase_count >= phase_count_max) {
        phase_available = 1;
        p_delta_average = (float)p_delta_sum / (float)phase_count;
        if (phase_wrap)
          phase_average = -1;
        else
          phase_average /= phase_count;
      }
    }
    if (dump_phase) {
      Serial.print(F("count="));
      Serial.print((int)delta_count);
      Serial.print(F(" phase="));
      Serial.print(phase);
      Serial.print(F(" delta="));
      Serial.println(p_delta);
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
    displ_setCursor(0,0);
#if 0
    if (phase < 100) displ_print (F(" "));
    if (phase < 10) displ_print (F(" "));
    displ_print(phase);
#endif
    if (p_delta >= 0)
      displ_print(F(" "));
    if (abs(p_delta) < 100) displ_print (F(" "));
    if (abs(p_delta) < 10) displ_print (F(" "));
    displ_print(p_delta);
    //    displ_print(F("     "));    
    displ_setCursor(7,0); // LCD cursor on the right part of Line 0
    if (hour < 10) displ_print (F("0"));
    displ_print (hour);
    displ_print (F(":"));
    if (minute < 10) displ_print (F("0"));
    displ_print (minute);
    displ_print (F(":"));
    if (second < 10) displ_print (F("0"));
    displ_print (second);
    //   displ_print (F("Z"));  // UTC Time Indicator
  }
  if (blink_alarm) {
    alarm_led = !alarm_led;
    digitalWrite(ALARM_LED, alarm_led);
  }
  if (blink_lock) {
    lock_led = !lock_led;
    digitalWrite(LOCK_LED, lock_led);
  }
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

void print_float(float v, int n) {
    int e = 0;
    float f = v;
    while (f != 0.0 && fabs(f) < 1.01) {
      f *= 10.0;
      e--;
    }
    Serial.print(f,1);
    Serial.print(F("e"));
    Serial.print(e);
    if (n)
        Serial.print(F("\n")); 
}
void setup()
{
  Serial.begin(9600);  // Define the GPS port speed
  Wire.begin(1);    // I2C bus address = 1
  SI5351aStart();  
  SI5351aSetFreq(SYNTH_MS_1, 10000000000LL,2);     // 10MHz, divide by 4
  SI5351aSetFreq(SYNTH_MS_0, 10000000000LL,0);    // 10MHz
  SI5351aSetFreq(SYNTH_MS_2,100000000000LL,0);     // 100MHz
  //  SI5351_write(CLK_ENABLE_CONTROL,0b00000100); // Turn OFF CLK2, not used
  SI5351_write(CLK_ENABLE_CONTROL,0b00000000); // Turn ON CLK2

  //Set up Timer1 as a frequency counter - input at pin 5
  TCCR1B = 0;     //Disable Timer during setup
  TCCR1A = 0;     //Reset
  TCNT1  = 0;     //Reset counter to zero
  TIFR1  = 1;     //Reset overflow
  TIMSK1 = 1;     //Turn on overflow flag

  // Set up the LCD's number of columns and rows 
#ifdef LCD

  lcd.begin(16,2); 
  lcd.display();           // initialize LCD
#endif
#ifdef OLED
  if(!display.begin(SSD1306_EXTERNALVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.display();
  display.clearDisplay();
  display.setTextSize(3);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text

#endif
  // GPS 1pps input
  pinMode(ppsPin, INPUT);
  analogReference(INTERNAL);
  displ_setCursor(0,1);
  displ_print(F(" F2DC V.5.2")); // display version number on the LCD
  delay(1000);

  // Set up IO switches
  pinMode(ALARM_LED, OUTPUT); // Alarm LED for weird measured_count
  digitalWrite(ALARM_LED, HIGH);
  pinMode(LOCK_LED, OUTPUT);  // Lock lED
  digitalWrite(LOCK_LED, LOW);
  pinMode(OPEN_LOOP, INPUT_PULLUP ); // Alarm LED for weird measured_count

  // Set Arduino D2 for external interrupt input on the rising edge of GPS 1PPS
  attachInterrupt(0, PPSinterrupt, RISING);  

  displ_setCursor(0,1);
  displ_print(F("Waiting for GPS"));
  Serial.print(F("high_gain="));
    int e = 0;
    float f = high_gain;
    while (f != 0.0 && fabs(f) < 1.01) {
      f *= 10.0;
      e--;
    }
    Serial.print(f,1);
    Serial.print(F("e"));
    Serial.println(e);

   Serial.print(F("gain="));
    e = 0;
    f = gain;
    while (f != 0.0 && fabs(f) < 1.01) {
      f *= 10.0;
      e--;
    }
    Serial.print(f,1);
    Serial.print(F("e"));
    Serial.println(e);

  Serial.println(F("Waiting for GPS"));
  


  TCCR1B = 0;    //Turn off Counter

    TCNT1 = 0;       //Reset counter to zero
    low_count = 0;
    high_count = 0;
    TCCR1B = 7;    //Clock on rising edge of pin 5

  // Turn OFF CLK2 
  //  SI5351.output_enable(SI5351_CLK2,0); 

  // Set up parameters
  calfact_x10 = CALFACT_START;
  target_count=CAL_FREQ*duration;

#ifdef DAC
  dac.begin(0x60);
#endif
}

char testKey()
{
  if(Serial.available() > 0)   // see if incoming serial data:
  {
    return(Serial.read());  // read oldest byte in serial buffer:
  }
  return(0);
}

long old_dac = 0;
long old_dac_high = 0;
#define INITIAL_DAC_HIGH  115
int dac_high = INITIAL_DAC_HIGH;

void set_dac() {
  if (dac_manual != 0)
    dac_out = dac_manual;
  else
    dac_out = (calfact_x10) * dac_mult + dac_base;

  if (dac_out < 0)   
  { dac_out = 0;}
#if 0
  if (dac_out > 65535)
  { dac_out = 65535;}
#else
  if (dac_out > 4095)
  { dac_out = 4095;}
#endif

      if (dac_high < 0)
        dac_high = 0;
      if (dac_high > 255)
        dac_high = 255;

  if (dac_out != old_dac || old_dac_high != dac_high) {

    analogWrite(DAC_HIGH,dac_high);
    dac.setVoltage(dac_out, false);
//    delay (abs(old_dac - dac_out)/10);
    old_dac = dac_out;
    old_dac_high = dac_high;
  }
}

#if 0
int eeprom_start = 0;


void do_eeprom(char c) {
  switch(c) {
    case 'c':
      for (int i=0; i<EEPROM.length(); i += 4) {
        EEPROM[i] = 0;
        EEPROM[i+1] = 0;
      }
      break;
    case 'd':
      eeprom_init();
      for (int i=0; i < eeprom_start; i += 4) {
        int d = EEPROM[i++] + (EEPROM[i++] <<8);
        float f = EEPROM[i]  >= 128 ? EEPROM[i++] - 256 : EEPROM[i++]);
        
        int e =  EEPROM[i++];
        Serial.print(F("corr="));
        Serial.print(f);
        Serial.print(F("e-"));
        Serial.print(e);
        Serial.print(F("dac="));
        Serial.print(d);
      }
      break;
  }
}


void eeprom_init(void) {
  eeprom_start = 0;
  while (eeprom_start < EEPROM.length()-4 && EEPROM[eeprom_start] == 0 && EEPROM[eeprom_start+1] == 0 )
    eeprom_start += 4; 
}

void eeprom_log(float corr, int dac) {
  EEPROM[eeprom_start++] = (dac & 0xff);
  EEPROM[eeprom_start++] = ((dac >> 8) & 0xff);
  int e = 0;
  float f = cor;
  while (f != 0.0 && fabs(f) < 1.01) {
    f *= 10.0;
    e--;
  }
  EEPROM[eeprom_start++] = 0xff & ((int) (f * 10.0));
  EEPROM[eeprom_start++] = e;
}
#endif

//******************************************************************
// Loop 
void loop()
{
  int update = false;
  int phase_locked = false;
  int lock = 0; 
  String str = "";
  int64_t target_freq,actual_freq;
  char c = testKey();
  switch(c) {
#if 0
    case 'e':
    do_eeprom(testKey());
    break;
#endif
  case 'p':
    dump_phase = !dump_phase;
    break;
  case 'd':
    dac_manual = Serial.parseInt();
    Serial.print(F("dac_manual = "));
    Serial.println(dac_manual); 
    set_dac();
    return;
  case 'h':
    dac_high = Serial.parseInt();
    Serial.print(F("dac_high = "));
    Serial.println(dac_high); 
    set_dac();
    return;
  case 'a':
    dac_manual = 0;
    break;
  case '+':
    target_duration *= 2;
    phase_count_max *= 2;
    if (target_duration < 1)
      target_duration = 1;
    if (phase_count_max < 1)
      phase_count_max = 1;
    pulse_count = 0;
    phase_count = 0;
    break;
  case '-':
    target_duration /= 2;
    phase_count_max /= 2;
    pulse_count = 0;
    phase_count = 0;
    break;
  case '>':
    dac_manual += 100;
    break;
  case '<':
    dac_manual -= 100;
    break;
  case 'r':
    calfact_x10 = 0;
    break;  
  case 'f':
    force_duration = Serial.parseInt();
    phase_count = 0;
    pulse_count = 0;
    break;
  case 'u':
    phase_start_duration = Serial.parseInt();
    phase_count = 0;
    pulse_count = 0;
    break;
  case 't':
    dac_mult = Serial.parseFloat();
    Serial.print(F("dac_mult = "));
    Serial.println(dac_mult); 
    break;
  case 'c':
 // high dac

    dac_manual = 2047;
    dac_high = 0;
    set_dac();
    Serial.println(F("wait 5s"));
    delay(5000);
    target_duration = 16;
    pulse_count = 0;
    count_available = 0;
    while(!count_available) { delay(2000);Serial.print(pulse_count);Serial.print(" "); }
    Serial.print(F("acount=")); str = ToString(measured_count);  Serial.print(str);
    Serial.print(F(" dcount=")); Serial.println((int)(measured_count -target_count));
    long low_count = measured_count;

    dac_manual = 2047;
    dac_high = 255;
    set_dac();
    Serial.println(F("wait 5s"));
    delay(5000);
    target_duration = 16;
    pulse_count = 0;
    count_available = 0;
    while(!count_available) { delay(2000);Serial.print(pulse_count);Serial.print(" "); }
    Serial.print(F("acount=")); str = ToString(measured_count);  Serial.print(str);
    Serial.print(F(" dcount=")); Serial.println((int)(measured_count -target_count));
    long high_count = measured_count;

    Serial.print(F("high range=")); 
    print_float( ((float)high_count - (float)low_count)/(float)target_count, true);

    

 // low dac 
    dac_high = INITIAL_DAC_HIGH;
    dac_manual = 1;
    set_dac();
    Serial.println(F("wait 5s"));
    delay(5000);
    target_duration = 80;
    pulse_count = 0;
    count_available = 0;
    while(!count_available) { delay(2000);Serial.print(pulse_count);Serial.print(" "); }
    Serial.print(F("acount=")); str = ToString(measured_count);  Serial.print(str);
    Serial.print(F(" dcount=")); Serial.println((int)(measured_count -target_count));
    low_count = measured_count;
    dac_manual = 4095;
    set_dac();
    Serial.println(F("wait 5s"));
    delay(5000);
///    target_duration = 80;
    pulse_count = 0;
    count_available = 0;
    phase_available = 0;
    while(!count_available) { delay(2000);Serial.print(pulse_count);Serial.print(" "); }
    Serial.print(F("acount=")); str = ToString(measured_count);  Serial.print(str);
    Serial.print(F(" dcount=")); Serial.println((int)(measured_count -target_count));
    high_count = measured_count; 
    
    Serial.print(F("low range=")); 
    print_float(((float)high_count - (float)low_count)/(float)target_count, true);

    
    dac_manual = 0;
    set_dac();
    count_available = 0;
    phase_available = 0;
    pulse_count = 0;
    return;
  }
  if (validGPSflag == 0) 
    GPSprocess( ); //If GPS is selected, wait for valid NMEA data
  else if (USE_PHASE_DETECTOR && target_duration >= phase_start_duration && count_available && phase_available) {

    if (!digitalRead(OPEN_LOOP)) {

    // Speed
    if ( false &&fabs(p_delta_average) < 2.0 && !phase_wrap && prev_phase_average != -1 && fabs(phase_average - prev_phase_average) < 100) {
      speed = ( phase_average - prev_phase_average ) / phase_count;
      corr = -speed * 0.5e-9;
      min_corr = 1e-13;
      speed_available = 1;
    } else 
    // Phase
    if (fabs(p_delta_average) < 5.0) {
      corr = -p_delta_average * 0.25e-9;      // Half speed to avoid overcompensations
      min_corr = 22e-9 / phase_count;
    }
    else {
      corr = -p_delta_average * 0.5e-9;     // resolution of phase detector
      min_corr = 22e-9 / phase_count;
    }
    if (fabs(corr) > 20.0 * fabs(min_corr)) {
      phase_count_max /= 2;
    }
    if (fabs(corr) < 2.0 * fabs(min_corr)) {
      lock = 1;
      phase_count_max *= 2;
      if (phase_count_max > 400)
        phase_count_max = 400;
    }
    if (force_duration)
      phase_count_max = force_duration;
    target_duration = phase_count_max;
    }
    
    
    if (fabs(p_delta_average) < 5.0) {
      blink_lock = false;
      digitalWrite(LOCK_LED,HIGH);   // final lock
    }
    else
      blink_lock = true;
    phase_locked = true;
    update = true;
  } else if(count_available && target_duration < phase_start_duration) // Frequency calculation data count_available                                   
  {
    digitalWrite(ALARM_LED, LOW);
    if (measured_count-target_count >= 0)               
      corr =(float)(measured_count-target_count)/ (float)target_count;
    else
      corr=(float) (measured_count+1-target_count )/ (float)target_count;

    min_corr = 1/(float)target_count;

#if 0
    if(measdif_x10<-5000000 || measdif_x10>+5000000) // Impossible error, alarm, not used
    {
      blink_alarm = true;
      Serial.print(F("Alarm "));
      alarm = 1;
      target_duration /= 2;
      if (target_duration < MIN_DURATION)
        target_duration = MIN_DURATION;

    }
    else  
#endif
    {
      blink_alarm = false;
      alarm = 0;
      if (!digitalRead(OPEN_LOOP)) {
       if(fabs(corr) == 0) // Within threshold, increase duration
       {
        lock = 1;
        target_duration = duration * 2;
        if (target_duration > MAX_TIME)
          target_duration = MAX_TIME;
       }

       if (target_duration < phase_start_duration || !phase_locked) {   // Not yet in phase detection mode
        if(abs(measured_count -target_count) > 10) // Too large, increase speed
        {
          target_duration = duration / 2;
          if (target_duration < MIN_DURATION)
            target_duration = MIN_DURATION;
        }
       }
      update = true;
      }

    }
    if (force_duration)
      target_duration = force_duration;

  }

  if (update && ! digitalRead(OPEN_LOOP) && ((SEARCH_OPTIMUM /* &&  abs(measdif_x10) > 3 */)|| fabs(corr) > 0 )) {

    if ((hour*60LL+minute)*60LL+second < 20LL) {
      calfact_x10=0; //freeze
      dac_high = INITIAL_DAC_HIGH;
    } else if (fabs(corr) > 1.5*high_gain) {  // use high dac
      dac_high -=  corr / high_gain;
      calfact_x10 = 0;
    } else   
      calfact_x10=calfact_x10 - ( corr / gain ); // compute the new calfact

    if (calfact_x10 > (float)MAX_CALFACT/dac_mult)
      calfact_x10 = (float)MAX_CALFACT/dac_mult;
    if (calfact_x10 < (float)MIN_CALFACT/dac_mult)
      calfact_x10 = (float)MIN_CALFACT/dac_mult;

    LCDmeasdif(); // Call the display Error E (measdif) routine

#ifdef DAC

    set_dac();

#else    

    target_freq = 10000000000;
    XtalFreq_x1000 = INITIAL_XTAL - (INITIAL_XTAL / 10000000LL ) * calfact_x10/ 10000LL;     // calfact is versus 10MHz in 1/1000 Hz

    // 
    // This routine searches the best combination of PLL and fractional divider settings to minimize the frequency error
    uint32_t a,b,c, f_a=A_STOP,f_b=0,f_c;
#define C_START (1048575UL - 10000UL)
    f_c = c = C_START;
    uint32_t delta = 100000000;

    if (SEARCH_OPTIMUM)         // XTAL drifts when changing VCO PLL settings so skip
    {
      SI5351_write(CLK_ENABLE_CONTROL,0b00000111); // Turn OFF all output
      for (a = A_START; a<=A_STOP; a++) {
        for (b = 524287UL; b <524303UL; b++) {
          //          for (c = C_START-20; c< C_START; c++) {
          SI5351aActualPLLFreq(a,b,c);
          actual_freq = SI5351aActualFreq(target_freq);
          if (abs(actual_freq - target_freq) < delta) {
            delta = abs(actual_freq - target_freq);
            f_a = a;
            f_b = b;
            f_c = c;
          }
          //          }
        }
      }
    }
    SI5351aActualPLLFreq(f_a,f_b,f_c);
    SI5351aSetPLLFreq(f_a,f_b,f_c);
    actual_freq = SI5351aActualFreq(target_freq);
#if 0
    Serial.print(F("a,b,c="));
    Serial.print(f_a);
    Serial.print(F(","));
    Serial.print(f_b);
    Serial.print(F(","));
    Serial.print(f_c);
    Serial.println();
#endif
    SI5351aSetFreq(SYNTH_MS_1,target_freq,2);  // 2.5MHz
    actual_freq=SI5351aSetFreq(SYNTH_MS_0,target_freq,0);    // 10MHz

    if (SEARCH_OPTIMUM)
      SI5351_write(CLK_ENABLE_CONTROL,0b00000100); // Turn ON CLK0 and CLK1

#endif
    //    p_delta_sum = 0;
    //    p_delta_average = 0.0;
  }

  if (dac_manual != 0) {
    set_dac();
  }

  if (update) {
    Serial.print(hour);
    Serial.print(F(":"));
    if (minute < 10) Serial.print(F("0"));
    Serial.print(minute);
    Serial.print(F(":"));
    if (second < 10) Serial.print(F("0"));
    Serial.print(second);
    Serial.print(F(" dur="));

    if (phase_locked) {
        Serial.print(phase_count);  
      if (speed_available) {
        Serial.print(F(" speed=")); Serial.print(speed, 4); 
      } else {
        Serial.print(F(" p_average=")); Serial.print(p_delta_average); 
      }
      Serial.print(F(" phase=")); Serial.print(phase_average);
    } else {
      Serial.print(duration); 
      Serial.print(F(" acount=")); str = ToString(measured_count);  Serial.print(str);
      Serial.print(F(" dcount=")); Serial.print((int)(measured_count -target_count));
    }

    Serial.print(F(" calfact="));
    Serial.print(calfact_x10);
    Serial.print(F(" acount="));
    str = ToString(measured_count);  Serial.print(str);

    Serial.print(F(" mincorr="));
    int e = 0;
    float f = min_corr;
    while (f != 0.0 && fabs(f) < 1.01) {
      f *= 10.0;
      e--;
    }
    Serial.print(f,1);
    Serial.print(F("e"));
    Serial.print(e);
 

    Serial.print(F(" corr="));
    e = 0;
    f = corr;
    while (f != 0.0 && fabs(f) < 1.01) {
      f *= 10.0;
      e--;
    }
    Serial.print(f,1);
    Serial.print(F("e"));
    Serial.print(e);

    prev_calfact_x10 = calfact_x10;
#ifdef DAC
    Serial.print(F(" hdac = "));
    Serial.print(dac_high);
    Serial.print(F(" dac = "));
    Serial.print(dac_out);
#endif
    if (target_freq!=actual_freq) {
      Serial.print(F(" Freq_Error = "));
      Serial.print((int)(target_freq-actual_freq));
    }
    if (digitalRead(OPEN_LOOP))
      Serial.print(F(" OPEN"));
    if (lock) 
      Serial.println(F(" Lock"));
    else
      Serial.println(F(" "));

  }
  if(update) {
    pulse_count = 0;         // Restart count measurement
    phase_count = 0;
    count_available= 0;   // Restart
    phase_available = 0;
    speed_available = 0;
    corr = 0;
  }


}
//************************************
// Display on the LCD the difference E between measured CLK0 and theoretical 100e6 
void LCDmeasdif()
{
  displ_setCursor(0,1);
  displ_print(F("                "));
  displ_setCursor(0,1);
  if (alarm)
    displ_print(F("> ")); 
  displ_print(corr);
  displ_print(F(" ")); 
  displ_print(duration); 
  displ_print(F("s ")); 
  displ_print(calfact_x10);
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
          displ_setCursor(0,1);
          displ_print(F("               "));
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
  Serial.print(F("ab="));
  Serial.print(a);
  Serial.print(F(","));
  Serial.print(b);
  Serial.print(F(", rest="));
  Serial.print((long) rest);
  String str;
  Serial.print(F(" target="));
  str = ToString(freq_x1000); Serial.print(str);
  Serial.print(F(" actual="));
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
  Serial.print(F("abc="));
  Serial.print(a);
  Serial.print(F(","));
  Serial.print(b);
  Serial.print(F(","));
  Serial.print(c);

  String str;
  Serial.print(F(" PLLFreq="));
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

  SI5351aSetPLLFreq((uint32_t)A_STOP,(uint32_t)0,(uint32_t)0xFFFFFUL);
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
