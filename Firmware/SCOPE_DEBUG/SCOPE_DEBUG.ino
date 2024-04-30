#include <avr/io.h>//for fast PWM
#include "fix_fft.h"//spectrum analyze

//OLED display setting
#include <SPI.h>//for OLED display
#include <Adafruit_GFX.h>//for OLED display
#include <Adafruit_SSD1306.h> //for OLED display
#include <FastGPIO.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
                        OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

//rotery encoder setting
#define ENCODER_OPTIMIZE_INTERRUPTS //contermeasure of rotery encoder noise
#define ENCODER_COUNT_PER_ROTATION 4
#include <Encoder.h>
#define ENCODER_SW_PIN 5
Encoder myEnc(2, 4);//rotery encoder digitalRead pin
float oldPosition  = -999;//rotery encoder counter
float newPosition = -999;


byte mode = 1;//1=low freq oscilo , 2=high freq oscilo , 3 = mid freq oscilo with external trig , 4 = spectrum analyze
byte old_mode = 1;//for initial setting when mode change.
byte param_select = 0;
byte param = 0;
byte param1 = 1;
bool param1_select = 0;
byte param2 = 1;
bool param2_select = 0;
int rfrs = 0;//display refresh rate

bool trig = 0;//external trig
bool old_trig = 0;//external trig , OFF -> ON detect
bool old_SW = 0;//push sw, OFF -> ON detect
bool SW = 0;//push sw

bool sw = 0;//push button
bool old_sw;//countermeasure of sw chattering
unsigned long sw_timer = 0;//countermeasure of sw chattering

unsigned long trigTimer = 0;
unsigned long hideTimer = 0;//hide parameter count
bool hide = 0; //1=hide,0=not hide

char data[128], im[128] , cv[128]; //data and im are used for spectrum , cv is used for oscilo.

int clk_val = 0;

void setup() {
 Serial.begin(57600);
 //display setting
 display.begin(SSD1306_SWITCHCAPVCC);
 display.clearDisplay();
 display.setTextSize(0);
 display.setTextColor(WHITE);
 analogReference(DEFAULT);

 //pin mode setting
 pinMode(3, OUTPUT) ;//offset voltage
 pinMode(5, INPUT_PULLUP);//push sw
 pinMode(6, INPUT) ;//input is high impedance -> no active 2pole filter , output is active 2pole filter
 pinMode(7, INPUT); //external triger detect
 

 //fast pwm setting
 TCCR2B &= B11111000;
 TCCR2B |= B00000001;

 //fast ADC setting
 ADCSRA = ADCSRA & 0xf8;//fast ADC *8 speed
 ADCSRA = ADCSRA | 0x04;//fast ADC *8 speed
};

void loop() {



  // Read CPU clock rate and convert to string
  String clockrate = String(F_CPU / 1000000) + " MHz";

  // Clear OLED display and print clock rate and other debug info
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  // Draw dot in bottom right corner of OLED display
  display.drawRect(120, 60, 4, 4, WHITE);

  unsigned long start = micros();
  unsigned long startMillis = millis();
  // display text for referencing platform
  display.setCursor(15, 2);
#if F_CPU == 32000000
  display.print("LGT8F328P");
  display.setCursor(5, 12);
  display.print("CPU speed: ");
  display.println("32 MHz");
#elif F_CPU == 16000000
  display.print("Arduino nano v3.0");
  display.setCursor(14, 12);
  display.print("CPU speed: ");
  display.println("16 MHz");
#endif
  // display frame rate for performance measure
  display.setCursor(7, 32);
  //Serial.print("Free RAM: ");
  //Serial.println(freeMemory());
  //Serial.print("Millis: ");
  //Serial.println(millis());
  // loop time for performance measure
  //display.setCursor(8, 44);
  //display.print("time: ");
  //display.print(millis() - start);
  //display.print("ms");
  // reset loop time  for performance measure
  start = millis();
  display.display();
  delay(10);


oldPosition = newPosition;
  //-----------------Rotery encoder read----------------------
  newPosition = myEnc.read() / ENCODER_COUNT_PER_ROTATION;

  if ( newPosition   < oldPosition ) {//turn left

    Serial.println("left");
    display.setTextSize(2);
    display.setCursor(50, 20);
    display.println("left");
    display.display();
    oldPosition = newPosition;
  }
  if (digitalRead(7) != clk_val) {
    clk_val = digitalRead(7);
    display.fillRect(120, 60, 4, 4, WHITE);
    //Serial.println("TRG");
    Serial.println("External Trigger Detected");
  }

  else if ( newPosition    > oldPosition ) {//turn right
    Serial.println("right");
    display.setTextSize(2);
    display.setCursor(50, 20);
    display.println("right");
    display.display();
  }
  sw = 1;
  if ((!FastGPIO::Pin<ENCODER_SW_PIN>::isInputHigh()) && ( sw_timer + 300 <= millis() )) {
    sw_timer = millis();
    Serial.println("click");
    display.setTextSize(2);
    display.setCursor(50, 20);
    display.println("click");
    display.fillRect(120, 60, 4, 4, WHITE);
    display.display();
    sw = 0;
  }


};

// debug outpuf for setup function and OLED display
// show for 5 seconds
void debug_display()
{
  // measure time for debug
  unsigned long start = micros();
  unsigned long startMillis = millis();
  // clear display for debugging performance
  display.clearDisplay();
  // display text for referencing platform
  display.setCursor(15, 2);
#if F_CPU == 32000000
  display.print("LGT8F328P");
  display.setCursor(5, 12);
  display.print("CPU speed: ");
  display.println("32 MHz");
#elif F_CPU == 16000000
  display.print("Arduino nano v3.0");
  display.setCursor(5, 12);
  display.print("CPU speed: ");
  display.println("16 MHz");
#endif
  // calculate and display elapsed time for performance measure
  //display.setCursor(5, 20);
  //display.print("elapsed time : ");
  //display.print(millis() - start);
  //display.print(" ms");
  // display frame rate for performance measure
  display.setCursor(7, 32);
  display.print("frame rate : ");
  display.print(1000 / (millis() - start));
  display.print(" fps");
  // loop time for performance measure
  //display.setCursor(12, 44);
  //display.print("loop time : ");
  //display.print(millis() - start);
  //display.print(" ms");
  // reset loop time  for performance measure
  start = millis();
  delay(10);
}

// Function to calculate free RAM
int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
