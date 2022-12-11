//#define DEBUG

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h>
#include <EEPROM.h>
#include <Keypad.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

#include <Wire.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x20 for a 16 chars and 2 line display

#define PWM_INPUT_PIN   2   //pin used to receive PWM signe from RAMP or UNO itself

#define PWM_OUTPUT_PIN   5   
#define WS_OUTPUT        6 

#define MAX_NB_MENU 3

#define NUMPIXELS 8 // Popular NeoPixel ring size



// 4*3 keypad
/*
const byte ROWS = 4; //rows
const byte COLS = 3; //columns
//define the symbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {8, 9, 10, 11}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {A0, A1, A2}; //connect to the column pinouts of the keypad
*/
const byte ROWS = 5; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'A','B','#','*'},
  {'1','2','3','U'},
  {'4','5','6','D'},
  {'7','8','9','E'},
  {'L','0','R','V'}
};
byte rowPins[ROWS] = {8, 9, 10, 11, 12}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {A3, A2, A1, A0}; //connect to the column pinouts of the keypad


typedef struct {
  short MaxPWM;   // %
  short MaxTemp;  // °
  short ZToFocus; // 1/10 mm
//  short ZDelta;   // mm
  short Size;     // mm
  short Speed;     // mm/s
  short PWM;      // %
  short Mode;
  short Step;     // 1/10 mm
  short Duration;  // ms
  short Thickness;     // 1/10 mm
} Parameters;

Parameters g_Parameters;


//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
Adafruit_NeoPixel pixels(NUMPIXELS, WS_OUTPUT, NEO_GRB + NEO_KHZ800);



char g_IsSignal = 0;
unsigned long g_PWMHighDuration, g_PWMLowDuration, g_StartPeriodLH, g_StartPeriodHL;

char Tmp[32];
long DebugTimer;
String g_String;

// internal values to count events
unsigned char Counter10ms;
unsigned char Tick100ms;
unsigned char Counter100ms;
unsigned char Tick500ms;
unsigned char Counter500ms;
unsigned char Tick1s;


// global variables
//short g_JoystickValue;
short g_keyboardValue;
short g_SerialValue;
long g_Freq, g_Cycle;
short g_Menu;
short g_Item;
short g_EditMode;
short g_EditCnt;
short g_Edit;

/**********************************************************************************************************/
/**********************************************************************************************************/

void TimerCallBback() 
{
  Counter10ms++;
  if(Counter10ms>=10)
  {
      Tick100ms = 1;
      Counter10ms = 0;
  }
}


void UpdatePixels( int nb, int r, int g, int b )
{
  pixels.clear(); // Set all pixel colors to 'off'

  for(int i=0; i<nb; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(r, g, b));

    pixels.show();   // Send the updated pixel colors to the hardware.

  }
}

void GetInputs( void )
{
int aval;

  g_keyboardValue = customKeypad.getKey();

  // manage input char
  if (Serial.available()) {      // If anything comes in Serial (USB),
    g_SerialValue = Serial.read();   
  }
  else
  {
    g_SerialValue = 0;
  }
  
}




// interrup procedure to manage the RC signal
// should occur more than 20 times per sec
void PWM_InputCallBback()
{
  g_IsSignal = 1;
  
  // going HIGH to LOW
  if(digitalRead(PWM_INPUT_PIN) == LOW) // High to low
  { 
    g_StartPeriodHL = micros();
    g_PWMHighDuration = g_StartPeriodHL - g_StartPeriodLH;
  }
  else // going LOW to HIGH
  {
    g_StartPeriodLH = micros();
    g_PWMLowDuration = g_StartPeriodLH - g_StartPeriodHL;
  }
}

/**********************************************************************************************************/
/**********************************************************************************************************/

void setup() 
{
int i;  

  Serial.begin(9600);
  Serial.println(";System Started");
  
  // Init PINs
  pinMode(PWM_INPUT_PIN, INPUT);
//  pinMode(MODE_INPUT_PIN, INPUT_PULLUP);
//  pinMode(JOY_INPUT_PIN, INPUT);

//  pinMode(ENA_OUTPUT_PIN, OUTPUT);
  pinMode(PWM_OUTPUT_PIN, OUTPUT);

  g_PWMHighDuration = 0;
  g_PWMLowDuration = 0;
  g_StartPeriodLH = 0;
  g_StartPeriodHL = 0;


//  analogReference(DEFAULT);
  LoadCalibration();


  displaySetup();
  StartupScreen();

  attachInterrupt(digitalPinToInterrupt(PWM_INPUT_PIN),PWM_InputCallBback,CHANGE);

  // init timer and counters 
  Timer1.initialize(10000);         //Set a 10 ms call back period
  Timer1.attachInterrupt(TimerCallBback);  // attaches callback() as a timer overflow interrupt

  g_keyboardValue = 0;

  analogWrite(PWM_OUTPUT_PIN, (g_Parameters.PWM*256)/100 );
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  UpdatePixels( 8, 0, 0, 32);
//  lcd.cursor();

  g_Menu = 0;
  g_Item = 0;

  g_EditMode = 0;
  g_EditCnt = 0;
  g_Edit = 0;
}

/**********************************************************************************************************/
/**********************************************************************************************************/

short g_keyboardPValue = 0;
short g_SerialPValue = 0;
long Power;
long nbpx;

void loop() 
{
int i, updatestatus=0;
//int PosX, PosY, PosBTN;
char customKey = 0, updatedisplay = 0;


  if(Tick100ms)
  {
    updatestatus = 1;
    Tick100ms = 0;
    Counter100ms++;
    if(Counter100ms >=10)
    {
      Tick1s = 1;
      Counter100ms = 0;
    }

    Counter500ms++;
    if(Counter500ms >=5)
    {
      Tick500ms = 1;
      Counter500ms = 0;
    }
  }


  if( updatestatus )
  {
    updatestatus = 0;
    GetInputs();

    // manage keyboard or serial inputs
    if( (g_keyboardPValue == 0) && (g_keyboardPValue != g_keyboardValue) )
    {
      customKey = g_keyboardValue;
    }
    g_keyboardPValue = g_keyboardValue;

    if( (g_SerialPValue == 0) && (g_SerialPValue != g_SerialValue) )
    {
      customKey = g_SerialValue;
    }
    g_SerialPValue = g_SerialValue;
    
    if(customKey)
    {
      switch( customKey )
      {
        case 'A': // F1 // RUN
            if( g_EditMode == 0 )
            {
              if( g_Menu == 1 )  // move Z
              {
                runZMove();    
              }
              if( g_Menu == 2 )  // run test 
              {
                runTest();    
              }
            }  
          break;   
        case 'B': // F2
          break;   
        case '#':
          if( g_EditMode == 0 )
          {
            g_Edit = 1 - g_Edit;
            updatedisplay = 1;  
          }
          break;   
        case '*':
          break;   
          
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
        case '0':
          if( ( g_EditMode ) && ( g_EditCnt<30 ) )
          {
            Tmp[g_EditCnt++] = customKey ;
            Tmp[g_EditCnt] = 0;
            updatedisplay = 1;  
          }
          break;   
          
        case 'R': //RIGHT
          if( g_EditMode == 0 )
          {
            g_Item = 0;
            g_Menu++;
            if(g_Menu > MAX_NB_MENU)
              g_Menu = 0;
            updatedisplay = 1;  
          }
          break;   
        case 'L': //LEFT
          if( g_EditMode == 0 )
          {
            g_Item = 0;
            g_Menu--;
            if(g_Menu<0)
              g_Menu = MAX_NB_MENU;
            updatedisplay = 1;  
          }
          break;   
          
        case 'U': //UP
        
          if( g_EditMode == 0 )
          {
            g_Item --;
            updatedisplay = 1;  
          }
          else
          {
            if( g_EditCnt > 0 )
            {
              g_EditCnt--;
              Tmp[g_EditCnt] = 0;
              updatedisplay = 1;  
            }
          }
          break;   
        case 'D': //DOWN
          if( g_EditMode == 0 )
          {
            g_Item ++;
            updatedisplay = 1;  
          }
          break;   

        case 'E': //ESC
          if( g_EditMode )
          {
            g_EditMode = 0;
            updatedisplay = 1;  
          }
          
          break;   
          
        case 'V': //VAL
          if( g_Edit )
          {
            if( g_EditMode == 0 )
            {
              g_EditMode = 1;
  //            lcd.blink();
              g_EditCnt= 0 ;
              Tmp[g_EditCnt] = 0;
              updatedisplay = 1;  
            }
            else
            {
  //            lcd.noBlink();
              if( g_EditCnt > 0 )
              {
                StoreData();
              }
              g_EditMode = 0;
              updatedisplay = 1;  
            }
          }
          break;   
      }
    }

    // update RBG LED
    if( g_IsSignal )
    {
      // PWM rate : assuming that high is enabled
      Power = (100 * g_PWMHighDuration) / (g_PWMHighDuration + g_PWMLowDuration);
      if( Power > (g_Parameters.MaxPWM+1) )
      {
        UpdatePixels( 8, 128, 0, 0);    
      }
      else
      {
        nbpx = ((8*Power)+1) / g_Parameters.MaxPWM;
        UpdatePixels( nbpx, 0, 32, 0);    
      }
      
    }
    else
    {
      UpdatePixels( 8, 0, 0, 0);  
    }

    
  }
  
  if( Tick500ms )  // automatic actions
  {
    Tick500ms = 0;
    if( g_Menu  ==  0)
      updatedisplay = 1;
  }



  if( updatedisplay )
  {
    updatedisplay = 0;
    switch( g_Menu )
    {
        case 0 :  // Menu 1 : test Menu
          UpdateMonitorMode();
          break;  
        case 1 :  // Menu 0 : Z position and PWM value
          g_Item = UpdateSetupZPosition( g_Item );
          break;  
        case 2 :  // Menu 1 : test Menu
          g_Item = UpdateSetupTest( g_Item );
          break;  
        case 3 :  // Menu 2 : Setup Menu
          g_Item = UpdateSetupConfig( g_Item );
          break;  
    }
  }

  

  if( Tick1s )
  {
    Tick1s = 0;
    if( g_IsSignal )
    {
      g_IsSignal = 0;
      // compute frequecy
      g_Freq = 1000000 / (g_PWMHighDuration + g_PWMLowDuration);

      // PWM rate : assuming that high is enabled
      g_Cycle = (100 * g_PWMHighDuration) / (g_PWMHighDuration + g_PWMLowDuration);
    }
    else
    {
      if(digitalRead(PWM_INPUT_PIN) == LOW)
      {
          g_Freq = -1;
          g_Cycle = 100;
      }
      else
      {
          g_Freq = -1;
          g_Cycle = 0;
      }
    }
/*
    Serial.println(g_PWMHighDuration);
    Serial.println(g_PWMLowDuration);
    if(g_IsSignal)
    {
      Serial.print(";Freq=");
      Serial.print(g_Freq);
      Serial.print("   Cycle=");
      Serial.println(g_Cycle);
    }
*/
  }
 
}



 
/*
void loop()
{
  // when characters arrive over the serial port...
  if (Serial.available()) {
    // wait a bit for the entire message to arrive
    delay(100);
    // clear the screen
    lcd.clear();
    // read all the available characters
    while (Serial.available() > 0) {
      // display each character to the LCD
      lcd.write(Serial.read());
    }
  }
}
*/
