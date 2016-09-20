#include <Wire.h>
#include "RTClib.h"

RTC_DS1307 rtc_1307;
RTC_Millis rtc_millis;


char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// constants won't change. They're used here to
// set pin numbers:
const int buttonPin = 12;     // the number of the pushbutton pin
const int ledPin =  11;      // the number of the LED pin

const int programButtonPin = 8;     // the number of the pushbutton pin
const int programLedPin =  7;      // the number of the LED pin


const int relay1Pin = 3;
const int relay2_3Pin = 4;
const unsigned long TIMER_PERIOD = 1800000;
unsigned long StartTime = 0;
bool isProgramStart = false;
char BT_data = 0; //Variable for storing received data

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
enum SystemState
{
  Init = 0,
  Irritate_Off,
  Irritate_On,
  Standby,
  Timer_Running,
  Timer_Timeout,
  Program_Start
};

SystemState State = Init;

float calibrationFactor = 4.5;

float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
byte sensorInterrupt = 0;  // 0 = digital pin 2
byte sensorPin       = 2;
unsigned long oldTime;
volatile byte pulseCount;  

void pulseCounter()
{
  // Increment the pulse counter
  pulseCount++;
}


void setup() 
{
  Serial.begin(38400);
   if (! rtc_1307.begin()) 
   {
      Serial.println("Couldn't find RTC");
      while (1);
   }

   if (! rtc_1307.isrunning()) 
   {
      Serial.println("RTC is NOT running!");
      // following line sets the RTC to the date & time this sketch was compiled
      rtc_1307.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  pinMode(programLedPin, OUTPUT);
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2_3Pin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(programButtonPin, INPUT_PULLUP);

  InitFlowRateSensor();
  
}


void loop() {
  switch(State)
  {
    case Init:
    {
      Serial.println("INIT");
      SyncSoftwareRTC();
      CloseValve();
      State = Standby;
      Serial.println("STANDBY");
    } 
    break;
   
   case Program_Start:
   {
    
     isProgramStart = true;

    digitalWrite(programLedPin, LOW);
    delay(500);
    digitalWrite(programLedPin, HIGH);
    delay(500);

    DateTime now = rtc_millis.now();    
    int currentHour = now.hour();
    int currentMinute = now.minute();
    //int modulo = currentMinute % 3;
    if (currentHour == 23 && currentMinute >= 9 && currentMinute < 20) 
    {
      OpenValve();
      State = Timer_Running;
      StartTime = millis();
      //State = Timer_Running;
      Serial.println("Timer_Running");
      PrintCurrentTime(rtc_millis.now());   
    }
   }
   break;
    
    case Standby:
    digitalWrite(ledPin, HIGH);
    digitalWrite(programLedPin, HIGH);
    
    // read the state of the pushbutton value:
    buttonState = digitalRead(buttonPin);
    delay(150);
    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH: 
    if (buttonState == LOW) 
    {
      // turn VALVE on:
      OpenValve();
      State = Timer_Running;
      StartTime = millis();
      Serial.print("StartTime = ");
      Serial.println(StartTime);
      Serial.println("Timer_Running");  
    }

    // read the state of the pushbutton value:
    buttonState = digitalRead(programButtonPin);
    delay(150);
    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH: 
    if (buttonState == LOW) 
    {
      State = Program_Start;
      Serial.println("Program_Start");  
    }  

    if(Serial.available() > 0)  // Send data only when you receive data:
    {
      BT_data = Serial.read();        //Read the  incoming  data and store it into variable data
      Serial.print(BT_data);          //Print Value inside data in Serial monitor
      Serial.print("\n");          //New line
      if(BT_data == '1')              // Checks whether value of data is equal to 1
      {
         // turn VALVE on:
        OpenValve();
        State = Timer_Running;
        StartTime = millis();
        Serial.print("StartTime = ");
        Serial.println(StartTime);
        Serial.println("Timer_Running");  
      }
   }
   break;

    case Timer_Running:
    
    digitalWrite(ledPin, LOW);
    delay(500);
    digitalWrite(ledPin, HIGH);
    delay(500);
    unsigned long CurrentTime = millis();

      //Serial.print("CurrentTime = ");
      //Serial.println(CurrentTime);
      //Serial.print("StartTime = ");
      //Serial.println(StartTime);
   // Serial.println("CurrentTime = %d", CurrentTime);
   // Serial.println("StartTime = %d", StartTime);
    unsigned long Diff = CurrentTime - StartTime;
    //Serial.print("Diff = ");
      //Serial.println(Diff);
//    Serial.println("CurrentTime - StartTime = %d", Diff);
    MeasureFlowRate();
    if (Diff > TIMER_PERIOD)
    {
      Serial.println("Timer_Timeout");
      PrintCurrentTime(rtc_millis.now());   
      CloseValve(); 
      if (isProgramStart)
        State = Program_Start;
      else     
        State = Standby;
    }
    break;


  }
}   
    
/*
    case Program_Start:

    isProgramStart = true;
    
    digitalWrite(programLedPin, LOW);
    delay(500);
    digitalWrite(programLedPin, HIGH);
    delay(500);

    DateTime now = rtc_millis.now();
    int currentHour = now.hour();
    int currentMinute = now.minute();
    if (currentHour > 6 && currentHour < 7 && currentMinute > 0 && currentMinute < 10) 
    {
      State = Timer_Running;
    }
    break;
  */  
  


  void OpenValve()
  {
    Serial.println("Open_Valve");
    digitalWrite(relay1Pin, HIGH);
    digitalWrite(relay2_3Pin, HIGH);
    delay(25);
    digitalWrite(relay1Pin, LOW);
  }
  void CloseValve()
  {
    Serial.println("Close_Valve");
    digitalWrite(relay1Pin, HIGH);
    digitalWrite(relay2_3Pin, LOW);
    delay(25);
    digitalWrite(relay1Pin, LOW);   
  }

  void SyncSoftwareRTC()
  {
    DateTime now = rtc_1307.now();
    PrintCurrentTime(now);
    rtc_millis.begin(now);
    PrintCurrentTime(rtc_millis.now());    
  }

  void PrintCurrentTime(DateTime now)
  {
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    
  }


void InitFlowRateSensor()
{
    pinMode(sensorPin, INPUT);
  digitalWrite(sensorPin, HIGH);

  pulseCount        = 0;
  flowRate          = 0.0;
  flowMilliLitres   = 0;
  totalMilliLitres  = 0;
  oldTime           = 0;

  // The Hall-effect sensor is connected to pin 2 which uses interrupt 0.
  // Configured to trigger on a FALLING state change (transition from HIGH
  // state to LOW state)
  attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
}
void MeasureFlowRate()
{
   if((millis() - oldTime) > 1000)    // Only process counters once per second
  { 
    // Disable the interrupt while calculating flow rate and sending the value to
    // the host
    detachInterrupt(sensorInterrupt);
        
    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    
    // Note the time this processing pass was executed. Note that because we've
    // disabled interrupts the millis() function won't actually be incrementing right
    // at this point, but it will still return the value it was set to just before
    // interrupts went away.
    oldTime = millis();
    
    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;
    
    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
      
    PrintFlowRateMeasure();

    // Reset the pulse counter so we can start incrementing again
    pulseCount = 0;
    
    // Enable the interrupt again now that we've finished sending output
    attachInterrupt(sensorInterrupt, pulseCounter, FALLING);
  }
}
void PrintFlowRateMeasure()
{
  
 unsigned int frac;
    
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(int(flowRate));  // Print the integer part of the variable
    Serial.print(".");             // Print the decimal point
    // Determine the fractional part. The 10 multiplier gives us 1 decimal place.
    frac = (flowRate - int(flowRate)) * 10;
    Serial.print(frac, DEC) ;      // Print the fractional part of the variable
    Serial.print("L/min");
    // Print the number of litres flowed in this second
    Serial.print("  Current Liquid Flowing: ");             // Output separator
    Serial.print(flowMilliLitres);
    Serial.print("mL/Sec");

    // Print the cumulative total of litres flowed since starting
    Serial.print("  Output Liquid Quantity: ");             // Output separator
    Serial.print(totalMilliLitres);
    Serial.println("mL"); 
}
  
