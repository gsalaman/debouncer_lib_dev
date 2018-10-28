// SoftwareSerial is used to communicate with the XBee

#include <SoftwareSerial.h>

#define BUTTON_PIN 10

/******** HEADER ******/
#include <Arduino.h>

typedef enum
{
  PIN_HIGH,
  PIN_LOW, 
  SWITCHING_LOW,
  SWITCHING_HIGH
} _debounceStateType;

class Debouncer
{
  public:
    Debouncer(int pin);
    Debouncer(int pin, int debounceTime);
    void debug(bool on);
    int read( void );
  private:
    bool               _debug;
    _debounceStateType _state;  
    int                _debounceTime;
    unsigned long      _transitionStart;
    int                _consecutiveReads;
    int                _pin;
    void               _init(int pin, int debounceTime);
};

/********** cpp *********/
#define DEFAULT_DEBOUNCE_TIME 50  // in ms;
void Debouncer::_init(int pin, int debounceTime)
{
  int currentPinVal;
  
  _pin = pin;
  _debounceTime = debounceTime;
  _consecutiveReads = 0;
  _debug = false;

  currentPinVal = digitalRead(_pin);
  
  if (currentPinVal == HIGH)
  {
    _state = PIN_HIGH;
  }
  else
  {
    _state = PIN_LOW;
  }
}

Debouncer::Debouncer(int pin)
{
  _init(pin, DEFAULT_DEBOUNCE_TIME);
}

Debouncer::Debouncer(int pin, int debounceTime)
{
  _init(pin, debounceTime);
}

void Debouncer::debug(bool on)
{
  _debug = on;
}

int Debouncer::read( void )
{
  unsigned long currentTime;
  int currentRead;
  int filteredRead=-1;

  currentRead = digitalRead(_pin);

  switch (_state)
  {
    case PIN_HIGH:
      if (currentRead == LOW)
      {
        // this is a transition from high to low.  we want to start marking time.
        // when the pin is low for long enough, then we'll actually return a LOW.
        _transitionStart = millis();
        _state = SWITCHING_LOW;
        _consecutiveReads = 1;

        if (_debug)
        {
          Serial.print("DEBOUNCE PIN ");
          Serial.print(_pin);
          Serial.print(" HIGH->LOW: ");
          Serial.println(_transitionStart);
        }
      }

      // irrespective of whether the pin is *currently* HIGH or LOW, we still want to return it
      // in the HIGH state. 
      filteredRead = HIGH;
    break;

    case PIN_LOW:
      // If our pin was currently low, and we read HIGH, start marking time.
      // when the pin is HIGH for long enough, then we'll actually return HIGH
      if (currentRead == HIGH)
      {
        _transitionStart = millis();
        _state = SWITCHING_HIGH;
        _consecutiveReads = 1;

        if (_debug)
        {
          Serial.print("DEBOUNCE PIN ");
          Serial.print(_pin);
          Serial.print(" LOW->HIGH: ");
          Serial.println(_transitionStart);
        }
      }

      // irrespective of whether the pin is *currently* high or low, we still want to return
      // it in the LOW state.
      filteredRead = LOW;
    break;

    case SWITCHING_LOW:
      // Okay, so we've previously detected a transition from HIGH to LOW.  
      // We want to make sure the pin stays in the low state for long enough to ACTUALLY return a LOW.  
      if (currentRead == LOW)
      {
        // So far, so good.  Last read was LOW, this one is too.  Have we been LOW for long enough?
        currentTime = millis();
        _consecutiveReads++;
        if (currentTime > _transitionStart + _debounceTime)
        {
          filteredRead = LOW;
          _state = PIN_LOW;

          if (_debug)
          {
            Serial.print("DEBOUNCED PIN ");
            Serial.print(_pin);
            Serial.print(" to LOW.  Reads: ");
            Serial.println(_consecutiveReads);
          }
        }
        else
        { 
          // stay in this state a while longer...
          filteredRead = HIGH;
        }
      }
      else
      {
        // so we were going to LOW, but then the pin bounced back to HIGH before we stayed long enough.  
        // hello, bounce.  Go back to the straight "HIGH" state.
        if (_debug)
        {
          Serial.print("*** DEBOUNCE PIN ");
          Serial.print(_pin);
          Serial.print(" bounced LOW->HIGH.  Reads: ");
          Serial.print(_consecutiveReads);
          Serial.println("  Staying HIGH");
        }

        filteredRead = HIGH;
        _state = PIN_HIGH;
      }
    break;

    case SWITCHING_HIGH:
      // Okay, so we've previously detected a transition from low to high.  
      // We want to make sure the pin stays in the high state for long enough to ACTUALLY return HIGH.  
      if (currentRead == HIGH)
      {
        // So far, so good.  Last read was high, this one is too.  Have we been HIGH for long enough?
        //                                      (insert Grateful Dead joke here ^^^^)
        currentTime = millis();
        _consecutiveReads++;
        if (currentTime > _transitionStart + _debounceTime)
        {
          filteredRead = HIGH;
          _state = PIN_HIGH;
          
          if (_debug)
          {
            Serial.print("DEBOUNCED PIN ");
            Serial.print(_pin);
            Serial.print(" to HIGH.  Reads: ");
            Serial.println(_consecutiveReads);
          }
        }
        else
        { 
          // stay in this state a while longer...
          filteredRead = LOW;
        }
      }
      else
      {
        // so we were going to HIGH, but then the pin switched back to low before we stayed long enough.  
        // hello, bounce.  Go back to the straight "LOW" state.
        if (_debug)
        {
          Serial.print("*** DEBOUNCE PIN ");
          Serial.print(_pin);
          Serial.print(" bounced HIGH->LOW.  Reads: ");
          Serial.print(_consecutiveReads);
          Serial.println("  Staying LOW");
        }

        filteredRead = 0;
        _state = PIN_LOW;
      }
    break;
  }

  if (filteredRead == -1) Serial.println("YOU MISSED A CLAUSE IN DEBOUNCING!!!");
  return filteredRead;
  
}  // end of read

/***********  CLIENT CODE HERE *****************/
Debouncer debounceButton(BUTTON_PIN);

/*=====================================================================
 * Function: setup
 */
void setup() 
{  
  Serial.begin(9600); // Start serial communication
  pinMode(BUTTON_PIN, INPUT_PULLUP); 

  debounceButton.debug(true);
  
  Serial.println("Debounce framework initialized");
  
} // end of setup



/*=====================================================================
 * Function: loop
 */
void loop() 
{
  static int last_button_state = HIGH;
  int current_button_state;
  static int presses=0;
  
  //current_button_state = digitalRead(BUTTON_PIN);
  current_button_state = debounceButton.read();

  if (current_button_state != last_button_state)
  {
    presses++;
    Serial.print("Button now ");
    Serial.print(current_button_state);
    Serial.print(" presses: ");
    Serial.println(presses);

    last_button_state = current_button_state;
  }

  
          
}  // end of loop
