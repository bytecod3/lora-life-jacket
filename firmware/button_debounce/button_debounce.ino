#define BUTTON 16
#define DEBOUNCE_DELAY 50

int lastSteadyState = LOW; // previous steady state from the input pin
int lastFlickerableState = LOW; // previous flickerable state from the input pin
int currentState; // the current reading from the input pin

unsigned long lastDebounceTime = 0; // the last time the output pin was toggled

void setup() {
  
  Serial.begin(115200);
  pinMode(BUTTON, INPUT);
}

void loop() { 

  // read the state of the button
  currentState = digitalRead(BUTTON);

  if(currentState != lastFlickerableState) {
    lastDebounceTime = millis();
    lastFlickerableState = currentState;
  }

  if( (millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    // if the button state has changed
    if ( (lastSteadyState == HIGH) && (currentState == LOW) ) {
      
      Serial.println("The button is pressed");

      
    } else if( (lastSteadyState == LOW) && (currentState == HIGH) ) {
      // Serial.println("The button is released");
    }

    // save the last steady state
    lastSteadyState = currentState;
  }

}
