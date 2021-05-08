#include <Servo.h>


int convertStringToInt(String text);

Servo esc;
Servo servo;


int led = 13;
bool led_on = false;

int throttle = 90;
int steeringAngle = 90;
String command;
String action;

String inString;
bool stringComplete = false;


void setup() {
  esc.attach(9, 1000, 2000);
  delay(500);
  esc.write(90);
  delay(500);

  servo.attach(10);
  Serial.begin(115200);

  pinMode(led, OUTPUT);
}

void loop() {
  
  if (stringComplete){
      toggleLed();
      stringComplete = false;
      command = String(inString);
      
      if (command.length() > 2){
        action = command.substring(0, 3);
        //Serial.println("action: " + action + "|");
        if (action == "mot") {
          throttle = convertStringToInt(command.substring(3));
            if (throttle > 90)
              throttle = min(throttle, 140);
            else
              throttle = max(throttle, 50);
  
            esc.write(throttle);
            Serial.println(inString + " throttle: " + String(throttle));
  
          } else if (action == "ser") {
          steeringAngle = convertStringToInt(command.substring(3));
            if (steeringAngle > 90)
              steeringAngle = min(steeringAngle, 180);
            else
              steeringAngle = max(steeringAngle, 0);
            servo.write(steeringAngle);
            Serial.println(inString + " steeringAngle: " + String(steeringAngle));
          }
      }
        
        inString = String();
        
    }
    
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    inString += inChar;

    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

int convertStringToInt(String text) {
    
  int arr[text.length()];
  int i;
  int convertedCharValue;

 
  for (i = 0 ; i < text.length() ; i++) {
    convertedCharValue = text[i] - '0';
    if (convertedCharValue >= 0 && convertedCharValue < 10)
      arr[i] = convertedCharValue;
    else
      break;
  }
  int finalValue = 0;
  for (int j = 0; j < i; j++) {
    finalValue = finalValue * 10 + arr[j];
  }
  return finalValue;
}



void toggleLed(){
  if(led_on){
    led_on = false;
    digitalWrite(led, LOW);
  }
    
   else{
    led_on = true;
    digitalWrite(led, HIGH);
   }
    
}
