
#define SOUND_SPEED 0.034

const int trigPin = 5;
const int echoPin = 18;
double duration, mm;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);

mm = (duration * SOUND_SPEED/2);

  Serial.println(mm);

  delay(1000);
}
