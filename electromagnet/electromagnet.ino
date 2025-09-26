// Define the analog pin the thermistor is connected to
const int thermistorPin = A13; 
const float SERIES_RESISTOR = 4700.0;
const float VCC = 5.0;

const float NOMINAL_RESISTANCE = 100000.0; // Resistance at 25°C
const float NOMINAL_TEMPERATURE = 25.0;   // Nominal temperature in Celsius
const float B_PARAMETER = 3950;         // Beta parameter

const float target_temp = 50.00;

int K = 10;

void setup() {
  Serial.begin(9600); 
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  //pinMode(40, OUTPUT);
  
 
  TCCR2A = 0;
  TCCR2B = 0;


  TCCR2A |= (1 << WGM21) | (1 << WGM20);
  TCCR2A |= (1 << COM2A1);
  //OCR2A = 100;


  TCCR2A |= (1 << COM2B1);
  //OCR2B = 255;

  TCCR2B |= (1 << CS20);
}
void heat(float temp){
  float diff = target_temp - temp;
  if(diff<0) 
    OCR2B = 255;
  else
    OCR2B = 0;
  float deno = 1 + (55 * exp(-0.1 * diff));
  int val = 255/deno;
  val = (val+K) < 255 ? (val + K) : val;
  OCR2A = val;
  Serial.print("REG: ");
  Serial.println(val);
}

void loop() {
  double analog_val = analogRead(thermistorPin); 
  float v_out = (analog_val * VCC) / 1023.0;
  float thermistor_resistance = (SERIES_RESISTOR * v_out) / (VCC - v_out);
  float temp = 1.0 / ( (1.0 / (NOMINAL_TEMPERATURE + 273.15)) + (log(thermistor_resistance / NOMINAL_RESISTANCE) / B_PARAMETER) );
  temp = temp - 273.15;
  Serial.println(temp ,DEC); 
  heat(temp);

  delay(1000); 
}


