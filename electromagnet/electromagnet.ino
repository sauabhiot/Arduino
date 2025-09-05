// Define the analog pin the thermistor is connected to
const int thermistorPin = A13; 
const float SERIES_RESISTOR = 4700.0;
const float VCC = 5.0;

const float NOMINAL_RESISTANCE = 100000.0; // Resistance at 25°C
const float NOMINAL_TEMPERATURE = 25.0;   // Nominal temperature in Celsius
const float B_PARAMETER = 3977.71;         // Beta parameter

void setup() {
  Serial.begin(9600); 
  pinMode(10, OUTPUT);
  
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  TCCR2A |= (1 << WGM21) | (1 << WGM20);
  TCCR2B |= (1 << WGM22);

  TCCR2A |= (1 << COM2B1);

  OCR2A = 199;

  TCCR2B |= (1 << CS20);

  OCR2B = OCR2A/2;

  sei();
  interrupts();
  TIMSK2=  (1<<OCIE2A);
}

void loop() {
  double analog_val = analogRead(thermistorPin); 
  float v_out = (analog_val * VCC) / 1023.0;
  float thermistor_resistance = (SERIES_RESISTOR * v_out) / (VCC - v_out);
  Serial.print("Therm_res: ");
  Serial.println(thermistor_resistance ,DEC); 
  float temp_kelvin = 1.0 / ( (1.0 / (NOMINAL_TEMPERATURE + 273.15)) + (log(thermistor_resistance / NOMINAL_RESISTANCE) / B_PARAMETER) );
  Serial.println(temp_kelvin-273.15 ,DEC); 
  delay(1000); 
  for (int i = 0; i <= OCR2A; i++) {
    OCR2B = i;
    delay(10);
  }
  analogWrite(10, 100);
}


