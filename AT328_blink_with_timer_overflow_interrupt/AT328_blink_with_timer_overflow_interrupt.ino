/*
1 sec
T/2 = 0.5 sec

base freq = 16 MHz
set prescalar to base freq/1024 = 16000000/1024= 15625

counting 15625 will take 1 sec

*/

#define BASE_FREQ 16000000UL


ISR(TIMER1_OVF_vect){
  PORTB ^= (1<<PORTB5);
  TCNT1 = 65526 - (BASE_FREQ/1024); 
  
} 

void setup() {
  Serial.begin(115200);
  DDRB |= (1<<DDB5); // Set DDB5(Data Direction Bit of Port B) to 1, This makes PortB as output port
  TCNT1 = 65526 - (BASE_FREQ/1024);
  TCCR1B =(1<<CS10) | (1<<CS12);
  TCCR1A = 0x00;
  TIMSK1= (1<<TOIE1);
  sei();
}

void loop() {
  Serial.println(PORTB>0?5:0);
  delay(50);
}
