
void setup() {
  Serial.begin(115200);
  DDRB=0x20;   
}

void loop() {
    PORTB=0x20;
    volatile unsigned long x=0;
    for(unsigned long i=0;i<500000;i++){x++;}     
             
    PORTB=0;           
    for(unsigned long i=0;i<500000;i++){x--;}
}
