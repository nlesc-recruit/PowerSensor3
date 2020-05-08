uint16_t challa= 0xAAFF;
uint8_t grappie1;
uint8_t grappie2;
uint16_t *pointer;

void setup() {
  Serial.begin(9600);
  
  pointer = &challa;

  memcpy(&grappie1, &pointer, sizeof(uint8_t));
  memcpy(&grappie2, &pointer + sizeof(grappie1), sizeof(grappie2)); 
}


void loop() {
  Serial.println(grappie1, BIN);
  Serial.println(grappie2, BIN);
  delay(1000);
}
