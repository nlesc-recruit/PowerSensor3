#define MAX_SENSORS 5

struct EEPROM {
  struct Sensor {
    double volt;
    double type;
    double level;
  }sensor[MAX_SENSORS];
};

EEPROM recv;

const uint16_t SIZE_16B = sizeof(recv)/2;

uint16_t eeprom[SIZE_16B + 1];


void setup() {
  Serial.begin(9600);

  for (int i = 0; i < MAX_SENSORS; i++) {
  recv.sensor[i].volt = 3.3;
  recv.sensor[i].type = .185;
  recv.sensor[i].level = 3.3455;
  }  

  uint16_t *p_data = (uint16_t *)&recv.sensor;
  uint16_t *p_eeprom = (uint16_t *)eeprom;

  memcpy(p_eeprom, &SIZE_16B, sizeof(uint16_t));
  p_eeprom++; 

  for (int i = 0; i < SIZE_16B; i++) {
    memcpy(p_eeprom, p_data, sizeof(uint16_t));
    p_data++;
    p_eeprom++;
  }

  recv.sensor[0].volt = 0;
  recv.sensor[0].type = 0;
  recv.sensor[0].level = 0;
  
  p_data = (uint16_t *)&recv.sensor;
  p_eeprom = (uint16_t *)eeprom;

  uint16_t size;

  memcpy(&size, p_eeprom, sizeof(uint16_t));
  p_eeprom++;

  for (int i = 0; i < size; i++) {
    memcpy(p_data, p_eeprom, sizeof(uint16_t));
    p_data++;
    p_eeprom++;
  }

 
}


void loop() {
  Serial.println("Copied: ");
  for (int i = 0; i < MAX_SENSORS; i++) {
  Serial.println(recv.sensor[i].volt);
  if (recv.sensor[i].type == .185){
  Serial.println(recv.sensor[i].type);}
  else {Serial.println(0);}
  Serial.println(recv.sensor[i].level);
  }

  delay(1000);
}
