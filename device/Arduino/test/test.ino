struct Sensor {
  float volt;
  float type;
  float level;
};

Sensor sensor;

const uint16_t SIZE_16B = sizeof(sensor)/2;

uint16_t eeprom[SIZE_16B];

void setup() {
  Serial.begin(9600);

  sensor.volt = 3.3;
  sensor.type = .185;
  sensor.level = 3.3455;

  uint16_t *p_data = (uint16_t *)&sensor;
  uint16_t *p_eeprom = (uint16_t *)eeprom;

  for (int i = 0; i < SIZE_16B; i++) {
    memcpy(p_eeprom, p_data, sizeof(uint16_t));
    p_data++;
    p_eeprom++;
  } 
}


void loop() {
  Serial.println("Copied: ");
  Serial.println(sensor.volt, BIN);
  Serial.println(sensor.type, BIN);
  Serial.println(sensor.level, BIN);
  Serial.println("To: ");
  for (int i = 0; i < SIZE_16B; i++) {
    Serial.println(eeprom[i], BIN);
  }

  delay(1000);
}
