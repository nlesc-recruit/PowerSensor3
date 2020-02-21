




uint8_t sensor = 4;
  uint16_t level = 314;
  uint8_t marker = 1;

  std::cout << "Sending: sensor no: " << (uint16_t) sensor << "\t level: " << level << "\t marker: " << (uint16_t) marker << std::endl;


  uint8_t bytes[2];

  // sending
  uint8_t firstByte = (1 << 7) | (sensor << 4) | (level >> 6);
  uint8_t secondByte = (0 << 7) | (marker << 6) | (level & 0x3F);

  bytes[0] = firstByte;
  bytes[1] = secondByte;

  std::bitset<8> x(firstByte);
  std::bitset<8> y(secondByte);

  std::cout << x << '\t' << y << std::endl;

  /*
  1. lees n bytes uit de serial buffer.
  2. check de eerste byte of het een begin byte is.
  2a. eerste byte is eerste byte, correct -> 3.
  2b. eerste byte is geen eerste byte, incorrect. n++ -> 1.
  3. extract info. 
  */


  for (int i = 0; i < 2; i++) {
    if(bytes[0] & 0x80) {
      
    } else {

    }
  }

  // receiving
  sensor = (firstByte >> 4) & 0x7;
  level = ((firstByte & 0xF) << 6) | (secondByte & 0x3F);
  marker = ((secondByte >> 6) & 0x1);




  std::cout << "Receiving: sensor no: " << (uint16_t) sensor << "\t level: " << level << "\t marker: " << (uint16_t) marker << std::endl;



