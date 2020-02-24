bool PowerSensor::readLevelFromDevice(unsigned &sensorNumber, unsigned &level)
    {
      // two 8-bit integer buffers, currently to store the whole ADC DR of 32 bits (needs to be downscaled for writing performance);
      uint8_t buffer[2]; 

      // return value storage and bytesRead counter;
      uint8_t returnValue, bytesRead = 0;

      while (true) 
      {
        // read N amount and save in the buffer, N is determined by subtracting amount of bytes it already received from the total buffer size expected;
        if ((returnValue = ::read(fd, (char *) &buffer + bytesRead, sizeof(buffer) - bytesRead)) < 0)
        {
          perror("read device");
          exit(1);
        }
        // if the amount of bytes it received is equal to the amount it expected, also checks if the return value of read is 0;
        else if ((bytesRead += returnValue) == sizeof(buffer)) //if ((bytesRead += returnValue) == sizeof buffer)
        {
          // if the received corresponds to kill signal, return false to terminate the IOthread;
          if (buffer[0] == 0xFF && buffer[1] == 0xE0)
          {
            return false;
          }
          // saves received sensornumber and checks if it is withing bounds;
          else if ((sensorNumber = buffer[0] >> 5) < MAX_SENSORS &&
                    // checks if second byte corresponds with predetermined second byte format;
                    (buffer[1] & 0xE0) == 0xE0)                    
          {
            // extracts the level from the buffers;
            level = ((buffer[0] & 0x1F) << 5) | (buffer[1] & 0x1F);
            return true;
          }
          else
          {
            // if a byte is lost, drop the first byte and try again;
            buffer[0] = buffer[1];
            bytesRead = 1;
          }

          // reconstruct the uint from individual bytes;
          level = buffer[0] << 8 | buffer[1];
          return true;
        }
      }
    }




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



bool PowerSensor::readLevelFromDevice(unsigned &sensorNumber, unsigned &level, unsigned &marker)
    {
      // two 8-bit integer buffers, currently to store the whole ADC DR of 32 bits (needs to be downscaled for writing performance);
      uint8_t buffer[2]; 

      // return value storage and bytesRead counter;
      uint8_t returnValue, bytesRead = 0;

      while (true) 
      {
        // read N amount and save in the buffer, N is determined by subtracting amount of bytes it already received from the total buffer size expected;
        if ((returnValue = ::read(fd, (char *) &buffer + bytesRead, sizeof(buffer) - bytesRead)) < 0)
        {
          perror("read device");
          exit(1);
        }
        // if the amount of bytes it received is equal to the amount it expected, also checks if the return value of read is 0;
        else if ((bytesRead += returnValue) == sizeof(buffer)) //if ((bytesRead += returnValue) == sizeof buffer)
        {
          // if the received corresponds to kill signal, return false to terminate the IOthread;
          if (buffer[0] == 0xFF && buffer[1] == 0xE0)
          {
            return false;
          }
          // checks if first byte corresponds with predetermined first byte format;
          else //if (buffer[0] & 0x80)                    
          {
            countera++;
            // extracts sensor number;
            //sensorNumber = (buffer[0] >> 5) & 0x7;

            // extracts the level from the buffers;
            //level = ((buffer[0] & 0xF) << 6) | (buffer[1] & 0x3F);

            // checks if there is a marker present;
            //marker = (buffer[1] >> 6) & 0x1; 

            return true;
          }
          /*else
          {
            counterb++;
            // if a byte is lost, drop the first byte and try again;
            buffer[0] = buffer[1];
            bytesRead = 1;
          }*/
        } 
      }
    }


      uint8_t buffer[2];
  uint8_t currentSensor = 0;
  uint8_t sendMarkerNext = 0;

  uint16_t level = 0;

  for (uint16_t i = 0; i < 1024; i ++) {
    buffer[0] = ((currentSensor & 0x7) << 4) | ((i & 0x3C0) >> 6) | (1 << 7);// 0x80 | (currentSensor << 4) | (level >> 6));
    buffer[1] = ((sendMarkerNext << 6) | (i & 0x3F)) & ~(1 << 7); //(sendMarkerNext << 6) | (level & 0x3F));

    std::bitset<8> x(buffer[0]);
    std::bitset<8> y(buffer[1]);
    std::cout << i << '\t';
    if ((buffer[0] & 0x80) && ((buffer[1] & 0x80) == 0)) {
      std::cout << 'A' << ' ' << x << '\t' << y << std::endl;
    }
    else
    {
      std::cout << 'B' << ' ' << x << '\t' << y << std::endl;
    }
    
  }