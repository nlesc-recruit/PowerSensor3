void Blink(uint8_t amount) {
  // Blink LED, note that outputs are inverted: LOW is on, HIGH is off
  for (uint8_t i = 0; i < amount; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
  }
}

void configureADCCommon() {
  LL_ADC_CommonInitTypeDef ADCCommonConfig;
  LL_ADC_CommonStructInit(&ADCCommonConfig);

  ADCCommonConfig.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;

  // Apply settings
  if (LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADCCommonConfig) != SUCCESS) {
    Blink(1);
    exit(1);
  }
}

void configureADC() {
  LL_ADC_InitTypeDef ADCConfig;
  LL_ADC_StructInit(&ADCConfig);

  ADCConfig.Resolution = LL_ADC_RESOLUTION_10B;  // 10-bit ADC resolution
  ADCConfig.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;  // right-align data within 16b register
  // enable scan only if there is more than one rank to convert
  ADCConfig.SequencersScanMode = ADC_SCANMODES[SENSORS - 1] ==
    LL_ADC_REG_SEQ_SCAN_DISABLE ? LL_ADC_SEQ_SCAN_DISABLE: LL_ADC_SEQ_SCAN_ENABLE;

  if (LL_ADC_Init(ADC1, &ADCConfig) != SUCCESS) {
    Blink(2);
    exit(1);
  }
}

void configureADCChannels() {
  LL_ADC_REG_InitTypeDef ADCChannelConfig;
  LL_ADC_REG_StructInit(&ADCChannelConfig);

  ADCChannelConfig.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;   // trigger conversion from software
  ADCChannelConfig.SequencerLength = ADC_SCANMODES[SENSORS - 1];  // number of ranks to convert
  ADCChannelConfig.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;  // not using discontinous mode
  ADCChannelConfig.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;  // enable continuous conversion mode
  ADCChannelConfig.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;  // Allow unlimited transfers to DMA

  if (LL_ADC_REG_Init(ADC1, &ADCChannelConfig) != SUCCESS) {
    Blink(3);
    exit(1);
  }

  // Set which channels will be converted and in which order, as well as their sampling time
  for (uint8_t i = 0; i < SENSORS; i++) {
    LL_ADC_REG_SetSequencerRanks(ADC1, ADC_RANKS[i], ADC_CHANNELS[i]);
    LL_ADC_SetChannelSamplingTime(ADC1, ADC_CHANNELS[i], LL_ADC_SAMPLINGTIME_15CYCLES);
  }
}

void configureDMA() {
  // Create DMA config struct and fill with defaults
  LL_DMA_InitTypeDef DMAConfig;
  LL_DMA_StructInit(&DMAConfig);

  DMAConfig.PeriphOrM2MSrcAddress =  LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA);
  DMAConfig.MemoryOrM2MDstAddress = (uint32_t) &dmaBuffer;  // target is the buffer in RAM
  DMAConfig.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMAConfig.Mode = LL_DMA_MODE_CIRCULAR;
  DMAConfig.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMAConfig.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMAConfig.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
  DMAConfig.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
  DMAConfig.NbData = SENSORS;
  DMAConfig.Channel = LL_DMA_CHANNEL_0;

  // write config
  if (LL_DMA_Init(DMA2, LL_DMA_STREAM_0, &DMAConfig) != SUCCESS) {
    Blink(5);
    exit(1);
  }

  // enable interrupt on transfer-complete
  LL_DMA_EnableIT_TC(DMA2, LL_DMA_STREAM_0);

  // enable the DMA stream
  LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_0);
}

extern "C" void DMA2_Stream0_IRQHandler() {
  static uint16_t t = 0;
  // loop over sensors and store each value at current location in averaging buffer
  for (uint8_t i = 0; i < SENSORS; i++) {
    avgBuffer[i][currentSample] = dmaBuffer[i];
  }
  currentSample++;

  if (currentSample == numSampleToAverage / 2) {
    // if we are halfway, store the timestamp
    t = micros();
  } else if (currentSample >= numSampleToAverage) {
    /* if the buffer is full, process the values
     * for timestamp packet, we use the sensor id 0b111 and set the marker bit
     * the host can recognize that this is not a sensor value because the marker bit
     * can only be set for sensor 0
     * so the timestamp packets are
     * 1 111 TTTT, where T are the upper 4 bits of the timestamp
     * 0 1 TTTTTT, where T are the lower 4 bits of the timestamp
     */
    uint16_t t = micros();
    serialData[0] = (0b1111 << 4) | ((t & 0x3C0) >> 6);
    serialData[1] = ((0b01) << 6) | (t & 0x3F);

    // process the sensor values
    for (uint8_t i = 0; i < SENSORS; i++) {
      // calculate average level of current sensor
      uint16_t level = 0.;
      for (uint8_t j = 0; j < numSampleToAverage; j++) {
        level += avgBuffer[i][j];
      }
      level /= numSampleToAverage;
#ifndef NODISPLAY
      // store in sensorValues for display purposes
      sensorLevels[i] = level;
#endif
      // add metadata to remaining bits: 2 bytes available with 10b sensor value
      // First byte: 1 iii aaaa
      // where iii is the sensor id, a are the upper 4 bits of the level
      serialData[2*i + 2] = ((i & 0x7) << 4) | ((level & 0x3C0) >> 6) | (1 << 7);
      // Second byte: 0 m bbbbbb
      // where m is the marker bit, b are the lower 6 bits of the level
      serialData[2*i + 3] = ((sendMarkerNext << 6) | (level & 0x3F)) & ~(1 << 7);

      counter++;
      sendMarkerNext = false;
    }
    // finally reset the sampling counter and trigger the sending of data in the main loop
    currentSample = 0;
    if (streamValues | sendSingleValue) {
      sendData = true;
      sendSingleValue = false;
    }
  }

  // clear DMA TC flag
  LL_DMA_ClearFlag_TC0(DMA2);
}
