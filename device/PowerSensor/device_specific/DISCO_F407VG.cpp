uint32_t counter;
__IO uint32_t dmaBuffer[PAIRS];  // DMA reads both ADCs at the same time to one 32b value

void Blink(uint8_t amount) {
  for (uint8_t i = 0; i < amount; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
  }
}

void configureADCCommon() {
  LL_ADC_CommonInitTypeDef ADCCommonConfig;
  LL_ADC_CommonStructInit(&ADCCommonConfig);

  ADCCommonConfig.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADCCommonConfig.Multimode = LL_ADC_MULTI_DUAL_REG_SIMULT;  // regular simultaneous mode
  ADCCommonConfig.MultiDMATransfer = LL_ADC_MULTI_REG_DMA_UNLMT_2; // allow unlimited DMA transfers. MODE2 = half-words by ADC pairs
  ADCCommonConfig.MultiTwoSamplingDelay = LL_ADC_MULTI_TWOSMP_DELAY_5CYCLES; // fastest possible mode

  // Apply settings
  if (LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADCCommonConfig) != SUCCESS) {
    Blink(1);
    exit(1);
  }
}

void configureADC(ADC_TypeDef* adc) {
  LL_ADC_InitTypeDef ADCConfig;
  LL_ADC_StructInit(&ADCConfig);

  ADCConfig.Resolution = LL_ADC_RESOLUTION_10B; // 10-bit ADC resolution
  ADCConfig.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT; // right-align data within 16b register
  ADCConfig.SequencersScanMode = ADC_SCANMODES[PAIRS - 1] == LL_ADC_REG_SEQ_SCAN_DISABLE ? LL_ADC_SEQ_SCAN_DISABLE: LL_ADC_SEQ_SCAN_ENABLE;  // enable scan only if there is more than one rank to convert

  if (LL_ADC_Init(adc, &ADCConfig) != SUCCESS) {
    Blink(2);
    exit(1);
  }
}

void configureADCChannels(ADC_TypeDef* adc, const bool master) {
  LL_ADC_REG_InitTypeDef ADCChannelConfig;
  LL_ADC_REG_StructInit(&ADCChannelConfig);

  ADCChannelConfig.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;   // trigger conversion from software
  ADCChannelConfig.SequencerLength = ADC_SCANMODES[PAIRS - 1];  // number of ranks to convert
  ADCChannelConfig.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE; // not using discontinous mode
  ADCChannelConfig.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS; // enable continuous conversion mode
  ADCChannelConfig.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED; // Allow unlimited transfers to DMA

  if (LL_ADC_REG_Init(adc, &ADCChannelConfig) != SUCCESS) {
    Blink(3);
    exit(1);
  }

  // Set which channels will be converted and in which order, as well as their sampling time
  // ADC1 does even sensors, ADC2 odd sensors
  for (uint8_t i = !master; i < SENSORS; i+=2) {
    LL_ADC_REG_SetSequencerRanks(adc, ADC_RANKS[i/2], ADC_CHANNELS[i]);
    LL_ADC_SetChannelSamplingTime(adc, ADC_CHANNELS[i], LL_ADC_SAMPLINGTIME_144CYCLES);
  }

}

void configureDMA() {
  // Create DMA config struct and fill with defaults
  LL_DMA_InitTypeDef DMAConfig;
  LL_DMA_StructInit(&DMAConfig);

  DMAConfig.PeriphOrM2MSrcAddress =  LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA_MULTI);  // macro handles obtaining common data register for multi-ADC mode
  DMAConfig.MemoryOrM2MDstAddress = (uint32_t) &dmaBuffer[0]; // target is the buffer in RAM
  DMAConfig.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  DMAConfig.Mode = LL_DMA_MODE_CIRCULAR;
  DMAConfig.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  DMAConfig.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  DMAConfig.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_WORD;
  DMAConfig.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_WORD;
  DMAConfig.NbData = PAIRS;
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
  // for timestamp packet, we use the sensor id 0b111 and set the marker bit
  // the host can recognize that this is not a sensor value because the marker bit
  // can only be set for sensor 0
  // so the timestamp packets are
  // 1 111 TTTT, where T are the upper 4 bits of the timestamp
  // 0 1 TTTTTT, where T are the lower 4 bits of the timestamp
  uint16_t t = micros();
  serialData[0] = (0b1111 << 4) | ((t & 0x3C0) >> 6);
  serialData[1] = ((0b01) << 6) | (t & 0x3F);

  for (uint8_t i = 0; i < SENSORS; i++) {
    // even sensors are current values from ADC1
    // odd sensors are voltage values from ADC2
    uint16_t level;
    if (i % 2 == 0) {
      level = __LL_ADC_MULTI_CONV_DATA_MASTER_SLAVE(LL_ADC_MULTI_MASTER, dmaBuffer[i/2]);
    } else {
      level = __LL_ADC_MULTI_CONV_DATA_MASTER_SLAVE(LL_ADC_MULTI_SLAVE, dmaBuffer[i/2]);
    }
#ifndef NODISPLAY
    if (displayEnabled) {
      // store in sensorValues for display purposes
      sensorLevels[i] = level;
    }
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

  // trigger sending data to host if enabled
  if (streamValues | sendSingleValue) {
    sendData = true;
    sendSingleValue = false;
  }

  // clear DMA TC flag
  LL_DMA_ClearFlag_TC0(DMA2);
}
