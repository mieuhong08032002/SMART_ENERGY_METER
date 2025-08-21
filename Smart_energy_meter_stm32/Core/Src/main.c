#include "main.h"
I2C_HandleTypeDef hi2c1;
#define EEPROM_ADDR 0xA4
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
ADE_Name ADE_test;
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
// ADE delay nonblocking
ADE_State ade_state = ADE_STATE_FREQ;
uint32_t ade_last_tick = 0;
const uint32_t ade_delay = 200; // ms
// FREQ
uint32_t period_val;
double freq_real;
float freq_dec;
// VRMS
uint32_t voltage_reg;
double voltage_real;
float voltage_dec;
int vA = 4450; // tham so hieu chinh
int vB = 0;
// IRMS
uint32_t current_reg;
double current_real;
float current_dec;
int iA = 81; // tham so hieu chinh
int iB = 0;
// temp
int8_t temp_val;
float temp_dec;
int tA = 146;
int tB = 837000;
// energy
uint32_t  S_power_reg;
int32_t P_power_reg,Q_power_reg;
float P_power_dec, Q_power_dec, S_power_dec, FP_dec;
float energy;
double old_energy;
uint32_t last_meas, last_wave, last_power, millis_energy;
// uart
uint8_t data_tx[100];
uint8_t keyValue[7];
uint8_t datarx[11];
uint8_t data_r[1];
volatile bool uartDataReceived = false;
// app
uint8_t buff_11[11];
uint16_t id = DEVICE_ID;
float balance = 0;
float newbalance = 0;
float energy_rate = 3000;
int rate_calc_energy = 50;
// thanh ghi
uint8_t scale = 0x00;
uint8_t pga1 = 0x04;
uint8_t pga2 = 0x01;
//
bool debugLever = true;
uint8_t sttRelay = 1;
uint32_t irms_raw;
uint32_t vrms_raw;
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();

  RS485_Init();
  Kalman_Filters_Init();
  ADE_init(&ADE_test, &hspi1, GPIOA, GPIO_PIN_4);
  max7219_Init(1, &hspi2, GPIOA, GPIO_PIN_8);
  max7219_Clean();
  max7219_Decode_Off();
  UART_LOG("Start Aplication V1.0415 \n");
  ledStart();
  int last_time = HAL_GetTick();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
  // Test_ADE7753_Mode_Register(&ADE_test);
   // test_EEPROM_Energy();
  read_initial_settings();
  HAL_Delay(1000);
  while (1)
  {
    // HAL_Delay(10);
    // uint32_t voltage = readRegisters(&ADE_test, 0x17, 3); // VRMS (3 byte)
    // HAL_Delay(10);
    // uint32_t current = readRegisters(&ADE_test, 0x16, 3); // IRMS (3 byte)
    // HAL_Delay(10);
    // write16(&ADE_test, ADE7753_MODE, 0x8C);
    // HAL_Delay(1);
    // uint16_t frequency = readRegisters(&ADE_test, 0x27,2); // FREQ (2 byte)
    // HAL_Delay(10);

    // get_Frequency(&ADE_test);
    // HAL_Delay(10);

    // get_Voltage(&ADE_test);
    // HAL_Delay(10);

    // get_Current(&ADE_test);
    // HAL_Delay(10);

    // get_temp(&ADE_test);
    // HAL_Delay(10);

    // get_Power(&ADE_test);
    // HAL_Delay(10);
    ADE_Task_NonBlocking();
    Update_Energy_And_Balance();
    Check_Payment();
    Control_Relay();
    max7219_PrintFloat(energy / 1000.0f);
    loop_rs485_check();
  }
}
void Update_Energy_And_Balance(void){
  if (energy - old_energy > rate_calc_energy)
    {
                                                 // wh
      balance = balance - energy_rate * (energy - old_energy) / 1000; // tinh tien theo KwH
      newbalance = balance;
      old_energy = energy; 
      EEPROM_Write_Float(&hi2c1, ADD_ENERGY, energy);
    }
}
void Check_Payment(void)
{
  if (newbalance > balance)
  {
    EEPROM_Write_Float(&hi2c1, ADD_BALANCE, newbalance);
    balance = EEPROM_Read_Float(&hi2c1, ADD_BALANCE); // UART_LOG(" PAYMEMT: %0.2f \r\n",balance);
  }
}
void Control_Relay(void)
{
  if (balance <= 0)
  {
    RELAY(OFF);
    balance = 0;
  }
  else
  {
    RELAY(ON & sttRelay);
  }
}
void reset_stm()
{
  // UART_LOG("Lỗi nghiêm trọng, reset STM32...\r\n");
  balance = 0.0f;
  newbalance= balance;
  EEPROM_Write_Float(&hi2c1, ADD_BALANCE, balance);
  EEPROM_Write_Float(&hi2c1, ADD_ENERGY, 0.0f);
  HAL_Delay(10);
  //if (EEPROM_Read_Float(&hi2c1, ADD_BALANCE) == 0.0) NVIC_SystemReset(); // Reset luôn
  // 	__HAL_SPI_DISABLE(&hspi1);
  // 	HAL_Delay(1);  // cho chắc
  // __HAL_SPI_ENABLE(&hspi1);
}
void addBalance(uint32_t amount)
{
  newbalance = balance + amount * 1.0;
}
void setEnergyRate(uint32_t e_rate)
{
  energy_rate = (float)e_rate;
  EEPROM_Write_Float(&hi2c1, ADD_ENERGY_RATE, energy_rate);
}
void uart_log()
{
  sprintf((char *)data_tx, "DIEN AP- V : %0.2f  VRMS %d \n", voltage_dec * 1.0, vrms_raw);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)data_tx, strlen(data_tx), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);

  sprintf((char *)data_tx, "DONG DIEN- I : %0.3f IRMS %d \n", current_dec * 1.0, irms_raw);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)data_tx, strlen(data_tx), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);

  sprintf((char *)data_tx, "CONG SUAT-P : %0.2f  W\n", P_power_dec * 1.0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)data_tx, strlen(data_tx), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);

  sprintf((char *)data_tx, "NĂNG LƯỢNG-A : %0.3f  Wh\n", energy * 1.0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)data_tx, strlen(data_tx), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);

  sprintf((char *)data_tx, "TAN SO- HZ : %0.2f \n", freq_dec);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)data_tx, strlen(data_tx), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);

  sprintf((char *)data_tx, "NHIET DO- °C : %0.2f BALANCE:  %0.2f \n",FP_dec , balance);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)data_tx, strlen(data_tx), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
}
void send11ByteData(uint8_t key, uint32_t value)
{
  buff_11[0] = 0xAA;
  buff_11[1] = DEVICE_ID >> 8;
  buff_11[2] = DEVICE_ID;
  buff_11[3] = key;
  buff_11[4] = (uint8_t)(value >> 24);
  buff_11[5] = (uint8_t)(value >> 16);
  buff_11[6] = (uint8_t)(value >> 8);
  buff_11[7] = (uint8_t)(value >> 0);
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 1; pos < 8; pos++)
  {
    crc ^= (uint16_t)buff_11[pos];

    for (int i = 8; i != 0; i--)
    {
      if ((crc & 0x0001) != 0)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  buff_11[8] = (uint8_t)(crc >> 8);
  buff_11[9] = (uint8_t)(crc);
  buff_11[10] = 0xBB;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
  HAL_UART_Transmit(&huart1, buff_11, 11, 100);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
}
uint16_t crc16_modbus(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < length; pos++)
  {
    crc ^= (uint16_t)data[pos];

    for (int i = 8; i != 0; i--)
    {
      if ((crc & 0x0001) != 0)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}
void loop_rs485_check()
{
  if (rs485_rx_done)
  {
    rs485_rx_done = 0;

    uint16_t id = 0;
    uint8_t key = 0;
    uint32_t value = 0;

    if (rx_buffer[0] == 0xAA && rx_buffer[10] == 0xBB)
    {
      id = (rx_buffer[1] << 8) | rx_buffer[2];
      key = rx_buffer[3];
      value = (rx_buffer[4] << 24) | (rx_buffer[5] << 16) | (rx_buffer[6] << 8) | rx_buffer[7];
      uint16_t crc_received = (rx_buffer[8] << 8) | rx_buffer[9];
      uint16_t crc_calculated = RS485_Calculate_CRC(&rx_buffer[1], 7);

      if (crc_received == crc_calculated)
      {
        RS485_Handle_Command(id, key, value); // xử lý user
      }
      else
      {
       // UART_LOG("CRC Error\n");
      }
    }
    else
    {
      //UART_LOG("Frame format error\n");
    }

    HAL_UART_Receive_IT(&huart1, rx_buffer, FRAME_SIZE);
  }

  // Timeout check
  if ((HAL_GetTick() - last_byte_tick) > 50 && !rs485_rx_done)
  {
    // Timeout, có thể reset nhận lại nếu cần
    HAL_UART_AbortReceive(&huart1);
    HAL_UART_Receive_IT(&huart1, rx_buffer, FRAME_SIZE);
    last_byte_tick = HAL_GetTick();
    //UART_LOG("Timeout → Restart Rx\n");
  }
}
void RS485_Handle_Command(uint16_t id, uint8_t key, uint32_t value)
{
  if (id != DEVICE_ID)
    return;

  switch (key)
  {
  case 0x61: // Nạp tiền
    addBalance(value);
    send11ByteData(0x0F, 0x00); // ok
    break;
  case 0x62:                                  // số dư
    send11ByteData(key, (uint32_t)(balance)); // ok
    break;
  case 0x51:
    setEnergyRate(value);
    send11ByteData(0x0F, 0x00); // ok
    break;
  case 0xA1: // VRMS
    send11ByteData(key, (uint32_t)(voltage_dec * 100));
    break;
  case 0xB1: // IRMS
    send11ByteData(key, (uint32_t)(current_dec * 100));
    break;
  case 0xC1: // Power
    send11ByteData(key, (uint32_t)(P_power_dec * 100));
    break;
  case 0xD1: // ON-OFF
    //RELAY(value);
    sttRelay = value;
    send11ByteData(key, (uint32_t)(value));
    break;
  case 0x91: // F_hz
    send11ByteData(key, (uint32_t)(freq_dec * 100));
    break;
  case 0x81:
    send11ByteData(key, (uint32_t)(energy)); // Wh
    break;
  case 0xC2:
    send11ByteData(key, (uint32_t)(FP_dec * 100)); // %
    break;
  case 0xE0:
    reset_stm();
    send11ByteData(0x0F, 0x00); // ok
    break;
  case 0xF0:
    uart_log();
    break;
  default:
    // HAL_UART_Transmit(&huart1, (uint8_t *)("Unknown Key!"), 13, 100);
    break;
  }
}
void ADE_Task_NonBlocking(void)
{
  if (HAL_GetTick() - ade_last_tick < ade_delay)
    return;

  ade_last_tick = HAL_GetTick(); // cập nhật lại thời gian mỗi bước

  switch (ade_state)
  {
  case ADE_STATE_FREQ:
    get_Frequency(&ADE_test);
    ade_state = ADE_STATE_VOLT;
    break;
  case ADE_STATE_VOLT:
    get_Voltage(&ADE_test);
    ade_state = ADE_STATE_CURR;
    break;
  case ADE_STATE_CURR:
    get_Current(&ADE_test);
    ade_state = ADE_STATE_PWR;
    break;
  case ADE_STATE_PWR:
    get_Power(&ADE_test);
    ade_state = ADE_STATE_FREQ; // quay lại vòng lặp
    break;
  default:
    ade_state = ADE_STATE_FREQ;
    break;
  }
}
void get_Voltage(ADE_Name *test)
{
  write16(test, ADE7753_MODE, 0x8C);
  vrms_raw = getVRMS(test, 1);
  voltage_real = (vrms_raw * vA + vB) / 10000;
  voltage_dec = 0.942 * voltage_real / 1000.0;
  //voltage_dec =  voltage_real / 1000.0;
  voltage_dec = Get_Filtered_Voltage((float)voltage_dec); // filter
}
// void get_Voltage(ADE_Name *ade)
// {
//     write16(ade, ADE7753_MODE, 0x008C);
//      vrms_raw = getVRMS(ade, 1);
//      vrms_raw = vrms_raw -1600;
//     const float vrms_full_scale = 1561400.0f;
//     const float full_scale_voltage = 500.0f;
//     float voltage_real = (vrms_raw * full_scale_voltage) / vrms_full_scale;
//     voltage_dec = Get_Filtered_Voltage(voltage_real);
// }
void get_Frequency(ADE_Name *test)
{
  write16(test, ADE7753_MODE, 0x8C);
  period_val = getPERIOD(test, 1);
  period_val = (period_val << 1);
  freq_dec = 447443.125f / period_val;
}
void get_temp(ADE_Name *test)
{
  write16(test, ADE7753_MODE, 0x8C);
  temp_val = getTEMP(test, 1);
  temp_val = temp_val;
  temp_dec = (temp_val * 1.5f) - 25.0f;
}
void get_Current(ADE_Name *test)
{
  write16(test, ADE7753_MODE, 0x8C);
  current_reg = getIRMS(test, 1);
  current_real = (current_reg * iA + iB) / 1000;
  current_dec = current_real / 1000;
  current_dec = Get_Filtered_Current((float)current_dec); // filter
}
void get_Current1(ADE_Name *ade)
{
    write16(ade, ADE7753_MODE, 0x008C); 
     irms_raw = getIRMS(ade, 1); 
     irms_raw =irms_raw - 600;
    const float full_scale_current = 100.0f; // tương ứng IRMS = 1868467
    const float irms_full_scale = 1868467.0f;
    float current_real = (irms_raw * full_scale_current) / irms_full_scale;
    current_dec = Get_Filtered_Current(current_real);
}
void get_Power(ADE_Name *test)
{ //write16(test, ADE7753_MODE, 0x008C); 
  P_power_reg = readRegisters(test, ADE7753_LAENERGY,3);
  Q_power_reg = read24bits_s(test, ADE7753_LVARENERGY);
  S_power_reg = readRegisters(test, ADE7753_LVAENERGY,3);
  getResetInterruptStatus(test);
  FP_dec = 0.827 * P_power_reg / (S_power_reg); // 0.827 factor from ADE7753 datasheet
  // if (FP_dec > 1)
  // {
  //   FP_dec = 1.00;
  // }
  S_power_dec = voltage_dec * current_dec;
  P_power_dec = voltage_dec * current_dec * FP_dec;
  Q_power_dec = sqrt(S_power_dec * S_power_dec - P_power_dec * P_power_dec);
  // FP_dec = FP_dec * 100.00;
  energy = energy + (P_power_dec * (HAL_GetTick() - millis_energy) / 3600000.0); // wh
  millis_energy = HAL_GetTick();
  HAL_Delay(0);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  ;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 | GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
