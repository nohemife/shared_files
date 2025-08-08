#include "main.h"
#include "stm32f1xx_hal.h"

// === Настройка пинов SWD ===
#define SWD_CLK_PORT    GPIOA
#define SWD_CLK_PIN     GPIO_PIN_0

#define SWD_DIO_PORT    GPIOA
#define SWD_DIO_PIN     GPIO_PIN_1

// Макросы для работы с пинами
#define SWD_CLK_HIGH()  HAL_GPIO_WritePin(SWD_CLK_PORT, SWD_CLK_PIN, GPIO_PIN_SET)
#define SWD_CLK_LOW()   HAL_GPIO_WritePin(SWD_CLK_PORT, SWD_CLK_PIN, GPIO_PIN_RESET)

#define SWD_DIO_HIGH()  HAL_GPIO_WritePin(SWD_DIO_PORT, SWD_DIO_PIN, GPIO_PIN_SET)
#define SWD_DIO_LOW()   HAL_GPIO_WritePin(SWD_DIO_PORT, SWD_DIO_PIN, GPIO_PIN_RESET)

#define SWD_DIO_READ()  HAL_GPIO_ReadPin(SWD_DIO_PORT, SWD_DIO_PIN)

// Настройка направления DIO (вход/выход)
#define SWD_DIO_OUTPUT() do { \
    GPIO_InitTypeDef gpio = {0}; \
    gpio.Pin = SWD_DIO_PIN; \
    gpio.Mode = GPIO_MODE_OUTPUT_PP; \
    gpio.Pull = GPIO_NOPULL; \
    gpio.Speed = GPIO_SPEED_FREQ_HIGH; \
    HAL_GPIO_Init(SWD_DIO_PORT, &gpio); \
} while(0)

#define SWD_DIO_INPUT() do { \
    GPIO_InitTypeDef gpio = {0}; \
    gpio.Pin = SWD_DIO_PIN; \
    gpio.Mode = GPIO_MODE_INPUT; \
    gpio.Pull = GPIO_NOPULL; \
    HAL_GPIO_Init(SWD_DIO_PORT, &gpio); \
} while(0)

// Задержка (зависит от тактовой частоты)
#define SWD_DELAY()     for(volatile int i = 0; i < 10; i++)

// Прототипы функций SWD
void SWD_Init(void);
void SWD_WriteBit(uint8_t bit);
uint8_t SWD_ReadBit(void);
void SWD_WriteByte(uint8_t byte);
uint8_t SWD_ReadByte(void);
uint32_t SWD_ReadReg(uint8_t addr);
void SWD_WriteReg(uint8_t addr, uint32_t data);
void SWD_ResetSequence(void);
void SWD_JTAG2SWD(void);
void SWD_Connect(void);
void SWD_WriteMemory(uint32_t addr, uint32_t data);
uint32_t SWD_ReadMemory(uint32_t addr);

// Инициализация GPIO для SWD
void SWD_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // SWCLK - выход
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = SWD_CLK_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SWD_CLK_PORT, &gpio);

    // Начально DIO как выход
    SWD_DIO_OUTPUT();
    SWD_CLK_LOW();
    SWD_DIO_HIGH();

    // Сброс целевого МК (опционально)
    // Добавьте управление NRST, если нужно
}

// Запись одного бита
void SWD_WriteBit(uint8_t bit) {
    if (bit) {
        SWD_DIO_HIGH();
    } else {
        SWD_DIO_LOW();
    }
    SWD_DELAY();
    SWD_CLK_HIGH();
    SWD_DELAY();
    SWD_CLK_LOW();
    SWD_DELAY();
}

// Чтение одного бита
uint8_t SWD_ReadBit(void) {
    uint8_t bit;
    SWD_CLK_HIGH();
    SWD_DELAY();
    bit = SWD_DIO_READ();
    SWD_CLK_LOW();
    SWD_DELAY();
    return bit;
}

// Отправка 8 бит (младший бит первым)
void SWD_WriteByte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        SWD_WriteBit(byte & 1);
        byte >>= 1;
    }
}

// Чтение 8 бит
uint8_t SWD_ReadByte(void) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (SWD_ReadBit() << i);
    }
    return byte;
}

// === SWD протокол ===

// Сброс и переход из JTAG в SWD
void SWD_ResetSequence(void) {
    // 50 тактов без данных
    for (int i = 0; i < 50; i++) {
        SWD_WriteBit(1);
    }
}

// Отправка команды перехода JTAG -> SWD
void SWD_JTAG2SWD(void) {
    // Специальная последовательность
    uint8_t sequence[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E};
    for (int i = 0; i < 8; i++) {
        SWD_WriteByte(sequence[i]);
    }
    SWD_WriteByte(0x00); // Turnaround
    SWD_WriteByte(0x00);
}

// Подключение к целевому устройству
void SWD_Connect(void) {
    SWD_ResetSequence();
    SWD_JTAG2SWD();

    // Чтение IDCODE как проверка
    uint32_t idcode = SWD_ReadReg(0x00); // DP_READ, IDCODE
    if (idcode == 0x0BC11477 || idcode == 0x1BA01477) {
        // Успешно подключено
    }
}

// Чтение регистра DP/AP
uint32_t SWD_ReadReg(uint8_t addr) {
    // Формирование команды: [start=1] [APnDP=bit] [RnW=1] [A2,A3] [parity] [stop=0] [park=1]
    uint8_t cmd = 0x00;
    cmd |= 0x01; // start
    cmd |= (addr & 0x01) << 1; // APnDP
    cmd |= 0x04; // RnW
    cmd |= ((addr >> 1) & 0x03) << 2; // A2,A3
    // Parity
    int parity = __builtin_popcount(cmd & 0x0F) % 2;
    cmd |= parity << 6;
    cmd |= 0x80; // Park

    SWD_WriteByte(cmd);

    // Turnaround
    SWD_WriteBit(0); // Turnaround
    SWD_WriteBit(1);

    // Чтение 32 бит + parity
    uint32_t data = 0;
    for (int i = 0; i < 32; i++) {
        data |= (SWD_ReadBit() << i);
    }
    uint8_t parity_rx = SWD_ReadBit();

    // Turnaround
    SWD_WriteBit(0);
    SWD_WriteBit(0);

    // Проверка parity (опционально)
    return data;
}

// Запись в регистр DP/AP
void SWD_WriteReg(uint8_t addr, uint32_t data) {
    uint8_t cmd = 0x00;
    cmd |= 0x01; // start
    cmd |= (addr & 0x01) << 1; // APnDP
    cmd |= 0x00; // WnR
    cmd |= ((addr >> 1) & 0x03) << 2; // A2,A3
    int parity = __builtin_popcount(cmd & 0x0F) % 2;
    cmd |= parity << 6;
    cmd |= 0x80; // Park

    SWD_WriteByte(cmd);

    // Turnaround
    SWD_WriteBit(0);
    SWD_WriteBit(1);

    // Запись данных (32 бита)
    for (int i = 0; i < 32; i++) {
        SWD_WriteBit((data >> i) & 1);
    }

    // Parity (не используется при записи)
    SWD_WriteBit(0); // dummy

    // Turnaround
    SWD_WriteBit(0);
    SWD_WriteBit(0);
}

// === Работа с памятью ===

// Запись 32-битного слова в память
void SWD_WriteMemory(uint32_t addr, uint32_t data) {
    // Включаем MEM-AP
    SWD_WriteReg(0x01, 0x00000000); // SELECT

    // Устанавливаем адрес
    SWD_WriteReg(0x02, addr); // RnW=0, REG=2 (SELECT)

    // Записываем данные
    SWD_WriteReg(0x03, data); // RnW=0, REG=3 (DRW)
}

// Чтение 32-битного слова
uint32_t SWD_ReadMemory(uint32_t addr) {
    SWD_WriteReg(0x01, 0x00000000); // SELECT
    SWD_WriteReg(0x02, addr);       // Set address

    SWD_WriteReg(0x03, 0);          // Dummy write, запускает чтение
    return SWD_ReadReg(0x03);       // Чтение данных
}

// === MAIN ===
int main(void) {
    HAL_Init();
    SystemClock_Config(); // Стандартная конфигурация (72 МГц от PLL)

    SWD_Init();

    // Подключаемся к целевому МК
    SWD_Connect();

    // Пример: чтение IDCODE через Debug Port
    uint32_t idcode = SWD_ReadReg(0x00);
    if (idcode == 0x0BC11477 || idcode == 0x1BA01477) {
        // Подключено успешно
        // Пример: чтение памяти по адресу 0x08000000 (начало флеша)
        uint32_t flash_word = SWD_ReadMemory(0x08000000);
    }

    while (1) {
        // Можно добавить логику программирования флеша
    }
}

// Стандартная функция конфигурации тактирования (для STM32F103)
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
