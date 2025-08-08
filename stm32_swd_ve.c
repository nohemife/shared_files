// STM32F103VET6 (мастер) → прошивает STM32F103CBU6 (цель) через SWD (SWCLK, SWDIO) и принимает бинарный файл по UART (например, от ПК).

// 📐 Подключение:
// PA0 (SWCLK)
// SWCLK
// PA1 (SWDIO)
// SWDIO
// PA2 (NRST)
// NRST (если есть)
// GND
// GND
// USART1_TX (PA9)
// RX (ПК)
// USART1_RX (PA10)
// TX (ПК)

// ✅ Особенности STM32F1:
// Flash size: 512 КБ (VET6), 128 КБ (CBU6).
// Flash page size: 1 КБ.
// Flash base: 0x08000000.
// 🔧 Функции:
// SWD_Init() — инициализация пинов.
// SWD_Connect() — подключение к целевому МК.
// FLASH_Unlock() — разблокировка флеша.
// FLASH_ErasePage(addr) — стирание страницы.
// FLASH_ProgramWord(addr, data) — запись 32-битного слова.
// Приём бинарника по UART (с заголовком: размер, адрес) и прошивка.

#include "main.h"
#include "stm32f1xx_hal.h"

// ==== Пины SWD ====
#define SWD_CLK_PORT    GPIOA
#define SWD_CLK_PIN     GPIO_PIN_0

#define SWD_DIO_PORT    GPIOA
#define SWD_DIO_PIN     GPIO_PIN_1

#define NRST_PORT       GPIOA
#define NRST_PIN        GPIO_PIN_2

// ==== Макросы ====
#define SWD_CLK_HIGH()  HAL_GPIO_WritePin(SWD_CLK_PORT, SWD_CLK_PIN, GPIO_PIN_SET)
#define SWD_CLK_LOW()   HAL_GPIO_WritePin(SWD_CLK_PORT, SWD_CLK_PIN, GPIO_PIN_RESET)

#define SWD_DIO_HIGH()  HAL_GPIO_WritePin(SWD_DIO_PORT, SWD_DIO_PIN, GPIO_PIN_SET)
#define SWD_DIO_LOW()   HAL_GPIO_WritePin(SWD_DIO_PORT, SWD_DIO_PIN, GPIO_PIN_RESET)
#define SWD_DIO_READ()  HAL_GPIO_ReadPin(SWD_DIO_PORT, SWD_DIO_PIN)

#define NRST_LOW()      HAL_GPIO_WritePin(NRST_PORT, NRST_PIN, GPIO_PIN_RESET)
#define NRST_HIGH()     HAL_GPIO_WritePin(NRST_PORT, NRST_PIN, GPIO_PIN_SET)

#define SWD_DELAY()     for(volatile int i = 0; i < 5; i++)

// ==== Настройка DIO ====
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

// ==== Прототипы ====
void SWD_Init(void);
void SWD_WriteBit(uint8_t bit);
uint8_t SWD_ReadBit(void);
void SWD_WriteByte(uint8_t byte);
uint8_t SWD_ReadByte(void);
uint32_t SWD_ReadDP(uint8_t addr);
void SWD_WriteDP(uint8_t addr, uint32_t data);
uint32_t SWD_ReadAP(uint8_t addr);
void SWD_WriteAP(uint8_t addr, uint32_t data);
void SWD_ResetSequence(void);
void SWD_JTAG2SWD(void);
void SWD_Connect(void);
void FLASH_Unlock(void);
void FLASH_Lock(void);
uint32_t FLASH_ErasePage(uint32_t addr);
uint32_t FLASH_ProgramWord(uint32_t addr, uint32_t data);
void FLASH_MassErase(void);
void UART_ReceiveAndProgram(void);

// ==== Глобальные ====
UART_HandleTypeDef huart1;
uint8_t rx_byte;

// ==== Инициализация SWD ====
void SWD_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // SWCLK
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = SWD_CLK_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SWD_CLK_PORT, &gpio);

    // DIO
    SWD_DIO_OUTPUT();
    SWD_CLK_LOW();
    SWD_DIO_HIGH();

    // NRST
    gpio.Pin = NRST_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(NRST_PORT, &gpio);
    NRST_HIGH(); // Выход из сброса
}

// ==== Битовые операции ====
void SWD_WriteBit(uint8_t bit) {
    if (bit) SWD_DIO_HIGH();
    else     SWD_DIO_LOW();
    SWD_DELAY();
    SWD_CLK_HIGH();
    SWD_DELAY();
    SWD_CLK_LOW();
    SWD_DELAY();
}

uint8_t SWD_ReadBit(void) {
    SWD_CLK_HIGH();
    SWD_DELAY();
    uint8_t bit = SWD_DIO_READ();
    SWD_CLK_LOW();
    SWD_DELAY();
    return bit;
}

void SWD_WriteByte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        SWD_WriteBit(byte & 1);
        byte >>= 1;
    }
}

// ==== Команды SWD ====
uint32_t SWD_ReadDP(uint8_t addr) {
    uint8_t cmd = 0x01 | ((addr & 1) << 1) | ((addr & 6) << 1) | (1 << 4);
    int parity = __builtin_popcount(cmd & 0x1F) % 2;
    cmd |= (parity << 6);
    cmd |= 0x80;

    SWD_WriteByte(cmd);
    SWD_WriteBit(0); SWD_WriteBit(1); // turnaround

    uint32_t data = 0;
    for (int i = 0; i < 32; i++) {
        data |= (SWD_ReadBit() << i);
    }
    SWD_ReadBit(); // parity
    SWD_WriteBit(0); SWD_WriteBit(0); // turnaround
    return data;
}

void SWD_WriteDP(uint8_t addr, uint32_t data) {
    uint8_t cmd = 0x01 | ((addr & 1) << 1) | ((addr & 6) << 1);
    int parity = __builtin_popcount(cmd & 0x1F) % 2;
    cmd |= (parity << 6);
    cmd |= 0x80;

    SWD_WriteByte(cmd);
    SWD_WriteBit(0); SWD_WriteBit(1);

    for (int i = 0; i < 32; i++) {
        SWD_WriteBit((data >> i) & 1);
    }
    SWD_WriteBit(0); // parity
    SWD_WriteBit(0); SWD_WriteBit(0);
}

uint32_t SWD_ReadAP(uint8_t addr) {
    SWD_WriteDP(0x04, 0x00000000 | (addr & 0x30)); // SELECT
    SWD_WriteByte(0x01 | ((addr & 1) << 1) | (1 << 4) | ((addr & 6) << 1));
    int parity = __builtin_popcount(SWD_ReadByte() & 0x1F) % 2;
    // (реальная команда выше — упрощённый вариант)
    // На практике: SELECT + READ
    SWD_WriteDP(0x04, (addr & 0xF0)); // SELECT
    SWD_WriteDP(0x00, 0); // Deselect turnaround
    return SWD_ReadDP(0x0C); // RDBUFF
}

void SWD_WriteAP(uint8_t addr, uint32_t data) {
    SWD_WriteDP(0x04, (addr & 0xF0)); // SELECT
    SWD_WriteDP(0x04, data);
}

// ==== Подключение ====
void SWD_ResetSequence(void) {
    for (int i = 0; i < 50; i++) SWD_WriteBit(1);
}

void SWD_JTAG2SWD(void) {
    uint8_t seq[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x9E};
    for (int i = 0; i < 8; i++) SWD_WriteByte(seq[i]);
    SWD_WriteBit(0); SWD_WriteBit(1);
    SWD_WriteBit(0); SWD_WriteBit(0);
}

void SWD_Connect(void) {
    SWD_ResetSequence();
    SWD_JTAG2SWD();

    // Reset and connect
    SWD_WriteDP(0x04, 0x00000000); // CTRL/STAT
    uint32_t id = SWD_ReadDP(0x00); // IDCODE
    if (id == 0x1BA01477 || id == 0x0BC11477) {
        // Подключено
    }
}

// ==== Работа с флешем ====
void FLASH_Unlock(void) {
    // AP 0x00: CSW
    SWD_WriteAP(0x00, 0x23000000); // CSW: 32-bit, PROT, ADDRINC
    // Отправляем команду разблокировки
    SWD_WriteAP(0x04, 0xE000EDF0); // SELECT: FLASH
    SWD_WriteAP(0x04, 0x45670192); // Write KEY1
    SWD_WriteAP(0x04, 0xC56D0192); // Write KEY2
}

void FLASH_Lock(void) {
    SWD_WriteAP(0x04, 0xE000EDF0);
    SWD_WriteAP(0x04, 0x00000001); // LOCK
}

uint32_t FLASH_ErasePage(uint32_t addr) {
    SWD_WriteAP(0x04, 0xE000EDF4); // SELECT: FLASH_CR
    SWD_WriteAP(0x04, 0x00000002); // PG = 0, PER = 1
    SWD_WriteAP(0x04, 0xE000EDF8); // SELECT: FLASH_AR
    SWD_WriteAP(0x04, addr);       // Address
    SWD_WriteAP(0x04, 0xE000EDF4);
    SWD_WriteAP(0x04, 0x00000042); // STRT = 1
    // Ждём завершения (упрощённо)
    HAL_Delay(100);
    return 0;
}

uint32_t FLASH_ProgramWord(uint32_t addr, uint32_t data) {
    SWD_WriteAP(0x04, 0xE000EDF4); // SELECT: FLASH_CR
    SWD_WriteAP(0x04, 0x00000001); // PG = 1

    // Устанавливаем адрес
    SWD_WriteAP(0x04, addr);
    SWD_WriteAP(0x04, data);

    HAL_Delay(10);
    return 0;
}

// ==== Приём по UART и прошивка ====
void UART_ReceiveAndProgram(void) {
    uint32_t addr, size;
    uint8_t buffer[1024];
    uint32_t offset = 0;

    // Ожидаем заголовок: [addr:4][size:4]
    while (HAL_UART_Receive(&huart1, (uint8_t*)&addr, 4, 1000) != HAL_OK);
    while (HAL_UART_Receive(&huart1, (uint8_t*)&size, 4, 1000) != HAL_OK);

    // Стирание
    for (uint32_t a = addr; a < addr + size; a += 1024) {
        FLASH_ErasePage(a);
    }

    // Приём данных
    while (offset < size) {
        uint32_t chunk = (size - offset > 1024) ? 1024 : (size - offset);
        HAL_UART_Receive(&huart1, &buffer[offset], chunk, 1000);
        offset += chunk;
    }

    // Запись
    for (int i = 0; i < size; i += 4) {
        uint32_t word = *(uint32_t*)&buffer[i];
        FLASH_ProgramWord(addr + i, word);
    }

    FLASH_Lock();
}


void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    // Инициализация
    SWD_Init();
    MX_USART1_UART_Init(); // UART1 на 115200

    // Подключиться к целевому МК
    SWD_Connect();

    // Разблокировать флеш
    FLASH_Unlock();

    // Ждём данные
    UART_ReceiveAndProgram();

    while (1) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500);
    }
}


// with open("firmware.bin", "rb") as f:
//     data = f.read()

// addr = 0x08000000
// size = len(data)
// header = addr.to_bytes(4, 'little') + size.to_bytes(4, 'little')

// ser.write(header)
// ser.write(data)
