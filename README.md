# LoggerSoft
GD32E230C8T6 + BME280 + RA-01
# Подготовка среды
## IDE (Keil)
- Установить Keil5 с кряком (например, [отсюда](https://cloud.mail.ru/public/6Mi5/LJMCEYaYY))
- Скачать и установить [пак](https://keilpack.azureedge.net/pack/ARM.CMSIS.5.7.0.pack)
## Пак и конфигуратор
- С [сайта](https://www.gd32mcu.com/en/download/0?kw=GD32E2) загрузить `GD32E23x AddOn` и `GD32 Embedded Builder`
- Установить пак из архива `GD32E23x AddOn`
- Установить `JDK` [этой](https://mirrors.huaweicloud.com/java/jdk/8u152-b16/) версии (`jdk-8u152-windows-x64.exe`)
- Распаковать архив с `GD32 Embedded Builder`

# pin mapping
| PIN | Назначение |
| - | - |
| PA2 | UART0_TX |
| PA3 | UART0_RX |
| PA4 | RA_CS (ERRATA!) |
| PA5 | SPI0_SCK |
| PA6 | SPI0_MISO |
| PA7 | SPI0_MOSI |
| PA8 | BME_INT_CS |
| PA9 | RA_RST |
| PA10 | RA_DIO0 |
| PA11 | RA_DIO1 |
| PA12 | W25Q_CS |
| PA13 | SWDIO |
| PA14 | SWCLK |
| PB1 | ADC_BAT |
| PB13 | SPI1_SCK |
| PB14 | SPI1_MISO |
| PB15 | SPI1_MOSI |
| PC14 | OSC32 |
| PC15 | OSC32 |
| PF0 | OSCIN |
| PF1 | OSCOUT |
| PF6 | I2C1_SCL |
| PF7 | I2C1_SDA |
