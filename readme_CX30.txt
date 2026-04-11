IAR Workbench and PlatformIO STM8 C lang Quadrocopter project
Use STM8S_StdPeriph_Lib v2.3.1
Controller STM8S005K6T6C
HSI f_cpu=16MHz
Pin configuration:
PA1 - LED control. Active level High. After init flashes (period 300ms) while payload from remote control (BK2425) will be recived. After it is on alwayse
PB2 - Analog input ADC. Battery voltage from divider with ratio 42/(42+49)
PB4 - I2C_SCL connected to MPU6052
PB5 - I2C_SDA connected to MPU6052
PC1 - PWM out for Right front motor (16KHz)
PC2 - PWM out for Right back motor (16KHz)
PC3 - LED control. Active level High. After init it is off. When payload (BK2425) is recived it turns on. When more than 10 requests have no payload than it starts flash.
PC5 - SPI_SCK connected to BK2425
PC5 - SPI_MOSI connected to BK2425
PC5 - SPI_MOSO connected to BK2425
PE5 - SPI_SS connected to BK2425
PD0 - LED control. Active high level. Battery voltage > 3.2V - LED On. Battery voltage <= 3.2V - LED Flashes with period 500ms. Range 3.0V to 4.2V
PD2 - PWM out for Left back motor (16KHz)
PD3 - PWM out for Left front motor (16KHz)

SPI
STM8 is master
clock 4MHz
File with init sequence of chip BK2425: BK_Init.txt
Data from remote control:
SPI_CMD 0x61 (Read RX payload) 8 bytes:
0x80 default value. Left stick Y position (0x0 - 0xFF) - Vertial speed control
0x80 default value. Left stick X position (0x0 - 0xFF) - Afterburn control
0x40 default value. Left bottom rocker. Left button click -1, Right click +1 (0x0E - 0x72), Long Beep when cross 0x40. Value to adjust quadcopter pitch orientation
0x80 default value. Right stick Y position (0x60 - 0xA0). Quadrocoper pitch control
0x80 default value. Right stick X position (0x60 - 0xA0). Quadrocoper roll control
0x40 default value. Reseved
0x40 default value. Right bottom rocker. Left button click -1, Right click +1 (0x0E - 0x72), Long Beep when cross 0x40. Value to adjust quadcopter roll orientation
0x00 default value. 0x40 - Left upper 2nd button
Pool rate: 330Hz when in Flight mode (payload recived) and 20Hz after init and if there is no payload more than 10 requests

I2C
clock 200KHz
File with init sequence of chip MPU6052: MPU_Init.txt
Request data from MPU:
1. 3 axis of Accel
2. Temperatue
3. 3 axis of Gyro
Pool rate: 330Hz when in Flight mode (payload recived) and after init no Pool and if there is no payload from BK2425 more than 10 requests than stop MPU pool

Motors.
PWM frequency 16kHz
Normal flight duty cycle from 20% - 85%
Afterburn duty cycle from 85% - 95%

Use fixedpoint arithmetic.
Sliding mode control loops.
1. Pitch control. Right stick X controls pitch angel (maximum angels +-30 deg).
2. Roll control. Right stick Y controls roll angel (maximum angels +-30 deg).
When stick is in dead zone +-3 from default value than target Roll = 0 and target Pitch = 0

Sliding mode vertical speed control loop.
Left stick uses to control it. Dead zone +-10 from default value.
When Y pos in dead zone than keep vertival speed equal 0 (keep altitude)

Calculate state of quadrocopter:
Roll, Pitch, Yaw - angels
Wroll, Wpitch, Wyaw - angular speed
x, y, z - center mass position (initial condition (0, 0, 0) takeoff point)
Vx, Vy, Vz - center mass speed
Ax, Ay, Az - center mass acceleration
