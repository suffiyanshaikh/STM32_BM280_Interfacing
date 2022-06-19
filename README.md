# STM32L476_BMP280_Interfacing

### Overview
The BMP280 is an absolute barometric pressure sensor which can be interface with micro-controller using SPI or I2C.The project shows how to read temperature 
from BMP280 using I2C communication.STM32L476 is used as a Host controller.

### Hardware details

* BMP280 Sensor Module
* Nucleo-L476RG Board

 [BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf).
 
 [Nucleo-L476RG datasheet](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf).
 
### Hardware Configuration

**Connections**

|BMP280 Pins |STM32L476 Pins|
|:-----------|:-------------:|
|CSB         | 3.3V/VCC      |
|SDI         | SDA           |
|SCK         | SCK           |
|SDO         | Gnd/0V        |
|VCC         | 3.3V          |
|GND         |Gnd/0V


**Cube MX View **
![image](https://user-images.githubusercontent.com/42150715/174488153-aff00b97-0920-4bbb-8f74-0b58f25d800a.png)

### How to Use the project
* Clone the repository in your local system.
* Open the project with STM32cube IDE Build and flash the project in your board.
* Run any serial monitor to check desired the output.

**Example Output**
![image](https://user-images.githubusercontent.com/42150715/174488599-ad96e501-d43e-4a83-b7e7-5dade6448947.png)






