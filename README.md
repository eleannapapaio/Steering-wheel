### Introduction
This project's aim is to program a steering-wheel, so when the user turns it, the model of the CubeSat will also turn. It was used to demonstrate the ADCS subsystem of the satellite in an EXPO. The nRF24L01 module that we insert in the steering wheel communicates with the same module in the CubeSat model, so the motor driver in the model, makes it move in the correct direction.
In order to make the model of the CubeSat turn around with the use of a steering wheel, we need a motor driver (Dual Motor Driver Module L298N), two Arduino, two nRF24L01 Transceiver Modules, an MPU6050, a power source, a reaction wheel and a steering wheel. 
### Testing
I conducted tests to assure that the driver and the MPU6050 are working fine. 
##### Driver Testing 
To test the driver I used [testing-driver](https://github.com/eleannapapaio/Steering-wheel/blob/main/testing_driver.ino) . If you want to test the driver you should use an Arduino, a motor driver, a reaction wheel and a power source. The wiring is as following:
- +12V with the positive terminal of the power supply
- common ground (GND driver with GND arduino) with the negative terminal
- ENA with pin 9
- IN1 with pin 8
- IN2 with pin 7
- OUT1 with reaction wheel
- OUT2 with reaction wheel

Assign 9V on power supply and 2A over voltage

![image](https://github.com/user-attachments/assets/8bc30a0b-ece1-42ea-b522-9cb390eeeb02)

The driver works properly only if the reaction wheel turns on the on side, stops for a second and continue turning on the other side. 

##### MPU6050 Testing 
In order to test that the MPU6050 (accelerometer/magnetometer/gyroscope) is working properly I used [test_MPU6050](https://github.com/eleannapapaio/Steering-wheel/blob/main/test_MPU9250.ino) . The wiring is as following:
- Connect VCC to a 5V output pin on the Arduino.
- Connect GND to a ground pin on the Arduino.
- Connect SDA to the Arduino's analog pin 4 (A4).
- Connect SCL to the Arduino's analog pin 5 (A5).

![image](https://github.com/user-attachments/assets/46a04a84-ffe9-4c2f-8150-4d204e29c8e2)


To check whether the MPU6050 is functioning properly I moved the component towards all axes (x-y-z). While moving the MPU6050, all the values in all three axes should change at the serial monitor. 
If you use the script as it is, the only data that will be printed at your serial monitor, will be the magnetometer's values on x,y and z axes. 
```
 Serial.print("\tmagX: " + String(mX));
    Serial.print("\tmaxY: " + String(mY));
    Serial.print("\tmagZ: " + String(mZ));
```
In order to check that the accelerometer and the gyroscope is also working on all three axes, you should change at the script the `mX`, `my` , `mz` and write `aX`, `aY` , `aZ` and afterwards `gX`, `gY` , `gZ`. When you apply these changes, move again the MPU6050 towards all axes and check that the values are changing at the serial monitor. If all the values in all axes are continuesly changing, it means that the MPU6050 is ready for use. 
You might see that the magnetometer and the gyroscope for example are working well, but the accelerometer keeps printing the same values. This indicates that the MPU6050 (basically the accelerometer) is not working properly and you should not use it for the project. Another chance, is that you might take different values for the x and y axes, while the values are always the same at the z axis. This is also a good indication that the MPU6050 is not functioning well. 

##### nRF24L01 Testing 
As soon we assured that both the motor driver and the MPU6050 are fine, we continued with the testing of the nRF24L01 Transceiver Modules. For this we connected all the components together in the exact the same way they should be in the project as well. 

##### Receiver wiring

1. **nRF24L01 Wireless Transceiver Modules**:
    
    - Connect VCC to a 3.3V output pin on the Arduino (or a separate 3.3V power supply).
    - Connect GND to a ground pin on the Arduino.
    - Connect CE to digital pin 9 on the Arduino.
    - Connect CNS to digital pin 10 on the Arduino.
    - Connect MOSI, MISO, and SCK to the corresponding SPI pins on the Arduino (usually pins 11, 12, and 13, respectively).
2. **MPU6050 IMU**:
    - Connect VCC to a 3.3V output pin on the Arduino.
    - Connect GND to a ground pin on the Arduino.
    - Connect SDA to the Arduino's analog pin 4 (A4).
    - Connect SCL to the Arduino's analog pin 5 (A5).
3. **Motor Driver**:

	- Connect +12V with the positive terminal of the power supply.
	- Common ground (GND driver with GND arduino) with the negative terminal.
	- Connect ENA with pin 6.
	- Connect IN1 with pin 3.
	- Connect IN2 with pin 4.
	- Connect OUT1 with reaction wheel.
	- Connect OUT2 with reaction wheel.
	
![image](https://github.com/user-attachments/assets/d4c54a20-d315-4411-b33a-b079a42c9d4d)


##### Transmitter wiring

To wire the NRF24L01 module to an Arduino board, you'll need to connect it according to the following pin configuration:

NRF24L01 Module to Arduino connections:

- VCC to 3.3V on Arduino
- GND to GND on Arduino
- CE to Pin 9 on Arduino
- CSN to Pin 10 on Arduino
- SCK to SCK (SPI Clock) on Arduino (usually Pin 13)
- MOSI to MOSI (SPI Master Out Slave In) on Arduino (usually Pin 11)
- MISO to MISO (SPI Master In Slave Out) on Arduino (usually Pin 12)

![image](https://github.com/user-attachments/assets/c3b285b2-1a56-4c8a-a870-8971d41f89ee)


##### Scripts 

In order to test the nRF24L01 modules I used [test_receiver](https://github.com/eleannapapaio/Steering-wheel/blob/main/test_receiver.ino) script and [test_transmitter](https://github.com/eleannapapaio/Steering-wheel/blob/main/test_transmitter.ino) script. With these scripts you will not see the reaction wheel turning, yet we only test the functionality of the modules. If they both work fine, this is what we expect to see at the receiver's serial monitor:
```
aaa
Response=<data_received>

```
If the transmitter's nRF24L01 module is not working properly, this is what will be printed at the transmitter's serial monitor:
```
aaa
```
This could happen if there are issues with the NRF24L01 module, such as not being able to establish a connection with the receiver, or if the receiver is not responding.
If the transmission is successful, it won't print anything else, as there are no other print statements in the code that would execute in a successful transmission scenario.

If everything seems to be working fine, the testing process is over. If you did all the above and something is not working properly, there might be an issue with the **Arduino** or the **USB** cable. 
**Make sure to check at the beginning of the process that both these two are working properly**, because afterwards with more components, it will be difficult to spot the error.

### Wiring
In order to perform the final project, we need to do the wiring, on the receiver and the transmitter.

##### Receiver wiring

The wiring should be as following:
1. **nRF24L01 Wireless Transceiver Modules**:
    - Connect VCC to a 3.3V output pin on the Arduino (or a separate 3.3V power supply).
    - Connect GND to a ground pin on the Arduino.
    - Connect CE to digital pin 9 on the Arduino.
    - Connect CNS to digital pin 10 on the Arduino.
    - Connect MOSI, MISO, and SCK to the corresponding SPI pins on the Arduino (usually pins 11, 12, and 13, respectively).
2. **MPU6050 IMU**:
    - Connect VCC to a 3.3V output pin on the Arduino.
    - Connect GND to a ground pin on the Arduino.
    - Connect SDA to the Arduino's analog pin 4 (A4).
    - Connect SCL to the Arduino's analog pin 5 (A5).
3. **Motor Driver**:

	-  Connect +12V with the positive terminal of the power supply.
	- +5V with 5V pin on the arduino
	- Common ground (GND driver with GND arduino) with the negative terminal.
	- Connect ENA with pin 6.
	- Connect IN1 with pin 3.
	- Connect IN2 with pin 4.
	- Connect OUT1 with reaction wheel.
	- Connect OUT2 with reaction wheel.

![image](https://github.com/user-attachments/assets/63423550-5177-4ff9-badc-a2856593a82c)


After you have done the wiring, you connect the arduino with the receiver script, with a USB cable.

##### Transmitter wiring

The wiring should be as following:
nRF24L01 Module to Arduino connections:

- VCC to 3.3V on Arduino
- GND to GND on Arduino
- CE to Pin 9 on Arduino
- CSN to Pin 10 on Arduino
- SCK to SCK (SPI Clock) on Arduino (usually Pin 13)
- MOSI to MOSI (SPI Master Out Slave In) on Arduino (usually Pin 11)
- MISO to MISO (SPI Master In Slave Out) on Arduino (usually Pin 12)

Then you should also connect the three cables from the steering wheel, to the Arduino. 
- Red cable to the Arduino's analog pin 0 (A0).
- Yellow cable to the Arduino's 5V.
- Black cable to GND.

![image](https://github.com/user-attachments/assets/fea5217a-abdd-40f8-8a51-69c84ab8e8c3)


**You also connect the transmitter's script with the Arduino, with a USB cable. Make sure to select the appropriate PORT in both scripts and also select 9600 baud rate. 
Double check that the wiring is correct and try to have the two nRF24L01 modules close to each other.**


