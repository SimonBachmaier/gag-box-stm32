# Gag Box Software

Software for STM32 MCU to measure and send acceleration, gyrooscope and GPS data to the CAN bus.  
Software was developed as a student in the Formula Student Team "Strohm und Söhne e.V." in Nuernberg Germany for use in our next car "Nora 7".  
  
The project was generated with CubeMX and edited/built using STM32 SysteWworkbench.
  
The software was developed for a custom board (board developed by other members of the team) containing the following components:
- MCU: STM32F303
- Inertial measurement: BMI160
- GPS Receiver: Venus638FLPx
    
Communication with the BMI160 is handled over I2C, with the Venus638FLPx over UART.  
CAN messages containing the measured data are being send continuessly at a certain rate.  
  
This project was my first embedded project and my first project writing C. There are certain areas that can be rewritten a lot better, for example:
- The way the UART messages from the GPS receiver are handled (learn how the IRQ handler works and implement ring buffer)
- Seperate the application code better from the CubeMX generated Code