## fsae-analog-to-can-converter
This STM32 MCU project is intended to be used for taking a variety of sensor data, 
from wheel speed to brake temperature to suspension strain, and transmitting that data over CAN
to the race car's engine control unit, where it is logged for analysis, or used in other vital
car functions such as traction controlling. The values it reads come from two separate analog-to-digital
converters located on the same custom PCB as the STM32 that this code is running on. 
