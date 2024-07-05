# Components
   - Arduino MEGA 2560 Board: The main microcontroller.
   - RTC DS3231: Real-Time Clock for accurate timekeeping.
   - DFRobot_SIM7000 module: For GSM/GPRS communication with ThingSpeak.
   - Analog Pins (A0, A1): For reading voltage and current signals.
   - TimerOne: For precise sampling of voltage and current.
# Setup
     Hardware Connections
     1) Voltage and Current Sensors:
        - Connect the voltage sensor to A0.
        - Connect the current sensor to A1.
     2) SIM7000 Module:
        - Connect TX to Arduino pin 10.
        - Connect RX to Arduino pin 7.
     3) RTC DS3231:
        - Connect to the I2C pins (SDA and SCL) on the Arduino.

# Software Setup 
     1) Install the required libraries through the Arduino Library Manager.
     2) Configure your ThingSpeak API key by setting the THINGSPEAK_API_KEY macro in the code.
