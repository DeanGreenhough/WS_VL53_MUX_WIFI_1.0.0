# WS_VL53_MUX_WIFI_1.0.0
Water Softener Project

  This project incorporates ultra low power usage for the ESP32, only taking a single 
  power measurement every 24 hours. Battery chemistry LiFePo4, no regulator used to save   
  20% overhead
  
  The Program uses 2 x VL53 Lidar sensors to measure the distance in a salt block water softener.
  Utilises TPL5110, ATtiny13a, ESP32, NPN & PNP Mosfets, INA219 for onboard testing only.
  ESP32 will hold its own power supply on, until self power down (determined by ESP32).
  
  TPL5110 35nA standby mode, wakes every 2 hours, wakes the ATtiny13a and checks a counter.
  The counter is stored in NV memory (EEPROM) and in my case at >12 turns on  PNP Mosfet to power ESP32.
  
  On Handover, Attiny13a shuts down (2.3mA for 162mS every 2 hours)
  ESP32 turns on another NPN Mosfet to take over from the ATtiny Mosfet and ESP32 self powers 
  down when work complete (currently 3-4 Secs every 24 hours) 

  For the purposes of testing MQTT data is sent to a Server based on a Rasp Pi and displayed on Grafana using influxDB,
  Node Red is used to covert the JSON payload  back to an int, to enable its storage on influxDB (not shown in this code)

  An ePaper display is used to display the readings, omitted due to its length.
  
  
