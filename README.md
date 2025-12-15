# Automatic Irrigation System Prototype using ATMega328p (Arduino Uno R3), written in pure C
![Image of the Prototype](https://github.com/PabloDeges/irrigation_system_prototype/blob/main/IMG_9050.png)
![Circuit Diagram of the Irrigation System](https://github.com/PabloDeges/irrigation_system_prototype/blob/main/circuit-2.png)

The heart of the irrigation system is an FC-28 soil moisture sensor, which was very affordable but is not suitable for a final system due to its low quality and rapid
sensor drift. It is supported by a DHT11 sensor, which measures air temperature and humidity and outputs a digital signal. In addition, 4 LEDs in red, yellow, green, and blue are installed as a simple status indicator. The only thing that is not yet installed is the water pump, as none was available, but it is included in the circuit diagram and is controlled by a transistor, as it is assumed that the water pump
requires more power than 5V, which is the maximum output voltage of the Arduino.
The sensor values of the soil moisture sensor are represented abstractly by 4 LEDs.
Only one LED can light up at a time, and each color corresponds to a
specific value range/status.
RED: The soil is very dry, water immediately
YELLOW: The soil is slightly moist, watering is not yet necessary (should be the normal state
)
GREEN: The soil is very moist, which is the case when the plant has just been watered
BLUE: The soil is much too moist, increased risk of mold
The program logic is based on a simple threshold decision: if
Soil Moisture > 800 (very dry), activate water pump until Soil Moisture is in
the range of 450-380 (very moist), where the values are the outputs of the ADC,
which range from 0 to 1023 (10bit ADC).

However, the lowest value, which is reached when the sensor is fully immersed in water, is in reality at approx. 350.
This threshold logic is supported by the measurements of the air temperature and humidity sensor, as the sensor values influence the duration of the deep sleep phases.

The idea behind this was that if the plant has just been watered, it will take several
hours to days before the plant needs to be watered again.
Unnecessarily high sampling rates would therefore only consume unnecessary energy.
There is therefore an initial time period “RAW_TTN_CHECK_SEC” which is defined as 12 hours.
This Depending on the air temperature and humidity, this can be extended or shortened
    to a minimum of 4 hours and a maximum of 20 hours.
    The reasoning behind this is that when the air temperature is higher, the water in the soil evaporates faster
    than when the air temperature is cooler, and therefore it should be sampled more often
    to prevent the soil from becoming too dry. Similarly, there is a similar logic
    for humidity: when the air is particularly dry, the pressure gradient is higher
    compared to the moist soil and facilitates the evaporation of water,
    so it should also be sampled more often.
    If the sensor value exceeds a certain value, the weight
  for the sensor, e.g., +/- 6h for the air temperature, is added to the initial time period.

At elevated temperatures and particularly dry air, the sampling rate is therefore reduced.
-> time_until_next_check = RAW_TTN_CHECK_SEC - TIME_WEIGHT_TEMP - TIME_WEIGHT_TEMP_SEC
