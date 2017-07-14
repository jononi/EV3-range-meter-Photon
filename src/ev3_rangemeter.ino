/******************************************************************************
* Project EV3-range-meter
* Description: This EV3 compatible sensor is a range meter using the ToF VL53L0X
The sensor will be automatically detected as an ultrasonic sensor on Port 1
* Author: Jaafar Benabdallah
* Date: July 2017
****************************************************************************
Hardware connections:
Plug the Photon-EV3-Universal sensor module to Port 1 on the EV3 computer

******************************************************************************/

#include "EV3UARTEmulation.h"
#include "VL53L0X.h"

EV3UARTEmulation ev3_sensor(&Serial1,TYPE_ULTRASONIC , 38400); // emulates ultrasonic sensor
VL53L0X range_sensor;

#define serial_debug

#ifdef serial_debug
SerialLogHandler logHandler(115200, LOG_LEVEL_WARN, {
	{ "app", LOG_LEVEL_INFO },
	{ "app.custom", LOG_LEVEL_INFO }
	});
	#endif

	uint32_t	last_reading = 0;

	bool			ev3_sensor_on = false;
	uint32_t	mode = 0;

	uint32_t	range_sensor_status = 0;
	uint8_t 	rangeData[14];

	uint32_t	distance_mm = 10000; // 'infinity' distance in mm
	uint32_t	distance_in = 400;  // 'infinity' distance in in


	void setup()
	{
		Serial.begin(115200);
		Wire.begin();
		Particle.variable("EV3_status", ev3_sensor_on);
		Particle.variable("EV3_mode", mode);
		Particle.variable("VL53_status", range_sensor_status);
		Particle.variable("Range_mm", distance_mm);

		if (range_sensor.init()) {
			range_sensor_status = 1;
			#ifdef serial_debug
			Log.info("VL53L0X initialized. id: %02X", range_sensor.readReg(0xC0));
			#endif
		}
		else {
			range_sensor_status = 0;
			#ifdef serial_debug
			Log.info("VL53L0X not found at: %02X", range_sensor.getAddress());
			#endif
		}

		config_range_sensor(80); // continuous mode interval in ms

		ev3_sensor.create_mode("US-DIST-CM", true, DATA16, 1, 4, 1);
		ev3_sensor.create_mode("US-DIST-IN", true, DATA16, 1, 4, 1);

		ev3_sensor_on = ev3_sensor.reset();

	}


	void loop() {
		// get data from the range-meter, regardless of connection with EV3 brick status
		range_sensor.readMulti(0x14, &rangeData[0], 14); //RESULT_RANGE_STATUS
		byte devError = (rangeData[0] & 0x78) >> 3; // Check for errors
		range_sensor_status = (devError == 0x0B)? 1 : devError;

		distance_mm = range_sensor.readRangeContinuousMillimeters();

		#ifdef serial_debug
		Log.info("d = %d mm\n", distance_mm);
		#endif

		ev3_sensor.heart_beat();

		if ( millis() - last_reading > 100) {

			distance_in = (uint32_t)(distance_mm / 25.4);

			switch (ev3_sensor.get_current_mode()) {
				case 0:
				mode = 0;
				ev3_sensor.send_data16((int16_t)(distance_mm / 10));
				break;
				case 1:
				mode = 1;
				ev3_sensor.send_data16(distance_in & 0xFFFF);
				break;
			}
			last_reading = millis();
		}
	}


	void config_range_sensor(uint32_t continuous_time_interval_ms) {
		range_sensor.setTimeout(500); // after this time (ms), stop checking for sensor presence
		// long range settings: (more sensitive to noise like background lights)
		// lower the return signal rate limit (default is 0.25 MCPS)
		range_sensor.setSignalRateLimit(0.25);
		// increase laser pulse periods (defaults are 14 and 10 PCLKs, always even number)
		range_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
		range_sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
		// higher speed: reduce timing budget to 20 ms (default is about 33 ms)
		// or high accuracy: increase timing budget to 200 ms
		range_sensor.setMeasurementTimingBudget(60000);
		// Start continuous back-to-back mode (take readings as fast as possible).
		// To use continuous timed mode instead, provide a desired inter-measurement
		// period in ms (e.g. range_sensor.startContinuous(100)).
		range_sensor.startContinuous(continuous_time_interval_ms);
	}
