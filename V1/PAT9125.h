/*
 * PAT9125.h
 *
 * Created: 03/01/2018 21:29:04
 *  Author: David
 */ 

#ifndef PAT9125_H_
#define PAT9125_H_

bool OTS_Sensor_Init();									// Initialise the sensor, returning true if successful
bool OTS_Sensor_ReadMotion(int16_t& dx, int16_t& dy);	// If any motion has occurred since the last read, update dx and dy and return true
uint8_t OTS_Sensor_Read_shutter();						// Return the shutter register (5 bits)
uint8_t OTS_Sensor_Read_brightness();					// Return the brightness register (8 bits)

#endif /* PAT9125_H_ */
