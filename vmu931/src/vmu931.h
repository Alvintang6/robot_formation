/*
 * vmu931 library header
 *
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 *
 */
 
/**
 ******************************************************************************
 *
 *  \mainpage vmu931 documentation
 *  \see https://github.com/bmegli/vmu931
 * 
 *  \copyright  Copyright (C) 2018 Bartosz Meglicki
 *  \file       vmu931.h
 *  \brief     Library public interface header
 *
 ******************************************************************************
 */

#ifndef VMU931_H_
#define VMU931_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** \addtogroup interface Public interface
 *  @{
 */

/**
 * @struct vmu
 * @brief Internal library data passed around by the user.
 * @see vmu_init, vmu_close
 */
struct vmu;

/**
 * @struct vmu_tx
 * @brief Data streamed for Heading.
 * @see vmu_stream, vmu_head, vmu_read_all
 */
struct vmu_tx
{
	uint32_t timestamp_ms; //!< Miliseconds elapsed since device was plugged in
	float x; //!< Between 0.0f and 360.0f degrees
};

/**
 * @struct vmu_txyz
 * @brief Data streamed for Accelerometers, Gyroscopes, Magnetometers and Euler angles
 * @see vmu_stream, vmu_resolution, vmu_accel, vmu_gyro, vmu_mag, vmu_euler, vmu_read_all
 */
struct vmu_txyz
{
	uint32_t timestamp_ms; //!< Miliseconds elapsed since device was plugged in
	float x; //!< Euler [-180.0f, 180.0f], Mag [-4800.0f, 4800.0f], Accel and Gyro as resolution
	float y; //!< Euler [-90.0f, 90.0f], Mag [-4800.0f, 4800.0f],  Accel and Gyro as resolution
	float z; //!< Euler [-180.0f, 180.0f], Mag [-4800.0f, 4800.0f], Accel and Gyro as resolution
};

/**
 * @struct vmu_twxyz
 * @brief Data streamed for Quaternions
 * @see vmu_stream, vmu_quat, vmu_read_all
 */
struct vmu_twxyz
{
	uint32_t timestamp_ms; //!< miliseconds elapsed since device was plugged in
	float w; //!< between -1.0f and 1.0f
	float x; //!< between -1.0f and 1.0f
	float y; //!< between -1.0f and 1.0f
	float z; //!< between -1.0f and 1.0f
};

/* TEXT RELATED */

enum {VMU_MAX_STRING_SIZE=255};

/**
 * @struct vmu_text
 * @brief Textual data returned by device and library
 * 
 * The strings are terminated with '\0'.
 * 
 * @see vmu_selftest, vmu_calibrate
 */
struct vmu_text
{
	char text[VMU_MAX_STRING_SIZE]; //!< '\0' terminated character data
};

/* STATUS RELATED */

/**
  * @brief Constants used for determining enabled sensors.
  * @see vmu_status
  */
enum vmu_sensors_enum {VMU_SENSORS_ACCEL=0x01, VMU_SENSORS_GYRO=0x02, VMU_SENSORS_MAG=0x04};

/**
  * @brief Constants used for determining and setting sensor resolution.
  * @see vmu_status, vmu_resolution
  */
enum vmu_resolution_enum {VMU_RESOLUTION_ACCEL_2G=0x01, VMU_RESOLUTION_ACCEL_4G=0x02,
	  VMU_RESOLUTION_ACCEL_8G=0x04, VMU_RESOLUTION_ACCEL_16G=0x08,
	  VMU_RESOLUTION_GYRO_250DPS=0x10, VMU_RESOLUTION_GYRO_500DPS=0x20,
	  VMU_RESOLUTION_GYRO_1000DPS=0x40, VMU_RESOLUTION_GYRO_2000DPS=0x80};

/**
  * @brief Constants used for setting and determining streamed data.
  * @see vmu_stream, vmu_status
  */
enum vmu_stream_enum { VMU_STREAM_ACCEL=0x01,  VMU_STREAM_GYRO = 0x02, VMU_STREAM_QUAT=0x04,  
	VMU_STREAM_MAG=0x08, VMU_STREAM_EULER=0x10, VMU_STREAM_HEAD=0x40 };

/**
 * @struct vmu_status
 * @brief Device status - enabled sensors, resolution, rate and streamed data
 *
 * @see vmu_status, vmu_sensors_enum, vmu_resolution_enum, vmu_stream_enum
 */
struct vmu_status
{
	uint8_t sensors; //!< bitwise AND with ::vmu_sensors_enum, e.g. sensors & VMU_SENSORS_ACCEL
	uint8_t resolution; //!<  bitwise AND with ::vmu_resolution_enum, e.g. resolution & VMU_RESOLUTION_ACCEL_8G
	uint8_t low_rate; //!< 0 for 1000 Hz, 1 for 200 Hz
	uint32_t stream; //!< bitwise AND with ::vmu_stream_enum, e.g. stream & VMU_STREAM_EULER
};

/**
 * @struct vmu_size
 * @brief Array sizes for \p vmu_data arrays
 *
 * @see vmu_data
 */
struct vmu_size
{
	int accel;
	int gyro;
	int mag;
	int euler;
	int quat;
	int head;	
	int text;
	int status;
};

/**
 * @struct vmu_data
 * @brief Structure with multiple types of data returned from the device.
 * 
 * Supply only arrays of data your are interested in. Set arrays sizes in \p size member.
 * 
 * @see vmu_read_all, vmu_stream
 */
struct vmu_data
{
	struct vmu_txyz *accel;
	struct vmu_txyz *gyro;
	struct vmu_txyz *mag;
	struct vmu_txyz *euler;
	struct vmu_twxyz *quat;
	struct vmu_tx *head;
	struct vmu_text *text;
	struct vmu_status *status;

	struct vmu_size size; //array sizes
};

/**
  * @brief Constants returned by most of library functions
  */
enum vmu_retval_enum {
	VMU_ERROR=-1, //!< error occured with errno set
	VMU_OK=0, //!< succesfull execution
	VMU_DATA_PENDING=1 //!< succesfull execution and more data pending without blocking
	};

/** @name Init and teardown
 */
///@{
	
/**
 * @brief initialize internal library data.
 * @param tty device like "/dev/ttyACM0" 
 * @return
 * - pointer to internal library data 
 * - NULL on error with errno set
 * 
 * @see vmu_close
 * 
 * Example:
 * @code
 * struct vmu *v=vmu_init("/dev/ttyACM0");
 * @endcode
 */
struct vmu *vmu_init(const char *tty);

/**
 * @brief free library resources
 *
 * Frees memory and restores terminal settings.
 *
 * May be safely called with NULL argument.
 *
 * @param v pointer to internal library data
 * @return 
 * - VMU_OK on success
 * - VMU_ERROR on error, query errno for the details
 * 
 * Example:
 * @code
 * vmu_close(v);
 * @endcode
 */
int vmu_close(struct vmu *v);

///@}

/** @name Convinient read
 *  Simplified read functions for single type of data (e.g. only euler).
 *  If you need to read multiple types of data use \p vmu_read_all instead.
 * 
 *  Functions will block waiting for data unless last call returned value > \p size.
 *  Timeout with return value VMU_ERROR and errno EAGAIN indicates device is not sending data type for some reason.
 * 
 *  Other data types are discarded silently without parsing. 
 * 
 *  Perfomance hints: 
 *  - use ::vmu_stream so that device streams only data you need
 * 
 * @param v pointer to internal library data
 * @param data user supplied array
 * @param size user supplied array size
 * @return
 * - value > \p size indicates user array was filled and more data is pending (without blocking)
 * - value <= \p size indicates number of data points returned in array (next call will block)
 * - VMU_ERROR indicates error, query errno for the details
 * 
 * @see vmu_stream, vmu_read_all
 * 
 * Example:
 * @code
 * int status, i;
 * struct vmu_txyz euler[10];
 * 
 * while( (status=vmu_euler(v, euler, 10)) != VMU_ERROR  )
 * {
 *    for(i=0; i<status && i<10; ++i)
 *       printf("[%lu ms] x=%f y=%f z=%f\n", euler[i].timeout_ms, euler[i].x, euler[i].y, euler[i].z);
 * }
 * @endcode
 */
///@{
/** @brief Read only accelerometer data. */
int vmu_accel(struct vmu *v, struct vmu_txyz *data, int size);
/** @brief Read only gyroscope data data. */
int vmu_gyro(struct vmu *v, struct vmu_txyz *data, int size);
/** @brief Read only magnetometer data. */
int vmu_mag(struct vmu *v, struct vmu_txyz *data, int size);
/** @brief Read only euler data. */
int vmu_euler(struct vmu *v, struct vmu_txyz *data, int size);
/** @brief Read only quaternion data. */
int vmu_quat(struct vmu *v, struct vmu_twxyz *data, int size);
/** @brief Read only heading data. */
int vmu_head(struct vmu *v, struct vmu_tx *data, int size);
///@}

/**
 * @brief Read multiple types of data simultanously.
 * 
 * Use this function if you need to read multiple types of data (e.g. euler and magnetometer).
 * 
 * If you care only about single data type use one of convinience functions instead.
 * 
 * Function will block waiting for data unless last call returned VMU_DATA_PENDING.
 * Timeout with return value VMU_ERROR and errno EAGAIN indicates device is not sending data types for some reason.
 *
 * Data types with 0 size in \p data parameter are discarded silently without parsing.
 * 
 * Perfomance hints: 
 *  - use ::vmu_stream so that device streams only data you need
 * 
 * @param v pointer to internal library data
 * @param data user supplied arrays with sizes
 * @return 
 * - VMU_OK indicates user arrays in \p data parameter were filled
 * - VMU_DATA_PENDING indicates at least one array in \p data parameter was filled completely and more data is pending (without blocking)
 * - VMU_ERROR indicates error, query errno for the details
 * 
 * @see vmu_stream, vmu_accel, vmu_gyro, vmu_mag, vmu_euler, vmu_quat, vmu_head
 */
int vmu_read_all(struct vmu *v, struct vmu_data *data);

/**
 * @brief Make the device stream particular data types.
 * 
 * Function blocks until one of:
 * - new streaming parameters are set
 * - error occurs (including timeout with errno EAGAIN)
 * 
 * @param v pointer to internal library data
 * @param stream logical sum of values from ::vmu_stream_enum
 * @return
 * - VMU_OK indicates device will stream requested data types
 * - VMU_ERROR indicates error, query errno for the details
 *  
 * @see vmu_stream_enum
 * 
 * Example:
 * @code
 * vmu_stream(v, VMU_STREAM_EULER | VMU_STREAM_MAG);
 * @endcode
 */
int vmu_stream(struct vmu *v, uint32_t stream);

/**
 * @brief Set accelerometer and/or gyroscope resolutions.
 * 
 * Blocks until resolution is set, error or timeout occurs.
 * 
 * Streaming Heading, Euler or Quaternion data forces:
 * - 2000 dps for gyroscope
 * - 2g for accelerometer
 * 
 * It setting resolution only for accelerometer or gyroscope the second remains unchanged.
 * 
 * errno values:
 * - EINVAL if \p resolution argument is not sane
 * - ENOTSUP if requested incompatible resolution while streaming Heading, Euler or Quaternion data
 * - EAGAIN if timeout occured during communication with the device
 * - other IO specific errno values are possible
 * 
 * @param v pointer to internal library data
 * @param resolution logical combination of ::vmu_resolution_enum values
 * @return
 * - VMU_OK if setting resolution succeded
 * - VMU_ERROR on IO error or invalid \p resolution argument, query errno for the details
 * 
 * @see vmu_resolution_enum
 * 
 * Example:
 * @code
 * vmu_resolution(v, VMU_RESOLUTION_ACCEL_16G | VMU_RESOLUTION_GYRO_1000DPS);
 * @endcode
 */
int vmu_resolution(struct vmu *v, uint32_t resolution);

//commands

/**
 * @brief Command device to perform selftest.
 * 
 * Blocks for around 2 seconds while device performs selftest.
 * All textual data output by the device is concatenated in \p data argument.
 * All non-textual data is discarded during the process.
 * 
 * The device needs to remain still on a flat surface with its Z-axis pointing upward during the process.
 * 
 * @param v pointer to internal library data
 * @param data argument filled with textual data
 * @return 
 * - VMU_OK if selftest succeded, query \p data for the details
 * - VMU_ERROR on IO error, selftest failure or timeout, query \p data and errno for the details
 */
int vmu_selftest(struct vmu *v, struct vmu_text *data);

/**
 * @brief Command device to perform calibration.
 * 
 * Blocks for around 4 seconds while device performs calibration.
 * All textual data output by the device is concatenated in \p data argument.
 * All non-textual data is discarded during the process.
 * The device needs to remain still on a flat surface with its Z-axis pointing upward during the process.
 * You don't have to repeat this procedure every time you power on the sensor.
 * 
 * Note - there is undocummented (VMU User Guide 1.3) magnetometer calibration procedure.
 * Magnetometer calibration is essential to reduce drift on Z (heading) axis.
 * While powering up the sensor move it around in multiple orientations.
 * Do it until the Euler angles indicates 0 degrees while the sensor has its Y axis pointing North.
 * As of firmware 1.01 this has to be repeated every time you power on the sensor.
 *  
 * @param v pointer to internal library data
 * @param data argument filled with textual data
 * @return 
 * - VMU_OK if calibration succeded, query \p data for the details
 * - VMU_ERROR on IO error, calibration failure or timeout, query \p data and errno for the details

 */
int vmu_calibrate(struct vmu *v, struct vmu_text *data);

/**
 * @brief Retrieve device status.
 * 
 * Blocks until device returns status, error or timeout occurs.
 * 
 * Non status data types are discarded while waiting for device status.
 * 
 * @param v pointer to internal library data
 * @param data argument filled with status data
 * @return
 * - VMU_OK on success
 * - VMU_ERROR on failure, query errno for the details
 */
int vmu_status(struct vmu *v, struct vmu_status *data);

/**
 * @brief Get file descriptor used for serial communication with the device
 *
 * Library user should not directly read or write from or to descriptor.
 * This function is intended to be used in synchronous I/O multiplexing (select, poll).
 *
 * @param v pointer to internal library data
 * @return file descriptor
 */
int vmu_fd(struct vmu *v);

/** @}*/



#ifdef __cplusplus
}
#endif

#endif //VMU931_H_
