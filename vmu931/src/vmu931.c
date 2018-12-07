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

#include "vmu931.h"

#include <stdint.h> //uint8_t, int16_t, int32_t
#include <unistd.h> //read, close
#include <fcntl.h> //O_RDWR file open flag
#include <termios.h> //struct termios, tcgetattr, tcsetattr, cfsetispeed, tcflush
#include <string.h> //memcpy
#include <sys/select.h> //select
#include <malloc.h> //malloc, free
#include <errno.h> //errno
#include <endian.h> //htobe32, be32toh
#include <time.h> //time, difftime

//temp
//#include <stdio.h>

/* TUNABLE CONSTANTS */
enum {VMU_BUFFER_SIZE=1024};

// timeouts
enum {VMU_READ_TIMEOUT_MS=50,
	  VMU_CALIBRATION_TIMEOUT_MS=4000, //typically around 3 seconds and garbage for less than 100 ms
      VMU_SELFTEST_TIMEOUT_MS=2000, //typically around 1,5 seconds and garbage for less than 100 ms
      VMU_STATUS_TIMEOUT_MS=500, //typically 40 ms, but it happends that status fails for some time after sending commands
	  VMU_POST_COMMAND_DELAY_MS=200, //after commands device sometimes doesn't respond for a while
	  VMU_COMMAND_INTERBYTE_DELAY_MS=1 //variense vmu_utils.h mentions a bug, wait at least 1 ms between command chars 
	  };

/* NON TUNABLE CONSTANTS */

// start end delimeters
enum {VMU_START_OF_MESSAGE_STRING=0x02, VMU_END_OF_MESSAGE_STRING=0x03,
	  VMU_START_OF_MESSAGE_DATA=0x01, VMU_END_OF_MESSAGE_DATA=0x04}; 
	  
// payload sizes	  
enum {VMU_TXYZ_MSG_SIZE=20, VMU_TWXYZ_MSG_SIZE=24, VMU_HEAD_MSG_SIZE=12, VMU_STATUS_MSG_SIZE=11, VMU_COMMAND_SIZE=4}; 
enum {VMU_NON_PAYLOAD_SIZE=4};

// offsets
enum {VMU_START_OF_MESSAGE_OFFSET=0, VMU_MESSAGE_SIZE_OFFSET=1, VMU_MESSAGE_TYPE_OFFSET=2, VMU_MSG_PAYLOAD_OFFSET=3};

// validation return values
enum {VMU_INVALID_MESSAGE=-1, VMU_NEED_MORE_DATA=0, VMU_VAlID_MESSAGE=1};

// message processing return values
enum {VMU_NO_SPACE_IN_USER_ARRAY=-1, VMU_MESSAGE_PROCESSED=0};

// masks for resolution bits of ACCEL and GYRO
enum {VMU_RESOLUTION_ACCEL_MASK = VMU_RESOLUTION_ACCEL_2G | VMU_RESOLUTION_ACCEL_4G | VMU_RESOLUTION_ACCEL_8G | VMU_RESOLUTION_ACCEL_16G,
VMU_RESOLUTION_GYRO_MASK = VMU_RESOLUTION_GYRO_250DPS | VMU_RESOLUTION_GYRO_500DPS | VMU_RESOLUTION_GYRO_1000DPS | VMU_RESOLUTION_GYRO_2000DPS};

// internal library data
struct vmu
{
	int fd;
	int data_pending;
	struct termios initial_termios;
	struct termios actual_termios;
	uint8_t buffer[VMU_BUFFER_SIZE];
	int buffer_bytes;
};

/* Init and teardown */

struct vmu *vmu_init(const char *tty);
int vmu_close(struct vmu *v);

/* Data reading functions */

int vmu_accel(struct vmu *v, struct vmu_txyz *data, int size);
int vmu_gyro(struct vmu *v, struct vmu_txyz *data, int size);
int vmu_mag(struct vmu *v, struct vmu_txyz *data, int size);
int vmu_euler(struct vmu *v, struct vmu_txyz *data, int size);
int vmu_quat(struct vmu *v, struct vmu_twxyz *data, int size);
int vmu_head(struct vmu *v, struct vmu_tx *data, int size);

int vmu_read_all(struct vmu *v, struct vmu_data *data);

/* Message validation */

static int validate_message(struct vmu *v, int from);
static int is_valid_message_start(uint8_t c);
static int is_valid_message_start_end(uint8_t msg_start, uint8_t msg_end);
static int is_valid_length_for_message_type(uint8_t msg_start,uint8_t msg_type, uint8_t msg_length);

/* Message processing and decoding */

static int process_message(uint8_t *msg, struct vmu_data *data, struct vmu_size *counters);

static void decode_message_string(uint8_t *msg, struct vmu_text *data);
static void decode_message_tx(uint8_t *msg, struct vmu_tx *data);
static void decode_message_txyz(uint8_t *msg, struct vmu_txyz *data);
static void decode_message_twxyz(uint8_t *msg, struct vmu_twxyz *data);
static void decode_message_status(uint8_t *msg, struct vmu_status *data);

static uint32_t decode_uint32(uint8_t *encoded);
static float decode_float(uint8_t *encoded);

/* Stream settings functions */

int vmu_stream(struct vmu *v, uint32_t stream);
int vmu_resolution(struct vmu *v, uint32_t resolution);

static int validate_resolution(uint32_t resolution);
static int resolution_matches(uint32_t requested, uint32_t current);

/* Test, calibration and device status */

int vmu_selftest(struct vmu *v, struct vmu_text *data);
int vmu_calibrate(struct vmu *v, struct vmu_text *data);

static int wait_string(struct vmu *v, int total_timeout_ms, const char *awaited_string, struct vmu_text *data);

int vmu_status(struct vmu *v, struct vmu_status *data);

/* Low level IO */
static int recv(struct vmu *v, int timeout_ms);
static int send(struct vmu *v, const char *command);

/* Generic helpers */
static void sleep_ms(int ms);

/* ---------------------- IMPLEMENTATION ----------------------------- */

/* Init and teardown functions */

struct vmu *vmu_init(const char *tty)
{
	struct vmu *v;

	v=(struct vmu*)malloc(sizeof(struct vmu));

	if( v == NULL )
		return NULL;

	v->buffer_bytes=0;
	v->data_pending=0;
	
	if ( (v->fd=open(tty, O_RDWR)) ==-1 )
	{
		free(v);
		return NULL;
	}

	if(tcgetattr(v->fd, &v->initial_termios) < 0)
	{
		close(v->fd);
		free(v);
		return NULL;
	}	
	
	v->actual_termios.c_iflag=v->actual_termios.c_oflag=v->actual_termios.c_lflag=0;
	v->actual_termios.c_cflag=CS8|CREAD|CLOCAL; //8 bit characters	
	v->actual_termios.c_cc[VMIN]=1;
	v->actual_termios.c_cc[VTIME]=0;

	//printf("tcsetattr\n");
	if(tcsetattr(v->fd, TCSANOW, &v->actual_termios) < 0)
	{
		close(v->fd);
		free(v);
		return NULL;
	}
	
	if(tcflush(v->fd, TCIOFLUSH) < 0)
	{
		close(v->fd);
		free(v);
		return NULL;
	}
	
	// from man (TO DO)
	// Note that tcsetattr() returns success if any of the  requested  changes
    // could  be  successfully  carried  out.  Therefore, when making multiple
    // changes it may be necessary to follow this call with a further call  to
    // tcgetattr() to check that all changes have been performed successfully.
	//
    // this is still edge case to consider, some settings may have not been made
	// solution tcgetattr and check settings we made for equality
	
	return v;
}

int vmu_close(struct vmu *v)
{
	int error=0;

	if(v == NULL)
		return VMU_OK;

	// Note that tcsetattr() returns success if any of the  requested  changes
	// could  be  successfully  carried  out.  Therefore, when making multiple
	// changes it may be necessary to follow this call with a further call  to
	// tcgetattr() to check that all changes have been performed successfully.
	error |= tcsetattr(v->fd, TCSANOW, &v->initial_termios) < 0;
	error |= close(v->fd) < 0;

	free(v);

	if(error)
		return VMU_ERROR;
	
	return VMU_OK;
}

/* Data reading functions */

int vmu_accel(struct vmu *v, struct vmu_txyz *data, int size)
{
	int ret;

	struct vmu_data vdata = {0};
	vdata.accel=data;
	vdata.size.accel=size;
	
	if( (ret=vmu_read_all(v, &vdata)) == VMU_ERROR )
		return VMU_ERROR;
	
	if(ret == VMU_DATA_PENDING)
		return size+1;

	return vdata.size.accel;
}

int vmu_gyro(struct vmu *v, struct vmu_txyz *data, int size)
{
	int ret;

	struct vmu_data vdata = {0};
	vdata.gyro=data;
	vdata.size.gyro=size;
	
	if( (ret=vmu_read_all(v, &vdata)) == VMU_ERROR )
		return VMU_ERROR;
	
	if(ret == VMU_DATA_PENDING)
		return size+1;

	return vdata.size.gyro;
}

int vmu_mag(struct vmu *v, struct vmu_txyz *data, int size)
{
	int ret;

	struct vmu_data vdata = {0};
	vdata.mag=data;
	vdata.size.mag=size;
	
	if( (ret=vmu_read_all(v, &vdata)) == VMU_ERROR )
		return VMU_ERROR;
	
	if(ret == VMU_DATA_PENDING)
		return size+1;

	return vdata.size.mag;
}

int vmu_euler(struct vmu *v, struct vmu_txyz *data, int size)
{
	int ret;

	struct vmu_data vdata = {0};
	vdata.euler=data;
	vdata.size.euler=size;
	
	if( (ret=vmu_read_all(v, &vdata)) == VMU_ERROR )
		return VMU_ERROR;
	
	if(ret == VMU_DATA_PENDING)
		return size+1;

	return vdata.size.euler;
}

int vmu_quat(struct vmu *v, struct vmu_twxyz *data, int size)
{
	int ret;

	struct vmu_data vdata = {0};
	vdata.quat=data;
	vdata.size.quat=size;
	
	if( (ret=vmu_read_all(v, &vdata)) == VMU_ERROR )
		return VMU_ERROR;
	
	if(ret == VMU_DATA_PENDING)
		return size+1;

	return vdata.size.quat;
}

int vmu_head(struct vmu *v, struct vmu_tx *data, int size)
{
	int ret;

	struct vmu_data vdata = {0};
	vdata.head=data;
	vdata.size.head=size;
	
	if( (ret=vmu_read_all(v, &vdata)) == VMU_ERROR )
		return VMU_ERROR;
	
	if(ret == VMU_DATA_PENDING)
		return size+1;

	return vdata.size.head;
}

int vmu_read_all(struct vmu* v, struct vmu_data *data)
{
	int valid, msg_process_status=VMU_MESSAGE_PROCESSED, offset=0;
	struct vmu_size counters={0};

	if( recv(v, VMU_READ_TIMEOUT_MS) == VMU_ERROR )
	{
		data->size=counters;
		return VMU_ERROR;
	}
	
	while( (valid=validate_message(v, offset)) != VMU_NEED_MORE_DATA  )
	{
		if(valid == VMU_INVALID_MESSAGE)
		{	//try luck starting from the next byte
			++offset;
			continue;
		}
		//otherwise VMU_VALID_MESSAGE
		if( (msg_process_status=process_message(v->buffer+offset, data, &counters)) == VMU_NO_SPACE_IN_USER_ARRAY)
			break; 
		//otherwise VMU_MESSAGE_PROCESSED
		offset+=v->buffer[offset+VMU_MESSAGE_SIZE_OFFSET];
	}
	
	memmove(v->buffer, v->buffer+offset, v->buffer_bytes-offset);
	v->buffer_bytes -= offset;
	
	data->size = counters; 
		
	if(msg_process_status == VMU_NO_SPACE_IN_USER_ARRAY)
	{
		v->data_pending=1;
		return VMU_DATA_PENDING;
	}
	//otherwise valid == VMU_NEED_MORE_DATA
	v->data_pending=0;
	return VMU_OK;
}

/* Message validation */

// returns VMU_INVALID_MESSAGE or VMU_NEED_MORE_DATA or VMU_VALID_MESSAGE
static int validate_message(struct vmu *v, int from)
{
	int pending_bytes=v->buffer_bytes-from;
	uint8_t msg_start, msg_size=UINT8_MAX, msg_end, msg_type;
	
	if(pending_bytes == 0)
		return VMU_NEED_MORE_DATA;
	
	if(pending_bytes >= 1)
	{
		msg_start=v->buffer[from];
		if(!is_valid_message_start(msg_start))
			return VMU_INVALID_MESSAGE; 
	}	
	
	if(pending_bytes >= 3)
	{
		msg_size=v->buffer[from+1];
		msg_type=v->buffer[from+2];
		
		if(!is_valid_length_for_message_type(msg_start, msg_type, msg_size))
			return VMU_INVALID_MESSAGE;
	}
		
	if(pending_bytes >= msg_size ) //start, length, type/reserved, end so we have end of message
	{
		msg_end=v->buffer[from + msg_size -1];
		
		if(!is_valid_message_start_end(msg_start, msg_end))
			return VMU_INVALID_MESSAGE; 

		// if we got that far:
		// - message has correct start
		// - message has valid length for start/type
		// - message end delimter matches start delimeter
		// we conclude that it is a valid message
		return VMU_VAlID_MESSAGE;
	}
	
	return VMU_NEED_MORE_DATA;	
}

static int is_valid_message_start(uint8_t c)
{
	return c == VMU_START_OF_MESSAGE_STRING || c == VMU_START_OF_MESSAGE_DATA;
}

static int is_valid_message_start_end(uint8_t msg_start, uint8_t msg_end)
{
	return (msg_start==VMU_START_OF_MESSAGE_STRING && msg_end==VMU_END_OF_MESSAGE_STRING) ||
			(msg_start==VMU_START_OF_MESSAGE_DATA && msg_end==VMU_END_OF_MESSAGE_DATA);
}

static int is_valid_length_for_message_type(uint8_t msg_start,uint8_t msg_type, uint8_t msg_length)
{
	return msg_start == (VMU_START_OF_MESSAGE_DATA &&
	( ( (msg_type == 'a' || msg_type=='g' || msg_type=='c' || msg_type=='e') && msg_length==VMU_TXYZ_MSG_SIZE ) ||
		(msg_type == 'q' && msg_length==VMU_TWXYZ_MSG_SIZE) || (msg_type=='h' && msg_length==VMU_HEAD_MSG_SIZE) ||
		(msg_type == 's' && msg_length==VMU_STATUS_MSG_SIZE)) )
		|| 
	    (msg_start == VMU_START_OF_MESSAGE_STRING && msg_length > 0);
}

/* Message processing and decoding */

//returns VMU_MESSAGE_PROCESSED or VMU_NO_SPACE_IN_USER_ARRAY
static int process_message(uint8_t *msg, struct vmu_data *data, struct vmu_size *counters)
{
	const uint8_t msg_start=msg[VMU_START_OF_MESSAGE_OFFSET];
	const uint8_t msg_type=msg[VMU_MESSAGE_TYPE_OFFSET];
		
	if(msg_start == VMU_START_OF_MESSAGE_STRING)
	{
		if(data->size.text == 0) //ignore the message of not collected type
			return VMU_MESSAGE_PROCESSED; 

		if(counters->text >= data->size.text)
			return VMU_NO_SPACE_IN_USER_ARRAY;
		
		//otherwise we have place for the message
		decode_message_string(msg, data->text + counters->text); 
		++counters->text;

		return VMU_MESSAGE_PROCESSED;
	}
	
	//otherwise msg_start == VMU_START_OF_MESSAGE_DATA
	switch(msg_type)
	{
		case 'a':
			if(data->size.accel==0)
				return VMU_MESSAGE_PROCESSED;
			if(counters->accel >= data->size.accel)
				return VMU_NO_SPACE_IN_USER_ARRAY;
			
			decode_message_txyz(msg, data->accel + counters->accel);
			
			++counters->accel;
			break;
		case 'g':
			if(data->size.gyro==0)
				return VMU_MESSAGE_PROCESSED;
			if(counters->gyro >= data->size.gyro)
				return VMU_NO_SPACE_IN_USER_ARRAY;
			
			decode_message_txyz(msg, data->gyro + counters->gyro);
			
			++counters->gyro;
			break;
		case 'c':
			if(data->size.mag==0)
				return VMU_MESSAGE_PROCESSED;
			if(counters->mag >= data->size.mag)
				return VMU_NO_SPACE_IN_USER_ARRAY;
			
			decode_message_txyz(msg, data->mag + counters->mag);
			
			++counters->mag;
			break;
		case 'e':
			if(data->size.euler==0)
				return VMU_MESSAGE_PROCESSED;
			if(counters->euler >= data->size.euler)
				return VMU_NO_SPACE_IN_USER_ARRAY;
			
			decode_message_txyz(msg, data->euler + counters->euler);
			
			++counters->euler;
			break;
		case 'q':
			if(data->size.quat==0)
				return VMU_MESSAGE_PROCESSED;
			if(counters->quat >= data->size.quat)
				return VMU_NO_SPACE_IN_USER_ARRAY;
			
			decode_message_twxyz(msg, data->quat + counters->quat);
			
			++counters->quat;
			break;
		case 'h':
			if(data->size.head==0)
				return VMU_MESSAGE_PROCESSED;
			if(counters->head >= data->size.head)
				return VMU_NO_SPACE_IN_USER_ARRAY;
			
			decode_message_tx(msg, data->head + counters->head);
			
			++counters->head;
			break;
		case 's':
			if(data->size.status==0)
				return VMU_MESSAGE_PROCESSED;
			if(counters->status >= data->size.status)
				return VMU_NO_SPACE_IN_USER_ARRAY;

			decode_message_status(msg, data->status + counters->status);

			++counters->status;
			break;
		default:
			;//fprintf(stderr, "unsupported message type: %c\n", msg_type);
	}
	
	//if we got that far the message is of unsupported type
	return VMU_MESSAGE_PROCESSED; 
}

/* Message level decoding */

void decode_message_string(uint8_t* msg, struct vmu_text *data)
{
	const uint8_t payload_size=msg[VMU_MESSAGE_SIZE_OFFSET]-VMU_NON_PAYLOAD_SIZE;
	memcpy(data->text, msg+VMU_MSG_PAYLOAD_OFFSET, payload_size);
	data->text[payload_size]='\0';
}
static void decode_message_tx(uint8_t *msg, struct vmu_tx *data)
{
	uint8_t *payload=msg+VMU_MSG_PAYLOAD_OFFSET;
	data->timestamp_ms=decode_uint32(payload);
	data->x=decode_float(payload+sizeof(uint32_t));
}
static void decode_message_txyz(uint8_t *msg, struct vmu_txyz *data)
{
	uint8_t *payload=msg+VMU_MSG_PAYLOAD_OFFSET;
	data->timestamp_ms=decode_uint32(payload);
	data->x=decode_float(payload+sizeof(uint32_t));
	data->y=decode_float(payload+2*sizeof(uint32_t));
	data->z=decode_float(payload+3*sizeof(uint32_t));
}
static void decode_message_twxyz(uint8_t *msg, struct vmu_twxyz *data)
{
	uint8_t *payload=msg+VMU_MSG_PAYLOAD_OFFSET;
	data->timestamp_ms=decode_uint32(payload);
	data->w=decode_float(payload+sizeof(uint32_t));
	data->x=decode_float(payload+2*sizeof(uint32_t));
	data->y=decode_float(payload+3*sizeof(uint32_t));
	data->z=decode_float(payload+4*sizeof(uint32_t));
}

static void decode_message_status(uint8_t *msg, struct vmu_status *data)
{
	uint8_t *payload=msg+VMU_MSG_PAYLOAD_OFFSET;
	data->sensors=payload[0];
	data->resolution=payload[1];
	data->low_rate=payload[2];
	data->stream=decode_uint32(payload+3);	
}

/* Data type level decoding */

static uint32_t decode_uint32(uint8_t *encoded)
{
	uint32_t temp;
	memcpy(&temp, encoded, sizeof(temp));
	return be32toh(temp);
}
static float decode_float(uint8_t *encoded)
{
	uint32_t temp;
	float tempf;
	memcpy(&temp, encoded, sizeof(temp));
	temp=be32toh(temp);
	memcpy(&tempf, &temp, sizeof(temp));
	return tempf;	
}

/* Stream settings functions */

int vmu_stream(struct vmu *v, uint32_t stream)
{
	struct vmu_status status;

	if(vmu_status(v, &status) == VMU_ERROR)
	{
		//perror("status in stream failed\n");
		return VMU_ERROR;
	}
	if(stream == status.stream)
		return VMU_OK; //nothing to do
	
	//toggle streaming to match the requested state
	if( (stream & VMU_STREAM_ACCEL) != (status.stream & VMU_STREAM_ACCEL) )
		if(send(v, "vara") == VMU_ERROR)
			 return VMU_ERROR;
			 
	if( (stream & VMU_STREAM_GYRO) != (status.stream & VMU_STREAM_GYRO) )
		if(send(v, "varg") == VMU_ERROR)
			 return VMU_ERROR;
			 
	if( (stream & VMU_STREAM_MAG) != (status.stream & VMU_STREAM_MAG) )
		if(send(v, "varc") == VMU_ERROR)
			 return VMU_ERROR;

	if( (stream & VMU_STREAM_QUAT) != (status.stream & VMU_STREAM_QUAT) )
		if(send(v, "varq") == VMU_ERROR)
			 return VMU_ERROR;

	if( (stream & VMU_STREAM_EULER) != (status.stream & VMU_STREAM_EULER) )
		if(send(v, "vare") == VMU_ERROR)
			 return VMU_ERROR;

	if( (stream & VMU_STREAM_HEAD) != (status.stream & VMU_STREAM_HEAD) )
		if(send(v, "varh") == VMU_ERROR)
			 return VMU_ERROR;
			
	sleep_ms(VMU_POST_COMMAND_DELAY_MS);		
	
	if(vmu_status(v, &status) == VMU_ERROR)
	{
		//perror("status failed\n");
		return VMU_ERROR;
	}

	if(stream == status.stream)
		return VMU_OK;
	
	//printf("stream failed\n");
	return VMU_ERROR;		
}

int vmu_resolution(struct vmu *v, uint32_t resolution)
{
	struct vmu_status status;
	
	if(validate_resolution(resolution) == VMU_ERROR)
	{	
		errno = EINVAL;
		return VMU_ERROR;
	}
	
	if(vmu_status(v, &status) == VMU_ERROR)
		return VMU_ERROR; //errno with some IO error or EAGAIN on timeout
	
	if( resolution_matches(resolution, status.resolution) == VMU_OK)
		return VMU_OK; //nothing to be done
		
	//streaming heading, euler or quaterion forces 2000 dps gyro and 2 g accelerometer, no changes allowed
	if( (status.stream & VMU_STREAM_HEAD) || (status.stream & VMU_STREAM_EULER) || (status.stream & VMU_STREAM_QUAT))
	{ 	
		errno = ENOTSUP;
		return VMU_ERROR;
	}

	//if we got that far we need to set requested resolution	
	if( (resolution & VMU_RESOLUTION_GYRO_250DPS) && (send(v, "var0") == VMU_ERROR) )
			return VMU_ERROR;
	if( (resolution & VMU_RESOLUTION_GYRO_500DPS) && (send(v, "var1") == VMU_ERROR) )
			return VMU_ERROR;
	if( (resolution & VMU_RESOLUTION_GYRO_1000DPS) && (send(v, "var2") == VMU_ERROR) )
			return VMU_ERROR;
	if( (resolution & VMU_RESOLUTION_GYRO_2000DPS) && (send(v, "var3") == VMU_ERROR) )
			return VMU_ERROR;

	if( (resolution & VMU_RESOLUTION_ACCEL_2G) && (send(v, "var4") == VMU_ERROR)  )
			return VMU_ERROR;
	if( (resolution & VMU_RESOLUTION_ACCEL_4G) && (send(v, "var5") == VMU_ERROR) )
			return VMU_ERROR;
	if( (resolution & VMU_RESOLUTION_ACCEL_8G) && (send(v, "var6") == VMU_ERROR) )
			return VMU_ERROR;
	if( (resolution & VMU_RESOLUTION_ACCEL_16G) && (send(v, "var7") == VMU_ERROR) )
			return VMU_ERROR;

	//finally confirm that resolution was set
	if(vmu_status(v, &status) == VMU_ERROR)
		return VMU_ERROR; //errno with some IO error or EAGAIN on timeout
	
	return resolution_matches(resolution, status.resolution);
}

//check whether argument is sane:
// - up to one for gyro
// - up to one for accel
// - at least one set
static int validate_resolution(uint32_t resolution)
{
	int suma, sumg;

	suma = ((resolution & VMU_RESOLUTION_ACCEL_2G) !=0) +
	((resolution & VMU_RESOLUTION_ACCEL_4G) !=0) +
	((resolution & VMU_RESOLUTION_ACCEL_8G) !=0) +
	((resolution & VMU_RESOLUTION_ACCEL_16G) !=0);

	sumg = ((resolution & VMU_RESOLUTION_GYRO_250DPS) !=0) +
	((resolution & VMU_RESOLUTION_GYRO_500DPS) !=0) +
	((resolution & VMU_RESOLUTION_GYRO_1000DPS) !=0) +
	((resolution & VMU_RESOLUTION_GYRO_2000DPS) !=0);
	
	if(suma>1 || sumg>1 || (suma+sumg) == 0)
		return VMU_ERROR;
	
	return VMU_OK;
}

//prerequisities:
//- requested resolution validated (at least ACCEL or GYRO set, sane combination) 
//returns:
// VMU_SUCCES if requested resolution matches current resolution
// VMU_ERROR otherwise
static int resolution_matches(uint32_t requested, uint32_t current)
{
	if(requested == current) 
		return VMU_OK; //nothing to be done
	
	if( !(requested & VMU_RESOLUTION_ACCEL_MASK) )
		if( (requested & VMU_RESOLUTION_GYRO_MASK) ==  (current & VMU_RESOLUTION_GYRO_MASK) )
			return VMU_OK;
		
	if( !(requested & VMU_RESOLUTION_GYRO_MASK) )
		if( (requested & VMU_RESOLUTION_ACCEL_MASK) ==  (current & VMU_RESOLUTION_ACCEL_MASK) )
			return VMU_OK;			

	return VMU_ERROR;
}

/* Test, calibration and device status */

// blocks for 2 seconds
// returns: 
// - VMU_OK on success, you may query data->text for more information
// - VMU_ERROR on IO error, calibration error or timeout, query data->text for the reason
int vmu_selftest(struct vmu* v, struct vmu_text *data)
{
	if(data)
		data->text[0]='\0';

	if(send(v, "vart") == VMU_ERROR)
		return VMU_ERROR;

	// Typically vmu returns these sequences of messages:
	// 1: Self-test started. | Test passed. Your device works fine.
	// 2: Self-test started. | Gyro failed. | Accel failed.
	return wait_string(v, VMU_SELFTEST_TIMEOUT_MS, "Test passed.", data);
}

// blocks for 4 seconds
// returns: 
// - VMU_OK on success, you may query data->text for more information
// - VMU_ERROR on IO error, calibration error or timeout, query data->text for the reason
int vmu_calibrate(struct vmu *v, struct vmu_text *data)
{
	if(data)
		data->text[0]='\0';

	if(send(v, "varl") == VMU_ERROR)
		return VMU_ERROR;
	
	// Typically vmu returns these sequences of messages:
	// 1: Calibration started. | Calibration completed.
	// 2: Calibration started. | Gyro failed. | Accel failed. | Calibration cannot be completed due to device failure.
	return wait_string(v, VMU_CALIBRATION_TIMEOUT_MS, "Calibration completed.", data);
}

static int wait_string(struct vmu *v, int total_timeout_ms, const char *awaited_string, struct vmu_text *data)
{
	const double TIMEOUT_S=total_timeout_ms/1000.0;
	time_t start;
	int bytes_copied=0, len, copy_max;	
	struct vmu_data vmu_data = {0};
	struct vmu_text vmu_text;	
	int status=VMU_ERROR, awaited_string_length=strlen(awaited_string);

	vmu_data.text=&vmu_text;
	
	if(data)
		data->text[0]='\0';
		
	start=time(NULL);
	//wait even if finished, VMU returns garbage for a while after calibration/test
	while( difftime(time(NULL), start)<TIMEOUT_S )
	{
		vmu_data.size.text=1;
		
		if( vmu_read_all(v, &vmu_data) == VMU_ERROR && errno != EAGAIN )
			return VMU_ERROR;

		if(vmu_data.size.text == 0)
			continue;

		//this could be early termination point if not for garbage after calibration/selftest
		if(strncmp(awaited_string, vmu_data.text->text, awaited_string_length) == 0)
			status = VMU_OK;

		if(data) //concatenate messages from vmu for user
		{
			len=strlen(vmu_data.text->text);	
			copy_max = (bytes_copied+len < VMU_MAX_STRING_SIZE -1 ) ? len : VMU_MAX_STRING_SIZE - 1 - bytes_copied;  				
			strncpy(data->text+bytes_copied, vmu_data.text->text, copy_max);
			bytes_copied += copy_max;
			data->text[bytes_copied]='\0';
		}
	}
				
	return status;	
}

int vmu_status(struct vmu *v, struct vmu_status *data)
{
	const double TIMEOUT_S=VMU_STATUS_TIMEOUT_MS/1000.0;
	time_t start;
	struct vmu_data vmu_data = {0};
	vmu_data.status=data;
	
	if(send(v, "vars") == VMU_ERROR)
		return VMU_ERROR;

	start=time(NULL);
	//typically 40 ms	
	while( difftime(time(NULL), start)<TIMEOUT_S )
	{
		vmu_data.size.status=1;
		
		if( vmu_read_all(v, &vmu_data) == VMU_ERROR && errno != EAGAIN )
			return VMU_ERROR;

		if(vmu_data.size.status == 0)
			continue;
		//otherwise we got status
		return VMU_OK;
	}
	return VMU_ERROR;
}

/* Low level IO */

static int recv(struct vmu *v, int timeout_ms)
{
	int ret;
	struct timeval tv={0};
	fd_set rfds;

	if(v->data_pending)
		return VMU_OK;

	FD_ZERO(&rfds);
	FD_SET(v->fd, &rfds);

	tv.tv_usec = VMU_READ_TIMEOUT_MS*1000; 	
		
	if( (ret = select(v->fd+1, &rfds, NULL, NULL, &tv)) < 0 )
		return VMU_ERROR;

	if (ret == 0) //timeout
	{
		errno = EAGAIN;
		return VMU_ERROR;
	}
	
	if( (ret = read(v->fd, v->buffer+v->buffer_bytes, VMU_BUFFER_SIZE-v->buffer_bytes )) < 0 )
		return VMU_ERROR;
	if( ret == 0 )
	{ //EOF - device unplugged
		errno = ENODEV;
		return VMU_ERROR;
	}

	v->buffer_bytes += ret;
	
	return VMU_OK;
}


/* prerequisities:
 * - vmu931_init called successfully
 * - command of length VMU_COMMAND_SIZE (4)
 * 
 * returns:
 * - VMU_OK on success
 * - VMU_ERROR on error and errno is set
*/  
static int send(struct vmu *v, const char *command)
{
	int bytes_sent=0, status;

	//byte by byte, variense vmu_utils.h mentions a bug:
	//-wait at least 1 ms between command charcters
	while(bytes_sent < VMU_COMMAND_SIZE)
	{
		if( (status=write(v->fd, command + bytes_sent, 1)) < 0)
			return VMU_ERROR;
		bytes_sent+=status;
		sleep_ms(VMU_COMMAND_INTERBYTE_DELAY_MS);
	}

	return VMU_OK;
}

int vmu_fd(struct vmu *v)
{
	return v->fd;
}

/* Generic Helpers */

static void sleep_ms(int ms)
{
	static struct timespec sleep_time = {0};
	sleep_time.tv_nsec=1000000*ms;
	nanosleep(&sleep_time, NULL);
}
