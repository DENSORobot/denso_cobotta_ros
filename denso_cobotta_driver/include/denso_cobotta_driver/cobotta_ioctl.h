/*
 * Copyright (C) 2018-2019  DENSO WAVE INCORPORATED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */


#ifndef _COBOTTA_IOCTL_H
#define _COBOTTA_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define JOINT_MAX		8

typedef enum SRV_BRAKE_STATE {
	SRV_BRAKE_LOCK		=  0,
	SRV_BRAKE_RELEASE	=  1,
	SRV_BRAKE_NOCHANGE	= -1,
	SRV_BRAKE_NONE		= -1,
} SRV_BRAKE_STATE;


typedef struct SRV_ERROR {
	uint32_t		ErrorCode;
	uint32_t		SubCode;
} SRV_ERROR;

typedef struct IOCTL_DATA_CHECKSTATE {
	int			Arm;
	SRV_ERROR		Err;
} IOCTL_DATA_CHECKSTATE;

typedef struct IOCTL_DATA_BRAKE {
	long			ArmNo;
	long			BrakeState[JOINT_MAX];
} IOCTL_DATA_BRAKE;

typedef IOCTL_DATA_BRAKE	IOCTL_DATA_GETBRAKE;
typedef IOCTL_DATA_BRAKE	IOCTL_DATA_SETBRAKE;

typedef struct SRV_COMM_SEND {
	long			ArmNo;
	uint32_t		Discontinuous;
	uint32_t		DisableCurLim;
	uint32_t		StayHere;
	long			Position[JOINT_MAX];
	uint16_t		CurrentLimit[JOINT_MAX];
	int16_t			CurrentOffset[JOINT_MAX];
} SRV_COMM_SEND;

typedef struct SRV_COMM_SEND_RECV {
        int			Result;
        uint32_t		BuffState;
} SRV_COMM_SEND_RECV;

typedef struct IOCTL_DATA_UPDATE {
	SRV_COMM_SEND		Send;
	SRV_COMM_SEND_RECV	Recv;
} IOCTL_DATA_UPDATE;


typedef struct SRV_COMM_RECV {
	int			Result;
	uint32_t		TimeStamp;
	long			Encoder[JOINT_MAX];
	uint16_t		CurrentQ[JOINT_MAX];
	uint16_t		CurrentU[JOINT_MAX];
	uint16_t		CurrentV[JOINT_MAX];
} SRV_COMM_RECV;

typedef struct IOCTL_DATA_GETENC {
	int			Arm;
	SRV_COMM_RECV		Recv;
} IOCTL_DATA_GETENC;

typedef struct IOCTL_DATA_GETSTATE {
	uint32_t		Index;		/* IN  */
	long			Result;		/* OUT */
	long			Value;		/* OUT */
} IOCTL_DATA_GETSTATE;

typedef struct IOCTL_DATA_PUTSTATE {
	uint32_t		Index;		/* IN  */
	long			Value;		/* IN  */
	long			Result;		/* OUT */
} IOCTL_DATA_PUTSTATE;

typedef struct IOCTL_DATA_MINI_OUTPUT {
	uint16_t		Data;		/* IN  */
	uint16_t		Mask;		/* IN  */
} IOCTL_DATA_MINI_OUTPUT;

typedef	struct {
	long			Value;
	long			Result;
} IOCTL_DATA_FRAM_ARG;

typedef	struct {
	uint32_t		Start;
	uint32_t		Length;
	long			Result;
	uint8_t			*Data;
} IOCTL_DATA_FRAM_READ;

typedef	struct {
	uint32_t		Start;
	uint32_t		Length;
	long			Result;
	uint8_t			*Data;
} IOCTL_DATA_FRAM_WRITE;

typedef	struct {
	long			Index;
	uint32_t		Size;
} IOCTL_DATA_FRAM_LENGTH;


/*!< COBOTTA Acyclic Communication Data */
typedef struct SRV_CB_ACYCLIC_DATA {
	uint16_t	Address;
	uint16_t	Value;
} SRV_CB_ACYCLIC_DATA;

/*!< COBOTTA Acyclic Communication Status */
typedef struct SRV_CB_ACYCLIC_STATUS {
	uint16_t	SendStatus;
	uint16_t	RecvStatus;
} SRV_CB_ACYCLIC_STATUS;

/*!< COBOTTA Acyclic Communication Request (Single/Single) */
typedef struct SRV_CB_ACYCLIC_1_1_REQ {
	long		ArmNo;
	long		JointNo;
	SRV_CB_ACYCLIC_DATA	Data;
} SRV_CB_ACYCLIC_1_1_REQ;

/*!< COBOTTA Acyclic Communication Response (Single/Single) */
typedef struct SRV_CB_ACYCLIC_1_1_RESULT {
	long Result;
	SRV_CB_ACYCLIC_STATUS   Status;
	SRV_CB_ACYCLIC_DATA     Data;
} SRV_CB_ACYCLIC_1_1_RESULT;

/*!< COBOTTA Acyclic Communication Request (Double/Single) */
typedef struct SRV_CB_ACYCLIC_2_1_REQ {
	long    ArmNo;
	long    JointNo;
	SRV_CB_ACYCLIC_DATA     Data[2];
	SRV_CB_ACYCLIC_DATA     __dummy;
} SRV_CB_ACYCLIC_2_1_REQ;

/*!< COBOTTA Acyclic Communication Response (Double/Single) */
typedef struct SRV_CB_ACYCLIC_2_1_RESULT {
	long Result;
	SRV_CB_ACYCLIC_STATUS   Status;
	SRV_CB_ACYCLIC_DATA     Data[2];
} SRV_CB_ACYCLIC_2_1_RESULT;

/*!< COBOTTA Acyclic Communication Request (Single/All) */
typedef struct SRV_CB_ACYCLIC_1_ALL_REQ {
	SRV_CB_ACYCLIC_DATA Data[9];
} SRV_CB_ACYCLIC_1_ALL_REQ;

/*!< COBOTTA Acyclic Communication Response (Single/All) */
typedef struct SRV_CB_ACYCLIC_1_ALL_RESULT {
	long Result;
	SRV_CB_ACYCLIC_STATUS   Status[9];
	SRV_CB_ACYCLIC_DATA     Data[9];
} SRV_CB_ACYCLIC_1_ALL_RESULT;

/*!< COBOTTA Acyclic Communication Request (Double/All) */
typedef struct SRV_CB_ACYCLIC_2_ALL_REQ {
	SRV_CB_ACYCLIC_DATA Data[2][9];
} SRV_CB_ACYCLIC_2_ALL_REQ;

/*!< COBOTTA Acyclic Communication Response (Double/All) */
typedef struct SRV_CB_ACYCLIC_2_ALL_RESULT {
	long Result;
	SRV_CB_ACYCLIC_STATUS   Status[9];
	SRV_CB_ACYCLIC_DATA     Data[2][9];
} SRV_CB_ACYCLIC_2_ALL_RESULT;

typedef	struct {
	long	InLength;			/* IN  */
	long	OutLength;			/* OUT */
	union {
		SRV_CB_ACYCLIC_1_1_REQ		req_1_1;
		SRV_CB_ACYCLIC_2_1_REQ		req_2_1;
		SRV_CB_ACYCLIC_1_ALL_REQ	req_1_all;
		SRV_CB_ACYCLIC_2_ALL_REQ	req_2_all;
	};					/* IN  */
	union {
		SRV_CB_ACYCLIC_1_1_RESULT	result_1_1;
		SRV_CB_ACYCLIC_2_1_RESULT	result_2_1;
		SRV_CB_ACYCLIC_1_ALL_RESULT	result_1_all;
		SRV_CB_ACYCLIC_2_ALL_RESULT	result_2_all;
	};					/* OUT */
} IOCTL_DATA_CB_ACYCLIC_COMM;


typedef	struct {
	uint32_t		Code;
	long			Result;
} IOCTL_DATA_SAFETY_SEND;

typedef struct {
	SRV_ERROR		Err;
} IOCTL_DATA_SAFETY_CHECKSTATE;

typedef struct {
	uint32_t		driver_ver;
	char			fpga_ver[2][0x100];
	uint16_t		servo_mcu_ver[9];
	uint16_t		safety_mcu_ver[9][2];
} IOCTL_DATA_GET_VERSION;


#define COBOTTA_MAGIC			'f'


#define COBOTTA_IOCTL_SRV_CHECKSTATE	_IOWR (COBOTTA_MAGIC,  4, IOCTL_DATA_CHECKSTATE)
#define COBOTTA_IOCTL_CLR_ERROR		_IO (COBOTTA_MAGIC,  5)

#define COBOTTA_IOCTL_MOTOR_ON		_IOR (COBOTTA_MAGIC,  8, long)
#define COBOTTA_IOCTL_MOTOR_OFF		_IO (COBOTTA_MAGIC,  9)
#define COBOTTA_IOCTL_MOTOR_STATE	_IOR (COBOTTA_MAGIC,  10, long)

#define COBOTTA_IOCTL_SRV_GETBRAKE	_IOWR (COBOTTA_MAGIC,  11, IOCTL_DATA_GETBRAKE)
#define COBOTTA_IOCTL_SRV_SETBRAKE	_IOWR (COBOTTA_MAGIC,  12, IOCTL_DATA_SETBRAKE)

#define COBOTTA_IOCTL_SRV_UPDATE	_IOWR (COBOTTA_MAGIC,  13, IOCTL_DATA_UPDATE)
#define COBOTTA_IOCTL_SRV_GETENC	_IOWR (COBOTTA_MAGIC,  14, IOCTL_DATA_GETENC)


#define COBOTTA_IOCTL_SRV_GETSTATE	_IOWR (COBOTTA_MAGIC,  16, IOCTL_DATA_GETSTATE)
#define COBOTTA_IOCTL_SRV_PUTSTATE	_IOWR (COBOTTA_MAGIC,  17, IOCTL_DATA_PUTSTATE)

#define COBOTTA_IOCTL_SRV_GETSRVENABLE	_IOR (COBOTTA_MAGIC,  96, unsigned long)
#define COBOTTA_IOCTL_SRV_SETSRVENABLE	_IOW (COBOTTA_MAGIC,  97, unsigned long)

#define COBOTTA_IOCTL_MINI_INPUT	_IOR (COBOTTA_MAGIC,  100, unsigned short)
#define COBOTTA_IOCTL_MINI_OUTPUT	_IOW (COBOTTA_MAGIC,  101, IOCTL_DATA_MINI_OUTPUT)
#define COBOTTA_IOCTL_MINI_OUTPUT_READ	_IOR (COBOTTA_MAGIC,  102, unsigned short)

#define COBOTTA_IOCTL_FRAM_LOAD		_IOWR (COBOTTA_MAGIC,  112, IOCTL_DATA_FRAM_ARG)
#define COBOTTA_IOCTL_FRAM_READ		_IOWR (COBOTTA_MAGIC,  114, IOCTL_DATA_FRAM_READ)
#define COBOTTA_IOCTL_FRAM_WRITE	_IOWR (COBOTTA_MAGIC,  115, IOCTL_DATA_FRAM_WRITE)
#define COBOTTA_IOCTL_FRAM_CHECK	_IOR (COBOTTA_MAGIC,  116, long)

#define COBOTTA_IOCTL_FRAM_LENGTH	_IOWR (COBOTTA_MAGIC,  118, IOCTL_DATA_FRAM_LENGTH)

#define COBOTTA_IOCTL_CB_ACYCLIC_COMM	_IOWR (COBOTTA_MAGIC,  120, IOCTL_DATA_CB_ACYCLIC_COMM)

#define COBOTTA_IOCTL_SAFETY_SEND	_IOWR (COBOTTA_MAGIC,  134, IOCTL_DATA_SAFETY_SEND)
#define COBOTTA_IOCTL_SAFETY_CHECKSTATE	_IOR (COBOTTA_MAGIC,  140, IOCTL_DATA_SAFETY_CHECKSTATE)

#define COBOTTA_IOCTL_ERRCODE_OUTPUT	_IOW (COBOTTA_MAGIC, 199, long)

#define COBOTTA_IOCTL_GET_VERSION	_IOR (COBOTTA_MAGIC, 200, IOCTL_DATA_GET_VERSION)

#endif /* _COBOTTA_IOCTL_H */
