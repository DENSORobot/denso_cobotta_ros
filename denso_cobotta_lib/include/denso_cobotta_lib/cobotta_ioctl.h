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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301,
 * USA.
 */

#ifndef _COBOTTA_IOCTL_H
#define _COBOTTA_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define ARM_MAX 4
#define JOINT_MAX 8

typedef enum SRV_BRAKE_STATE {
	SRV_BRAKE_LOCK = 0,
	SRV_BRAKE_RELEASE = 1,
	SRV_BRAKE_NOCHANGE = -1,
	SRV_BRAKE_NONE = -1,
} SRV_BRAKE_STATE;

typedef struct IOCTL_DATA_RESULT {
	long result; /* OUT */
} IOCTL_DATA_RESULT;

typedef struct SRV_ERROR {
	uint32_t error_code;
	uint32_t sub_code;
} SRV_ERROR;

typedef struct IOCTL_DATA_CHECKSTATE {
	long arm_no;
	long result; /* OUT */
	SRV_ERROR err;
} IOCTL_DATA_CHECKSTATE;

typedef struct IOCTL_DATA_MOTOR_STATE {
	long result; /* OUT */
	long state;  /* OUT */
} IOCTL_DATA_MOTOR_STATE;

typedef struct IOCTL_DATA_BRAKE {
	long arm_no;
	long result;
	long brake_state[JOINT_MAX];
} IOCTL_DATA_BRAKE;

typedef IOCTL_DATA_BRAKE IOCTL_DATA_GETBRAKE;
typedef IOCTL_DATA_BRAKE IOCTL_DATA_SETBRAKE;

typedef struct SRV_COMM_SEND {
	long arm_no;
	uint32_t discontinuous;
	uint32_t disable_cur_lim;
	uint32_t stay_here;
	long position[JOINT_MAX];
	uint16_t current_limit[JOINT_MAX];
	int16_t current_offset[JOINT_MAX];
} SRV_COMM_SEND;

typedef struct SRV_COMM_SEND_RECV {
	long result;
	uint32_t buff_state;
} SRV_COMM_SEND_RECV;

typedef struct IOCTL_DATA_UPDATE {
	SRV_COMM_SEND send;
	SRV_COMM_SEND_RECV recv;
} IOCTL_DATA_UPDATE;

typedef struct SRV_COMM_RECV {
	long result;
	uint32_t time_stamp;
	long encoder[JOINT_MAX];
	uint16_t current_q[JOINT_MAX];
	uint16_t current_u[JOINT_MAX];
	uint16_t current_v[JOINT_MAX];
} SRV_COMM_RECV;

typedef struct IOCTL_DATA_GETENC {
	long arm_no;
	SRV_COMM_RECV recv;
} IOCTL_DATA_GETENC;

typedef struct IOCTL_DATA_GETSTATE {
	uint32_t index; /* IN  */
	long result;    /* OUT */
	long value;     /* OUT */
} IOCTL_DATA_GETSTATE;

typedef struct IOCTL_DATA_PUTSTATE {
	uint32_t index; /* IN  */
	long value;     /* IN  */
	long result;    /* OUT */
} IOCTL_DATA_PUTSTATE;

typedef struct IOCTL_DATA_GRIPPER_GETSTATE {
	long result;	/* OUT */
	long hold;	/* OUT */
} IOCTL_DATA_GRIPPER_GETSTATE;

typedef struct IOCTL_DATA_GET_ALLSTATE {
	long result;	/* OUT */
	uint32_t checkstate_num[ARM_MAX];
	uint32_t safety_checkstate_num;
	int motor_state;
	int safety_state;
	long brake_state[ARM_MAX][JOINT_MAX];
	long emgstop_state;
	long protectstop_state;
	long function_btn_state;
	long hand_open_btn_state;
	long hand_close_btn_state;
	long ip_reset_btn_state;
	long gripper_state;
	long fatal_error;
	long error;
	long safety_error;
	uint16_t mini_in;
	uint16_t mini_out;
} IOCTL_DATA_GET_ALLSTATE;

typedef struct IOCTL_DATA_SRVENABLE {
	long result; /* OUT */
	unsigned long enable;
} IOCTL_DATA_SRVENABLE;

typedef IOCTL_DATA_SRVENABLE IOCTL_DATA_GETSRVENABLE;
typedef IOCTL_DATA_SRVENABLE IOCTL_DATA_SETSRVENABLE;

typedef struct IOCTL_DATA_MINI_INPUT {
	long result;   /* OUT */
	uint16_t data; /* OUT */
} IOCTL_DATA_MINI_INPUT;

typedef struct IOCTL_DATA_MINI_OUTPUT {
	uint16_t data; /* IN  */
	uint16_t mask; /* IN  */
	long result;   /* OUT */
} IOCTL_DATA_MINI_OUTPUT;

typedef struct IOCTL_DATA_MINI_OUTPUT_READ {
	long result;   /* OUT */
	uint16_t data; /* OUT */
} IOCTL_DATA_MINI_OUTPUT_READ;

typedef struct {
	long value;
	long result; /* OUT */
} IOCTL_DATA_FRAM_ARG;

typedef struct {
	uint32_t start;
	uint32_t length;
	long result; /* OUT */
	uint8_t *data;
} IOCTL_DATA_FRAM_READ;

typedef struct {
	uint32_t start;
	uint32_t length;
	long result; /* OUT */
	uint8_t *data;
} IOCTL_DATA_FRAM_WRITE;

typedef struct {
	long result; /* OUT */
	long check;  /* OUT */
} IOCTL_DATA_FRAM_CHECK;

typedef struct {
	long index;
	long result; /* OUT */
	uint32_t size;
} IOCTL_DATA_FRAM_LENGTH;

/*!< COBOTTA acyclic communication data */
typedef struct SRV_CB_ACYCLIC_DATA {
	uint16_t address;
	uint16_t value;
} SRV_CB_ACYCLIC_DATA;

/*!< COBOTTA acyclic communication status */
typedef struct SRV_CB_ACYCLIC_STATUS {
	uint16_t send_status;
	uint16_t recv_status;
} SRV_CB_ACYCLIC_STATUS;

/*!< COBOTTA acyclic communication request (single/single) */
typedef struct SRV_CB_ACYCLIC_1_1_REQ {
	long arm_no;
	long joint_no;
	SRV_CB_ACYCLIC_DATA data;
} SRV_CB_ACYCLIC_1_1_REQ;

/*!< COBOTTA acyclic communication response (single/single) */
typedef struct SRV_CB_ACYCLIC_1_1_RESULT {
	long result; /* OUT */
	SRV_CB_ACYCLIC_STATUS status;
	SRV_CB_ACYCLIC_DATA data;
} SRV_CB_ACYCLIC_1_1_RESULT;

/*!< COBOTTA acyclic communication request (double/single) */
typedef struct SRV_CB_ACYCLIC_2_1_REQ {
	long arm_no;
	long joint_no;
	SRV_CB_ACYCLIC_DATA data[2];
	SRV_CB_ACYCLIC_DATA __dummy;
} SRV_CB_ACYCLIC_2_1_REQ;

/*!< COBOTTA acyclic communication response (double/single) */
typedef struct SRV_CB_ACYCLIC_2_1_RESULT {
	long result; /* OUT */
	SRV_CB_ACYCLIC_STATUS status;
	SRV_CB_ACYCLIC_DATA data[2];
} SRV_CB_ACYCLIC_2_1_RESULT;

/*!< COBOTTA acyclic communication request (single/all) */
typedef struct SRV_CB_ACYCLIC_1_ALL_REQ {
	SRV_CB_ACYCLIC_DATA data[9];
} SRV_CB_ACYCLIC_1_ALL_REQ;

/*!< COBOTTA acyclic communication response (single/all) */
typedef struct SRV_CB_ACYCLIC_1_ALL_RESULT {
	long result; /* OUT */
	SRV_CB_ACYCLIC_STATUS status[9];
	SRV_CB_ACYCLIC_DATA data[9];
} SRV_CB_ACYCLIC_1_ALL_RESULT;

/*!< COBOTTA acyclic communication request (double/all) */
typedef struct SRV_CB_ACYCLIC_2_ALL_REQ {
	SRV_CB_ACYCLIC_DATA data[2][9];
} SRV_CB_ACYCLIC_2_ALL_REQ;

/*!< COBOTTA acyclic communication response (double/all) */
typedef struct SRV_CB_ACYCLIC_2_ALL_RESULT {
	long result; /* OUT */
	SRV_CB_ACYCLIC_STATUS status[9];
	SRV_CB_ACYCLIC_DATA data[2][9];
} SRV_CB_ACYCLIC_2_ALL_RESULT;

typedef struct {
	long in_length;  /* IN  */
	long out_length; /* OUT */
	union {
		SRV_CB_ACYCLIC_1_1_REQ req_1_1;
		SRV_CB_ACYCLIC_2_1_REQ req_2_1;
		SRV_CB_ACYCLIC_1_ALL_REQ req_1_all;
		SRV_CB_ACYCLIC_2_ALL_REQ req_2_all;
	}; /* IN  */
	union {
		SRV_CB_ACYCLIC_1_1_RESULT result_1_1;
		SRV_CB_ACYCLIC_2_1_RESULT result_2_1;
		SRV_CB_ACYCLIC_1_ALL_RESULT result_1_all;
		SRV_CB_ACYCLIC_2_ALL_RESULT result_2_all;
	}; /* OUT */
} IOCTL_DATA_CB_ACYCLIC_COMM;

typedef struct {
	uint32_t code;
	long result; /* OUT */
} IOCTL_DATA_SAFETY_SEND;

typedef struct {
	long result; /* OUT */
	SRV_ERROR err;
} IOCTL_DATA_SAFETY_CHECKSTATE;

typedef struct {
	long enable; /* IN  */
	long result; /* OUT */
} IOCTL_DATA_ERRCODE_OUTPUT;

typedef struct {
	long result; /* OUT */
	uint32_t driver_ver;
	char fpga_ver[2][0x100];
	uint16_t servo_mcu_ver[9];
	uint16_t safety_mcu_ver[9][2];
} IOCTL_DATA_GET_VERSION;

#define COBOTTA_MAGIC 'f'

#define COBOTTA_IOCTL_SRV_CHECKSTATE                                           \
	_IOWR(COBOTTA_MAGIC, 4, IOCTL_DATA_CHECKSTATE)
#define COBOTTA_IOCTL_CLR_ERROR _IOR(COBOTTA_MAGIC, 5, IOCTL_DATA_RESULT)

#define COBOTTA_IOCTL_MOTOR_ON _IOR(COBOTTA_MAGIC, 8, IOCTL_DATA_RESULT)
#define COBOTTA_IOCTL_MOTOR_OFF _IOR(COBOTTA_MAGIC, 9, IOCTL_DATA_RESULT)
#define COBOTTA_IOCTL_MOTOR_STATE                                              \
	_IOR(COBOTTA_MAGIC, 10, IOCTL_DATA_MOTOR_STATE)

#define COBOTTA_IOCTL_SRV_GETBRAKE _IOWR(COBOTTA_MAGIC, 11, IOCTL_DATA_GETBRAKE)
#define COBOTTA_IOCTL_SRV_SETBRAKE _IOWR(COBOTTA_MAGIC, 12, IOCTL_DATA_SETBRAKE)

#define COBOTTA_IOCTL_SRV_UPDATE _IOWR(COBOTTA_MAGIC, 13, IOCTL_DATA_UPDATE)
#define COBOTTA_IOCTL_SRV_GETENC _IOWR(COBOTTA_MAGIC, 14, IOCTL_DATA_GETENC)

#define COBOTTA_IOCTL_SRV_GETSTATE _IOWR(COBOTTA_MAGIC, 16, IOCTL_DATA_GETSTATE)
#define COBOTTA_IOCTL_SRV_PUTSTATE _IOWR(COBOTTA_MAGIC, 17, IOCTL_DATA_PUTSTATE)

#define COBOTTA_IOCTL_GRIPPER_GETSTATE                                         \
	_IOR(COBOTTA_MAGIC, 18, IOCTL_DATA_GRIPPER_GETSTATE)
#define COBOTTA_IOCTL_GET_ALLSTATE                                         \
	_IOR(COBOTTA_MAGIC, 19, IOCTL_DATA_GET_ALLSTATE)

#define COBOTTA_IOCTL_SRV_GETSRVENABLE                                         \
	_IOR(COBOTTA_MAGIC, 96, IOCTL_DATA_GETSRVENABLE)
#define COBOTTA_IOCTL_SRV_SETSRVENABLE                                         \
	_IOWR(COBOTTA_MAGIC, 97, IOCTL_DATA_SETSRVENABLE)

#define COBOTTA_IOCTL_MINI_INPUT _IOR(COBOTTA_MAGIC, 100, IOCTL_DATA_MINI_INPUT)
#define COBOTTA_IOCTL_MINI_OUTPUT                                              \
	_IOWR(COBOTTA_MAGIC, 101, IOCTL_DATA_MINI_OUTPUT)
#define COBOTTA_IOCTL_MINI_OUTPUT_READ                                         \
	_IOR(COBOTTA_MAGIC, 102, IOCTL_DATA_MINI_OUTPUT_READ)

#define COBOTTA_IOCTL_FRAM_LOAD _IOWR(COBOTTA_MAGIC, 112, IOCTL_DATA_FRAM_ARG)
#define COBOTTA_IOCTL_FRAM_READ _IOWR(COBOTTA_MAGIC, 114, IOCTL_DATA_FRAM_READ)
#define COBOTTA_IOCTL_FRAM_WRITE                                               \
	_IOWR(COBOTTA_MAGIC, 115, IOCTL_DATA_FRAM_WRITE)
#define COBOTTA_IOCTL_FRAM_CHECK _IOR(COBOTTA_MAGIC, 116, IOCTL_DATA_FRAM_CHECK)

#define COBOTTA_IOCTL_FRAM_LENGTH                                              \
	_IOWR(COBOTTA_MAGIC, 118, IOCTL_DATA_FRAM_LENGTH)

#define COBOTTA_IOCTL_CB_ACYCLIC_COMM                                          \
	_IOWR(COBOTTA_MAGIC, 120, IOCTL_DATA_CB_ACYCLIC_COMM)

#define COBOTTA_IOCTL_SAFETY_SEND                                              \
	_IOWR(COBOTTA_MAGIC, 134, IOCTL_DATA_SAFETY_SEND)
#define COBOTTA_IOCTL_SAFETY_CHECKSTATE                                        \
	_IOR(COBOTTA_MAGIC, 140, IOCTL_DATA_SAFETY_CHECKSTATE)

#define COBOTTA_IOCTL_ERRCODE_OUTPUT                                           \
	_IOWR(COBOTTA_MAGIC, 199, IOCTL_DATA_ERRCODE_OUTPUT)

#define COBOTTA_IOCTL_GET_VERSION                                              \
	_IOR(COBOTTA_MAGIC, 200, IOCTL_DATA_GET_VERSION)

#endif /* _COBOTTA_IOCTL_H */
