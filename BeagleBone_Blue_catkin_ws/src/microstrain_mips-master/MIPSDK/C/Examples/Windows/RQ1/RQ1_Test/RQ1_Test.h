/////////////////////////////////////////////////////////////////////////////
//
// RQ1_Test.h 
//
// Test program for the RQ1
//
// Notes:  This program runs through most of the sdk functions supported
//         by the RQ1.  It does not permanently alter any of the device
//         settings.
//
//
// Written By: Nathan Miller and Gregg Carpenter
// 
//!@copyright 2014 Lord Microstrain Sensing Systems. 
//
//!@section CHANGES
//! 
//
//!@section LICENSE
//!
//! THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING 
//! CUSTOMERS WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER 
//! FOR THEM TO SAVE TIME. AS A RESULT, LORD MICROSTRAIN SENSING SYSTEMS
//! SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES 
//! WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT OF SUCH SOFTWARE AND/OR 
//! THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONNECTION 
//! WITH THEIR PRODUCTS.
//
/////////////////////////////////////////////////////////////////////////////


#ifndef _RQ1_TEST_H
#define _RQ1_TEST_H

///////////////////////////////////////////////////////////////////////////////
//
// Includes
//
///////////////////////////////////////////////////////////////////////////////

#include "mip_sdk.h"
#include "byteswap_utilities.h"
#include "mip_rq1_imu.h"
#include "mip_rq1.h"
#include <stdio.h>
#include <windows.h>

///////////////////////////////////////////////////////////////////////////////
//
// Defines
//
///////////////////////////////////////////////////////////////////////////////

#define MIP_SDK_RQ1_IMU_STANDARD_MODE	0x01
#define MIP_SDK_RQ1_IMU_DIRECT_MODE	0x02

#define NUM_COMMAND_LINE_ARGUMENTS 3

#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds

////////////////////////////////////////////////////////////////////////////////
//
// Function Prototypes
//
////////////////////////////////////////////////////////////////////////////////

//Help Functions
void print_header();
void print_command_line_usage();
void print_packet_stats();

//MIP Parser Packet Callback Functions
void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
void gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);

//Hardware specific status functions
u16 mip_3dm_cmd_hw_specific_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer);
u16 mip_3dm_cmd_hw_specific_imu_device_status(mip_interface *device_interface, u16 model_number, u8 status_selector, u8 *response_buffer);

#endif
