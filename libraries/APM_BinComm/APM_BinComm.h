// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-
//
// Copyright (c) 2010 Michael Smith. All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//	  notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//	  notice, this list of conditions and the following disclaimer in the
//	  documentation and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.	IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.

/// @file		BinComm.h
/// @brief		Definitions for the ArduPilot Mega binary communications
///				library.

#ifndef APM_BinComm_h
#define APM_BinComm_h

#include <string.h>
#include <inttypes.h>
#include "WProgram.h"

///
/// @class		BinComm
/// @brief		Class providing protocol en/decoding services for the ArduPilot
///				Mega binary telemetry protocol.
///
/// The protocol definition, including structures describing
/// messages, MessageID values and helper functions for sending
/// and unpacking messages are automatically generated.
///
/// See protocol/protocol.def for a description of the message
/// definitions, and protocol/protocol.h for the generated
/// definitions.
///
/// Protocol messages are sent using the send_* functions defined in
/// protocol/protocol.h, and handled on reception by functions defined
/// in the handlerTable array passed to the constructor.
///
class BinComm {
public:
	struct MessageHandler;

	//////////////////////////////////////////////////////////////////////
	/// Constructor.
	///
	/// @param handlerTable			Array of callout functions to which
	///								received messages will be sent.	 More than
	///								one handler for a given messageID may be
	///								registered; handlers are called in the order
	///								they appear in the table.  A single handler
	///								may be registered for more than one message,
	///								as the message ID is passed to the handler
	///								when it is received.
	///
	/// @param interface			The stream that will be used
	///								for telemetry communications.
	///
	/// @param rxBuffSize		    Size of receive buffer allocated by interface.
	///								This is used to warn for buffer overflow.
	///

	///
	BinComm(const MessageHandler *handlerTable,
			Stream *interface = NULL);

	///
	/// Optional initialiser.
	///
	/// If the interface stream isn't known at construction time, it
	/// can be set here instead.
	///
	/// @param interface			The stream that will be used for telemetry
	///								communications.
	///
	void			init(Stream *interface);

private:
	/// OTA message header
	struct MessageHeader {
        uint8_t         length;
        uint8_t         messageID;
        uint8_t         messageVersion;
	};

	/// Incoming header/packet buffer
	/// XXX we could make this smaller
	union {
		uint8_t					bytes[0];
		MessageHeader			header;
		uint8_t					payload[256];
	} _decodeBuf;

	//////////////////////////////////////////////////////////////////////
	/// @name		Message pack/unpack utility functions
	///
	//@{

	/// Emit any scalar type.
	///
	/// @param x				Value to emit.
	///
	template <typename T> inline void _emit(const T x) { _send(&x, sizeof(T)); }

	/// Emit an array of any scalar type.
	///
	/// @param x				Array to emit.
	/// @param count			Array size.
	///
	template <typename T> inline void _emit(const T *values, uint8_t count) { _send(values, count * sizeof(T)); }
	

	/// Emit a fixed-size string, from a NUL-terminated buffer.
	///
	/// The string is NUL-padded if the buffer is larger than the string.
	///
	/// @param msg				The NUL-terminated string to emit.
	/// @param size				The maximum length of the string to emit.
	///
	inline void	_emit(const char *msg, uint8_t size) { 
		while (size--) {
			char	c = *msg;
			_emit(c);
			if (0 != c)
				msg++;
		}
	}

	/// Unpack any scalar type.
	///
	/// @param buf				Buffer pointer.
	/// @param x				Unpacked result.
	///
	template <typename T> inline void _unpack(uint8_t *&ptr, T &x) { x = *(T *)ptr; ptr += sizeof(T); }

	/// Unpack an array of any scalar type.
	///
	/// @param buf				Buffer pointer.
	/// @param values			Array to receive the unpacked values.
	///
	template <typename T> inline void _unpack(uint8_t *&ptr, T *values, uint8_t count) {
		memcpy(values, ptr, count * sizeof(T)); 
		ptr += count * sizeof(T); 
	}

	/// Unpack a string from a fixed-size buffer.
	///
	/// @param ptr				Buffer pointer.
	/// @param msg				Pointer to the result buffer.
	/// @param size				The size of the buffer.
	///
	inline void	_unpack(uint8_t *&ptr, char *msg, uint8_t size) {
		strncpy(msg, (char *)ptr, size); msg[size-1] = '\0'; ptr += size;
	}
	//@}

public:
	//////////////////////////////////////////////////////////////////////
	/// @name		Protocol definition
	///
	//@{
#include "protocol/protocol.h"
	//@}

	//////////////////////////////////////////////////////////////////////
	/// @name		Protocol magic numbers
	///
	/// @note The MessageID enum is automatically generated and thus not described here.
	/// 
	//@{
	
	/// Message serverities
	enum severities
	{
		SEVERITY_LOW 		= 1,
		SEVERITY_MEDIUM 	= 2,
		SEVERITY_HIGH 		= 3,
		SEVERITY_CRITICAL 	= 4,
	};

	/// Variables defined
	/// XXX these should probably be handled by the database/MIB?
	enum variableID {
		MSG_VAR_ROLL_MODE		= 0x00,
		MSG_VAR_PITCH_MODE		= 0x01,
		MSG_VAR_THROTTLE_MODE	= 0x02,
		MSG_VAR_YAW_MODE		= 0x03,
		MSG_VAR_ELEVON_TRIM_1	= 0x04,
		MSG_VAR_ELEVON_TRIM_2	= 0x05,

		MSG_VAR_INTEGRATOR_0	= 0x10,
		MSG_VAR_INTEGRATOR_1	= 0x11,
		MSG_VAR_INTEGRATOR_2	= 0x12,
		MSG_VAR_INTEGRATOR_3	= 0x13,
		MSG_VAR_INTEGRATOR_4	= 0x14,
		MSG_VAR_INTEGRATOR_5	= 0x15,
		MSG_VAR_INTEGRATOR_6	= 0x16,
		MSG_VAR_INTEGRATOR_7	= 0x17,

		MSG_VAR_KFF_0			= 0x1a,
		MSG_VAR_KFF_1			= 0x1b,
		MSG_VAR_KFF_2			= 0x1c,

		MSG_VAR_TARGET_BEARING	= 0x20,
		MSG_VAR_NAV_BEARING		= 0x21,
		MSG_VAR_BEARING_ERROR	= 0x22,
		MSG_VAR_CROSSTRACK_BEARING = 0x23,
		MSG_VAR_CROSSTRACK_ERROR = 0x24,
		MSG_VAR_ALTITUDE_ERROR	= 0x25,
		MSG_VAR_WP_RADIUS		= 0x26,
		MSG_VAR_LOITER_RADIUS	= 0x27,
		MSG_VAR_WP_MODE			= 0x28,
		MSG_VAR_LOOP_COMMANDS	= 0x29,
		MSG_VAR_NAV_GAIN_SCALER = 0x2a,
	};

	/// PID sets defined
	enum PIDSet {
		MSG_SERVO_ROLL			= 0,
		MSG_SERVO_PITCH			= 1,
		MSG_SERVO_RUDDER		= 2,
		MSG_SERVO_NAV_ROLL		= 3,
		MSG_SERVO_NAV_PITCH_ASP	= 4,
		MSG_SERVO_NAV_PITCH_ALT	= 5,
		MSG_SERVO_TE_THROTTLE	= 6,
		MSG_SERVO_ALT_THROTTLE	= 7,
		MSG_SERVO_ELEVATOR		= 8  // Added by Randy
	};

	//@}

	//////////////////////////////////////////////////////////////////////
	/// Message reception callout descriptor
	///
	/// An array of these handlers is passed to the constructor to delegate
	/// processing for received messages.
	///
	struct MessageHandler {
		MessageID		messageID;						///< messageID for which the handler will be called
		void			(* handler)(void *arg,
									uint8_t messageId, 
									uint8_t messageVersion,
									void *messageData); ///< function to be called
		void			*arg;							///< argument passed to function
	};

	//////////////////////////////////////////////////////////////////////
	/// @name		Decoder interface
	//@{

	/// Consume bytes from the interface and feed them to the decoder.
	///
	/// If a packet is completed, then any callbacks associated
	/// with the packet's messageID will be called.
	///
	/// If no bytes are passed to the decoder for a period determined
	/// by DEC_MESSAGE_TIMEOUT, the decode state machine will reset
	/// before processing the next byte.  This can help re-synchronise
	/// after a link loss or in-flight failure.
	///

	void					update(void);

	uint32_t				messagesReceived;		///< statistics
	uint32_t				badMessagesReceived;	///< statistics

	//@}

	//////////////////////////////////////////////////////////////////////
	/// @name		Encoder interface
	///
	///	Messages are normally encoded and sent using the
	/// send_msg_* functions defined in protocol/protocol.h.
	/// For each message type MSG_* there is a corresponding send_msg_*
	/// function which will construct and transmit the message.
	///
	//@{
	uint32_t				messagesSent;			///< statistics
	//@}


private:
	const MessageHandler	*_handlerTable; ///< callout table
	Stream					*_interface;	///< Serial port we send/receive using.

	/// Various magic numbers
	enum MagicNumbers {
		MSG_PREAMBLE_1			= 0x34,
		MSG_PREAMBLE_2			= 0x44,
		MSG_VERSION_1			= 1,
		MSG_VARIABLE_LENGTH		= 0xff
	};

	//////////////////////////////////////////////////////////////////////
	/// @name		Decoder state
	//@{
	uint8_t					_decodePhase;	///< decoder state machine phase
	uint8_t					_bytesIn;		///< bytes received in the current phase
	uint8_t					_bytesExpected; ///< bytes expected in the current phase
	uint8_t					_decoderSumA;	///< sum of incoming bytes
	uint8_t					_decoderSumB;	///< sum of _sumA values

	uint8_t					_messageID;		///< messageID from the packet being received
	uint8_t					_messageVersion;///< messageVersion from the packet being received

	unsigned long			_lastReceived;	///< timestamp of last byte reception
	//@}

	/// Decoder state machine.
	///
	/// @param inByte		  The byte to process.
	///
	void					_decode(uint8_t inByte);

	//////////////////////////////////////////////////////////////////////
	/// @name		Encoder state
	//@{
	uint8_t					_encoderSumA;	///< sum of outgoing bytes
	uint8_t					_encoderSumB;	///< sum of _sumA values
	//@}

	/// Start transmitting a message.
	///
	/// @param	messageId		The ID of the message to be sent
	/// @param	messageLength	The protocol-defined length of the message in bytes
	/// @param	messageVersion	The message version (optional)
	///
	void					_startMessage(uint8_t messageId, uint8_t messageLength, uint8_t messageVersion = 1);

	/// Send bytes as part of a message.
	///
	/// @param	bytes			Pointer to the byte(s) to send.
	/// @param	count			Count of bytes to send.
	void					_send(const void *bytes, uint8_t count);

	/// Finalise message transmission.
	///
	void					_endMessage(void);
};

#endif // BinComm_h
