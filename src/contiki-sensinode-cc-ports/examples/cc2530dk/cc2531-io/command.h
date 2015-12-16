/*
 * Copyright (c) 2015, Jie Zeng.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#ifndef CMD_H_
#define CMD_H_

/* Server raises command to client. */
/*
 * | magic (1B) | opcode (1B) | data |
 */

#define CMD_NULL		0x00
#define CMD_GET_INTERVAL	0x01
#define CMD_INC_INTERVAL	0x02
#define CMD_DES_INTERVAL	0x03

#define MAGIC_LEN		1
#define OPCODE_LEN		1
#define DEFAULT_HEADER_LEN	(MAGIC_LEN + OPCODE_LEN)

#define CMD_GET_INTERVAL_LEN	DEFAULT_HEADER_LEN
#define CMD_INC_INTERVAL_LEN	DEFAULT_HEADER_LEN + 1
#define CMD_DES_INTERVAL_LEN	DEFAULT_HEADER_LEN + 1

#define CMD_MAGIC_NUM		0xBC

/* Client raises message to server. */
#define MSG_NULL		0x00
#define MSG_INFO		0x01
#define MSG_INFO_ID		0x02

#endif // CMD_H_
