/*
Copyright (c) 2012, Aeste Works (M) Sdn Bhd.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the
   distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//===-- MBlazeMCTargetDesc.h - MBlaze Target Descriptions -------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides MBlaze specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef T3RASMCTARGETDESC_H
#define T3RASMCTARGETDESC_H

#include "llvm/Support/DataTypes.h"

namespace llvm {
class MCAsmBackend;
class MCContext;
class MCCodeEmitter;
class MCInstrInfo;
class MCObjectWriter;
class MCSubtargetInfo;
class Target;
class StringRef;
class raw_ostream;

extern Target TheT3RASTarget;

MCCodeEmitter *createT3RASMCCodeEmitter(const MCInstrInfo &MCII,
                                         const MCSubtargetInfo &STI,
                                         MCContext &Ctx);

MCAsmBackend *createT3RASAsmBackend(const Target &T, StringRef TT);

MCObjectWriter *createT3RASELFObjectWriter(raw_ostream &OS, uint8_t OSABI);
} // End llvm namespace

// Defines symbolic names for MBlaze registers.  This defines a mapping from
// register name to register number.
#define GET_REGINFO_ENUM
#include "T3RASGenRegisterInfo.inc"

// Defines symbolic names for the MBlaze instructions.
#define GET_INSTRINFO_ENUM
#include "T3RASGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "T3RASGenSubtargetInfo.inc"

#endif
