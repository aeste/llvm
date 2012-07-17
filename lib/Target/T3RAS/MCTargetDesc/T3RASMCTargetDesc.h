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
