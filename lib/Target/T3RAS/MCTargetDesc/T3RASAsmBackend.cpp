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

//===-- MBlazeAsmBackend.cpp - MBlaze Assembler Backend -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T3RASMCTargetDesc.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCELFSymbolFlags.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSectionMachO.h"
#include "llvm/MC/MCValue.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Support/ELF.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

static unsigned getFixupKindSize(unsigned Kind) {
  switch (Kind) {
  default: llvm_unreachable("invalid fixup kind!");
  case FK_Data_1: return 1;
  case FK_PCRel_2:
  case FK_Data_2: return 2;
  case FK_PCRel_4:
  case FK_Data_4: return 4;
  case FK_Data_8: return 8;
  }
}


namespace {

class T3RASAsmBackend : public MCAsmBackend {
public:
  T3RASAsmBackend(const Target &T)
    : MCAsmBackend() {
  }

  unsigned getNumFixupKinds() const {
    return 2;
  }

  bool mayNeedRelaxation(const MCInst &Inst) const;

  bool fixupNeedsRelaxation(const MCFixup &Fixup,
                            uint64_t Value,
                            const MCInstFragment *DF,
                            const MCAsmLayout &Layout) const;

  void relaxInstruction(const MCInst &Inst, MCInst &Res) const;

  bool writeNopData(uint64_t Count, MCObjectWriter *OW) const;

  unsigned getPointerSize() const {
    return 4;
  }
};

static unsigned getRelaxedOpcode(unsigned Op) {
    switch (Op) {
    default:            return Op;
    case T3RAS::ADDIK: return T3RAS::ADDIK32;
    case T3RAS::ORI:   return T3RAS::ORI32;
    case T3RAS::BRLID: return T3RAS::BRLID32;
    }
}

bool T3RASAsmBackend::mayNeedRelaxation(const MCInst &Inst) const {
  if (getRelaxedOpcode(Inst.getOpcode()) == Inst.getOpcode())
    return false;

  bool hasExprOrImm = false;
  for (unsigned i = 0; i < Inst.getNumOperands(); ++i)
    hasExprOrImm |= Inst.getOperand(i).isExpr();

  return hasExprOrImm;
}

bool T3RASAsmBackend::fixupNeedsRelaxation(const MCFixup &Fixup,
                                            uint64_t Value,
                                            const MCInstFragment *DF,
                                            const MCAsmLayout &Layout) const {
  // FIXME: Is this right? It's what the "generic" code was doing before,
  // but is X86 specific. Is it actually true for MBlaze also, or was it
  // just close enough to not be a big deal?
  //
  // Relax if the value is too big for a (signed) i8.
  return int64_t(Value) != int64_t(int8_t(Value));
}

void T3RASAsmBackend::relaxInstruction(const MCInst &Inst, MCInst &Res) const {
  Res = Inst;
  Res.setOpcode(getRelaxedOpcode(Inst.getOpcode()));
}

bool T3RASAsmBackend::writeNopData(uint64_t Count, MCObjectWriter *OW) const {
  if ((Count % 4) != 0)
    return false;

  for (uint64_t i = 0; i < Count; i += 4)
      OW->Write32(0x00000000);

  return true;
}
} // end anonymous namespace

namespace {
class ELFT3RASAsmBackend : public T3RASAsmBackend {
public:
  uint8_t OSABI;
  ELFT3RASAsmBackend(const Target &T, uint8_t _OSABI)
    : T3RASAsmBackend(T), OSABI(_OSABI) { }

  void applyFixup(const MCFixup &Fixup, char *Data, unsigned DataSize,
                  uint64_t Value) const;

  MCObjectWriter *createObjectWriter(raw_ostream &OS) const {
    return createT3RASELFObjectWriter(OS, OSABI);
  }
};

void ELFT3RASAsmBackend::applyFixup(const MCFixup &Fixup, char *Data,
                                     unsigned DataSize, uint64_t Value) const {
  unsigned Size = getFixupKindSize(Fixup.getKind());

  assert(Fixup.getOffset() + Size <= DataSize &&
         "Invalid fixup offset!");

  char *data = Data + Fixup.getOffset();
  switch (Size) {
  default: llvm_unreachable("Cannot fixup unknown value.");
  case 1:  llvm_unreachable("Cannot fixup 1 byte value.");
  case 8:  llvm_unreachable("Cannot fixup 8 byte value.");

  case 4:
    *(data+7) = uint8_t(Value);
    *(data+6) = uint8_t(Value >> 8);
    *(data+3) = uint8_t(Value >> 16);
    *(data+2) = uint8_t(Value >> 24);
    break;

  case 2:
    *(data+3) = uint8_t(Value >> 0);
    *(data+2) = uint8_t(Value >> 8);
  }
}
} // end anonymous namespace

MCAsmBackend *llvm::createT3RASAsmBackend(const Target &T, StringRef TT) {
  Triple TheTriple(TT);

  if (TheTriple.isOSDarwin())
    assert(0 && "Mac not supported on T3RAS");

  if (TheTriple.isOSWindows())
    assert(0 && "Windows not supported on T3RAS");

  uint8_t OSABI = MCELFObjectTargetWriter::getOSABI(TheTriple.getOS());
  return new ELFT3RASAsmBackend(T, OSABI);
}
