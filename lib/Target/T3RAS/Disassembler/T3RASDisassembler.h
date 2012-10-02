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

//===-- MBlazeDisassembler.h - Disassembler for MicroBlaze  -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file is part of the MBlaze Disassembler. It it the header for
// MBlazeDisassembler, a subclass of MCDisassembler.
//
//===----------------------------------------------------------------------===//

#ifndef T3RASDISASSEMBLER_H
#define T3RASDISASSEMBLER_H

#include "llvm/MC/MCDisassembler.h"

namespace llvm {
  
class MCInst;
class MemoryObject;
class raw_ostream;

struct EDInstInfo;
  
/// MBlazeDisassembler - Disassembler for all MBlaze platforms.
class T3RASDisassembler : public MCDisassembler {
public:
  /// Constructor     - Initializes the disassembler.
  ///
  T3RASDisassembler(const MCSubtargetInfo &STI) :
    MCDisassembler(STI) {
  }

  ~T3RASDisassembler() {
  }

  /// getInstruction - See MCDisassembler.
  MCDisassembler::DecodeStatus getInstruction(MCInst &instr,
                      uint64_t &size,
                      const MemoryObject &region,
                      uint64_t address,
                      raw_ostream &vStream,
                      raw_ostream &cStream) const;

  /// getEDInfo - See MCDisassembler.
  const EDInstInfo *getEDInfo() const;
};

} // namespace llvm
  
#endif
