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
