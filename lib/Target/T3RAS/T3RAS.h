//===-- MBlaze.h - Top-level interface for MBlaze ---------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in
// the LLVM MBlaze back-end.
//
//===----------------------------------------------------------------------===//

#ifndef TARGET_T3RAS_H
#define TARGET_T3RAS_H

#include "MCTargetDesc/T3RASBaseInfo.h"
#include "MCTargetDesc/T3RASMCTargetDesc.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {
  class T3RASTargetMachine;
  class FunctionPass;
  class MachineCodeEmitter;

  FunctionPass *createT3RASISelDag(T3RASTargetMachine &TM);
  FunctionPass *createT3RASDelaySlotFillerPass(T3RASTargetMachine &TM);

} // end namespace llvm;

#endif
