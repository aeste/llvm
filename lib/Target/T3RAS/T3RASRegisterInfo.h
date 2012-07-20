//===-- MBlazeRegisterInfo.h - MBlaze Register Information Impl -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MBlaze implementation of the TargetRegisterInfo
// class.
//
//===----------------------------------------------------------------------===//

#ifndef T3RASREGISTERINFO_H
#define T3RASREGISTERINFO_H

#include "T3RAS.h"
#include "llvm/Target/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "T3RASGenRegisterInfo.inc"

namespace llvm {
class T3RASSubtarget;
class TargetInstrInfo;
class Type;

namespace T3RAS {
  /// SubregIndex - The index of various sized subregister classes. Note that
  /// these indices must be kept in sync with the class indices in the
  /// MBlazeRegisterInfo.td file.
  enum SubregIndex {
    SUBREG_FPEVEN = 1, SUBREG_FPODD = 2
  };
}

struct T3RASRegisterInfo : public T3RASGenRegisterInfo {
  const T3RASSubtarget &Subtarget;
  const TargetInstrInfo &TII;

  T3RASRegisterInfo(const T3RASSubtarget &Subtarget,
                     const TargetInstrInfo &tii);

  /// Get PIC indirect call register
  static unsigned getPICCallReg();

  /// Code Generation virtual methods...
  const uint16_t *getCalleeSavedRegs(const MachineFunction* MF = 0) const;

  BitVector getReservedRegs(const MachineFunction &MF) const;

  void eliminateCallFramePseudoInstr(MachineFunction &MF,
                                     MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator I) const;

  /// Stack Frame Processing Methods
  void eliminateFrameIndex(MachineBasicBlock::iterator II,
                           int SPAdj, RegScavenger *RS = NULL) const;

  void processFunctionBeforeFrameFinalized(MachineFunction &MF) const;

  /// Debug information queries.
  unsigned getFrameRegister(const MachineFunction &MF) const;

  /// Exception handling queries.
  unsigned getEHExceptionRegister() const;
  unsigned getEHHandlerRegister() const;
};

} // end namespace llvm

#endif
