//===-- MBlazeRegisterInfo.cpp - MBlaze Register Information --------------===//
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

#define DEBUG_TYPE "T3RAS-frame-info"

#include "T3RASRegisterInfo.h"
#include "T3RAS.h"
#include "T3RASSubtarget.h"
#include "T3RASMachineFunction.h"
#include "llvm/Constants.h"
#include "llvm/Type.h"
#include "llvm/Function.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/STLExtras.h"

#define GET_REGINFO_TARGET_DESC
#include "T3RASGenRegisterInfo.inc"

using namespace llvm;

T3RASRegisterInfo::
T3RASRegisterInfo(const T3RASSubtarget &ST, const TargetInstrInfo &tii)
  : T3RASGenRegisterInfo(T3RAS::R15), Subtarget(ST), TII(tii) {}

unsigned T3RASRegisterInfo::getPICCallReg() {
  return T3RAS::R20;
}

//===----------------------------------------------------------------------===//
// Callee Saved Registers methods
//===----------------------------------------------------------------------===//

/// MBlaze Callee Saved Registers
const uint16_t* T3RASRegisterInfo::
getCalleeSavedRegs(const MachineFunction *MF) const {
  // MBlaze callee-save register range is R20 - R31
  static const uint16_t CalleeSavedRegs[] = {
    T3RAS::R20, T3RAS::R21, T3RAS::R22, T3RAS::R23,
    T3RAS::R24, T3RAS::R25, T3RAS::R26, T3RAS::R27,
    T3RAS::R28, T3RAS::R29, T3RAS::R30, T3RAS::R31,
    0
  };

  return CalleeSavedRegs;
}

BitVector T3RASRegisterInfo::
getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  Reserved.set(T3RAS::R0);
  Reserved.set(T3RAS::R1);
  Reserved.set(T3RAS::R2);
  Reserved.set(T3RAS::R13);
  Reserved.set(T3RAS::R14);
  Reserved.set(T3RAS::R15);
  Reserved.set(T3RAS::R16);
  Reserved.set(T3RAS::R17);
  Reserved.set(T3RAS::R18);
  Reserved.set(T3RAS::R19);
  return Reserved;
}

// This function eliminate ADJCALLSTACKDOWN/ADJCALLSTACKUP pseudo instructions
void T3RASRegisterInfo::
eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I) const {
  const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();

  if (!TFI->hasReservedCallFrame(MF)) {
    // If we have a frame pointer, turn the adjcallstackup instruction into a
    // 'addi r1, r1, -<amt>' and the adjcallstackdown instruction into
    // 'addi r1, r1, <amt>'
    MachineInstr *Old = I;
    int Amount = Old->getOperand(0).getImm() + 4;
    if (Amount != 0) {
      // We need to keep the stack aligned properly.  To do this, we round the
      // amount of space needed for the outgoing arguments up to the next
      // alignment boundary.
      unsigned Align = TFI->getStackAlignment();
      Amount = (Amount+Align-1)/Align*Align;

      MachineInstr *New;
      if (Old->getOpcode() == T3RAS::ADJCALLSTACKDOWN) {
        New = BuildMI(MF,Old->getDebugLoc(),TII.get(T3RAS::ADDIK),T3RAS::R1)
                .addReg(T3RAS::R1).addImm(-Amount);
      } else {
        assert(Old->getOpcode() == T3RAS::ADJCALLSTACKUP);
        New = BuildMI(MF,Old->getDebugLoc(),TII.get(T3RAS::ADDIK),T3RAS::R1)
                .addReg(T3RAS::R1).addImm(Amount);
      }

      // Replace the pseudo instruction with a new instruction...
      MBB.insert(I, New);
    }
  }

  // Simply discard ADJCALLSTACKDOWN, ADJCALLSTACKUP instructions.
  MBB.erase(I);
}

// FrameIndex represent objects inside a abstract stack.
// We must replace FrameIndex with an stack/frame pointer
// direct reference.
void T3RASRegisterInfo::
eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj,
                    RegScavenger *RS) const {
  MachineInstr &MI = *II;
  MachineFunction &MF = *MI.getParent()->getParent();
  MachineFrameInfo *MFI = MF.getFrameInfo();

  unsigned i = 0;
  while (!MI.getOperand(i).isFI()) {
    ++i;
    assert(i < MI.getNumOperands() &&
           "Instr doesn't have FrameIndex operand!");
  }

  unsigned oi = i == 2 ? 1 : 2;

  DEBUG(dbgs() << "\nFunction : " << MF.getFunction()->getName() << "\n";
        dbgs() << "<--------->\n" << MI);

  int FrameIndex = MI.getOperand(i).getIndex();
  int stackSize  = MFI->getStackSize();
  int spOffset   = MFI->getObjectOffset(FrameIndex);

  DEBUG(T3RASFunctionInfo *T3RASFI = MF.getInfo<T3RASFunctionInfo>();
        dbgs() << "FrameIndex : " << FrameIndex << "\n"
               << "spOffset   : " << spOffset << "\n"
               << "stackSize  : " << stackSize << "\n"
               << "isFixed    : " << MFI->isFixedObjectIndex(FrameIndex) << "\n"
               << "isLiveIn   : " << T3RASFI->isLiveIn(FrameIndex) << "\n"
               << "isSpill    : " << MFI->isSpillSlotObjectIndex(FrameIndex)
               << "\n" );

  // as explained on LowerFormalArguments, detect negative offsets
  // and adjust SPOffsets considering the final stack size.
  int Offset = (spOffset < 0) ? (stackSize - spOffset) : spOffset;
  Offset += MI.getOperand(oi).getImm();

  DEBUG(dbgs() << "Offset     : " << Offset << "\n" << "<--------->\n");

  MI.getOperand(oi).ChangeToImmediate(Offset);
  MI.getOperand(i).ChangeToRegister(getFrameRegister(MF), false);
}

void T3RASRegisterInfo::
processFunctionBeforeFrameFinalized(MachineFunction &MF) const {
  // Set the stack offset where GP must be saved/loaded from.
  MachineFrameInfo *MFI = MF.getFrameInfo();
  T3RASFunctionInfo *T3RASFI = MF.getInfo<T3RASFunctionInfo>();
  if (T3RASFI->needGPSaveRestore())
    MFI->setObjectOffset(T3RASFI->getGPFI(), T3RASFI->getGPStackOffset());
}

unsigned T3RASRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();

  return TFI->hasFP(MF) ? T3RAS::R19 : T3RAS::R1;
}

unsigned T3RASRegisterInfo::getEHExceptionRegister() const {
  llvm_unreachable("What is the exception register");
}

unsigned T3RASRegisterInfo::getEHHandlerRegister() const {
  llvm_unreachable("What is the exception handler register");
}
