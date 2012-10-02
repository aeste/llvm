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

//===-- MBlazeInstrInfo.cpp - MBlaze Instruction Information --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MBlaze implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "T3RASInstrInfo.h"
#include "T3RASTargetMachine.h"
#include "T3RASMachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/ScoreboardHazardRecognizer.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/ADT/STLExtras.h"

#define GET_INSTRINFO_CTOR
#include "T3RASGenInstrInfo.inc"

using namespace llvm;

T3RASInstrInfo::T3RASInstrInfo(T3RASTargetMachine &tm)
  : T3RASGenInstrInfo(T3RAS::ADJCALLSTACKDOWN, T3RAS::ADJCALLSTACKUP),
    TM(tm), RI(*TM.getSubtargetImpl(), *this) {}

static bool isZeroImm(const MachineOperand &op) {
  return op.isImm() && op.getImm() == 0;
}

/// isLoadFromStackSlot - If the specified machine instruction is a direct
/// load from a stack slot, return the virtual or physical register number of
/// the destination along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than loading from the stack slot.
unsigned T3RASInstrInfo::
isLoadFromStackSlot(const MachineInstr *MI, int &FrameIndex) const {
  if (MI->getOpcode() == T3RAS::LWI) {
    if ((MI->getOperand(1).isFI()) && // is a stack slot
        (MI->getOperand(2).isImm()) &&  // the imm is zero
        (isZeroImm(MI->getOperand(2)))) {
      FrameIndex = MI->getOperand(1).getIndex();
      return MI->getOperand(0).getReg();
    }
  }

  return 0;
}

/// isStoreToStackSlot - If the specified machine instruction is a direct
/// store to a stack slot, return the virtual or physical register number of
/// the source reg along with the FrameIndex of the loaded stack slot.  If
/// not, return 0.  This predicate must return 0 if the instruction has
/// any side effects other than storing to the stack slot.
unsigned T3RASInstrInfo::
isStoreToStackSlot(const MachineInstr *MI, int &FrameIndex) const {
  if (MI->getOpcode() == T3RAS::SWI) {
    if ((MI->getOperand(1).isFI()) && // is a stack slot
        (MI->getOperand(2).isImm()) &&  // the imm is zero
        (isZeroImm(MI->getOperand(2)))) {
      FrameIndex = MI->getOperand(1).getIndex();
      return MI->getOperand(0).getReg();
    }
  }
  return 0;
}

/// insertNoop - If data hazard condition is found insert the target nop
/// instruction.
void T3RASInstrInfo::
insertNoop(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI) const {
  DebugLoc DL;
  BuildMI(MBB, MI, DL, get(T3RAS::NOP));
}

void T3RASInstrInfo::
copyPhysReg(MachineBasicBlock &MBB,
            MachineBasicBlock::iterator I, DebugLoc DL,
            unsigned DestReg, unsigned SrcReg,
            bool KillSrc) const {
  llvm::BuildMI(MBB, I, DL, get(T3RAS::ADDK), DestReg)
    .addReg(SrcReg, getKillRegState(KillSrc)).addReg(T3RAS::R0);
}

void T3RASInstrInfo::
storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                    unsigned SrcReg, bool isKill, int FI,
                    const TargetRegisterClass *RC,
                    const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  BuildMI(MBB, I, DL, get(T3RAS::SWI)).addReg(SrcReg,getKillRegState(isKill))
    .addFrameIndex(FI).addImm(0); //.addFrameIndex(FI);
}

void T3RASInstrInfo::
loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     unsigned DestReg, int FI,
                     const TargetRegisterClass *RC,
                     const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  BuildMI(MBB, I, DL, get(T3RAS::LWI), DestReg)
      .addFrameIndex(FI).addImm(0); //.addFrameIndex(FI);
}

//===----------------------------------------------------------------------===//
// Branch Analysis
//===----------------------------------------------------------------------===//
bool T3RASInstrInfo::AnalyzeBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *&TBB,
                                    MachineBasicBlock *&FBB,
                                    SmallVectorImpl<MachineOperand> &Cond,
                                    bool AllowModify) const {
  // If the block has no terminators, it just falls into the block after it.
  MachineBasicBlock::iterator I = MBB.end();
  if (I == MBB.begin())
    return false;
  --I;
  while (I->isDebugValue()) {
    if (I == MBB.begin())
      return false;
    --I;
  }
  if (!isUnpredicatedTerminator(I))
    return false;

  // Get the last instruction in the block.
  MachineInstr *LastInst = I;

  // If there is only one terminator instruction, process it.
  unsigned LastOpc = LastInst->getOpcode();
  if (I == MBB.begin() || !isUnpredicatedTerminator(--I)) {
    if (T3RAS::isUncondBranchOpcode(LastOpc)) {
      TBB = LastInst->getOperand(0).getMBB();
      return false;
    }
    if (T3RAS::isCondBranchOpcode(LastOpc)) {
      // Block ends with fall-through condbranch.
      TBB = LastInst->getOperand(1).getMBB();
      Cond.push_back(MachineOperand::CreateImm(LastInst->getOpcode()));
      Cond.push_back(LastInst->getOperand(0));
      return false;
    }
    // Otherwise, don't know what this is.
    return true;
  }

  // Get the instruction before it if it's a terminator.
  MachineInstr *SecondLastInst = I;

  // If there are three terminators, we don't know what sort of block this is.
  if (SecondLastInst && I != MBB.begin() && isUnpredicatedTerminator(--I))
    return true;

  // If the block ends with something like BEQID then BRID, handle it.
  if (T3RAS::isCondBranchOpcode(SecondLastInst->getOpcode()) &&
      T3RAS::isUncondBranchOpcode(LastInst->getOpcode())) {
    TBB = SecondLastInst->getOperand(1).getMBB();
    Cond.push_back(MachineOperand::CreateImm(SecondLastInst->getOpcode()));
    Cond.push_back(SecondLastInst->getOperand(0));
    FBB = LastInst->getOperand(0).getMBB();
    return false;
  }

  // If the block ends with two unconditional branches, handle it.
  // The second one is not executed, so remove it.
  if (T3RAS::isUncondBranchOpcode(SecondLastInst->getOpcode()) &&
      T3RAS::isUncondBranchOpcode(LastInst->getOpcode())) {
    TBB = SecondLastInst->getOperand(0).getMBB();
    I = LastInst;
    if (AllowModify)
      I->eraseFromParent();
    return false;
  }

  // Otherwise, can't handle this.
  return true;
}

unsigned T3RASInstrInfo::
InsertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
             MachineBasicBlock *FBB,
             const SmallVectorImpl<MachineOperand> &Cond,
             DebugLoc DL) const {
  // Shouldn't be a fall through.
  assert(TBB && "InsertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 2 || Cond.size() == 0) &&
         "T3RAS branch conditions have two components!");

  unsigned Opc = T3RAS::BRID;
  if (!Cond.empty())
    Opc = (unsigned)Cond[0].getImm();

  if (FBB == 0) {
    if (Cond.empty()) // Unconditional branch
      BuildMI(&MBB, DL, get(Opc)).addMBB(TBB);
    else              // Conditional branch
      BuildMI(&MBB, DL, get(Opc)).addReg(Cond[1].getReg()).addMBB(TBB);
    return 1;
  }

  BuildMI(&MBB, DL, get(Opc)).addReg(Cond[1].getReg()).addMBB(TBB);
  BuildMI(&MBB, DL, get(T3RAS::BRID)).addMBB(FBB);
  return 2;
}

unsigned T3RASInstrInfo::RemoveBranch(MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator I = MBB.end();
  if (I == MBB.begin()) return 0;
  --I;
  while (I->isDebugValue()) {
    if (I == MBB.begin())
      return 0;
    --I;
  }

  if (!T3RAS::isUncondBranchOpcode(I->getOpcode()) &&
      !T3RAS::isCondBranchOpcode(I->getOpcode()))
    return 0;

  // Remove the branch.
  I->eraseFromParent();

  I = MBB.end();

  if (I == MBB.begin()) return 1;
  --I;
  if (!T3RAS::isCondBranchOpcode(I->getOpcode()))
    return 1;

  // Remove the branch.
  I->eraseFromParent();
  return 2;
}

bool T3RASInstrInfo::ReverseBranchCondition(SmallVectorImpl<MachineOperand>
                                               &Cond) const {
  assert(Cond.size() == 2 && "Invalid T3RAS branch opcode!");
  switch (Cond[0].getImm()) {
  default:            return true;
  case T3RAS::BEQ:   Cond[0].setImm(T3RAS::BNE); return false;
  case T3RAS::BNE:   Cond[0].setImm(T3RAS::BEQ); return false;
  case T3RAS::BGT:   Cond[0].setImm(T3RAS::BLE); return false;
  case T3RAS::BGE:   Cond[0].setImm(T3RAS::BLT); return false;
  case T3RAS::BLT:   Cond[0].setImm(T3RAS::BGE); return false;
  case T3RAS::BLE:   Cond[0].setImm(T3RAS::BGT); return false;
  case T3RAS::BEQI:  Cond[0].setImm(T3RAS::BNEI); return false;
  case T3RAS::BNEI:  Cond[0].setImm(T3RAS::BEQI); return false;
  case T3RAS::BGTI:  Cond[0].setImm(T3RAS::BLEI); return false;
  case T3RAS::BGEI:  Cond[0].setImm(T3RAS::BLTI); return false;
  case T3RAS::BLTI:  Cond[0].setImm(T3RAS::BGEI); return false;
  case T3RAS::BLEI:  Cond[0].setImm(T3RAS::BGTI); return false;
  case T3RAS::BEQD:  Cond[0].setImm(T3RAS::BNED); return false;
  case T3RAS::BNED:  Cond[0].setImm(T3RAS::BEQD); return false;
  case T3RAS::BGTD:  Cond[0].setImm(T3RAS::BLED); return false;
  case T3RAS::BGED:  Cond[0].setImm(T3RAS::BLTD); return false;
  case T3RAS::BLTD:  Cond[0].setImm(T3RAS::BGED); return false;
  case T3RAS::BLED:  Cond[0].setImm(T3RAS::BGTD); return false;
  case T3RAS::BEQID: Cond[0].setImm(T3RAS::BNEID); return false;
  case T3RAS::BNEID: Cond[0].setImm(T3RAS::BEQID); return false;
  case T3RAS::BGTID: Cond[0].setImm(T3RAS::BLEID); return false;
  case T3RAS::BGEID: Cond[0].setImm(T3RAS::BLTID); return false;
  case T3RAS::BLTID: Cond[0].setImm(T3RAS::BGEID); return false;
  case T3RAS::BLEID: Cond[0].setImm(T3RAS::BGTID); return false;
  }
}

/// getGlobalBaseReg - Return a virtual register initialized with the
/// the global base register value. Output instructions required to
/// initialize the register in the function entry block, if necessary.
///
unsigned T3RASInstrInfo::getGlobalBaseReg(MachineFunction *MF) const {
  T3RASFunctionInfo *T3RASFI = MF->getInfo<T3RASFunctionInfo>();
  unsigned GlobalBaseReg = T3RASFI->getGlobalBaseReg();
  if (GlobalBaseReg != 0)
    return GlobalBaseReg;

  // Insert the set of GlobalBaseReg into the first MBB of the function
  MachineBasicBlock &FirstMBB = MF->front();
  MachineBasicBlock::iterator MBBI = FirstMBB.begin();
  MachineRegisterInfo &RegInfo = MF->getRegInfo();
  const TargetInstrInfo *TII = MF->getTarget().getInstrInfo();

  GlobalBaseReg = RegInfo.createVirtualRegister(T3RAS::GPRRegisterClass);
  BuildMI(FirstMBB, MBBI, DebugLoc(), TII->get(TargetOpcode::COPY),
          GlobalBaseReg).addReg(T3RAS::R20);
  RegInfo.addLiveIn(T3RAS::R20);

  T3RASFI->setGlobalBaseReg(GlobalBaseReg);
  return GlobalBaseReg;
}
