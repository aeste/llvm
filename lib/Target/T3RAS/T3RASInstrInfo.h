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

//===-- MBlazeInstrInfo.h - MBlaze Instruction Information ------*- C++ -*-===//
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

#ifndef T3RASINSTRUCTIONINFO_H
#define T3RASINSTRUCTIONINFO_H

#include "T3RAS.h"
#include "T3RASRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetInstrInfo.h"

#define GET_INSTRINFO_HEADER
#include "T3RASGenInstrInfo.inc"

namespace llvm {

namespace T3RAS {

  // MBlaze Branch Codes
  enum FPBranchCode {
    BRANCH_F,
    BRANCH_T,
    BRANCH_FL,
    BRANCH_TL,
    BRANCH_INVALID
  };

  // MBlaze Condition Codes
  enum CondCode {
    // To be used with float branch True
    FCOND_F,
    FCOND_UN,
    FCOND_EQ,
    FCOND_UEQ,
    FCOND_OLT,
    FCOND_ULT,
    FCOND_OLE,
    FCOND_ULE,
    FCOND_SF,
    FCOND_NGLE,
    FCOND_SEQ,
    FCOND_NGL,
    FCOND_LT,
    FCOND_NGE,
    FCOND_LE,
    FCOND_NGT,

    // To be used with float branch False
    // This conditions have the same mnemonic as the
    // above ones, but are used with a branch False;
    FCOND_T,
    FCOND_OR,
    FCOND_NEQ,
    FCOND_OGL,
    FCOND_UGE,
    FCOND_OGE,
    FCOND_UGT,
    FCOND_OGT,
    FCOND_ST,
    FCOND_GLE,
    FCOND_SNE,
    FCOND_GL,
    FCOND_NLT,
    FCOND_GE,
    FCOND_NLE,
    FCOND_GT,

    // Only integer conditions
    COND_EQ,
    COND_GT,
    COND_GE,
    COND_LT,
    COND_LE,
    COND_NE,
    COND_INVALID
  };

  // Turn condition code into conditional branch opcode.
  inline static unsigned GetCondBranchFromCond(CondCode CC) {
	//TODO:Insert condition to check if variant has delay and included a branch condition without delays
    switch (CC) {
    default: llvm_unreachable("Unknown condition code");
    case COND_EQ: return T3RAS::BEQID;
    case COND_NE: return T3RAS::BNEID;
    case COND_GT: return T3RAS::BGTID;
    case COND_GE: return T3RAS::BGEID;
    case COND_LT: return T3RAS::BLTID;
    case COND_LE: return T3RAS::BLEID;
    }
  }

  /// GetOppositeBranchCondition - Return the inverse of the specified cond,
  /// e.g. turning COND_E to COND_NE.
  // CondCode GetOppositeBranchCondition(MBlaze::CondCode CC);

  /// MBlazeCCToString - Map each FP condition code to its string
  inline static const char *T3RASFCCToString(T3RAS::CondCode CC) {
    switch (CC) {
    default: llvm_unreachable("Unknown condition code");
    case FCOND_F:
    case FCOND_T:   return "f";
    case FCOND_UN:
    case FCOND_OR:  return "un";
    case FCOND_EQ:
    case FCOND_NEQ: return "eq";
    case FCOND_UEQ:
    case FCOND_OGL: return "ueq";
    case FCOND_OLT:
    case FCOND_UGE: return "olt";
    case FCOND_ULT:
    case FCOND_OGE: return "ult";
    case FCOND_OLE:
    case FCOND_UGT: return "ole";
    case FCOND_ULE:
    case FCOND_OGT: return "ule";
    case FCOND_SF:
    case FCOND_ST:  return "sf";
    case FCOND_NGLE:
    case FCOND_GLE: return "ngle";
    case FCOND_SEQ:
    case FCOND_SNE: return "seq";
    case FCOND_NGL:
    case FCOND_GL:  return "ngl";
    case FCOND_LT:
    case FCOND_NLT: return "lt";
    case FCOND_NGE:
    case FCOND_GE:  return "ge";
    case FCOND_LE:
    case FCOND_NLE: return "nle";
    case FCOND_NGT:
    case FCOND_GT:  return "gt";
    }
  }

  inline static bool isUncondBranchOpcode(int Opc) {
    switch (Opc) {
    default: return false;
    case T3RAS::BRI:
    case T3RAS::BRAI:
    case T3RAS::BRID:
    case T3RAS::BRAID:
      return true;
    }
  }

  inline static bool isCondBranchOpcode(int Opc) {
    switch (Opc) {
    default: return false;
    case T3RAS::BEQI: case T3RAS::BEQID:
    case T3RAS::BNEI: case T3RAS::BNEID:
    case T3RAS::BGTI: case T3RAS::BGTID:
    case T3RAS::BGEI: case T3RAS::BGEID:
    case T3RAS::BLTI: case T3RAS::BLTID:
    case T3RAS::BLEI: case T3RAS::BLEID:
      return true;
    }
  }
}

class T3RASInstrInfo : public T3RASGenInstrInfo {
  T3RASTargetMachine &TM;
  const T3RASRegisterInfo RI;
public:
  explicit T3RASInstrInfo(T3RASTargetMachine &TM);

  /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info.  As
  /// such, whenever a client has an instance of instruction info, it should
  /// always be able to get register info as well (through this method).
  ///
  virtual const T3RASRegisterInfo &getRegisterInfo() const { return RI; }

  /// isLoadFromStackSlot - If the specified machine instruction is a direct
  /// load from a stack slot, return the virtual or physical register number of
  /// the destination along with the FrameIndex of the loaded stack slot.  If
  /// not, return 0.  This predicate must return 0 if the instruction has
  /// any side effects other than loading from the stack slot.
  virtual unsigned isLoadFromStackSlot(const MachineInstr *MI,
                                       int &FrameIndex) const;

  /// isStoreToStackSlot - If the specified machine instruction is a direct
  /// store to a stack slot, return the virtual or physical register number of
  /// the source reg along with the FrameIndex of the loaded stack slot.  If
  /// not, return 0.  This predicate must return 0 if the instruction has
  /// any side effects other than storing to the stack slot.
  virtual unsigned isStoreToStackSlot(const MachineInstr *MI,
                                      int &FrameIndex) const;

  /// Branch Analysis
  virtual bool AnalyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                             MachineBasicBlock *&FBB,
                             SmallVectorImpl<MachineOperand> &Cond,
                             bool AllowModify) const;
  virtual unsigned InsertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                                MachineBasicBlock *FBB,
                                const SmallVectorImpl<MachineOperand> &Cond,
                                DebugLoc DL) const;
  virtual unsigned RemoveBranch(MachineBasicBlock &MBB) const;

  virtual bool ReverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond)
    const;

  virtual void copyPhysReg(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator I, DebugLoc DL,
                           unsigned DestReg, unsigned SrcReg,
                           bool KillSrc) const;
  virtual void storeRegToStackSlot(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator MBBI,
                                   unsigned SrcReg, bool isKill, int FrameIndex,
                                   const TargetRegisterClass *RC,
                                   const TargetRegisterInfo *TRI) const;

  virtual void loadRegFromStackSlot(MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator MBBI,
                                    unsigned DestReg, int FrameIndex,
                                    const TargetRegisterClass *RC,
                                    const TargetRegisterInfo *TRI) const;

  /// Insert nop instruction when hazard condition is found
  virtual void insertNoop(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator MI) const;

  /// getGlobalBaseReg - Return a virtual register initialized with the
  /// the global base register value. Output instructions required to
  /// initialize the register in the function entry block, if necessary.
  ///
  unsigned getGlobalBaseReg(MachineFunction *MF) const;
};

}

#endif
