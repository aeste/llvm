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

//===-- HazardPass.cpp - T3RAS hazard solver --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// A pass that attempts to sort instructions to solve data hazards. If
// instruction cant be moved forwards then a NOP is placed to move it backwards.
//
//===----------------------------------------------------------------------===//


#include "T3RAS.h"
#include "T3RASTargetMachine.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;
namespace {
  struct Filler : public MachineFunctionPass {
	//---------
	//definition section
	//---------
	//insert check for subtarget. If subtarget is T3RAS4T than disable
    TargetMachine &TM;
    const TargetInstrInfo *TII;

    static char ID;
    Filler(TargetMachine &tm)
      : MachineFunctionPass(ID), TM(tm), TII(tm.getInstrInfo()) { }

    virtual const char *getPassName() const {
      return "T3RAS hazard pass";
    }

    bool runOnMachineBasicBlock(MachineBasicBlock &MBB);
    bool runOnMachineFunction(MachineFunction &F) {
      bool Changed = false;
      for (MachineFunction::iterator FI = F.begin(), FE = F.end();
           FI != FE; ++FI)
        Changed |= runOnMachineBasicBlock(*FI);
      return Changed;
    }

  };
  char Filler::ID = 0;
} // end of anonymous namespace
static bool hasForwardingResults(MachineBasicBlock::iterator &I) {
//result is available before it is written just after execution
  unsigned op = I->getOpcode();//insert more opcodes here
  if (op == T3RAS::ADD || op == T3RAS::ADDI ||
     op == T3RAS::ADDK || op == T3RAS::ADDIK ||
     op == T3RAS::ADDC || op == T3RAS::ADDIC ||
    op == T3RAS::ADDKC || op == T3RAS::ADDIKC ||
      op == T3RAS::AND || op == T3RAS::ANDI ||
     op == T3RAS::ANDN || op == T3RAS::ANDNI ||
      op == T3RAS::CMP || op == T3RAS::CMPU ||
      op == T3RAS::MFS || op == T3RAS::MSRCLR ||
   op == T3RAS::MSRSET || op == T3RAS::MTS ||
       op == T3RAS::OR || op == T3RAS::ORI ||
     op == T3RAS::RSUB || op == T3RAS::RSUBI ||
    op == T3RAS::RSUBK || op == T3RAS::RSUBIK ||
    op == T3RAS::RSUBC || op == T3RAS::RSUBIC ||
   op == T3RAS::RSUBKC || op == T3RAS::RSUBIKC||
   op == T3RAS::SEXT16 || op == T3RAS::SEXT8||
      op == T3RAS::SRA || op == T3RAS::SRC||
      op == T3RAS::SRL || 
      op == T3RAS::XOR || op == T3RAS::XORI)
    return true;

  else return false;
}
//-------
//this is dependency check algorithm
//-------
static bool delayHasHazard(MachineBasicBlock &MBB,MachineBasicBlock::iterator &candidate
				//,MachineBasicBlock::iterator &slot
) {
int dependencyLength=4;
int t=0;
int forwardLength=2;
unsigned myinst=candidate->getOpcode();
MachineBasicBlock::iterator i=candidate;
//MachineBasicBlock::iterator dL=candidate;
//MachineBasicBlock::iterator fL=candidate;

//for(int k=forwardLength;k>0;k--)fL--;
//for(int k=dependencyLength;k>0;k--)dL--;
//if(candidate!=MBB.begin()){
//for(int k=dependencyLength;k>0;k--){t++;i--;if(i==MBB.begin())break;}
//}
//i=candidate;
//TODO:improve this algorithm and allow instructions to be moved forwards
bool hasHazard=false;
bool op_is_def=false;
unsigned op_def;
unsigned op;
//i->
	/*for(int k=t;k>0&&hasHazard==false&&i!=MBB.begin();k--){
		i--;//FIXME section
	  	unsigned op = i->getOpcode();
       		op_is_def= i->getOperand(op).isDef();
      		if(op_is_def) {
			op_def = i->getOperand(op).getReg();
			if(k!=4) {
			if(candidate->getOperand(op).isUse())hasHazard=true;
			if(i->mayLoad())hasHazard=true;
			}
		}
		if(hasForwardingResults(i)){
			if(k==2)hasHazard=false;
		}
		
	}*/
	if(hasHazard)return hasHazard;
	//else if(i->getOpcode()==T3RAS::IMM)return false;
	else return false;
}

  // Hazard check
 /* MachineBasicBlock::iterator a = candidate;
  MachineBasicBlock::iterator b = slot;
//check if instruction has hazard in current location
//if its a load instruction, check if theres a store instruction infront nearby. Check if they're the same
//if its a load instruction and theres the same store instruction, delete the load instruction.
//if it has check where the instruction depends on
//if dependent instruction has forwarding, look for a nop that can be replaced
//otherwise find how many NOPs is required to solve the data hazard
  // MBB layout:-
  //    candidate := a0 = operation(a1, a2)
  //    ...middle bit...
  //    slot := b0 = operation(b1, b2)

  // Possible hazards:-/
  // 1. a1 or a2 was written during the middle bit
  // 2. a0 was read or written during the middle bit
  // 3. a0 is one or more of {b0, b1, b2}
  // 4. b0 is one or more of {a1, a2}
  // 5. a accesses memory, and the middle bit
  //    contains a store operation.
  bool a_is_memory = candidate->mayLoad() || candidate->mayStore();

  // Determine the number of operands in the slot instruction and in the
  // candidate instruction.
  const unsigned aend = getLastRealOperand(a);
  const unsigned bend = getLastRealOperand(b);

  // Check hazards type 1, 2 and 5 by scanning the middle bit
  MachineBasicBlock::iterator m = a;
  for (++m; m != b; ++m) {
    for (unsigned aop = 0; aop<aend; ++aop) {
      bool aop_is_reg = a->getOperand(aop).isReg();
      if (!aop_is_reg) continue;

      bool aop_is_def = a->getOperand(aop).isDef();
      unsigned aop_reg = a->getOperand(aop).getReg();

      const unsigned mend = getLastRealOperand(m);
      for (unsigned mop = 0; mop<mend; ++mop) {
        bool mop_is_reg = m->getOperand(mop).isReg();
        if (!mop_is_reg) continue;

        bool mop_is_def = m->getOperand(mop).isDef();
        unsigned mop_reg = m->getOperand(mop).getReg();

        if (aop_is_def && (mop_reg == aop_reg))
            return true; // Hazard type 2, because aop = a0
        else if (mop_is_def && (mop_reg == aop_reg))
            return true; // Hazard type 1, because aop in {a1, a2}
      }
    }

    // Check hazard type 5
    if (a_is_memory && m->mayStore())
      return true;
  }

  // Check hazard type 3 & 4
  for (unsigned aop = 0; aop<aend; ++aop) {
    if (a->getOperand(aop).isReg()) {
      unsigned aop_reg = a->getOperand(aop).getReg();

      for (unsigned bop = 0; bop<bend; ++bop) {
        if (b->getOperand(bop).isReg() && !b->getOperand(bop).isImplicit()) {
          unsigned bop_reg = b->getOperand(bop).getReg();
          if (aop_reg == bop_reg)
            return true;
        }
      }
    }
  }

  return false;*/

static bool hasImmInstruction(MachineBasicBlock::iterator &candidate) {
    // Any instruction with an immediate mode operand greater than
    // 16-bits requires an implicit IMM instruction.
    unsigned numOper = candidate->getNumOperands();
    for (unsigned op = 0; op < numOper; ++op) {
        MachineOperand &mop = candidate->getOperand(op);

        // The operand requires more than 16-bits to represent.
        if (mop.isImm() && (mop.getImm() < -0x8000 || mop.getImm() > 0x7fff))
          return true;

        // We must assume that unknown immediate values require more than
        // 16-bits to represent.
        if (mop.isGlobal() || mop.isSymbol() || mop.isJTI() || mop.isCPI())
          return true;

        // FIXME: we could probably check to see if the FP value happens
        //        to not need an IMM instruction. For now we just always
        //        assume that FP values do.
        if (mop.isFPImm())
          return true;
    }

    return false;
}

static bool isDelayFiller(MachineBasicBlock &MBB,
                          MachineBasicBlock::iterator candidate) {
  if (candidate == MBB.begin())
    return false;

  --candidate;
  return (candidate->hasDelaySlot());
}
//define 2 types of instruction. First have results available at stage 3 and 5, second at stage 5
static bool hasUnknownSideEffects(MachineBasicBlock::iterator &I) {
  if (!I->hasUnmodeledSideEffects())
    return false;

  unsigned op = I->getOpcode();
  if (op == T3RAS::ADDK || op == T3RAS::ADDIK ||
      op == T3RAS::ADDC || op == T3RAS::ADDIC ||
      op == T3RAS::ADDKC || op == T3RAS::ADDIKC ||
      op == T3RAS::RSUBK || op == T3RAS::RSUBIK ||
      op == T3RAS::RSUBC || op == T3RAS::RSUBIC ||
      op == T3RAS::RSUBKC || op == T3RAS::RSUBIKC)
    return false;

  return true;
}

//--------------
//this performs the search for a suitable slot to place the instruction
//--------------
 //MachineBasicBlock::iterator
static bool findInstrPos(MachineBasicBlock &MBB,MachineBasicBlock::iterator slot) {
  MachineBasicBlock::iterator I = slot;
	MachineBasicBlock::iterator M=MBB.begin();
//  while (true) {
    if (I == MBB.begin()) return false;
    bool pos=delayHasHazard(MBB,I);
      //break;
	//need another of this to find data dependecy and possible locations for instructions
	//find suitable location infront if possible if hazard, else insert NOP
	
   //if instruction is terminator
	//if(I->isTerminator())
  //  if (I->hasDelaySlot() || I->isBranch() || isDelayFiller(MBB,I) ||
   //     I->isCall() || I->isReturn() || I->isBarrier() //|| hasUnknownSideEffects(I)
//)//change to indicate instructions that are required for the OP
   //   break;

    //if (hasImmInstruction(I)&&pos>0)pos--;
	//{//give some value to indicate 2 instructions.
     // continue;
	//}

    return pos;
//  }
//  return MBB.begin();//otherwise return start (no movement necessary)
}
//--------------
//this performs the action
//--------------
bool Filler::runOnMachineBasicBlock(MachineBasicBlock &MBB) {
  	bool Changed = false;
  	for (MachineBasicBlock::iterator I = MBB.begin(); I != MBB.end(); ++I){
		int i;
		if (TM.getSubtarget<T3RASSubtarget>().hasNoDelay())i=0;
		else i=TM.getSubtarget<T3RASSubtarget>().delays();
		if(TM.getSubtarget<T3RASSubtarget>().disableHazardPass())i=0;
		for(i=i;i>0;i--){
		
		bool D=false;
		MachineBasicBlock::iterator J = I;
		MachineBasicBlock::iterator JStart = MBB.begin();
		//MachineBasicBlock::iterator delayDistance=MBB.end();

		//for(int k=Delay;k>0;k--)delayDistance--;
        	D = findInstrPos(MBB,I);
		//for(int k=D;k>0;k--)delayDistance--;
      		Changed = true;
		D=true;
      //if (I >=delayDistance){
	//	--delayDistance;//moving the instruction back by inserting NOPs infront
	//	for(temp=D;temp>0;temp--)
        //	BuildMI(MBB, delayDistance, I->getDebugLoc(), TII->get(T3RAS::NOP));
		//if(J>J3){MBB.splice(J,&MBB,J2);delayMoved=true;}}}//if a NOP was added in the delay slot, move it infront of the terminator instruction and notify
	//	I=delayDistance;}
      //else	{
        //insert code to delete the NOP there after.
		JStart++;
			if(I!=MBB.begin()&&I!=JStart&&--J!=MBB.begin()){//if(hasImmInstruction(I))--J;
        		BuildMI(MBB, J, I->getDebugLoc(), TII->get(T3RAS::NOP));
			}//end of if based on beginning or not
		}//end of for loop based on number of delays
   	}//end of MBB loop
	return Changed;
}
//TODO:insert check here to ensure that there are only the said amount of delayslots by moving instructions forwards if there are too many. if moving the instructions are required, check if moving the first instruction will cause a hazard for it. If it does insert a NOP, than move all the other instructions that are required.
//After transfering the instructions, check if hazard is found for first instruction in delay slot. If found insert a NOP 
//TODO:Check start for NOP. If it does delete all the NOPs at start 
  

FunctionPass *llvm::createT3RASHazardPass(T3RASTargetMachine &tm) {
  return new Filler(tm);
}
