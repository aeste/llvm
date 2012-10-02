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

//===-- MBlazeMCInstLower.cpp - Convert MBlaze MachineInstr to an MCInst---===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower MBlaze MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//

#include "T3RASMCInstLower.h"
#include "T3RASInstrInfo.h"
#include "llvm/Constants.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/ADT/SmallString.h"
using namespace llvm;

MCSymbol *T3RASMCInstLower::
GetGlobalAddressSymbol(const MachineOperand &MO) const {
  switch (MO.getTargetFlags()) {
  default: llvm_unreachable("Unknown target flag on GV operand");
  case 0:  break;
  }

  return Printer.Mang->getSymbol(MO.getGlobal());
}

MCSymbol *T3RASMCInstLower::
GetExternalSymbolSymbol(const MachineOperand &MO) const {
  switch (MO.getTargetFlags()) {
  default: llvm_unreachable("Unknown target flag on GV operand");
  case 0:  break;
  }

  return Printer.GetExternalSymbolSymbol(MO.getSymbolName());
}

MCSymbol *T3RASMCInstLower::
GetJumpTableSymbol(const MachineOperand &MO) const {
  SmallString<256> Name;
  raw_svector_ostream(Name) << Printer.MAI->getPrivateGlobalPrefix() << "JTI"
                            << Printer.getFunctionNumber() << '_'
                            << MO.getIndex();
  switch (MO.getTargetFlags()) {
  default: llvm_unreachable("Unknown target flag on GV operand");
  case 0:  break;
  }

  // Create a symbol for the name.
  return Ctx.GetOrCreateSymbol(Name.str());
}

MCSymbol *T3RASMCInstLower::
GetConstantPoolIndexSymbol(const MachineOperand &MO) const {
  SmallString<256> Name;
  raw_svector_ostream(Name) << Printer.MAI->getPrivateGlobalPrefix() << "CPI"
                            << Printer.getFunctionNumber() << '_'
                            << MO.getIndex();

  switch (MO.getTargetFlags()) {
  default:
      llvm_unreachable("Unknown target flag on GV operand");

  case 0: break;
  }

  // Create a symbol for the name.
  return Ctx.GetOrCreateSymbol(Name.str());
}

MCSymbol *T3RASMCInstLower::
GetBlockAddressSymbol(const MachineOperand &MO) const {
  switch (MO.getTargetFlags()) {
  default: llvm_unreachable("Unknown target flag on GV operand");
  case 0: break;
  }

  return Printer.GetBlockAddressSymbol(MO.getBlockAddress());
}

MCOperand T3RASMCInstLower::
LowerSymbolOperand(const MachineOperand &MO, MCSymbol *Sym) const {
  // FIXME: We would like an efficient form for this, so we don't have to do a
  // lot of extra uniquing.
  const MCExpr *Expr = MCSymbolRefExpr::Create(Sym, Ctx);

  switch (MO.getTargetFlags()) {
  default:
      llvm_unreachable("Unknown target flag on GV operand");

  case 0: break;
  }

  if (!MO.isJTI() && MO.getOffset())
    Expr = MCBinaryExpr::CreateAdd(Expr,
                                   MCConstantExpr::Create(MO.getOffset(), Ctx),
                                   Ctx);
  return MCOperand::CreateExpr(Expr);
}

void T3RASMCInstLower::Lower(const MachineInstr *MI, MCInst &OutMI) const {
  OutMI.setOpcode(MI->getOpcode());

  for (unsigned i = 0, e = MI->getNumOperands(); i != e; ++i) {
    const MachineOperand &MO = MI->getOperand(i);

    MCOperand MCOp;
    switch (MO.getType()) {
    default: llvm_unreachable("unknown operand type");
    case MachineOperand::MO_Register:
      // Ignore all implicit register operands.
      if (MO.isImplicit()) continue;
      MCOp = MCOperand::CreateReg(MO.getReg());
      break;
    case MachineOperand::MO_Immediate:
      MCOp = MCOperand::CreateImm(MO.getImm());
      break;
    case MachineOperand::MO_MachineBasicBlock:
      MCOp = MCOperand::CreateExpr(MCSymbolRefExpr::Create(
                         MO.getMBB()->getSymbol(), Ctx));
      break;
    case MachineOperand::MO_GlobalAddress:
      MCOp = LowerSymbolOperand(MO, GetGlobalAddressSymbol(MO));
      break;
    case MachineOperand::MO_ExternalSymbol:
      MCOp = LowerSymbolOperand(MO, GetExternalSymbolSymbol(MO));
      break;
    case MachineOperand::MO_JumpTableIndex:
      MCOp = LowerSymbolOperand(MO, GetJumpTableSymbol(MO));
      break;
    case MachineOperand::MO_ConstantPoolIndex:
      MCOp = LowerSymbolOperand(MO, GetConstantPoolIndexSymbol(MO));
      break;
    case MachineOperand::MO_BlockAddress:
      MCOp = LowerSymbolOperand(MO, GetBlockAddressSymbol(MO));
      break;
    case MachineOperand::MO_FPImmediate: {
      bool ignored;
      APFloat FVal = MO.getFPImm()->getValueAPF();
      FVal.convert(APFloat::IEEEsingle, APFloat::rmTowardZero, &ignored);

      APInt IVal = FVal.bitcastToAPInt();
      uint64_t Val = *IVal.getRawData();
      MCOp = MCOperand::CreateImm(Val);
      break;
    }
    case MachineOperand::MO_RegisterMask:
      continue;
    }

    OutMI.addOperand(MCOp);
  }
}
