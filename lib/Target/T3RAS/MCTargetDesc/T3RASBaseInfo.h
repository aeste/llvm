//===-- MBlazeBaseInfo.h - Top level definitions for MBlaze -- --*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains small standalone helper functions and enum definitions for
// the MBlaze target useful for the compiler back-end and the MC libraries.
// As such, it deliberately does not include references to LLVM core
// code gen types, passes, etc..
//
//===----------------------------------------------------------------------===//

#ifndef T3RASBASEINFO_H
#define T3RASBASEINFO_H

#include "T3RASMCTargetDesc.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// MBlazeII - This namespace holds all of the target specific flags that
/// instruction info tracks.
///
namespace T3RASII {
  enum {
    // PseudoFrm - This represents an instruction that is a pseudo instruction
    // or one that has not been implemented yet.  It is illegal to code generate
    // it, but tolerated for intermediate implementation stages.
    FPseudo = 0,
    FRRR,
    FRRI,
    FCRR,
    FCRI,
    FRCR,
    FRCI,
    FCCR,
    FCCI,
    FRRCI,
    FRRC,
    FRCX,
    FRCS,
    FCRCS,
    FCRCX,
    FCX,
    FCR,
    FRIR,
    FRRRR,
    FRI,
    FC,
    FRR,
    FormMask = 63

    //===------------------------------------------------------------------===//
    // MBlaze Specific MachineOperand flags.
    // MO_NO_FLAG,

    /// MO_GOT - Represents the offset into the global offset table at which
    /// the address the relocation entry symbol resides during execution.
    // MO_GOT,

    /// MO_GOT_CALL - Represents the offset into the global offset table at
    /// which the address of a call site relocation entry symbol resides
    /// during execution. This is different from the above since this flag
    /// can only be present in call instructions.
    // MO_GOT_CALL,

    /// MO_GPREL - Represents the offset from the current gp value to be used
    /// for the relocatable object file being produced.
    // MO_GPREL,

    /// MO_ABS_HILO - Represents the hi or low part of an absolute symbol
    /// address.
    // MO_ABS_HILO

  };
}

static inline bool isT3RASRegister(unsigned Reg) {
  return Reg <= 31;
}

static inline bool isSpecialT3RASRegister(unsigned Reg) {
  switch (Reg) {
    case 0x0000 : case 0x0001 : case 0x0003 : case 0x0005 : 
    case 0x0007 : case 0x000B : case 0x000D : case 0x1000 : 
    case 0x1001 : case 0x1002 : case 0x1003 : case 0x1004 : 
    case 0x2000 : case 0x2001 : case 0x2002 : case 0x2003 : 
    case 0x2004 : case 0x2005 : case 0x2006 : case 0x2007 : 
    case 0x2008 : case 0x2009 : case 0x200A : case 0x200B : 
      return true;

    default:
      return false;
  }
}

/// getMBlazeRegisterNumbering - Given the enum value for some register, e.g.
/// MBlaze::R0, return the number that it corresponds to (e.g. 0).
static inline unsigned getT3RASRegisterNumbering(unsigned RegEnum) {
  switch (RegEnum) {
    case T3RAS::R0     : return 0;
    case T3RAS::R1     : return 1;
    case T3RAS::R2     : return 2;
    case T3RAS::R3     : return 3;
    case T3RAS::R4     : return 4;
    case T3RAS::R5     : return 5;
    case T3RAS::R6     : return 6;
    case T3RAS::R7     : return 7;
    case T3RAS::R8     : return 8;
    case T3RAS::R9     : return 9;
    case T3RAS::R10    : return 10;
    case T3RAS::R11    : return 11;
    case T3RAS::R12    : return 12;
    case T3RAS::R13    : return 13;
    case T3RAS::R14    : return 14;
    case T3RAS::R15    : return 15;
    case T3RAS::R16    : return 16;
    case T3RAS::R17    : return 17;
    case T3RAS::R18    : return 18;
    case T3RAS::R19    : return 19;
    case T3RAS::R20    : return 20;
    case T3RAS::R21    : return 21;
    case T3RAS::R22    : return 22;
    case T3RAS::R23    : return 23;
    case T3RAS::R24    : return 24;
    case T3RAS::R25    : return 25;
    case T3RAS::R26    : return 26;
    case T3RAS::R27    : return 27;
    case T3RAS::R28    : return 28;
    case T3RAS::R29    : return 29;
    case T3RAS::R30    : return 30;
    case T3RAS::R31    : return 31;
    case T3RAS::RPC    : return 0x0000;
    case T3RAS::RMSR   : return 0x0001;
    case T3RAS::REAR   : return 0x0003;
    case T3RAS::RESR   : return 0x0005;
    case T3RAS::RFSR   : return 0x0007;
    case T3RAS::RBTR   : return 0x000B;
    case T3RAS::REDR   : return 0x000D;
    case T3RAS::RPID   : return 0x1000;
    case T3RAS::RZPR   : return 0x1001;
    case T3RAS::RTLBX  : return 0x1002;
    case T3RAS::RTLBLO : return 0x1003;
    case T3RAS::RTLBHI : return 0x1004;
    case T3RAS::RPVR0  : return 0x2000;
    case T3RAS::RPVR1  : return 0x2001;
    case T3RAS::RPVR2  : return 0x2002;
    case T3RAS::RPVR3  : return 0x2003;
    case T3RAS::RPVR4  : return 0x2004;
    case T3RAS::RPVR5  : return 0x2005;
    case T3RAS::RPVR6  : return 0x2006;
    case T3RAS::RPVR7  : return 0x2007;
    case T3RAS::RPVR8  : return 0x2008;
    case T3RAS::RPVR9  : return 0x2009;
    case T3RAS::RPVR10 : return 0x200A;
    case T3RAS::RPVR11 : return 0x200B;
    default: llvm_unreachable("Unknown register number!");
  }
}

/// getRegisterFromNumbering - Given the enum value for some register, e.g.
/// MBlaze::R0, return the number that it corresponds to (e.g. 0).
static inline unsigned getT3RASRegisterFromNumbering(unsigned Reg) {
  switch (Reg) {
    case 0  : return T3RAS::R0;
    case 1  : return T3RAS::R1;
    case 2  : return T3RAS::R2;
    case 3  : return T3RAS::R3;
    case 4  : return T3RAS::R4;
    case 5  : return T3RAS::R5;
    case 6  : return T3RAS::R6;
    case 7  : return T3RAS::R7;
    case 8  : return T3RAS::R8;
    case 9  : return T3RAS::R9;
    case 10 : return T3RAS::R10;
    case 11 : return T3RAS::R11;
    case 12 : return T3RAS::R12;
    case 13 : return T3RAS::R13;
    case 14 : return T3RAS::R14;
    case 15 : return T3RAS::R15;
    case 16 : return T3RAS::R16;
    case 17 : return T3RAS::R17;
    case 18 : return T3RAS::R18;
    case 19 : return T3RAS::R19;
    case 20 : return T3RAS::R20;
    case 21 : return T3RAS::R21;
    case 22 : return T3RAS::R22;
    case 23 : return T3RAS::R23;
    case 24 : return T3RAS::R24;
    case 25 : return T3RAS::R25;
    case 26 : return T3RAS::R26;
    case 27 : return T3RAS::R27;
    case 28 : return T3RAS::R28;
    case 29 : return T3RAS::R29;
    case 30 : return T3RAS::R30;
    case 31 : return T3RAS::R31;
    default: llvm_unreachable("Unknown register number!");
  }
}

static inline unsigned getSpecialT3RASRegisterFromNumbering(unsigned Reg) {
  switch (Reg) {
    case 0x0000 : return T3RAS::RPC;
    case 0x0001 : return T3RAS::RMSR;
    case 0x0003 : return T3RAS::REAR;
    case 0x0005 : return T3RAS::RESR;
    case 0x0007 : return T3RAS::RFSR;
    case 0x000B : return T3RAS::RBTR;
    case 0x000D : return T3RAS::REDR;
    case 0x1000 : return T3RAS::RPID;
    case 0x1001 : return T3RAS::RZPR;
    case 0x1002 : return T3RAS::RTLBX;
    case 0x1003 : return T3RAS::RTLBLO;
    case 0x1004 : return T3RAS::RTLBHI;
    case 0x2000 : return T3RAS::RPVR0;
    case 0x2001 : return T3RAS::RPVR1;
    case 0x2002 : return T3RAS::RPVR2;
    case 0x2003 : return T3RAS::RPVR3;
    case 0x2004 : return T3RAS::RPVR4;
    case 0x2005 : return T3RAS::RPVR5;
    case 0x2006 : return T3RAS::RPVR6;
    case 0x2007 : return T3RAS::RPVR7;
    case 0x2008 : return T3RAS::RPVR8;
    case 0x2009 : return T3RAS::RPVR9;
    case 0x200A : return T3RAS::RPVR10;
    case 0x200B : return T3RAS::RPVR11;
    default: llvm_unreachable("Unknown register number!");
  }
}

} // end namespace llvm;

#endif
