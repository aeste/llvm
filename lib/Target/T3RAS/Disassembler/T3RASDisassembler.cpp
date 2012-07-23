//===-- MBlazeDisassembler.cpp - Disassembler for MicroBlaze  -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file is part of the MBlaze Disassembler. It contains code to translate
// the data produced by the decoder into MCInsts.
//
//===----------------------------------------------------------------------===//

#include "T3RAS.h"
#include "T3RASDisassembler.h"

#include "llvm/MC/EDInstInfo.h"
#include "llvm/MC/MCDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MemoryObject.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"

// #include "T3RASGenDecoderTables.inc"
// #include "T3RASGenRegisterNames.inc"
#include "T3RASGenEDInfo.inc"

namespace llvm {
extern const MCInstrDesc T3RASInsts[];
}

using namespace llvm;

const uint16_t UNSUPPORTED = -1;

static const uint16_t T3RASBinary2Opcode[] = {
  T3RAS::ADD,   T3RAS::RSUB,   T3RAS::ADDC,   T3RAS::RSUBC,   //00,01,02,03
  T3RAS::ADDK,  T3RAS::RSUBK,  T3RAS::ADDKC,  T3RAS::RSUBKC,  //04,05,06,07
  T3RAS::ADDI,  T3RAS::RSUBI,  T3RAS::ADDIC,  T3RAS::RSUBIC,  //08,09,0A,0B
  T3RAS::ADDIK, T3RAS::RSUBIK, T3RAS::ADDIKC, T3RAS::RSUBIKC, //0C,0D,0E,0F

  T3RAS::MUL,   T3RAS::BSRL,   //T3RAS::IDIV,
 T3RAS::GETD,    //10,11,12,13
  UNSUPPORTED,   //UNSUPPORTED,    //T3RAS::FADD,   UNSUPPORTED,     //14,15,16,17
  T3RAS::MULI,  T3RAS::BSRLI,  UNSUPPORTED,    T3RAS::GET,     //18,19,1A,1B
  UNSUPPORTED,   UNSUPPORTED,    UNSUPPORTED,    UNSUPPORTED,     //1C,1D,1E,1F

  T3RAS::OR,    T3RAS::AND,    T3RAS::XOR,    T3RAS::ANDN,    //20,21,22,23
  T3RAS::SEXT8, T3RAS::MFS,    T3RAS::BR,     T3RAS::BEQ,     //24,25,26,27
  T3RAS::ORI,   T3RAS::ANDI,   T3RAS::XORI,   T3RAS::ANDNI,   //28,29,2A,2B
  T3RAS::IMM,   T3RAS::RTSD,   T3RAS::BRI,    T3RAS::BEQI,    //2C,2D,2E,2F

  T3RAS::LBU,   T3RAS::LHU,    T3RAS::LW,     UNSUPPORTED,     //30,31,32,33
  T3RAS::SB,    T3RAS::SH,     T3RAS::SW,     UNSUPPORTED,     //34,35,36,37
  T3RAS::LBUI,  T3RAS::LHUI,   T3RAS::LWI,    UNSUPPORTED,     //38,39,3A,3B
  T3RAS::SBI,   T3RAS::SHI,    T3RAS::SWI,    UNSUPPORTED,     //3C,3D,3E,3F
};

static unsigned getRD(uint32_t insn) {
  if (!isT3RASRegister((insn>>21)&0x1F))
    return UNSUPPORTED;
  return getT3RASRegisterFromNumbering((insn>>21)&0x1F);
}

static unsigned getRA(uint32_t insn) {
  if (!getT3RASRegisterFromNumbering((insn>>16)&0x1F))
    return UNSUPPORTED;
  return getT3RASRegisterFromNumbering((insn>>16)&0x1F);
}

static unsigned getRB(uint32_t insn) {
  if (!getT3RASRegisterFromNumbering((insn>>11)&0x1F))
    return UNSUPPORTED;
  return getT3RASRegisterFromNumbering((insn>>11)&0x1F);
}

static int64_t getRS(uint32_t insn) {
  if (!isSpecialT3RASRegister(insn&0x3FFF))
    return UNSUPPORTED;
  return getSpecialT3RASRegisterFromNumbering(insn&0x3FFF);
}

static int64_t getIMM(uint32_t insn) {
    int16_t val = (insn & 0xFFFF);
    return val;
}

static int64_t getSHT(uint32_t insn) {
    int16_t val = (insn & 0x1F);
    return val;
}

static unsigned getFLAGS(int32_t insn) {
    return (insn & 0x7FF);
}

static int64_t getFSL(uint32_t insn) {
    int16_t val = (insn & 0xF);
    return val;
}

static unsigned decodeMUL(uint32_t insn) {
    switch (getFLAGS(insn)) {
    default: return UNSUPPORTED;
    case 0:  return T3RAS::MUL;
    //case 1:  return T3RAS::MULH;
    //case 2:  return T3RAS::MULHSU;
    //case 3:  return T3RAS::MULHU;
    }
}

static unsigned decodeSEXT(uint32_t insn) {
    switch (insn&0x7FF) {
    default:   return UNSUPPORTED;
    case 0x60: return T3RAS::SEXT8;
    /*case 0x68: return T3RAS::WIC;
    case 0x64: return T3RAS::WDC;
    case 0x66: return T3RAS::WDCC;
    case 0x74: return T3RAS::WDCF;*/
    case 0x61: return T3RAS::SEXT16;
    case 0x41: return T3RAS::SRL;
    case 0x21: return T3RAS::SRC;
    case 0x01: return T3RAS::SRA;
    //case 0xE0: return T3RAS::CLZ;
    }
}

static unsigned decodeBEQ(uint32_t insn) {
    switch ((insn>>21)&0x1F) {
    default:    return UNSUPPORTED;
    case 0x00:  return T3RAS::BEQ;
    case 0x10:  return T3RAS::BEQD;
    case 0x05:  return T3RAS::BGE;
    case 0x15:  return T3RAS::BGED;
    case 0x04:  return T3RAS::BGT;
    case 0x14:  return T3RAS::BGTD;
    case 0x03:  return T3RAS::BLE;
    case 0x13:  return T3RAS::BLED;
    case 0x02:  return T3RAS::BLT;
    case 0x12:  return T3RAS::BLTD;
    case 0x01:  return T3RAS::BNE;
    case 0x11:  return T3RAS::BNED;
    }
}

static unsigned decodeBEQI(uint32_t insn) {
    switch ((insn>>21)&0x1F) {
    default:    return UNSUPPORTED;
    case 0x00:  return T3RAS::BEQI;
    case 0x10:  return T3RAS::BEQID;
    case 0x05:  return T3RAS::BGEI;
    case 0x15:  return T3RAS::BGEID;
    case 0x04:  return T3RAS::BGTI;
    case 0x14:  return T3RAS::BGTID;
    case 0x03:  return T3RAS::BLEI;
    case 0x13:  return T3RAS::BLEID;
    case 0x02:  return T3RAS::BLTI;
    case 0x12:  return T3RAS::BLTID;
    case 0x01:  return T3RAS::BNEI;
    case 0x11:  return T3RAS::BNEID;
    }
}

static unsigned decodeBR(uint32_t insn) {
    switch ((insn>>16)&0x1F) {
    default:   return UNSUPPORTED;
    case 0x00: return T3RAS::BR;
    case 0x08: return T3RAS::BRA;
    case 0x0C: return T3RAS::BRK;
    case 0x10: return T3RAS::BRD;
    case 0x14: return T3RAS::BRLD;
    case 0x18: return T3RAS::BRAD;
    case 0x1C: return T3RAS::BRALD;
    }
}

static unsigned decodeBRI(uint32_t insn) {
    switch (insn&0x3FFFFFF) {
    default:        break;
    case 0x0020004: return T3RAS::IDMEMBAR;
    case 0x0220004: return T3RAS::DMEMBAR;
    case 0x0420004: return T3RAS::IMEMBAR;
    }

    switch ((insn>>16)&0x1F) {
    default:   return UNSUPPORTED;
    case 0x00: return T3RAS::BRI;
    case 0x08: return T3RAS::BRAI;
    case 0x0C: return T3RAS::BRKI;
    case 0x10: return T3RAS::BRID;
    case 0x14: return T3RAS::BRLID;
    case 0x18: return T3RAS::BRAID;
    case 0x1C: return T3RAS::BRALID;
    }
}

static unsigned decodeBSRL(uint32_t insn) {
    switch ((insn>>9)&0x3) {
    default:  return UNSUPPORTED;
    case 0x2: return T3RAS::BSLL;
    case 0x1: return T3RAS::BSRA;
    case 0x0: return T3RAS::BSRL;
    }
}

static unsigned decodeBSRLI(uint32_t insn) {
    switch ((insn>>9)&0x3) {
    default:  return UNSUPPORTED;
    case 0x2: return T3RAS::BSLLI;
    case 0x1: return T3RAS::BSRAI;
    case 0x0: return T3RAS::BSRLI;
    }
}

static unsigned decodeRSUBK(uint32_t insn) {
    switch (getFLAGS(insn)) {
    default:  return UNSUPPORTED;
    case 0x0: return T3RAS::RSUBK;
    case 0x1: return T3RAS::CMP;
    case 0x3: return T3RAS::CMPU;
    }
}
#ifdef float_instr
static unsigned decodeFADD(uint32_t insn) {
    switch (getFLAGS(insn)) {
    default:    return UNSUPPORTED;
    case 0x000: return T3RAS::FADD;
    case 0x080: return T3RAS::FRSUB;
    case 0x100: return T3RAS::FMUL;
    case 0x180: return T3RAS::FDIV;
    case 0x200: return T3RAS::FCMP_UN;
    case 0x210: return T3RAS::FCMP_LT;
    case 0x220: return T3RAS::FCMP_EQ;
    case 0x230: return T3RAS::FCMP_LE;
    case 0x240: return T3RAS::FCMP_GT;
    case 0x250: return T3RAS::FCMP_NE;
    case 0x260: return T3RAS::FCMP_GE;
    case 0x280: return T3RAS::FLT;
    case 0x300: return T3RAS::FINT;
    case 0x380: return T3RAS::FSQRT;
    }
}
#endif
static unsigned decodeGET(uint32_t insn) {
    switch ((insn>>10)&0x3F) {
    default:   return UNSUPPORTED;
    case 0x00: return T3RAS::GET;
    /*case 0x01: return T3RAS::EGET;
    case 0x02: return T3RAS::AGET;
    case 0x03: return T3RAS::EAGET;
    case 0x04: return T3RAS::TGET;
    case 0x05: return T3RAS::TEGET;
    case 0x06: return T3RAS::TAGET;
    case 0x07: return T3RAS::TEAGET;*/
    case 0x08: return T3RAS::CGET;
    /*case 0x09: return T3RAS::ECGET;
    case 0x0A: return T3RAS::CAGET;
    case 0x0B: return T3RAS::ECAGET;
    case 0x0C: return T3RAS::TCGET;
    case 0x0D: return T3RAS::TECGET;
    case 0x0E: return T3RAS::TCAGET;
    case 0x0F: return T3RAS::TECAGET;*/
    case 0x10: return T3RAS::NGET;
    /*case 0x11: return T3RAS::NEGET;
    case 0x12: return T3RAS::NAGET;
    case 0x13: return T3RAS::NEAGET;
    case 0x14: return T3RAS::TNGET;
    case 0x15: return T3RAS::TNEGET;
    case 0x16: return T3RAS::TNAGET;
    case 0x17: return T3RAS::TNEAGET;*/
    case 0x18: return T3RAS::NCGET;
    /*case 0x19: return T3RAS::NECGET;
    case 0x1A: return T3RAS::NCAGET;
    case 0x1B: return T3RAS::NECAGET;
    case 0x1C: return T3RAS::TNCGET;
    case 0x1D: return T3RAS::TNECGET;
    case 0x1E: return T3RAS::TNCAGET;
    case 0x1F: return T3RAS::TNECAGET;*/
    case 0x20: return T3RAS::PUT;
    /*case 0x22: return T3RAS::APUT;
    case 0x24: return T3RAS::TPUT;
    case 0x26: return T3RAS::TAPUT;
    case 0x28: return T3RAS::CPUT;
    case 0x2A: return T3RAS::CAPUT;
    case 0x2C: return T3RAS::TCPUT;
    case 0x2E: return T3RAS::TCAPUT;*/
    case 0x30: return T3RAS::NPUT;
    /*case 0x32: return T3RAS::NAPUT;
    case 0x34: return T3RAS::TNPUT;
    case 0x36: return T3RAS::TNAPUT;
    case 0x38: return T3RAS::NCPUT;
    case 0x3A: return T3RAS::NCAPUT;
    case 0x3C: return T3RAS::TNCPUT;
    case 0x3E: return T3RAS::TNCAPUT;*/
    }
}
#ifdef GETD_instr
static unsigned decodeGETD(uint32_t insn) {
    switch ((insn>>5)&0x3F) {
    default:   return UNSUPPORTED;
    case 0x00: return T3RAS::GETD;
    case 0x01: return T3RAS::EGETD;
    case 0x02: return T3RAS::AGETD;
    case 0x03: return T3RAS::EAGETD;
    case 0x04: return T3RAS::TGETD;
    case 0x05: return T3RAS::TEGETD;
    case 0x06: return T3RAS::TAGETD;
    case 0x07: return T3RAS::TEAGETD;
    case 0x08: return T3RAS::CGETD;
    case 0x09: return T3RAS::ECGETD;
    case 0x0A: return T3RAS::CAGETD;
    case 0x0B: return T3RAS::ECAGETD;
    case 0x0C: return T3RAS::TCGETD;
    case 0x0D: return T3RAS::TECGETD;
    case 0x0E: return T3RAS::TCAGETD;
    case 0x0F: return T3RAS::TECAGETD;
    case 0x10: return T3RAS::NGETD;
    case 0x11: return T3RAS::NEGETD;
    case 0x12: return T3RAS::NAGETD;
    case 0x13: return T3RAS::NEAGETD;
    case 0x14: return T3RAS::TNGETD;
    case 0x15: return T3RAS::TNEGETD;
    case 0x16: return T3RAS::TNAGETD;
    case 0x17: return T3RAS::TNEAGETD;
    case 0x18: return T3RAS::NCGETD;
    case 0x19: return T3RAS::NECGETD;
    case 0x1A: return T3RAS::NCAGETD;
    case 0x1B: return T3RAS::NECAGETD;
    case 0x1C: return T3RAS::TNCGETD;
    case 0x1D: return T3RAS::TNECGETD;
    case 0x1E: return T3RAS::TNCAGETD;
    case 0x1F: return T3RAS::TNECAGETD;
    case 0x20: return T3RAS::PUTD;
    case 0x22: return T3RAS::APUTD;
    case 0x24: return T3RAS::TPUTD;
    case 0x26: return T3RAS::TAPUTD;
    case 0x28: return T3RAS::CPUTD;
    case 0x2A: return T3RAS::CAPUTD;
    case 0x2C: return T3RAS::TCPUTD;
    case 0x2E: return T3RAS::TCAPUTD;
    case 0x30: return T3RAS::NPUTD;
    case 0x32: return T3RAS::NAPUTD;
    case 0x34: return T3RAS::TNPUTD;
    case 0x36: return T3RAS::TNAPUTD;
    case 0x38: return T3RAS::NCPUTD;
    case 0x3A: return T3RAS::NCAPUTD;
    case 0x3C: return T3RAS::TNCPUTD;
    case 0x3E: return T3RAS::TNCAPUTD;
    }
}
#endif
#ifdef T3RAS_IDIV
static unsigned decodeIDIV(uint32_t insn) {
    switch (insn&0x3) {
    default:  return UNSUPPORTED;
    case 0x0: return T3RAS::IDIV;
    case 0x2: return T3RAS::IDIVU;
    }
}
#endif

static unsigned decodeLBU(uint32_t insn) {
    switch ((insn>>9)&0x1) {
    default:  return UNSUPPORTED;
    case 0x0: return T3RAS::LBU;
    case 0x1: return T3RAS::LBUR;
    }
}

static unsigned decodeLHU(uint32_t insn) {
    switch ((insn>>9)&0x1) {
    default:  return UNSUPPORTED;
    case 0x0: return T3RAS::LHU;
    case 0x1: return T3RAS::LHUR;
    }
}

static unsigned decodeLW(uint32_t insn) {
    switch ((insn>>9)&0x3) {
    default:  return UNSUPPORTED;
    case 0x0: return T3RAS::LW;
    case 0x1: return T3RAS::LWR;
    case 0x2: return T3RAS::LWX;
    }
}

static unsigned decodeSB(uint32_t insn) {
    switch ((insn>>9)&0x1) {
    default:  return UNSUPPORTED;
    case 0x0: return T3RAS::SB;
    case 0x1: return T3RAS::SBR;
    }
}

static unsigned decodeSH(uint32_t insn) {
    switch ((insn>>9)&0x1) {
    default:  return UNSUPPORTED;
    case 0x0: return T3RAS::SH;
    case 0x1: return T3RAS::SHR;
    }
}

static unsigned decodeSW(uint32_t insn) {
    switch ((insn>>9)&0x3) {
    default:  return UNSUPPORTED;
    case 0x0: return T3RAS::SW;
    case 0x1: return T3RAS::SWR;
    case 0x2: return T3RAS::SWX;
    }
}

static unsigned decodeMFS(uint32_t insn) {
    switch ((insn>>15)&0x1) {
    default:   return UNSUPPORTED;
    case 0x0:
      switch ((insn>>16)&0x1) {
      default:   return UNSUPPORTED;
      case 0x0: return T3RAS::MSRSET;
      case 0x1: return T3RAS::MSRCLR;
      }
    case 0x1:
      switch ((insn>>14)&0x1) {
      default:   return UNSUPPORTED;
      case 0x0: return T3RAS::MFS;
      case 0x1: return T3RAS::MTS;
      }
    }
}

static unsigned decodeOR(uint32_t insn) {
    switch (getFLAGS(insn)) {
    default:    return UNSUPPORTED;
    case 0x000: return T3RAS::OR;
    //case 0x400: return T3RAS::PCMPBF;
    }
}

static unsigned decodeXOR(uint32_t insn) {
    switch (getFLAGS(insn)) {
    default:    return UNSUPPORTED;
    case 0x000: return T3RAS::XOR;
    //case 0x400: return T3RAS::PCMPEQ;
    }
}

static unsigned decodeANDN(uint32_t insn) {
    switch (getFLAGS(insn)) {
    default:    return UNSUPPORTED;
    case 0x000: return T3RAS::ANDN;
    //case 0x400: return T3RAS::PCMPNE;
    }
}

static unsigned decodeRTSD(uint32_t insn) {
    switch ((insn>>21)&0x1F) {
    default:   return UNSUPPORTED;
    case 0x10: return T3RAS::RTSD;
    case 0x11: return T3RAS::RTID;
    case 0x12: return T3RAS::RTBD;
    case 0x14: return T3RAS::RTED;
    }
}

static unsigned getOPCODE(uint32_t insn) {
  unsigned opcode = T3RASBinary2Opcode[ (insn>>26)&0x3F ];
  switch (opcode) {
  case T3RAS::MUL:     return decodeMUL(insn);
  case T3RAS::SEXT8:   return decodeSEXT(insn);
  case T3RAS::BEQ:     return decodeBEQ(insn);
  case T3RAS::BEQI:    return decodeBEQI(insn);
  case T3RAS::BR:      return decodeBR(insn);
  case T3RAS::BRI:     return decodeBRI(insn);
  case T3RAS::BSRL:    return decodeBSRL(insn);
  case T3RAS::BSRLI:   return decodeBSRLI(insn);
  case T3RAS::RSUBK:   return decodeRSUBK(insn);
  //case T3RAS::FADD:    return decodeFADD(insn);
  case T3RAS::GET:     return decodeGET(insn);
  //case T3RAS::GETD:    return decodeGETD(insn);
  //case T3RAS::IDIV:    return decodeIDIV(insn);
  case T3RAS::LBU:     return decodeLBU(insn);
  case T3RAS::LHU:     return decodeLHU(insn);
  case T3RAS::LW:      return decodeLW(insn);
  case T3RAS::SB:      return decodeSB(insn);
  case T3RAS::SH:      return decodeSH(insn);
  case T3RAS::SW:      return decodeSW(insn);
  case T3RAS::MFS:     return decodeMFS(insn);
  case T3RAS::OR:      return decodeOR(insn);
  case T3RAS::XOR:     return decodeXOR(insn);
  case T3RAS::ANDN:    return decodeANDN(insn);
  case T3RAS::RTSD:    return decodeRTSD(insn);
  default:              return opcode;
  }
}

const EDInstInfo *T3RASDisassembler::getEDInfo() const {
  return instInfoT3RAS;
}

//
// Public interface for the disassembler
//

MCDisassembler::DecodeStatus T3RASDisassembler::getInstruction(MCInst &instr,
                                        uint64_t &size,
                                        const MemoryObject &region,
                                        uint64_t address,
                                        raw_ostream &vStream,
                                        raw_ostream &cStream) const {
  // The machine instruction.
  uint32_t insn;
  uint64_t read;
  uint8_t bytes[4];

  // By default we consume 1 byte on failure
  size = 1;

  // We want to read exactly 4 bytes of data.
  if (region.readBytes(address, 4, (uint8_t*)bytes, &read) == -1 || read < 4)
    return Fail;

  // Encoded as a big-endian 32-bit word in the stream.
  insn = (bytes[0]<<24) | (bytes[1]<<16) | (bytes[2]<< 8) | (bytes[3]<<0);

  // Get the MCInst opcode from the binary instruction and make sure
  // that it is a valid instruction.
  unsigned opcode = getOPCODE(insn);
  if (opcode == UNSUPPORTED)
    return Fail;

  instr.setOpcode(opcode);

  unsigned RD = getRD(insn);
  unsigned RA = getRA(insn);
  unsigned RB = getRB(insn);
  unsigned RS = getRS(insn);

  uint64_t tsFlags = T3RASInsts[opcode].TSFlags;
  switch ((tsFlags & T3RASII::FormMask)) {
  default: 
    return Fail;

  case T3RASII::FC:
    break;

  case T3RASII::FRRRR:
    if (RD == UNSUPPORTED || RA == UNSUPPORTED || RB == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateReg(RB));
    instr.addOperand(MCOperand::CreateReg(RA));
    break;

  case T3RASII::FRRR:
    if (RD == UNSUPPORTED || RA == UNSUPPORTED || RB == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateReg(RA));
    instr.addOperand(MCOperand::CreateReg(RB));
    break;

  case T3RASII::FRR:
    if (RD == UNSUPPORTED || RA == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateReg(RA));
    break;

  case T3RASII::FRI:
    switch (opcode) {
    default: 
      return Fail;
    case T3RAS::MFS:
      if (RD == UNSUPPORTED)
        return Fail;
      instr.addOperand(MCOperand::CreateReg(RD));
      instr.addOperand(MCOperand::CreateImm(insn&0x3FFF));
      break;
    case T3RAS::MTS:
      if (RA == UNSUPPORTED)
        return Fail;
      instr.addOperand(MCOperand::CreateImm(insn&0x3FFF));
      instr.addOperand(MCOperand::CreateReg(RA));
      break;
    case T3RAS::MSRSET:
    case T3RAS::MSRCLR:
      if (RD == UNSUPPORTED)
        return Fail;
      instr.addOperand(MCOperand::CreateReg(RD));
      instr.addOperand(MCOperand::CreateImm(insn&0x7FFF));
      break;
    }
    break;

  case T3RASII::FRRI:
    if (RD == UNSUPPORTED || RA == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateReg(RA));
    switch (opcode) {
    default:
      instr.addOperand(MCOperand::CreateImm(getIMM(insn)));
      break;
    case T3RAS::BSRLI:
    case T3RAS::BSRAI:
    case T3RAS::BSLLI:
      instr.addOperand(MCOperand::CreateImm(insn&0x1F));
      break;
    }
    break;

  case T3RASII::FCRR:
    if (RA == UNSUPPORTED || RB == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RA));
    instr.addOperand(MCOperand::CreateReg(RB));
    break;

  case T3RASII::FCRI:
    if (RA == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RA));
    instr.addOperand(MCOperand::CreateImm(getIMM(insn)));
    break;

  case T3RASII::FRCR:
    if (RD == UNSUPPORTED || RB == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateReg(RB));
    break;

  case T3RASII::FRCI:
    if (RD == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateImm(getIMM(insn)));
    break;

  case T3RASII::FCCR:
    if (RB == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RB));
    break;

  case T3RASII::FCCI:
    instr.addOperand(MCOperand::CreateImm(getIMM(insn)));
    break;

  case T3RASII::FRRCI:
    if (RD == UNSUPPORTED || RA == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateReg(RA));
    instr.addOperand(MCOperand::CreateImm(getSHT(insn)));
    break;

  case T3RASII::FRRC:
    if (RD == UNSUPPORTED || RA == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateReg(RA));
    break;

  case T3RASII::FRCX:
    if (RD == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateImm(getFSL(insn)));
    break;

  case T3RASII::FRCS:
    if (RD == UNSUPPORTED || RS == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateReg(RS));
    break;

  case T3RASII::FCRCS:
    if (RS == UNSUPPORTED || RA == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RS));
    instr.addOperand(MCOperand::CreateReg(RA));
    break;

  case T3RASII::FCRCX:
    if (RA == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RA));
    instr.addOperand(MCOperand::CreateImm(getFSL(insn)));
    break;

  case T3RASII::FCX:
    instr.addOperand(MCOperand::CreateImm(getFSL(insn)));
    break;

  case T3RASII::FCR:
    if (RB == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RB));
    break;

  case T3RASII::FRIR:
    if (RD == UNSUPPORTED || RA == UNSUPPORTED)
      return Fail;
    instr.addOperand(MCOperand::CreateReg(RD));
    instr.addOperand(MCOperand::CreateImm(getIMM(insn)));
    instr.addOperand(MCOperand::CreateReg(RA));
    break;
  }

  // We always consume 4 bytes of data on success
  size = 4;

  return Success;
}

static MCDisassembler *createT3RASDisassembler(const Target &T,
                                                const MCSubtargetInfo &STI) {
  return new T3RASDisassembler(STI);
}

extern "C" void LLVMInitializeT3RASDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(TheT3RASTarget,
                                         createT3RASDisassembler);
}
