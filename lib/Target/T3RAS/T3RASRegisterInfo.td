//===-- MBlazeRegisterInfo.td - MBlaze Register defs -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the MicroBlaze register file
//===----------------------------------------------------------------------===//

// We have banks of 32 registers each.
class T3RASReg<string n> : Register<n> {
  field bits<5> Num;
  let Namespace = "T3RAS";
}

// Special purpose registers have 15-bit values
class T3RASSReg<string n> : Register<n> {
  field bits<15> Num;
  let Namespace = "T3RAS";
}

// MBlaze general purpose registers
class T3RASGPRReg<bits<5> num, string n> : T3RASReg<n> {
  let Num = num;
}

// MBlaze special purpose registers
class T3RASSPRReg<bits<15> num, string n> : T3RASSReg<n> {
  let Num = num;
}

//===----------------------------------------------------------------------===//
//  Registers
//===----------------------------------------------------------------------===//

let Namespace = "T3RAS" in {
  // General Purpose Registers
  def R0  : T3RASGPRReg< 0,  "r0">,   DwarfRegNum<[0]>;
  def R1  : T3RASGPRReg< 1,  "r1">,   DwarfRegNum<[1]>;
  def R2  : T3RASGPRReg< 2,  "r2">,   DwarfRegNum<[2]>;
  def R3  : T3RASGPRReg< 3,  "r3">,   DwarfRegNum<[3]>;
  def R4  : T3RASGPRReg< 4,  "r4">,   DwarfRegNum<[4]>;
  def R5  : T3RASGPRReg< 5,  "r5">,   DwarfRegNum<[5]>;
  def R6  : T3RASGPRReg< 6,  "r6">,   DwarfRegNum<[6]>;
  def R7  : T3RASGPRReg< 7,  "r7">,   DwarfRegNum<[7]>;
  def R8  : T3RASGPRReg< 8,  "r8">,   DwarfRegNum<[8]>;
  def R9  : T3RASGPRReg< 9,  "r9">,   DwarfRegNum<[9]>;
  def R10 : T3RASGPRReg< 10, "r10">,  DwarfRegNum<[10]>;
  def R11 : T3RASGPRReg< 11, "r11">,  DwarfRegNum<[11]>;
  def R12 : T3RASGPRReg< 12, "r12">,  DwarfRegNum<[12]>;
  def R13 : T3RASGPRReg< 13, "r13">,  DwarfRegNum<[13]>;
  def R14 : T3RASGPRReg< 14, "r14">,  DwarfRegNum<[14]>;
  def R15 : T3RASGPRReg< 15, "r15">,  DwarfRegNum<[15]>;
  def R16 : T3RASGPRReg< 16, "r16">,  DwarfRegNum<[16]>;
  def R17 : T3RASGPRReg< 17, "r17">,  DwarfRegNum<[17]>;
  def R18 : T3RASGPRReg< 18, "r18">,  DwarfRegNum<[18]>;
  def R19 : T3RASGPRReg< 19, "r19">,  DwarfRegNum<[19]>;
  def R20 : T3RASGPRReg< 20, "r20">,  DwarfRegNum<[20]>;
  def R21 : T3RASGPRReg< 21, "r21">,  DwarfRegNum<[21]>;
  def R22 : T3RASGPRReg< 22, "r22">,  DwarfRegNum<[22]>;
  def R23 : T3RASGPRReg< 23, "r23">,  DwarfRegNum<[23]>;
  def R24 : T3RASGPRReg< 24, "r24">,  DwarfRegNum<[24]>;
  def R25 : T3RASGPRReg< 25, "r25">,  DwarfRegNum<[25]>;
  def R26 : T3RASGPRReg< 26, "r26">,  DwarfRegNum<[26]>;
  def R27 : T3RASGPRReg< 27, "r27">,  DwarfRegNum<[27]>;
  def R28 : T3RASGPRReg< 28, "r28">,  DwarfRegNum<[28]>;
  def R29 : T3RASGPRReg< 29, "r29">,  DwarfRegNum<[29]>;
  def R30 : T3RASGPRReg< 30, "r30">,  DwarfRegNum<[30]>;
  def R31 : T3RASGPRReg< 31, "r31">,  DwarfRegNum<[31]>;

  // Special Purpose Registers
  def RPC    : T3RASSPRReg<0x0000, "rpc">,    DwarfRegNum<[32]>;
  def RMSR   : T3RASSPRReg<0x0001, "rmsr">,   DwarfRegNum<[33]>;
  def REAR   : T3RASSPRReg<0x0003, "rear">,   DwarfRegNum<[34]>;
  def RESR   : T3RASSPRReg<0x0005, "resr">,   DwarfRegNum<[35]>;
  def RFSR   : T3RASSPRReg<0x0007, "rfsr">,   DwarfRegNum<[36]>;
  def RBTR   : T3RASSPRReg<0x000B, "rbtr">,   DwarfRegNum<[37]>;
  def REDR   : T3RASSPRReg<0x000D, "redr">,   DwarfRegNum<[38]>;
  def RPID   : T3RASSPRReg<0x1000, "rpid">,   DwarfRegNum<[39]>;
  def RZPR   : T3RASSPRReg<0x1001, "rzpr">,   DwarfRegNum<[40]>;
  def RTLBX  : T3RASSPRReg<0x1002, "rtlbx">,  DwarfRegNum<[41]>;
  def RTLBLO : T3RASSPRReg<0x1003, "rtlblo">, DwarfRegNum<[42]>;
  def RTLBHI : T3RASSPRReg<0x1004, "rtlbhi">, DwarfRegNum<[43]>;
  def RTLBSX : T3RASSPRReg<0x1004, "rtlbsx">, DwarfRegNum<[44]>;
  def RPVR0  : T3RASSPRReg<0x2000, "rpvr0">,  DwarfRegNum<[45]>;
  def RPVR1  : T3RASSPRReg<0x2001, "rpvr1">,  DwarfRegNum<[46]>;
  def RPVR2  : T3RASSPRReg<0x2002, "rpvr2">,  DwarfRegNum<[47]>;
  def RPVR3  : T3RASSPRReg<0x2003, "rpvr3">,  DwarfRegNum<[48]>;
  def RPVR4  : T3RASSPRReg<0x2004, "rpvr4">,  DwarfRegNum<[49]>;
  def RPVR5  : T3RASSPRReg<0x2005, "rpvr5">,  DwarfRegNum<[50]>;
  def RPVR6  : T3RASSPRReg<0x2006, "rpvr6">,  DwarfRegNum<[51]>;
  def RPVR7  : T3RASSPRReg<0x2007, "rpvr7">,  DwarfRegNum<[52]>;
  def RPVR8  : T3RASSPRReg<0x2008, "rpvr8">,  DwarfRegNum<[53]>;
  def RPVR9  : T3RASSPRReg<0x2009, "rpvr9">,  DwarfRegNum<[54]>;
  def RPVR10 : T3RASSPRReg<0x200A, "rpvr10">, DwarfRegNum<[55]>;
  def RPVR11 : T3RASSPRReg<0x200B, "rpvr11">, DwarfRegNum<[56]>;

  // The carry bit. In the Microblaze this is really bit 29 of the
  // MSR register but this is the only bit of that register that we
  // are interested in modeling.
  def CARRY  : T3RASSPRReg<0x0000, "rmsr[c]">;
}

//===----------------------------------------------------------------------===//
// Register Classes
//===----------------------------------------------------------------------===//

def GPR : RegisterClass<"T3RAS", [i32,f32], 32, (sequence "R%u", 0, 31)>;

def SPR : RegisterClass<"T3RAS", [i32], 32, (add
  // Reserved
  RPC,
  RMSR,
  REAR,
  RESR,
  RFSR,
  RBTR,
  REDR,
  RPID,
  RZPR,
  RTLBX,
  RTLBLO,
  RTLBHI,
  RPVR0,
  RPVR1,
  RPVR2,
  RPVR3,
  RPVR4,
  RPVR5,
  RPVR6,
  RPVR7,
  RPVR8,
  RPVR9,
  RPVR10,
  RPVR11
  )>
{
  // None of the special purpose registers are allocatable.
  let isAllocatable = 0;
}

def CRC : RegisterClass<"T3RAS", [i32], 32, (add CARRY)> {
  let CopyCost = -1;
}
