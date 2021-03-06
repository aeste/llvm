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

//===-- MBlazeInstrFSL.td - MBlaze FSL Instruction defs ----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// FSL Instruction Formats
//===----------------------------------------------------------------------===//
class FSLGet<bits<6> op, bits<5> flags, string instr_asm, Intrinsic OpNode> :
             T3RASInst<op, FRCX, (outs GPR:$dst), (ins fslimm:$b),
                        !strconcat(instr_asm, " $dst, $b"),
                        [(set GPR:$dst, (OpNode immZExt4:$b))],IIC_FSLg>
{
    bits<5> rd;
    bits<4> fslno;

    let Inst{6-10}  = rd;
    let Inst{11-15} = 0x0;
    let Inst{16}    = 0x0;
    let Inst{17-21} = flags; // NCTAE
    let Inst{22-27} = 0x0;
    let Inst{28-31} = fslno;
}

class FSLGetD<bits<6> op, bits<5> flags, string instr_asm, Intrinsic OpNode> :
              T3RASInst<op, FRCR, (outs GPR:$dst), (ins GPR:$b),
                         !strconcat(instr_asm, " $dst, $b"),
                         [(set GPR:$dst, (OpNode GPR:$b))], IIC_FSLg>
{
    bits<5> rd;
    bits<5> rb;

    let Inst{6-10}  = rd;
    let Inst{11-15} = 0x0;
    let Inst{16-20} = rb;
    let Inst{21}    = 0x0;
    let Inst{22-26} = flags; // NCTAE
    let Inst{27-31} = 0x0;
}

class FSLPut<bits<6> op, bits<4> flags, string instr_asm, Intrinsic OpNode> :
             T3RASInst<op, FCRCX, (outs), (ins GPR:$v, fslimm:$b),
                        !strconcat(instr_asm, " $v, $b"),
                        [(OpNode GPR:$v, immZExt4:$b)], IIC_FSLp>
{
    bits<5> ra;
    bits<4> fslno;

    let Inst{6-10}  = 0x0;
    let Inst{11-15} = ra;
    let Inst{16}    = 0x1;
    let Inst{17-20} = flags; // NCTA
    let Inst{21-27} = 0x0;
    let Inst{28-31} = fslno;
}

class FSLPutD<bits<6> op, bits<4> flags, string instr_asm, Intrinsic OpNode> :
              T3RASInst<op, FCRR, (outs), (ins GPR:$v, GPR:$b),
                         !strconcat(instr_asm, " $v, $b"),
                         [(OpNode GPR:$v, GPR:$b)], IIC_FSLp>
{
    bits<5> ra;
    bits<5> rb;

    let Inst{6-10}  = 0x0;
    let Inst{11-15} = ra;
    let Inst{16-20} = rb;
    let Inst{21}    = 0x1;
    let Inst{22-25} = flags; // NCTA
    let Inst{26-31} = 0x0;
}

class FSLPutT<bits<6> op, bits<4> flags, string instr_asm, Intrinsic OpNode> :
              T3RASInst<op, FCX, (outs), (ins fslimm:$b),
                         !strconcat(instr_asm, " $b"),
                         [(OpNode immZExt4:$b)], IIC_FSLp>
{
    bits<4> fslno;

    let Inst{6-10}  = 0x0;
    let Inst{11-15} = 0x0;
    let Inst{16}    = 0x1;
    let Inst{17-20} = flags; // NCTA
    let Inst{21-27} = 0x0;
    let Inst{28-31} = fslno;
}

class FSLPutTD<bits<6> op, bits<4> flags, string instr_asm, Intrinsic OpNode> :
               T3RASInst<op, FCR, (outs), (ins GPR:$b),
                          !strconcat(instr_asm, " $b"),
                          [(OpNode GPR:$b)], IIC_FSLp>
{
    bits<5> rb;

    let Inst{6-10}  = 0x0;
    let Inst{11-15} = 0x0;
    let Inst{16-20} = rb;
    let Inst{21}    = 0x1;
    let Inst{22-25} = flags; // NCTA
    let Inst{26-31} = 0x0;
}

//===----------------------------------------------------------------------===//
// FSL Get Instructions
//===----------------------------------------------------------------------===//
def GET      : FSLGet<0x1B, 0x00, "get      ", int_T3RAS_fsl_get>;
def AGET     : FSLGet<0x1B, 0x02, "aget     ", int_T3RAS_fsl_aget>;
def CGET     : FSLGet<0x1B, 0x08, "cget     ", int_T3RAS_fsl_cget>;
def CAGET    : FSLGet<0x1B, 0x0A, "caget    ", int_T3RAS_fsl_caget>;
def EGET     : FSLGet<0x1B, 0x01, "eget     ", int_T3RAS_fsl_eget>;
def EAGET    : FSLGet<0x1B, 0x03, "eaget    ", int_T3RAS_fsl_eaget>;
def ECGET    : FSLGet<0x1B, 0x09, "ecget    ", int_T3RAS_fsl_ecget>;
def ECAGET   : FSLGet<0x1B, 0x0B, "ecaget   ", int_T3RAS_fsl_ecaget>;
def TGET     : FSLGet<0x1B, 0x04, "tget     ", int_T3RAS_fsl_tget>;
def TAGET    : FSLGet<0x1B, 0x06, "taget    ", int_T3RAS_fsl_taget>;
def TCGET    : FSLGet<0x1B, 0x0C, "tcget    ", int_T3RAS_fsl_tcget>;
def TCAGET   : FSLGet<0x1B, 0x0E, "tcaget   ", int_T3RAS_fsl_tcaget>;
def TEGET    : FSLGet<0x1B, 0x05, "teget    ", int_T3RAS_fsl_teget>;
def TEAGET   : FSLGet<0x1B, 0x07, "teaget   ", int_T3RAS_fsl_teaget>;
def TECGET   : FSLGet<0x1B, 0x0D, "tecget   ", int_T3RAS_fsl_tecget>;
def TECAGET  : FSLGet<0x1B, 0x0F, "tecaget  ", int_T3RAS_fsl_tecaget>;

let Defs = [CARRY] in {
  def NGET     : FSLGet<0x1B, 0x10, "nget     ", int_T3RAS_fsl_nget>;
  def NAGET    : FSLGet<0x1B, 0x12, "naget    ", int_T3RAS_fsl_naget>;
  def NCGET    : FSLGet<0x1B, 0x18, "ncget    ", int_T3RAS_fsl_ncget>;
  def NCAGET   : FSLGet<0x1B, 0x1A, "ncaget   ", int_T3RAS_fsl_ncaget>;
  def NEGET    : FSLGet<0x1B, 0x11, "neget    ", int_T3RAS_fsl_neget>;
  def NEAGET   : FSLGet<0x1B, 0x13, "neaget   ", int_T3RAS_fsl_neaget>;
  def NECGET   : FSLGet<0x1B, 0x19, "necget   ", int_T3RAS_fsl_necget>;
  def NECAGET  : FSLGet<0x1B, 0x1B, "necaget  ", int_T3RAS_fsl_necaget>;
  def TNGET    : FSLGet<0x1B, 0x14, "tnget    ", int_T3RAS_fsl_tnget>;
  def TNAGET   : FSLGet<0x1B, 0x16, "tnaget   ", int_T3RAS_fsl_tnaget>;
  def TNCGET   : FSLGet<0x1B, 0x1C, "tncget   ", int_T3RAS_fsl_tncget>;
  def TNCAGET  : FSLGet<0x1B, 0x1E, "tncaget  ", int_T3RAS_fsl_tncaget>;
  def TNEGET   : FSLGet<0x1B, 0x15, "tneget   ", int_T3RAS_fsl_tneget>;
  def TNEAGET  : FSLGet<0x1B, 0x17, "tneaget  ", int_T3RAS_fsl_tneaget>;
  def TNECGET  : FSLGet<0x1B, 0x1D, "tnecget  ", int_T3RAS_fsl_tnecget>;
  def TNECAGET : FSLGet<0x1B, 0x1F, "tnecaget ", int_T3RAS_fsl_tnecaget>;
}

//===----------------------------------------------------------------------===//
// FSL Dynamic Get Instructions
//===----------------------------------------------------------------------===//
def GETD      : FSLGetD<0x13, 0x00, "getd     ", int_T3RAS_fsl_get>;
def AGETD     : FSLGetD<0x13, 0x02, "agetd    ", int_T3RAS_fsl_aget>;
def CGETD     : FSLGetD<0x13, 0x08, "cgetd    ", int_T3RAS_fsl_cget>;
def CAGETD    : FSLGetD<0x13, 0x0A, "cagetd   ", int_T3RAS_fsl_caget>;
def EGETD     : FSLGetD<0x13, 0x01, "egetd    ", int_T3RAS_fsl_eget>;
def EAGETD    : FSLGetD<0x13, 0x03, "eagetd   ", int_T3RAS_fsl_eaget>;
def ECGETD    : FSLGetD<0x13, 0x09, "ecgetd   ", int_T3RAS_fsl_ecget>;
def ECAGETD   : FSLGetD<0x13, 0x0B, "ecagetd  ", int_T3RAS_fsl_ecaget>;
def TGETD     : FSLGetD<0x13, 0x04, "tgetd    ", int_T3RAS_fsl_tget>;
def TAGETD    : FSLGetD<0x13, 0x06, "tagetd   ", int_T3RAS_fsl_taget>;
def TCGETD    : FSLGetD<0x13, 0x0C, "tcgetd   ", int_T3RAS_fsl_tcget>;
def TCAGETD   : FSLGetD<0x13, 0x0E, "tcagetd  ", int_T3RAS_fsl_tcaget>;
def TEGETD    : FSLGetD<0x13, 0x05, "tegetd   ", int_T3RAS_fsl_teget>;
def TEAGETD   : FSLGetD<0x13, 0x07, "teagetd  ", int_T3RAS_fsl_teaget>;
def TECGETD   : FSLGetD<0x13, 0x0D, "tecgetd  ", int_T3RAS_fsl_tecget>;
def TECAGETD  : FSLGetD<0x13, 0x0F, "tecagetd ", int_T3RAS_fsl_tecaget>;

let Defs = [CARRY] in {
  def NGETD     : FSLGetD<0x13, 0x10, "ngetd    ", int_T3RAS_fsl_nget>;
  def NAGETD    : FSLGetD<0x13, 0x12, "nagetd   ", int_T3RAS_fsl_naget>;
  def NCGETD    : FSLGetD<0x13, 0x18, "ncgetd   ", int_T3RAS_fsl_ncget>;
  def NCAGETD   : FSLGetD<0x13, 0x1A, "ncagetd  ", int_T3RAS_fsl_ncaget>;
  def NEGETD    : FSLGetD<0x13, 0x11, "negetd   ", int_T3RAS_fsl_neget>;
  def NEAGETD   : FSLGetD<0x13, 0x13, "neagetd  ", int_T3RAS_fsl_neaget>;
  def NECGETD   : FSLGetD<0x13, 0x19, "necgetd  ", int_T3RAS_fsl_necget>;
  def NECAGETD  : FSLGetD<0x13, 0x1B, "necagetd ", int_T3RAS_fsl_necaget>;
  def TNGETD    : FSLGetD<0x13, 0x14, "tngetd   ", int_T3RAS_fsl_tnget>;
  def TNAGETD   : FSLGetD<0x13, 0x16, "tnagetd  ", int_T3RAS_fsl_tnaget>;
  def TNCGETD   : FSLGetD<0x13, 0x1C, "tncgetd  ", int_T3RAS_fsl_tncget>;
  def TNCAGETD  : FSLGetD<0x13, 0x1E, "tncagetd ", int_T3RAS_fsl_tncaget>;
  def TNEGETD   : FSLGetD<0x13, 0x15, "tnegetd  ", int_T3RAS_fsl_tneget>;
  def TNEAGETD  : FSLGetD<0x13, 0x17, "tneagetd ", int_T3RAS_fsl_tneaget>;
  def TNECGETD  : FSLGetD<0x13, 0x1D, "tnecgetd ", int_T3RAS_fsl_tnecget>;
  def TNECAGETD : FSLGetD<0x13, 0x1F, "tnecagetd", int_T3RAS_fsl_tnecaget>;
}

//===----------------------------------------------------------------------===//
// FSL Put Instructions
//===----------------------------------------------------------------------===//
def PUT     :  FSLPut<0x1B, 0x0, "put      ", int_T3RAS_fsl_put>;
def APUT    :  FSLPut<0x1B, 0x1, "aput     ", int_T3RAS_fsl_aput>;
def CPUT    :  FSLPut<0x1B, 0x4, "cput     ", int_T3RAS_fsl_cput>;
def CAPUT   :  FSLPut<0x1B, 0x5, "caput    ", int_T3RAS_fsl_caput>;
def TPUT    : FSLPutT<0x1B, 0x2, "tput     ", int_T3RAS_fsl_tput>;
def TAPUT   : FSLPutT<0x1B, 0x3, "taput    ", int_T3RAS_fsl_taput>;
def TCPUT   : FSLPutT<0x1B, 0x6, "tcput    ", int_T3RAS_fsl_tcput>;
def TCAPUT  : FSLPutT<0x1B, 0x7, "tcaput   ", int_T3RAS_fsl_tcaput>;

let Defs = [CARRY] in {
  def NPUT    :  FSLPut<0x1B, 0x8, "nput     ", int_T3RAS_fsl_nput>;
  def NAPUT   :  FSLPut<0x1B, 0x9, "naput    ", int_T3RAS_fsl_naput>;
  def NCPUT   :  FSLPut<0x1B, 0xC, "ncput    ", int_T3RAS_fsl_ncput>;
  def NCAPUT  :  FSLPut<0x1B, 0xD, "ncaput   ", int_T3RAS_fsl_ncaput>;
  def TNPUT   : FSLPutT<0x1B, 0xA, "tnput    ", int_T3RAS_fsl_tnput>;
  def TNAPUT  : FSLPutT<0x1B, 0xB, "tnaput   ", int_T3RAS_fsl_tnaput>;
  def TNCPUT  : FSLPutT<0x1B, 0xE, "tncput   ", int_T3RAS_fsl_tncput>;
  def TNCAPUT : FSLPutT<0x1B, 0xF, "tncaput  ", int_T3RAS_fsl_tncaput>;
}

//===----------------------------------------------------------------------===//
// FSL Dynamic Put Instructions
//===----------------------------------------------------------------------===//
def PUTD     :  FSLPutD<0x13, 0x0, "putd     ", int_T3RAS_fsl_put>;
def APUTD    :  FSLPutD<0x13, 0x1, "aputd    ", int_T3RAS_fsl_aput>;
def CPUTD    :  FSLPutD<0x13, 0x4, "cputd    ", int_T3RAS_fsl_cput>;
def CAPUTD   :  FSLPutD<0x13, 0x5, "caputd   ", int_T3RAS_fsl_caput>;
def TPUTD    : FSLPutTD<0x13, 0x2, "tputd    ", int_T3RAS_fsl_tput>;
def TAPUTD   : FSLPutTD<0x13, 0x3, "taputd   ", int_T3RAS_fsl_taput>;
def TCPUTD   : FSLPutTD<0x13, 0x6, "tcputd   ", int_T3RAS_fsl_tcput>;
def TCAPUTD  : FSLPutTD<0x13, 0x7, "tcaputd  ", int_T3RAS_fsl_tcaput>;

let Defs = [CARRY] in {
  def NPUTD    :  FSLPutD<0x13, 0x8, "nputd    ", int_T3RAS_fsl_nput>;
  def NAPUTD   :  FSLPutD<0x13, 0x9, "naputd   ", int_T3RAS_fsl_naput>;
  def NCPUTD   :  FSLPutD<0x13, 0xC, "ncputd   ", int_T3RAS_fsl_ncput>;
  def NCAPUTD  :  FSLPutD<0x13, 0xD, "ncaputd  ", int_T3RAS_fsl_ncaput>;
  def TNPUTD   : FSLPutTD<0x13, 0xA, "tnputd   ", int_T3RAS_fsl_tnput>;
  def TNAPUTD  : FSLPutTD<0x13, 0xB, "tnaputd  ", int_T3RAS_fsl_tnaput>;
  def TNCPUTD  : FSLPutTD<0x13, 0xE, "tncputd  ", int_T3RAS_fsl_tncput>;
  def TNCAPUTD : FSLPutTD<0x13, 0xF, "tncaputd ", int_T3RAS_fsl_tncaput>;
}
