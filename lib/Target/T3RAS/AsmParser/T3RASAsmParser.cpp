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

//===-- MBlazeAsmParser.cpp - Parse MBlaze asm to MCInst instructions -----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T3RASBaseInfo.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCAsmParser.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCTargetAsmParser.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/Twine.h"
using namespace llvm;

namespace {
struct T3RASOperand;

class T3RASAsmParser : public MCTargetAsmParser {
  MCAsmParser &Parser;

  MCAsmParser &getParser() const { return Parser; }
  MCAsmLexer &getLexer() const { return Parser.getLexer(); }

  void Warning(SMLoc L, const Twine &Msg) { Parser.Warning(L, Msg); }
  bool Error(SMLoc L, const Twine &Msg) { return Parser.Error(L, Msg); }

  T3RASOperand *ParseMemory(SmallVectorImpl<MCParsedAsmOperand*> &Operands);
  T3RASOperand *ParseRegister(unsigned &RegNo);
  T3RASOperand *ParseImmediate();
  T3RASOperand *ParseFsl();
  T3RASOperand* ParseOperand(SmallVectorImpl<MCParsedAsmOperand*> &Operands);

  virtual bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc);

  bool ParseDirectiveWord(unsigned Size, SMLoc L);

  bool MatchAndEmitInstruction(SMLoc IDLoc,
                               SmallVectorImpl<MCParsedAsmOperand*> &Operands,
                               MCStreamer &Out);

  /// @name Auto-generated Match Functions
  /// {

#define GET_ASSEMBLER_HEADER
#include "T3RASGenAsmMatcher.inc"

  /// }


public:
  T3RASAsmParser(MCSubtargetInfo &_STI, MCAsmParser &_Parser)
    : MCTargetAsmParser(), Parser(_Parser) {}

  virtual bool ParseInstruction(StringRef Name, SMLoc NameLoc,
                                SmallVectorImpl<MCParsedAsmOperand*> &Operands);

  virtual bool ParseDirective(AsmToken DirectiveID);
};

/// MBlazeOperand - Instances of this class represent a parsed MBlaze machine
/// instruction.
struct T3RASOperand : public MCParsedAsmOperand {
  enum KindTy {
    Token,
    Immediate,
    Register,
    Memory,
    Fsl
  } Kind;

  SMLoc StartLoc, EndLoc;

  union {
    struct {
      const char *Data;
      unsigned Length;
    } Tok;

    struct {
      unsigned RegNum;
    } Reg;

    struct {
      const MCExpr *Val;
    } Imm;

    struct {
      unsigned Base;
      unsigned OffReg;
      const MCExpr *Off;
    } Mem;

    struct {
      const MCExpr *Val;
    } FslImm;
  };

  T3RASOperand(KindTy K) : MCParsedAsmOperand(), Kind(K) {}
public:
  T3RASOperand(const T3RASOperand &o) : MCParsedAsmOperand() {
    Kind = o.Kind;
    StartLoc = o.StartLoc;
    EndLoc = o.EndLoc;
    switch (Kind) {
    case Register:
      Reg = o.Reg;
      break;
    case Immediate:
      Imm = o.Imm;
      break;
    case Token:
      Tok = o.Tok;
      break;
    case Memory:
      Mem = o.Mem;
      break;
    case Fsl:
      FslImm = o.FslImm;
      break;
    }
  }

  /// getStartLoc - Get the location of the first token of this operand.
  SMLoc getStartLoc() const { return StartLoc; }

  /// getEndLoc - Get the location of the last token of this operand.
  SMLoc getEndLoc() const { return EndLoc; }

  unsigned getReg() const {
    assert(Kind == Register && "Invalid access!");
    return Reg.RegNum;
  }

  const MCExpr *getImm() const {
    assert(Kind == Immediate && "Invalid access!");
    return Imm.Val;
  }

  const MCExpr *getFslImm() const {
    assert(Kind == Fsl && "Invalid access!");
    return FslImm.Val;
  }

  unsigned getMemBase() const {
    assert(Kind == Memory && "Invalid access!");
    return Mem.Base;
  }

  const MCExpr* getMemOff() const {
    assert(Kind == Memory && "Invalid access!");
    return Mem.Off;
  }

  unsigned getMemOffReg() const {
    assert(Kind == Memory && "Invalid access!");
    return Mem.OffReg;
  }

  bool isToken() const { return Kind == Token; }
  bool isImm() const { return Kind == Immediate; }
  bool isMem() const { return Kind == Memory; }
  bool isFsl() const { return Kind == Fsl; }
  bool isReg() const { return Kind == Register; }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    // Add as immediates when possible.  Null MCExpr = 0.
    if (Expr == 0)
      Inst.addOperand(MCOperand::CreateImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::CreateImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::CreateExpr(Expr));
  }

  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    Inst.addOperand(MCOperand::CreateReg(getReg()));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getImm());
  }

  void addFslOperands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    addExpr(Inst, getFslImm());
  }

  void addMemOperands(MCInst &Inst, unsigned N) const {
    assert(N == 2 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::CreateReg(getMemBase()));

    unsigned RegOff = getMemOffReg();
    if (RegOff)
      Inst.addOperand(MCOperand::CreateReg(RegOff));
    else
      addExpr(Inst, getMemOff());
  }

  StringRef getToken() const {
    assert(Kind == Token && "Invalid access!");
    return StringRef(Tok.Data, Tok.Length);
  }

  virtual void print(raw_ostream &OS) const;

  static T3RASOperand *CreateToken(StringRef Str, SMLoc S) {
    T3RASOperand *Op = new T3RASOperand(Token);
    Op->Tok.Data = Str.data();
    Op->Tok.Length = Str.size();
    Op->StartLoc = S;
    Op->EndLoc = S;
    return Op;
  }

  static T3RASOperand *CreateReg(unsigned RegNum, SMLoc S, SMLoc E) {
    T3RASOperand *Op = new T3RASOperand(Register);
    Op->Reg.RegNum = RegNum;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static T3RASOperand *CreateImm(const MCExpr *Val, SMLoc S, SMLoc E) {
    T3RASOperand *Op = new T3RASOperand(Immediate);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static T3RASOperand *CreateFslImm(const MCExpr *Val, SMLoc S, SMLoc E) {
    T3RASOperand *Op = new T3RASOperand(Fsl);
    Op->Imm.Val = Val;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static T3RASOperand *CreateMem(unsigned Base, const MCExpr *Off, SMLoc S,
                                  SMLoc E) {
    T3RASOperand *Op = new T3RASOperand(Memory);
    Op->Mem.Base = Base;
    Op->Mem.Off = Off;
    Op->Mem.OffReg = 0;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }

  static T3RASOperand *CreateMem(unsigned Base, unsigned Off, SMLoc S,
                                  SMLoc E) {
    T3RASOperand *Op = new T3RASOperand(Memory);
    Op->Mem.Base = Base;
    Op->Mem.OffReg = Off;
    Op->Mem.Off = 0;
    Op->StartLoc = S;
    Op->EndLoc = E;
    return Op;
  }
};

} // end anonymous namespace.

void T3RASOperand::print(raw_ostream &OS) const {
  switch (Kind) {
  case Immediate:
    getImm()->print(OS);
    break;
  case Register:
    OS << "<register R";
    OS << getT3RASRegisterNumbering(getReg()) << ">";
    break;
  case Token:
    OS << "'" << getToken() << "'";
    break;
  case Memory: {
    OS << "<memory R";
    OS << getT3RASRegisterNumbering(getMemBase());
    OS << ", ";

    unsigned RegOff = getMemOffReg();
    if (RegOff)
      OS << "R" << getT3RASRegisterNumbering(RegOff);
    else
      OS << getMemOff();
    OS << ">";
    }
    break;
  case Fsl:
    getFslImm()->print(OS);
    break;
  }
}

/// @name Auto-generated Match Functions
/// {

static unsigned MatchRegisterName(StringRef Name);

/// }
//
bool T3RASAsmParser::
MatchAndEmitInstruction(SMLoc IDLoc,
                        SmallVectorImpl<MCParsedAsmOperand*> &Operands,
                        MCStreamer &Out) {
  MCInst Inst;
  SMLoc ErrorLoc;
  unsigned ErrorInfo;

  switch (MatchInstructionImpl(Operands, Inst, ErrorInfo)) {
  default: break;
  case Match_Success:
    Out.EmitInstruction(Inst);
    return false;
  case Match_MissingFeature:
    return Error(IDLoc, "instruction use requires an option to be enabled");
  case Match_MnemonicFail:
      return Error(IDLoc, "unrecognized instruction mnemonic");
  case Match_ConversionFail:
    return Error(IDLoc, "unable to convert operands to instruction");
  case Match_InvalidOperand:
    ErrorLoc = IDLoc;
    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size())
        return Error(IDLoc, "too few operands for instruction");

      ErrorLoc = ((T3RASOperand*)Operands[ErrorInfo])->getStartLoc();
      if (ErrorLoc == SMLoc()) ErrorLoc = IDLoc;
    }

    return Error(ErrorLoc, "invalid operand for instruction");
  }

  llvm_unreachable("Implement any new match types added!");
}

T3RASOperand *T3RASAsmParser::
ParseMemory(SmallVectorImpl<MCParsedAsmOperand*> &Operands) {
  if (Operands.size() != 4)
    return 0;

  T3RASOperand &Base = *(T3RASOperand*)Operands[2];
  T3RASOperand &Offset = *(T3RASOperand*)Operands[3];

  SMLoc S = Base.getStartLoc();
  SMLoc O = Offset.getStartLoc();
  SMLoc E = Offset.getEndLoc();

  if (!Base.isReg()) {
    Error(S, "base address must be a register");
    return 0;
  }

  if (!Offset.isReg() && !Offset.isImm()) {
    Error(O, "offset must be a register or immediate");
    return 0;
  }

  T3RASOperand *Op;
  if (Offset.isReg())
    Op = T3RASOperand::CreateMem(Base.getReg(), Offset.getReg(), S, E);
  else
    Op = T3RASOperand::CreateMem(Base.getReg(), Offset.getImm(), S, E);

  delete Operands.pop_back_val();
  delete Operands.pop_back_val();
  Operands.push_back(Op);

  return Op;
}

bool T3RASAsmParser::ParseRegister(unsigned &RegNo,
                                    SMLoc &StartLoc, SMLoc &EndLoc) {
  return (ParseRegister(RegNo) == 0);
}

T3RASOperand *T3RASAsmParser::ParseRegister(unsigned &RegNo) {
  SMLoc S = Parser.getTok().getLoc();
  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

  switch (getLexer().getKind()) {
  default: return 0;
  case AsmToken::Identifier:
    RegNo = MatchRegisterName(getLexer().getTok().getIdentifier());
    if (RegNo == 0)
      return 0;

    getLexer().Lex();
    return T3RASOperand::CreateReg(RegNo, S, E);
  }
}

static unsigned MatchFslRegister(StringRef String) {
  if (!String.startswith("rfsl"))
    return -1;

  unsigned regNum;
  if (String.substr(4).getAsInteger(10,regNum))
    return -1;

  return regNum;
}

T3RASOperand *T3RASAsmParser::ParseFsl() {
  SMLoc S = Parser.getTok().getLoc();
  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

  switch (getLexer().getKind()) {
  default: return 0;
  case AsmToken::Identifier:
    unsigned reg = MatchFslRegister(getLexer().getTok().getIdentifier());
    if (reg >= 16)
      return 0;

    getLexer().Lex();
    const MCExpr *EVal = MCConstantExpr::Create(reg,getContext());
    return T3RASOperand::CreateFslImm(EVal,S,E);
  }
}

T3RASOperand *T3RASAsmParser::ParseImmediate() {
  SMLoc S = Parser.getTok().getLoc();
  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);

  const MCExpr *EVal;
  switch (getLexer().getKind()) {
  default: return 0;
  case AsmToken::LParen:
  case AsmToken::Plus:
  case AsmToken::Minus:
  case AsmToken::Integer:
  case AsmToken::Identifier:
    if (getParser().ParseExpression(EVal))
      return 0;

    return T3RASOperand::CreateImm(EVal, S, E);
  }
}

T3RASOperand *T3RASAsmParser::
ParseOperand(SmallVectorImpl<MCParsedAsmOperand*> &Operands) {
  T3RASOperand *Op;

  // Attempt to parse the next token as a register name
  unsigned RegNo;
  Op = ParseRegister(RegNo);

  // Attempt to parse the next token as an FSL immediate
  if (!Op)
    Op = ParseFsl();

  // Attempt to parse the next token as an immediate
  if (!Op)
    Op = ParseImmediate();

  // If the token could not be parsed then fail
  if (!Op) {
    Error(Parser.getTok().getLoc(), "unknown operand");
    return 0;
  }

  // Push the parsed operand into the list of operands
  Operands.push_back(Op);
  return Op;
}

/// Parse an mblaze instruction mnemonic followed by its operands.
bool T3RASAsmParser::
ParseInstruction(StringRef Name, SMLoc NameLoc,
                 SmallVectorImpl<MCParsedAsmOperand*> &Operands) {
  // The first operands is the token for the instruction name
  size_t dotLoc = Name.find('.');
  Operands.push_back(T3RASOperand::CreateToken(Name.substr(0,dotLoc),NameLoc));
  if (dotLoc < Name.size())
    Operands.push_back(T3RASOperand::CreateToken(Name.substr(dotLoc),NameLoc));

  // If there are no more operands then finish
  if (getLexer().is(AsmToken::EndOfStatement))
    return false;

  // Parse the first operand
  if (!ParseOperand(Operands))
    return true;

  while (getLexer().isNot(AsmToken::EndOfStatement) &&
         getLexer().is(AsmToken::Comma)) {
    // Consume the comma token
    getLexer().Lex();

    // Parse the next operand
    if (!ParseOperand(Operands))
      return true;
  }

  // If the instruction requires a memory operand then we need to
  // replace the last two operands (base+offset) with a single
  // memory operand.
  if (Name.startswith("lw") || Name.startswith("sw") ||
      Name.startswith("lh") || Name.startswith("sh") ||
      Name.startswith("lb") || Name.startswith("sb"))
    return (ParseMemory(Operands) == NULL);

  return false;
}

/// ParseDirective parses the MBlaze specific directives
bool T3RASAsmParser::ParseDirective(AsmToken DirectiveID) {
  StringRef IDVal = DirectiveID.getIdentifier();
  if (IDVal == ".word")
    return ParseDirectiveWord(2, DirectiveID.getLoc());
  return true;
}

/// ParseDirectiveWord
///  ::= .word [ expression (, expression)* ]
bool T3RASAsmParser::ParseDirectiveWord(unsigned Size, SMLoc L) {
  if (getLexer().isNot(AsmToken::EndOfStatement)) {
    for (;;) {
      const MCExpr *Value;
      if (getParser().ParseExpression(Value))
        return true;

      getParser().getStreamer().EmitValue(Value, Size, 0 /*addrspace*/);

      if (getLexer().is(AsmToken::EndOfStatement))
        break;

      // FIXME: Improve diagnostic.
      if (getLexer().isNot(AsmToken::Comma))
        return Error(L, "unexpected token in directive");
      Parser.Lex();
    }
  }

  Parser.Lex();
  return false;
}

extern "C" void LLVMInitializeT3RASAsmLexer();

/// Force static initialization.
extern "C" void LLVMInitializeT3RASAsmParser() {
  RegisterMCAsmParser<T3RASAsmParser> X(TheT3RASTarget);
  LLVMInitializeT3RASAsmLexer();
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "T3RASGenAsmMatcher.inc"
