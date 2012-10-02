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

//===-- MBlazeAsmLexer.cpp - Tokenize MBlaze assembly to AsmTokens --------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/T3RASBaseInfo.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCTargetAsmLexer.h"

#include "llvm/Support/TargetRegistry.h"

#include <string>
#include <map>

using namespace llvm;

namespace {
  
  class T3RASBaseAsmLexer : public MCTargetAsmLexer {
    const MCAsmInfo &AsmInfo;
    
    const AsmToken &lexDefinite() {
      return getLexer()->Lex();
    }
    
    AsmToken LexTokenUAL();
  protected:
    typedef std::map <std::string, unsigned> rmap_ty;
    
    rmap_ty RegisterMap;
    
    void InitRegisterMap(const MCRegisterInfo *info) {
      unsigned numRegs = info->getNumRegs();

      for (unsigned i = 0; i < numRegs; ++i) {
        const char *regName = info->getName(i);
        if (regName)
          RegisterMap[regName] = i;
      }
    }
    
    unsigned MatchRegisterName(StringRef Name) {
      rmap_ty::iterator iter = RegisterMap.find(Name.str());
      if (iter != RegisterMap.end())
        return iter->second;
      else
        return 0;
    }
    
    AsmToken LexToken() {
      if (!Lexer) {
        SetError(SMLoc(), "No MCAsmLexer installed");
        return AsmToken(AsmToken::Error, "", 0);
      }
      
      switch (AsmInfo.getAssemblerDialect()) {
      default:
        SetError(SMLoc(), "Unhandled dialect");
        return AsmToken(AsmToken::Error, "", 0);
      case 0:
        return LexTokenUAL();
      }
    }
  public:
    T3RASBaseAsmLexer(const Target &T, const MCAsmInfo &MAI)
      : MCTargetAsmLexer(T), AsmInfo(MAI) {
    }
  };
  
  class T3RASAsmLexer : public T3RASBaseAsmLexer {
  public:
    T3RASAsmLexer(const Target &T, const MCRegisterInfo &MRI,
                   const MCAsmInfo &MAI)
      : T3RASBaseAsmLexer(T, MAI) {
      InitRegisterMap(&MRI);
    }
  };
}

AsmToken T3RASBaseAsmLexer::LexTokenUAL() {
  const AsmToken &lexedToken = lexDefinite();
  
  switch (lexedToken.getKind()) {
  default:
    return AsmToken(lexedToken);
  case AsmToken::Error:
    SetError(Lexer->getErrLoc(), Lexer->getErr());
    return AsmToken(lexedToken);
  case AsmToken::Identifier:
  {
    unsigned regID = MatchRegisterName(lexedToken.getString().lower());
    
    if (regID) {
      return AsmToken(AsmToken::Register,
                      lexedToken.getString(),
                      static_cast<int64_t>(regID));
    } else {
      return AsmToken(lexedToken);
    }
  }
  }
}

extern "C" void LLVMInitializeT3RASAsmLexer() {
  RegisterMCAsmLexer<T3RASAsmLexer> X(TheT3RASTarget);
}

