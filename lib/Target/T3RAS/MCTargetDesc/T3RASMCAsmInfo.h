//===-- MBlazeMCAsmInfo.h - MBlaze asm properties --------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the MBlazeMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef T3RASTARGETASMINFO_H
#define T3RASTARGETASMINFO_H

#include "llvm/MC/MCAsmInfo.h"

namespace llvm {
  class Target;

  class T3RASMCAsmInfo : public MCAsmInfo {
    virtual void anchor();
  public:
    explicit T3RASMCAsmInfo();
  };

} // namespace llvm

#endif
