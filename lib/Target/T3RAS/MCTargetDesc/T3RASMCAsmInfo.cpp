//===-- MBlazeMCAsmInfo.cpp - MBlaze asm properties -----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the MBlazeMCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "T3RASMCAsmInfo.h"
using namespace llvm;

void T3RASMCAsmInfo::anchor() { }

T3RASMCAsmInfo::T3RASMCAsmInfo() {
  IsLittleEndian              = false;
  StackGrowsUp                = false;
  SupportsDebugInformation    = true;
  AlignmentIsInBytes          = false;
  PrivateGlobalPrefix         = "$";
  GPRel32Directive            = "\t.gpword\t";
}
