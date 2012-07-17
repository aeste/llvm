//===-- MBlazeTargetInfo.cpp - MBlaze Target Implementation ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "T3RAS.h"//this needs to be changed
#include "llvm/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target llvm::TheT3RASTarget;

extern "C" void LLVMInitializeT3RASTargetInfo() {
  RegisterTarget<Triple::T3RAS> X(TheT3RASTarget, "T3RAS", "T3RAS");
  //RegisterTarget<Triple::T3RAS> Y(TheT3RASTarget, "T3RAS1", "T3RAS1");
  //RegisterTarget<Triple::T3RAS> Z(TheT3RASTarget, "T3RAS4", "T3RAS4");
}
