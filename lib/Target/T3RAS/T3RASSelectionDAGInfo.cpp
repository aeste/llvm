//===-- MBlazeSelectionDAGInfo.cpp - MBlaze SelectionDAG Info -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MBlazeSelectionDAGInfo class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "T3RAS-selectiondag-info"
#include "T3RASTargetMachine.h"
using namespace llvm;

T3RASSelectionDAGInfo::T3RASSelectionDAGInfo(const T3RASTargetMachine &TM)
  : TargetSelectionDAGInfo(TM) {
}

T3RASSelectionDAGInfo::~T3RASSelectionDAGInfo() {
}
