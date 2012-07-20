//===-- MBlazeSelectionDAGInfo.h - MBlaze SelectionDAG Info -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the MBlaze subclass for TargetSelectionDAGInfo.
//
//===----------------------------------------------------------------------===//

#ifndef T3RASSELECTIONDAGINFO_H
#define T3RASSELECTIONDAGINFO_H

#include "llvm/Target/TargetSelectionDAGInfo.h"

namespace llvm {

class T3RASTargetMachine;

class T3RASSelectionDAGInfo : public TargetSelectionDAGInfo {
public:
  explicit T3RASSelectionDAGInfo(const T3RASTargetMachine &TM);
  ~T3RASSelectionDAGInfo();
};

}

#endif
