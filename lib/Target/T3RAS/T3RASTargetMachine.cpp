 //===-- MBlazeTargetMachine.cpp - Define TargetMachine for MBlaze ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Implements the info about MBlaze target spec.
//
//===----------------------------------------------------------------------===//

#include "T3RASTargetMachine.h"
#include "T3RAS.h"
#include "llvm/PassManager.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Target/TargetOptions.h"
using namespace llvm;

extern "C" void LLVMInitializeT3RASTarget() {
  // Register the target.
  RegisterTargetMachine<T3RASTargetMachine> X(TheT3RASTarget);
}

// DataLayout --> Big-endian, 32-bit pointer/ABI/alignment
// The stack is always 8 byte aligned
// On function prologue, the stack is created by decrementing
// its pointer. Once decremented, all references are done with positive
// offset from the stack/frame pointer, using StackGrowsUp enables
// an easier handling.
T3RASTargetMachine::
T3RASTargetMachine(const Target &T, StringRef TT,
                    StringRef CPU, StringRef FS, const TargetOptions &Options,
                    Reloc::Model RM, CodeModel::Model CM,
                    CodeGenOpt::Level OL)
  : LLVMTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL),
    Subtarget(TT, CPU, FS),
    DataLayout("E-p:32:32:32-i8:8:8-i16:16:16"),
    InstrInfo(*this),
    FrameLowering(Subtarget),
    TLInfo(*this), TSInfo(*this), ELFWriterInfo(*this),
    InstrItins(Subtarget.getInstrItineraryData()) {
}

namespace {
/// MBlaze Code Generator Pass Configuration Options.
class T3RASPassConfig : public TargetPassConfig {
public:
  T3RASPassConfig(T3RASTargetMachine *TM, PassManagerBase &PM)
    : TargetPassConfig(TM, PM) {}

  T3RASTargetMachine &getT3RASTargetMachine() const {
    return getTM<T3RASTargetMachine>();
  }

  virtual bool addInstSelector();
  virtual bool addPreEmitPass();
};
} // namespace

TargetPassConfig *T3RASTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new T3RASPassConfig(this, PM);
}

// Install an instruction selector pass using
// the ISelDag to gen MBlaze code.
bool T3RASPassConfig::addInstSelector() {
  PM->add(createT3RASISelDag(getT3RASTargetMachine()));
  return false;
}

// Implemented by targets that want to run passes immediately before
// machine code is emitted. return true if -print-machineinstrs should
// print out the code after the passes.
bool T3RASPassConfig::addPreEmitPass() {
  PM->add(createT3RASDelaySlotFillerPass(getT3RASTargetMachine()));
  return true;
}
//TODO:add a pre emit pass here to solve data hazards
