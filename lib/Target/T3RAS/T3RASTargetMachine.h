//===-- MBlazeTargetMachine.h - Define TargetMachine for MBlaze -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the MBlaze specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

//#ifndef T3RAS_TARGETMACHINE_H
#define T3RAS_TARGETMACHINE_H

#include "T3RASSubtarget.h"
#include "T3RASInstrInfo.h"
#include "T3RASISelLowering.h"
#include "T3RASSelectionDAGInfo.h"
#include "T3RASIntrinsicInfo.h"
#include "T3RASFrameLowering.h"
#include "T3RASELFWriterInfo.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetFrameLowering.h"

namespace llvm {
  class formatted_raw_ostream;

  class T3RASTargetMachine : public LLVMTargetMachine {
    T3RASSubtarget        Subtarget;
    const TargetData       DataLayout; // Calculates type size & alignment
    T3RASInstrInfo        InstrInfo;
    T3RASFrameLowering    FrameLowering;
    T3RASTargetLowering   TLInfo;
    T3RASSelectionDAGInfo TSInfo;
    T3RASIntrinsicInfo    IntrinsicInfo;
    T3RASELFWriterInfo    ELFWriterInfo;
    InstrItineraryData     InstrItins;

  public:
    T3RASTargetMachine(const Target &T, StringRef TT,
                        StringRef CPU, StringRef FS,
                        const TargetOptions &Options,
                        Reloc::Model RM, CodeModel::Model CM,
                        CodeGenOpt::Level OL);

    virtual const T3RASInstrInfo *getInstrInfo() const
    { return &InstrInfo; }

    virtual const InstrItineraryData *getInstrItineraryData() const
    {  return &InstrItins; }

    virtual const TargetFrameLowering *getFrameLowering() const
    { return &FrameLowering; }

    virtual const T3RASSubtarget *getSubtargetImpl() const
    { return &Subtarget; }

    virtual const TargetData *getTargetData() const
    { return &DataLayout;}

    virtual const T3RASRegisterInfo *getRegisterInfo() const
    { return &InstrInfo.getRegisterInfo(); }

    virtual const T3RASTargetLowering *getTargetLowering() const
    { return &TLInfo; }

    virtual const T3RASSelectionDAGInfo* getSelectionDAGInfo() const
    { return &TSInfo; }

    const TargetIntrinsicInfo *getIntrinsicInfo() const
    { return &IntrinsicInfo; }

    virtual const T3RASELFWriterInfo *getELFWriterInfo() const {
      return &ELFWriterInfo;
    }

    // Pass Pipeline Configuration
    virtual TargetPassConfig *createPassConfig(PassManagerBase &PM);
  };
} // End llvm namespace

//#endif
