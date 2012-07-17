//===-- MBlazeELFWriterInfo.cpp - ELF Writer Info for the MBlaze backend --===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements ELF writer information for the MBlaze backend.
//
//===----------------------------------------------------------------------===//

#include "T3RASELFWriterInfo.h"
#include "T3RASRelocations.h"
#include "llvm/Function.h"
#include "llvm/Support/ELF.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetMachine.h"

using namespace llvm;

//===----------------------------------------------------------------------===//
//  Implementation of the MBlazeELFWriterInfo class
//===----------------------------------------------------------------------===//

T3RASELFWriterInfo::T3RASELFWriterInfo(TargetMachine &TM)
  : TargetELFWriterInfo(TM.getTargetData()->getPointerSizeInBits() == 64,
                        TM.getTargetData()->isLittleEndian()) {
}

T3RASELFWriterInfo::~T3RASELFWriterInfo() {}

unsigned T3RASELFWriterInfo::getRelocationType(unsigned MachineRelTy) const {
  switch (MachineRelTy) {
  case T3RAS::reloc_pcrel_word:
    return ELF::R_MICROBLAZE_64_PCREL;
  case T3RAS::reloc_absolute_word:
    return ELF::R_MICROBLAZE_NONE;
  default:
    llvm_unreachable("unknown T3RAS machine relocation type");
  }
}

long int T3RASELFWriterInfo::getDefaultAddendForRelTy(unsigned RelTy,
                                                    long int Modifier) const {
  switch (RelTy) {
  case ELF::R_MICROBLAZE_32_PCREL:
    return Modifier - 4;
  case ELF::R_MICROBLAZE_32:
    return Modifier;
  default:
    llvm_unreachable("unknown T3RAS relocation type");
  }
}

unsigned T3RASELFWriterInfo::getRelocationTySize(unsigned RelTy) const {
  // FIXME: Most of these sizes are guesses based on the name
  switch (RelTy) {
  case ELF::R_MICROBLAZE_32:
  case ELF::R_MICROBLAZE_32_PCREL:
  case ELF::R_MICROBLAZE_32_PCREL_LO:
  case ELF::R_MICROBLAZE_32_LO:
  case ELF::R_MICROBLAZE_SRO32:
  case ELF::R_MICROBLAZE_SRW32:
  case ELF::R_MICROBLAZE_32_SYM_OP_SYM:
  case ELF::R_MICROBLAZE_GOTOFF_32:
    return 32;

  case ELF::R_MICROBLAZE_64_PCREL:
  case ELF::R_MICROBLAZE_64:
  case ELF::R_MICROBLAZE_GOTPC_64:
  case ELF::R_MICROBLAZE_GOT_64:
  case ELF::R_MICROBLAZE_PLT_64:
  case ELF::R_MICROBLAZE_GOTOFF_64:
    return 64;
  }

  return 0;
}

bool T3RASELFWriterInfo::isPCRelativeRel(unsigned RelTy) const {
  // FIXME: Most of these are guesses based on the name
  switch (RelTy) {
  case ELF::R_MICROBLAZE_32_PCREL:
  case ELF::R_MICROBLAZE_64_PCREL:
  case ELF::R_MICROBLAZE_32_PCREL_LO:
  case ELF::R_MICROBLAZE_GOTPC_64:
    return true;
  }

  return false;
}

unsigned T3RASELFWriterInfo::getAbsoluteLabelMachineRelTy() const {
  return T3RAS::reloc_absolute_word;
}

long int T3RASELFWriterInfo::computeRelocation(unsigned SymOffset,
                                                unsigned RelOffset,
                                                unsigned RelTy) const {
  assert((RelTy == ELF::R_MICROBLAZE_32_PCREL ||
          RelTy == ELF::R_MICROBLAZE_64_PCREL) &&
         "computeRelocation unknown for this relocation type");
  return SymOffset - (RelOffset + 4);
}
