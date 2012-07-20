//===-- MBlazeSubtarget.cpp - MBlaze Subtarget Information ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MBlaze specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "T3RASSubtarget.h"
#include "T3RAS.h"
#include "T3RASRegisterInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "T3RASGenSubtargetInfo.inc"

using namespace llvm;

T3RASSubtarget::T3RASSubtarget(const std::string &TT,
                                 const std::string &CPU,
                                 const std::string &FS):
  T3RASGenSubtargetInfo(TT, CPU, FS),
  HasBarrel(false), HasDiv(false), HasMul(false), HasPatCmp(false),
  HasFPU(false), HasMul64(false), HasSqrt(false)
{
  // Parse features string.
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = "T3RAS";
  ParseSubtargetFeatures(CPUName, FS);

  // Only use instruction scheduling if the selected CPU has an instruction
  // itinerary (the default CPU is the only one that doesn't).
  HasItin = CPUName != "T3RAS";
  DEBUG(dbgs() << "CPU " << CPUName << "(" << HasItin << ")\n");

  // Initialize scheduling itinerary for the specified CPU.
  InstrItins = getInstrItineraryForCPU(CPUName);

  // Compute the issue width of the MBlaze itineraries
  computeIssueWidth();
}

void T3RASSubtarget::computeIssueWidth() {
  InstrItins.IssueWidth = 1;
}

bool T3RASSubtarget::
enablePostRAScheduler(CodeGenOpt::Level OptLevel,
                      TargetSubtargetInfo::AntiDepBreakMode& Mode,
                      RegClassVector& CriticalPathRCs) const {
  Mode = TargetSubtargetInfo::ANTIDEP_CRITICAL;
  CriticalPathRCs.clear();
  CriticalPathRCs.push_back(&T3RAS::GPRRegClass);
  return HasItin && OptLevel >= CodeGenOpt::Default;
}
