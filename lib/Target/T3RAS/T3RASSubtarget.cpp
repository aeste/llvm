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
  HasBarrel(false), HasMul(false),//, HasDiv(false), HasPatCmp(false),
  //HasFPU(false), HasMul64(false), HasSqrt(false)
	HasNoDelay(false), DelaySlots(1),DisableHazardSolver(false)
{

  // Parse features string.
  std::string CPUName = CPU;
  if (CPUName.empty())
    CPUName = "T3RAS1T";
  ParseSubtargetFeatures(CPUName, FS);
  // Only use instruction scheduling if the selected CPU has an instruction
  // itinerary (the default CPU is the only one that doesn't).
  HasItin = CPUName != "MBlaze";
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
