//===-- MBlaze.td - Describe the MBlaze Target Machine -----*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This is the top level entry point for the MBlaze target.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Target-independent interfaces
//===----------------------------------------------------------------------===//

include "llvm/Target/Target.td"

//===----------------------------------------------------------------------===//
// Register File, Calling Conv, Instruction Descriptions
//===----------------------------------------------------------------------===//

include "T3RASRegisterInfo.td"
include "T3RASSchedule.td"
include "T3RASIntrinsics.td"
include "T3RASInstrInfo.td"
include "T3RASCallingConv.td"

def T3RASInstrInfo : InstrInfo;

//===----------------------------------------------------------------------===//
// T3RAS Subtarget features                                              //
//===----------------------------------------------------------------------===//

def FeatureBarrel      : SubtargetFeature<"barrel", "HasBarrel", "true",
                                "Implements barrel shifter">;
//def FeatureDiv         : SubtargetFeature<"div", "HasDiv", "true",
//                                "Implements hardware divider">;
def FeatureMul         : SubtargetFeature<"mul", "HasMul", "true",
                                "Implements hardware multiplier">;
//def FeaturePatCmp      : SubtargetFeature<"patcmp", "HasPatCmp", "true",
//                                "Implements pattern compare instruction">;
//def FeatureFPU         : SubtargetFeature<"fpu", "HasFPU", "true",
//                                "Implements floating point unit">;
//def FeatureMul64       : SubtargetFeature<"mul64", "HasMul64", "true",
//                                "Implements multiplier with 64-bit result">;
//def FeatureSqrt        : SubtargetFeature<"sqrt", "HasSqrt", "true",
//                                "Implements sqrt and floating point convert">;
def FeatureNoDelay         : SubtargetFeature<"hasnodelay", "HasNoDelay", "true",
                                "disables branching delay slots -default for T3RAS4T">;
def Feature1Delays         : SubtargetFeature<"1-delayslot", "DelaySlots", "1",
                                "enables 1 delay slot per branch -default for T3RAS2T">;
def Feature2Delays         : SubtargetFeature<"2-delayslot", "DelaySlots", "2",
                                "enables 2 delay slot per branch">;
def Feature3Delays         : SubtargetFeature<"3-delayslot", "DelaySlots", "3",
                                "enables 3 delay slot per branch -default for T3RAS">;
def FeatureNoHazardSolver         : SubtargetFeature<"nohazardsolver", "DisableHazardSolver", "true",
                                "disables hazard solver pass">;
//===----------------------------------------------------------------------===//
// MBlaze processors supported.
//===----------------------------------------------------------------------===//

def : Processor<"T3RAS1T",  T3RAS1TItineraries, [Feature3Delays]>;
def : Processor<"T3RAS2T", T3RAS2TItineraries, [Feature1Delays]>;
def : Processor<"T3RAS4T", T3RAS4TItineraries, [FeatureNoDelay]>;

//===----------------------------------------------------------------------===//
// Instruction Descriptions
//===----------------------------------------------------------------------===//

def T3RASAsmWriter : AsmWriter {
  string AsmWriterClassName  = "InstPrinter";
  bit isMCAsmWriter = 1;
}

//===----------------------------------------------------------------------===//
// Target Declaration
//===----------------------------------------------------------------------===//

def T3RAS : Target {
  let InstructionSet = T3RASInstrInfo;
  let AssemblyWriters = [T3RASAsmWriter];
}
