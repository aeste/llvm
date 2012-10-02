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

//===-- MBlazeSchedule.td - MBlaze Scheduling Definitions --*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// MBlaze functional units.
//===----------------------------------------------------------------------===//
def IF : FuncUnit;
def ID : FuncUnit;
def EX : FuncUnit;
def MA : FuncUnit;
def WB : FuncUnit;

//===----------------------------------------------------------------------===//
// Instruction Itinerary classes used for MBlaze
//===----------------------------------------------------------------------===//
def IIC_ALU    : InstrItinClass;
def IIC_ALUm   : InstrItinClass;
def IIC_ALUd   : InstrItinClass;
def IIC_SHT    : InstrItinClass;
def IIC_FSLg   : InstrItinClass;
def IIC_FSLp   : InstrItinClass;
def IIC_MEMs   : InstrItinClass;
def IIC_MEMl   : InstrItinClass;
/*def IIC_FPU    : InstrItinClass;
def IIC_FPUd   : InstrItinClass;
def IIC_FPUf   : InstrItinClass;
def IIC_FPUi   : InstrItinClass;
def IIC_FPUs   : InstrItinClass;
def IIC_FPUc   : InstrItinClass;*/
def IIC_BR     : InstrItinClass;
def IIC_BRc    : InstrItinClass;
def IIC_BRl    : InstrItinClass;
def IIC_lwi	:InstrItinClass;
def IIC_lw	:InstrItinClass;
//def IIC_WDC    : InstrItinClass;
def IIC_Pseudo : InstrItinClass;

//===----------------------------------------------------------------------===//
// MBlaze generic instruction itineraries.
//===----------------------------------------------------------------------===//
def T3RASGenericItineraries : ProcessorItineraries<[], [], []>;

//===----------------------------------------------------------------------===//
// T3RAS instruction itineraries for 1 Threaded CPU.
//===----------------------------------------------------------------------===//
include "T3RASSchedule1T.td"

//===----------------------------------------------------------------------===//
// T3RAS instruction itineraries for 2 Threaded CPU.
//===----------------------------------------------------------------------===//
include "T3RASSchedule2T.td"

//===----------------------------------------------------------------------===//
// MBlaze instruction itineraries for 4 Threaded CPU.
//===----------------------------------------------------------------------===//
include "T3RASSchedule4T.td"
