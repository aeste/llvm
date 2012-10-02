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

//===-- MBlazeRelocations.h - MBlaze Code Relocations -----------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the MBlaze target-specific relocation types.
//
//===----------------------------------------------------------------------===//

#ifndef T3RASRELOCATIONS_H
#define T3RASRELOCATIONS_H

#include "llvm/CodeGen/MachineRelocation.h"

namespace llvm {
  namespace T3RAS {
    enum RelocationType {
      /// reloc_pcrel_word - PC relative relocation, add the relocated value to
      /// the value already in memory, after we adjust it for where the PC is.
      reloc_pcrel_word = 0,

      /// reloc_picrel_word - PIC base relative relocation, add the relocated
      /// value to the value already in memory, after we adjust it for where the
      /// PIC base is.
      reloc_picrel_word = 1,

      /// reloc_absolute_word - absolute relocation, just add the relocated
      /// value to the value already in memory.
      reloc_absolute_word = 2,

      /// reloc_absolute_word_sext - absolute relocation, just add the relocated
      /// value to the value already in memory. In object files, it represents a
      /// value which must be sign-extended when resolving the relocation.
      reloc_absolute_word_sext = 3,

      /// reloc_absolute_dword - absolute relocation, just add the relocated
      /// value to the value already in memory.
      reloc_absolute_dword = 4
    };
  }
}

#endif
