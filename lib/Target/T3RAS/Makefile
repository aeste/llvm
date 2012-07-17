##===- lib/Target/MBlaze/Makefile --------------------------*- Makefile -*-===##
#
#                     The LLVM Compiler Infrastructure
#
# This file is distributed under the University of Illinois Open Source
# License. See LICENSE.TXT for details.
#
##===----------------------------------------------------------------------===##
LEVEL = ../../..
LIBRARYNAME = LLVMT3RASCodeGen
TARGET = T3RAS

# Make sure that tblgen is run, first thing.
BUILT_SOURCES = T3RASGenRegisterInfo.inc T3RASGenInstrInfo.inc \
		T3RASGenAsmWriter.inc \
                T3RASGenDAGISel.inc T3RASGenAsmMatcher.inc \
                T3RASGenCodeEmitter.inc T3RASGenCallingConv.inc \
                T3RASGenSubtargetInfo.inc T3RASGenIntrinsics.inc \
                T3RASGenEDInfo.inc

DIRS = InstPrinter AsmParser Disassembler TargetInfo MCTargetDesc

include $(LEVEL)/Makefile.common

