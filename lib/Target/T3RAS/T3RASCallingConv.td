//===- MBlazeCallingConv.td - Calling Conventions for MBlaze -*- tablegen -*-=//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for MBlaze architecture.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// MBlaze ABI Calling Convention
//===----------------------------------------------------------------------===//

def RetCC_T3RAS : CallingConv<[
  // i32 are returned in registers R3, R4
  CCIfType<[i32,f32], CCAssignToReg<[R3, R4]>>
]>;

def CC_T3RAS : CallingConv<[
  CCIfType<[i32,f32], CCCustom<"CC_T3RAS_AssignReg">>,
  CCIfType<[i32,f32], CCAssignToStack<4, 4>>
]>;
