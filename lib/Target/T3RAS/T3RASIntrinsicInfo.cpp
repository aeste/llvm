//===-- MBlazeIntrinsicInfo.cpp - Intrinsic Information -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the MBlaze implementation of TargetIntrinsicInfo.
//
//===----------------------------------------------------------------------===//

#include "T3RASIntrinsicInfo.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Function.h"
#include "llvm/Intrinsics.h"
#include "llvm/Module.h"
#include "llvm/Type.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/ErrorHandling.h"
#include <cstring>

using namespace llvm;

namespace T3RASIntrinsic {

  enum ID {
    last_non_T3RAS_intrinsic = Intrinsic::num_intrinsics-1,
#define GET_INTRINSIC_ENUM_VALUES
#include "T3RASGenIntrinsics.inc"
#undef GET_INTRINSIC_ENUM_VALUES
    , num_T3RAS_intrinsics
  };

#define GET_LLVM_INTRINSIC_FOR_GCC_BUILTIN
#include "T3RASGenIntrinsics.inc"
#undef GET_LLVM_INTRINSIC_FOR_GCC_BUILTIN
}

std::string T3RASIntrinsicInfo::getName(unsigned IntrID, Type **Tys,
                                         unsigned numTys) const {
  static const char *const names[] = {
#define GET_INTRINSIC_NAME_TABLE
#include "T3RASGenIntrinsics.inc"
#undef GET_INTRINSIC_NAME_TABLE
  };

  assert(!isOverloaded(IntrID) && "T3RAS intrinsics are not overloaded");
  if (IntrID < Intrinsic::num_intrinsics)
    return 0;
  assert(IntrID < T3RASIntrinsic::num_T3RAS_intrinsics &&
         "Invalid intrinsic ID");

  std::string Result(names[IntrID - Intrinsic::num_intrinsics]);
  return Result;
}

unsigned T3RASIntrinsicInfo::
lookupName(const char *Name, unsigned Len) const {
  if (Len < 5 || Name[4] != '.' || Name[0] != 'l' || Name[1] != 'l'
      || Name[2] != 'v' || Name[3] != 'm')
    return 0;  // All intrinsics start with 'llvm.'

#define GET_FUNCTION_RECOGNIZER
#include "T3RASGenIntrinsics.inc"
#undef GET_FUNCTION_RECOGNIZER
  return 0;
}

unsigned T3RASIntrinsicInfo::
lookupGCCName(const char *Name) const {
    return T3RASIntrinsic::getIntrinsicForGCCBuiltin("T3RAS",Name);
}

bool T3RASIntrinsicInfo::isOverloaded(unsigned IntrID) const {
  if (IntrID == 0)
    return false;

  unsigned id = IntrID - Intrinsic::num_intrinsics + 1;
#define GET_INTRINSIC_OVERLOAD_TABLE
#include "T3RASGenIntrinsics.inc"
#undef GET_INTRINSIC_OVERLOAD_TABLE
}

/// This defines the "getAttributes(ID id)" method.
#define GET_INTRINSIC_ATTRIBUTES
#include "T3RASGenIntrinsics.inc"
#undef GET_INTRINSIC_ATTRIBUTES

static FunctionType *getType(LLVMContext &Context, unsigned id) {
  Type *ResultTy = NULL;
  SmallVector<Type*, 8> ArgTys;
  bool IsVarArg = false;

#define GET_INTRINSIC_GENERATOR
#include "T3RASGenIntrinsics.inc"
#undef GET_INTRINSIC_GENERATOR

  return FunctionType::get(ResultTy, ArgTys, IsVarArg);
}

Function *T3RASIntrinsicInfo::getDeclaration(Module *M, unsigned IntrID,
                                                Type **Tys,
                                                unsigned numTy) const {
  assert(!isOverloaded(IntrID) && "T3RAS intrinsics are not overloaded");
  AttrListPtr AList = getAttributes((T3RASIntrinsic::ID) IntrID);
  return cast<Function>(M->getOrInsertFunction(getName(IntrID),
                                               getType(M->getContext(), IntrID),
                                               AList));
}
