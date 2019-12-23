#ifndef __CODEGEN_H__
#define __CODEGEN_H__

#include "header.h"
#include "symbolTable.h"

#ifdef __cplusplus
extern "C" {
#endif

void codeGen(AST_NODE *root);

#ifdef __cplusplus
}
#endif

#endif
