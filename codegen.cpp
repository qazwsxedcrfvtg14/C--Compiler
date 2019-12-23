#include "codegen.h"
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <unordered_map>
#include <variant>
#include <vector>
using namespace std;

int function_frame_size;

string cur_function_name;

enum class Inst {
  Mv,      // Move
  Fmv,     // Floating-point Move
  La,      // Load address
  Li,      // Load im
  Lw,      // Load global 32bit
  Ld,      // Load global 64bit
  Sw,      // Store global 32bit
  Sw_sym,  // Store global 32bit
  Sd,      // Store global 64bit
  Sd_sym,  // Store global 64bit
  Flw,     // Floating-point load global
  Flw_sym, // Floating-point load global
  Fsw,     // Floating-point store global
  Fsw_sym, // Floating-point store global
  Addi,    // Addi
  Add,     // Add
  Sub,     // Sub
  Mul,     // Mul
  Div,     // Div
  Fadd,    // Floating-point Add
  Fsub,    // Floating-point Sub
  Fmul,    // Floating-point Mul
  Fdiv,    // Floating-point Div
  And,     // And
  Or,      // Or
  Fneg,
  Fcvt_w_s,
  Fcvt_s_w,
  Slt,  // Set if <
  Sgt,  // Set if >
  Feq,  // Set if =
  Flt,  // Set if <
  Fgt,  // Set if >
  Fle,  // Set if <=
  Fge,  // Set if >=
  Seqz, // Set if = zero
  Snez, // Set if != zero
  Sltz, // Set if < zero
  Sgtz, // Set if > zero
  Slez, // Set if <= zero
  Sgez, // Set if >= zero
  Beq,
  Bne,
  Blt,
  Bgt,
  Ble,
  Bge,
  Beqz,
  Bnez,
  Bltz,
  Bgtz,
  Blez,
  Bgez,
  J,
  Jr,
  Jal,
  Call,
  Label,   // label:
  Segment, // .segment
};

enum class Reg {
  x0 = 0,
  zero = 0, // hardwired to 0, ignores writes
  x1 = 1,
  ra = 1, // return address for jumps
  x2 = 2,
  sp = 2, // stack pointer
  x3 = 3,
  gp = 3, // global pointer
  x4 = 4,
  tp = 4, // thread pointer
  x5 = 5,
  t0 = 5, // temporary register 0
  x6 = 6,
  t1 = 6, // temporary register 1
  x7 = 7,
  t2 = 7, // temporary register 2
  x8 = 8,
  s0 = 8, // saved register 0
  fp = 8, // frame pointer
  x9 = 9,
  s1 = 9, // saved register 1
  x10 = 10,
  a0 = 10, // return value or function argument 0
  x11 = 11,
  a1 = 11, // return value or function argument 1
  x12 = 12,
  a2 = 12, // function argument 2
  x13 = 13,
  a3 = 13, // function argument 3
  x14 = 14,
  a4 = 14, // function argument 4
  x15 = 15,
  a5 = 15, // function argument 5
  x16 = 16,
  a6 = 16, // function argument 6
  x17 = 17,
  a7 = 17, // function argument 7
  x18 = 18,
  s2 = 18, // saved register 2
  x19 = 19,
  s3 = 19, // saved register 3
  x20 = 20,
  s4 = 20, // saved register 4
  x21 = 21,
  s5 = 21, // saved register 5
  x22 = 22,
  s6 = 22, // saved register 6
  x23 = 23,
  s7 = 23, // saved register 7
  x24 = 24,
  s8 = 24, // saved register 8
  x25 = 25,
  s9 = 25, // saved register 9
  x26 = 26,
  s10 = 26, // saved register 10
  x27 = 27,
  s11 = 27, // saved register 11
  x28 = 28,
  t3 = 28, // temporary register 3
  x29 = 29,
  t4 = 29, // temporary register 4
  x30 = 30,
  t5 = 30, // temporary register 5
  x31 = 31,
  t6 = 31, // temporary register 6
  pc = 32,
};
enum class FReg {
  f0 = 0,
  ft0 = 0,
  f1 = 1,
  ft1 = 1,
  f2 = 2,
  ft2 = 2,
  f3 = 3,
  ft3 = 3,
  f4 = 4,
  ft4 = 4,
  f5 = 5,
  ft5 = 5,
  f6 = 6,
  ft6 = 6,
  f7 = 7,
  ft7 = 7,
  f8 = 8,
  fs0 = 8,
  f9 = 9,
  fs1 = 9,
  f10 = 10,
  fa0 = 10,
  f11 = 11,
  fa1 = 11,
  f12 = 12,
  fa2 = 12,
  f13 = 13,
  fa3 = 13,
  f14 = 14,
  fa4 = 14,
  f15 = 15,
  fa5 = 15,
  f16 = 16,
  fa6 = 16,
  f17 = 17,
  fa7 = 17,
  f18 = 18,
  fs2 = 18,
  f19 = 19,
  fs3 = 19,
  f20 = 20,
  fs4 = 20,
  f21 = 21,
  fs5 = 21,
  f22 = 22,
  fs6 = 12,
  f23 = 23,
  fs7 = 23,
  f24 = 24,
  fs8 = 24,
  f25 = 25,
  fs9 = 25,
  f26 = 26,
  fs10 = 26,
  f27 = 27,
  fs11 = 27,
  f28 = 28,
  ft8 = 28,
  f29 = 29,
  ft9 = 29,
  f30 = 30,
  ft10 = 30,
  f31 = 31,
  ft11 = 31,
  null = 32
};

/*const char reg_name[33][4] = {
    "x0",  "x1",  "x2",  "x3",  "x4",  "x5",  "x6",  "x7",  "x8",  "x9",  "x10",
    "x11", "x12", "x13", "x14", "x15", "x16", "x17", "x18", "x19", "x20", "x21",
    "x22", "x23", "x24", "x25", "x26", "x27", "x28", "x29", "x30", "x31",
   "pc"};*/
const char reg_name[33][4] = {
    "x0", "ra", "sp", "gp", "tp",  "t0",  "t1", "t2", "fp", "s1", "a0",
    "a1", "a2", "a3", "a4", "a5",  "a6",  "a7", "s2", "s3", "s4", "s5",
    "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6", "pc"};

const char freg_name[33][5] = {
    "ft0",  "ft1", "ft2", "ft3",  "ft4",  "ft5", "ft6", "ft7", "fs0",
    "fs1",  "fa0", "fa1", "fa2",  "fa3",  "fa4", "fa5", "fa6", "fa7",
    "fs2",  "fs3", "fs4", "fs5",  "fs6",  "fs7", "fs8", "fs9", "fs10",
    "fs11", "ft8", "ft9", "ft10", "ft11", "null"};

enum class FrameType {
  Unknown,
  Int,
  Float,
  IntArray,
  FloatArray,
};

struct FrameVar {
  int frame;
  FrameType type;
};

using Symbol = string;
using SReg = shared_ptr<Reg>;
using FSReg = shared_ptr<FReg>;
using SmartReg = variant<Reg, SReg, FrameVar, FReg, FSReg>;
using InstArg = variant<Reg, FReg, Symbol, int, double>;

using CodeDec = vector<pair<Inst, vector<InstArg>>>;
CodeDec codes, codes_tmp;

class Node {
public:
  Node(AST_NODE *_node) : node(_node) {}
  Node(AST_NODE &_node) : node(&_node) {}
  Node(Node &_node) : node(_node.node) {}
  Node child() const { return Node(node->child); }
  Node begin() const { return Node(node); }
  Node end() const { return Node(nullptr); }
  bool operator!=(Node target) const { return node != target.node; }
  bool operator==(Node target) const { return node == target.node; }
  void operator++() { node = node->rightSibling; }
  Node operator[](int level) const {
    auto result = node;
    for (int i = 0; result && i < level; i++)
      result = result->rightSibling;
    return result;
  }
  AST_NODE &operator*() const { return *node; }
  AST_NODE *operator->() const { return node; }
  operator bool() const { return static_cast<bool>(node); }
  auto &identifier() { return node->semantic_value.identifierSemanticValue; }
  auto &expr() { return node->semantic_value.exprSemanticValue; }
  auto &stmt() { return node->semantic_value.stmtSemanticValue; }
  auto &decl() { return node->semantic_value.declSemanticValue; }
  auto &cons() { return node->semantic_value.const1; }
  string name() { return identifier().identifierName; }

private:
  AST_NODE *node;
};

namespace Gen {
template <typename... Args> vector<InstArg> iargs(Args &&... args) {
  return {std::forward<Args>(args)...};
}
template <typename... Args>
pair<Inst, vector<InstArg>> insts(Inst inst, Args &&... args) {
  return make_pair(inst, iargs(std::forward<Args>(args)...));
}
void segment(Symbol seg) { codes.push_back(insts(Inst::Segment, seg)); }
void align(int num) { codes.push_back(insts(Inst::Segment, "align", num)); }
template <typename... Args> void label(Symbol lab, Args &&... args) {
  codes.push_back(insts(Inst::Label, lab, std::forward<Args>(args)...));
}
template <typename... Args> void inst(Inst ins, Args &&... args) {
  codes.push_back(insts(ins, std::forward<Args>(args)...));
}
} // namespace Gen
double getConstFValue(Node node) {
  if (node->nodeType == CONST_VALUE_NODE) {
    if (node->dataType == INT_TYPE)
      return node.cons()->const_u.intval;
    else
      return node.cons()->const_u.fval;
  } else {
    if (node->dataType == INT_TYPE)
      return node.expr().constEvalValue.iValue;
    else
      return node.expr().constEvalValue.fValue;
  }
}
string escape(string str) {
  /*string result;
  char tmp[3];
  for (unsigned char c : str) {
    if (c < 32 || c > 127) {
      sprintf(tmp, "%02x", c);
      result += "\\x"s + tmp;
    } else {
      result += c;
    }
  }
  return "\"" + result + "\\000\"";*/
  return str.substr(0, str.length() - 1) + "\\000\"";
}
int getConstIValue(Node node) { return static_cast<int>(getConstFValue(node)); }
void genStmt(Node node);
void genBlock(Node node);
SmartReg genFunctionCall(Node node);
SmartReg genExpr(Node node);
SmartReg genArrayLocation(Node node);
int const_id = 0;

queue<Reg> temp_queue;
Reg getNextTemp() {
  Reg reg = temp_queue.front();
  temp_queue.pop();
  temp_queue.push(reg);
  return reg;
}

queue<FReg> f_temp_queue;
FReg getNextFTemp() {
  FReg reg = f_temp_queue.front();
  f_temp_queue.pop();
  f_temp_queue.push(reg);
  return reg;
}

void F2I(FReg a, Reg b) { Gen::inst(Inst::Fcvt_w_s, b, a); }

void I2F(Reg a, FReg b) { Gen::inst(Inst::Fcvt_s_w, b, a); }

Reg getAddr(const SmartReg &reg, Reg avoid = Reg::x0) {
  if (auto r = get_if<FrameVar>(&reg)) {
    Reg tmp = getNextTemp();
    if (tmp == avoid)
      tmp = getNextTemp();
    Gen::inst(Inst::Addi, tmp, Reg::fp, -r->frame);
    return tmp;
  } else {
    fprintf(stderr, "empty getAddr");
    return Reg::x0;
  }
}

Reg getReg(const SmartReg &reg, Reg avoid = Reg::x0) {
  if (auto r = get_if<SReg>(&reg))
    return **r;
  else if (auto r = get_if<FrameVar>(&reg)) {
    Reg tmp = getNextTemp();
    if (tmp == avoid)
      tmp = getNextTemp();
    if (r->type == FrameType::Float) {
      FReg ftmp = getNextFTemp();
      Gen::inst(Inst::Flw, ftmp, -r->frame, Reg::fp);
      F2I(ftmp, tmp);
    } else {
      Gen::inst(Inst::Lw, tmp, -r->frame, Reg::fp);
    }
    return tmp;
  } else if (auto r = get_if<Reg>(&reg)) {
    return *r;
  } else if (auto r = get_if<FSReg>(&reg)) {
    Reg tmp = getNextTemp();
    if (tmp == avoid)
      tmp = getNextTemp();
    F2I(**r, tmp);
    return Reg::t0;
  } else if (auto r = get_if<FReg>(&reg)) {
    Reg tmp = getNextTemp();
    if (tmp == avoid)
      tmp = getNextTemp();
    F2I(*r, tmp);
    return tmp;
  } else {
    fprintf(stderr, "empty getReg");
    return Reg::x0;
  }
}

FReg getFReg(const SmartReg &reg, FReg avoid = FReg::null) {
  if (auto r = get_if<FSReg>(&reg))
    return **r;
  else if (auto r = get_if<FrameVar>(&reg)) {
    FReg tmp = getNextFTemp();
    if (tmp == avoid)
      tmp = getNextFTemp();
    if (r->type == FrameType::Int) {
      Reg itmp = getNextTemp();
      Gen::inst(Inst::Lw, itmp, -r->frame, Reg::fp);
      I2F(itmp, tmp);
    } else {
      Gen::inst(Inst::Flw, tmp, -r->frame, Reg::fp);
    }
    return tmp;
  } else if (auto r = get_if<FReg>(&reg)) {
    return *r;
  } else if (auto r = get_if<SReg>(&reg)) {
    I2F(**r, FReg::ft0);
    return FReg::ft0;
  } else if (auto r = get_if<Reg>(&reg)) {
    I2F(*r, FReg::ft0);
    return FReg::ft0;
  } else {
    fprintf(stderr, "empty getFReg");
    return FReg::null;
  }
}

void setReg(const SmartReg &reg, Reg src) {
  if (auto r = get_if<SReg>(&reg))
    Gen::inst(Inst::Mv, **r, src);
  else if (auto r = get_if<FrameVar>(&reg)) {
    if (r->type == FrameType::Float) {
      FReg ftmp = getNextFTemp();
      I2F(src, ftmp);
      Gen::inst(Inst::Fsw, ftmp, -r->frame, Reg::fp);
    } else {
      Gen::inst(Inst::Sw, src, -r->frame, Reg::fp);
    }
  } else if (auto r = get_if<FSReg>(&reg)) {
    FReg ftmp = getNextFTemp();
    I2F(src, ftmp);
    Gen::inst(Inst::Fmv, **r, ftmp);
  } else if (auto r = get_if<FReg>(&reg)) {
    FReg ftmp = getNextFTemp();
    I2F(src, ftmp);
    Gen::inst(Inst::Fmv, *r, ftmp);
  } else {
    fprintf(stderr, "empty setReg");
  }
}

void setFReg(const SmartReg &reg, FReg src) {
  if (auto r = get_if<FSReg>(&reg))
    Gen::inst(Inst::Fmv, **r, src);
  else if (auto r = get_if<FrameVar>(&reg)) {
    if (r->type == FrameType::Int) {
      Reg itmp = getNextTemp();
      F2I(src, itmp);
      Gen::inst(Inst::Sw, itmp, -r->frame, Reg::fp);
    } else {
      Gen::inst(Inst::Fsw, src, -r->frame, Reg::fp);
    }
  } else if (auto r = get_if<FSReg>(&reg)) {
    Reg itmp = getNextTemp();
    F2I(src, itmp);
    Gen::inst(Inst::Mv, **r, itmp);
  } else if (auto r = get_if<FReg>(&reg)) {
    Reg itmp = getNextTemp();
    F2I(src, itmp);
    Gen::inst(Inst::Mv, *r, itmp);
  } else {
    fprintf(stderr, "empty setFReg");
  }
}

vector<Reg> free_reg_pool = {Reg::s11, Reg::s10, Reg::s9, Reg::s8,
                             Reg::s7,  Reg::s6,  Reg::s5, Reg::s4,
                             Reg::s3,  Reg::s2,  Reg::s1};
vector<FReg> f_free_reg_pool = {FReg::fs11, FReg::fs10, FReg::fs9, FReg::fs8,
                                FReg::fs7,  FReg::fs6,  FReg::fs5, FReg::fs4,
                                FReg::fs3,  FReg::fs2,  FReg::fs1, FReg::fs0};
set<Reg> used_reg_pool;
set<FReg> f_used_reg_pool;

SReg getSavedReg() {
  if (free_reg_pool.empty())
    return nullptr;
  Reg reg = free_reg_pool.back();
  free_reg_pool.pop_back();
  used_reg_pool.insert(reg);
  return SReg(new Reg(reg), [reg](Reg *r) {
    delete r;
    free_reg_pool.push_back(reg);
  });
}

FSReg getSavedFReg() {
  if (f_free_reg_pool.empty())
    return nullptr;
  FReg reg = f_free_reg_pool.back();
  f_free_reg_pool.pop_back();
  f_used_reg_pool.insert(reg);
  return FSReg(new FReg(reg), [reg](FReg *r) {
    delete r;
    f_free_reg_pool.push_back(reg);
  });
}

SmartReg allocSavedIt() {
  auto sreg = getSavedReg();
  if (sreg) {
    return sreg;
  }
  FrameVar var;
  function_frame_size += 4;
  var.frame = function_frame_size;
  var.type = FrameType::Int;
  return var;
}

SmartReg allocSavedItF() {
  auto sreg = getSavedFReg();
  if (sreg) {
    return sreg;
  }
  FrameVar var;
  function_frame_size += 4;
  var.frame = function_frame_size;
  var.type = FrameType::Float;
  return var;
}

bool forceI(SmartReg &reg) {
  if (get_if<SReg>(&reg))
    return true;
  if (get_if<Reg>(&reg))
    return false;
  if (get_if<FSReg>(&reg) || get_if<FReg>(&reg)) {
    reg = getReg(reg);
  }
  if (auto r = get_if<FrameVar>(&reg)) {
    if (r->type != FrameType::Float)
      return true;
    reg = getReg(reg);
  }
  return false;
}

bool forceF(SmartReg &reg) {
  if (get_if<FSReg>(&reg))
    return true;
  if (get_if<FReg>(&reg))
    return false;
  if (get_if<SReg>(&reg) || get_if<Reg>(&reg)) {
    reg = getFReg(reg);
  }
  if (auto r = get_if<FrameVar>(&reg)) {
    if (r->type != FrameType::Int)
      return true;
    reg = getFReg(reg);
  }
  return false;
}

void genSavedIt(SmartReg &reg) {
  if (forceI(reg))
    return;
  auto sreg = getSavedReg();
  if (sreg) {
    Gen::inst(Inst::Mv, *sreg, getReg(reg));
    reg = sreg;
    return;
  }
  FrameVar var;
  function_frame_size += 4;
  var.frame = function_frame_size;
  var.type = FrameType::Int;
  Gen::inst(Inst::Sw, getReg(reg), -var.frame, Reg::fp);
  reg = var;
}

void genSavedItF(SmartReg &reg) {
  if (forceF(reg))
    return;
  auto sreg = getSavedFReg();
  if (sreg) {
    Gen::inst(Inst::Fmv, *sreg, getFReg(reg));
    reg = sreg;
    return;
  }
  FrameVar var;
  function_frame_size += 4;
  var.frame = function_frame_size;
  var.type = FrameType::Float;
  Gen::inst(Inst::Fsw, getFReg(reg), -var.frame, Reg::fp);
  reg = var;
}

unordered_map<int, SmartReg> bind_map;

void bindVar(int &varid, SmartReg &reg) {
  static int varid_num = 1;
  if (!varid)
    varid = varid_num++;
  bind_map[varid] = reg;
}

void resetFrame() {
  bind_map.clear();
  free_reg_pool = {Reg::s11, Reg::s10, Reg::s9, Reg::s8, Reg::s7, Reg::s6,
                   Reg::s5,  Reg::s4,  Reg::s3, Reg::s2, Reg::s1};
  f_free_reg_pool = {FReg::fs11, FReg::fs10, FReg::fs9, FReg::fs8,
                     FReg::fs7,  FReg::fs6,  FReg::fs5, FReg::fs4,
                     FReg::fs3,  FReg::fs2,  FReg::fs1, FReg::fs0};
  used_reg_pool.clear();
  f_used_reg_pool.clear();
}

Reg genIntBinOp(BINARY_OPERATOR op, Reg r1, Reg r2) {
  switch (op) {
  case BINARY_OP_ADD:
    Gen::inst(Inst::Add, Reg::t0, r1, r2);
    break;
  case BINARY_OP_SUB:
    Gen::inst(Inst::Sub, Reg::t0, r1, r2);
    break;
  case BINARY_OP_MUL:
    Gen::inst(Inst::Mul, Reg::t0, r1, r2);
    break;
  case BINARY_OP_DIV:
    Gen::inst(Inst::Div, Reg::t0, r1, r2);
    break;
  case BINARY_OP_AND:
    Gen::inst(Inst::And, Reg::t0, r1, r2);
    break;
  case BINARY_OP_OR:
    Gen::inst(Inst::Or, Reg::t0, r1, r2);
    break;
  case BINARY_OP_EQ:
    Gen::inst(Inst::Sub, Reg::t0, r1, r2);
    Gen::inst(Inst::Seqz, Reg::t0, Reg::t0);
    break;
  case BINARY_OP_GE:
    Gen::inst(Inst::Sub, Reg::t0, r1, r2);
    Gen::inst(Inst::Sltz, Reg::t0, Reg::t0);
    Gen::inst(Inst::Seqz, Reg::t0, Reg::t0);
    break;
  case BINARY_OP_LE:
    Gen::inst(Inst::Sub, Reg::t0, r2, r1);
    Gen::inst(Inst::Sltz, Reg::t0, Reg::t0);
    Gen::inst(Inst::Seqz, Reg::t0, Reg::t0);
    break;
  case BINARY_OP_NE:
    Gen::inst(Inst::Sub, Reg::t1, r1, r2);
    Gen::inst(Inst::Snez, Reg::t0, Reg::t1);
    break;
  case BINARY_OP_GT:
    Gen::inst(Inst::Sgt, Reg::t0, r1, r2);
    break;
  case BINARY_OP_LT:
    Gen::inst(Inst::Slt, Reg::t0, r1, r2);
    break;
  default:
    fprintf(stderr, "unknown in genIntBinOp\n");
  }
  return Reg::t0;
}

FReg genFloatBinOp(BINARY_OPERATOR op, FReg r1, FReg r2) {
  switch (op) {
  case BINARY_OP_ADD:
    Gen::inst(Inst::Fadd, FReg::ft0, r1, r2);
    break;
  case BINARY_OP_SUB:
    Gen::inst(Inst::Fsub, FReg::ft0, r1, r2);
    break;
  case BINARY_OP_MUL:
    Gen::inst(Inst::Fmul, FReg::ft0, r1, r2);
    break;
  case BINARY_OP_DIV:
    Gen::inst(Inst::Fdiv, FReg::ft0, r1, r2);
    break;
  case BINARY_OP_AND:
    F2I(r1, Reg::t0);
    F2I(r2, Reg::t1);
    Gen::inst(Inst::And, Reg::t0, Reg::t0, Reg::t1);
    I2F(Reg::t0, FReg::ft0);
    break;
  case BINARY_OP_OR:
    F2I(r1, Reg::t0);
    F2I(r2, Reg::t1);
    Gen::inst(Inst::Or, Reg::t0, Reg::t0, Reg::t1);
    I2F(Reg::t0, FReg::ft0);
    break;
  case BINARY_OP_EQ:
    Gen::inst(Inst::Feq, Reg::t0, r1, r2);
    I2F(Reg::t0, FReg::ft0);
    break;
  case BINARY_OP_GE:
    Gen::inst(Inst::Fge, Reg::t0, r1, r2);
    I2F(Reg::t0, FReg::ft0);
    break;
  case BINARY_OP_LE:
    Gen::inst(Inst::Fle, Reg::t0, r1, r2);
    I2F(Reg::t0, FReg::ft0);
    break;
  case BINARY_OP_NE:
    Gen::inst(Inst::Feq, Reg::t0, r1, r2);
    Gen::inst(Inst::Seqz, Reg::t0, Reg::t0);
    I2F(Reg::t0, FReg::ft0);
    break;
  case BINARY_OP_GT:
    Gen::inst(Inst::Fgt, Reg::t0, r1, r2);
    I2F(Reg::t0, FReg::ft0);
    break;
  case BINARY_OP_LT:
    Gen::inst(Inst::Flt, Reg::t0, r1, r2);
    I2F(Reg::t0, FReg::ft0);
    break;
  default:
    fprintf(stderr, "unknown in genIntBinOp\n");
  }
  return FReg::ft0;
}

Reg genIntUniOp(UNARY_OPERATOR op, Reg r1) {
  switch (op) {
  case UNARY_OP_POSITIVE:
    return r1;
    break;
  case UNARY_OP_NEGATIVE:
    Gen::inst(Inst::Sub, Reg::t0, Reg::x0, r1);
    break;
  case UNARY_OP_LOGICAL_NEGATION:
    Gen::inst(Inst::Seqz, Reg::t0, r1);
    break;
  default:
    fprintf(stderr, "unknown in genIntUniOp\n");
  }
  return Reg::t0;
}

FReg genFloatUniOp(UNARY_OPERATOR op, FReg r1) {
  switch (op) {
  case UNARY_OP_POSITIVE:
    return r1;
    break;
  case UNARY_OP_NEGATIVE:
    Gen::inst(Inst::Fneg, FReg::ft0, r1);
    break;
  case UNARY_OP_LOGICAL_NEGATION:
    F2I(r1, Reg::t1);
    Gen::inst(Inst::Seqz, Reg::t0, Reg::t1);
    I2F(Reg::t0, FReg::ft0);
    break;
  default:
    fprintf(stderr, "unknown in genFloatUniOp\n");
  }
  return FReg::ft0;
}

SmartReg genIntExpr(Node node) {
  static int gen_int_expr_id = 0;
  SmartReg result;
  switch (node->nodeType) {
  case EXPR_NODE:
    if (node.expr().isConstEval) {
      // Gen::segment("data");
      // Gen::label("_CONSTANT_" + to_string(const_id), ".word",
      //           getConstIValue(node));
      // Gen::segment("text");
      // Gen::inst(Inst::Lw, Reg::t0, "_CONSTANT_" + to_string(const_id));
      // const_id++;
      Gen::inst(Inst::Li, Reg::t0, getConstIValue(node));
      result = Reg::t0;
    } else if (node.expr().kind == BINARY_OPERATION) {
      switch (node.expr().op.binaryOp) {
      case BINARY_OP_AND: {
        SmartReg res = genExpr(node.child()[0]);
        Gen::inst(Inst::Mv, Reg::t0, getReg(res));
        Gen::inst(Inst::Beqz, Reg::t0,
                  "_short_int_" + to_string(gen_int_expr_id));
        SmartReg res2 = genExpr(node.child()[1]);
        Gen::inst(Inst::Mv, Reg::t0, getReg(res2));
        Gen::label("_short_int_" + to_string(gen_int_expr_id));
        result = Reg::t0;
        gen_int_expr_id++;
      } break;
      case BINARY_OP_OR: {
        SmartReg res = genExpr(node.child()[0]);
        Gen::inst(Inst::Mv, Reg::t0, getReg(res));
        Gen::inst(Inst::Bnez, getReg(res),
                  "_short_int_" + to_string(gen_int_expr_id));
        SmartReg res2 = genExpr(node.child()[1]);
        Gen::inst(Inst::Mv, Reg::t0, getReg(res2));
        Gen::label("_short_int_" + to_string(gen_int_expr_id));
        result = Reg::t0;
        gen_int_expr_id++;
      } break;
      default: {
        SmartReg res1 = genExpr(node.child()[0]);
        genSavedIt(res1);
        SmartReg res2 = genExpr(node.child()[1]);
        Reg r2 = getReg(res2);
        Reg r1 = getReg(res1, r2);
        result = genIntBinOp(node.expr().op.binaryOp, r1, r2);
      }
      }
    } else {
      SmartReg res = genExpr(node.child());
      result = genIntUniOp(node.expr().op.unaryOp, getReg(res));
    }
    break;
  case CONST_VALUE_NODE:
    switch (node.cons()->const_type) {
    case INTEGERC:
      // Gen::segment("data");
      // Gen::label("_CONSTANT_" + to_string(const_id), ".word",
      //           node.cons()->const_u.intval);
      // Gen::segment("text");
      // Gen::inst(Inst::Lw, Reg::t0, "_CONSTANT_" + to_string(const_id));
      // const_id++;
      Gen::inst(Inst::Li, Reg::t0, node.cons()->const_u.intval);
      result = Reg::t0;
      break;
    case STRINGC:
    case FLOATC:
    default:
      fprintf(stderr, "unknown in genIntExpr\n");
    }
    break;
  case IDENTIFIER_NODE:
    switch (node.identifier().kind) {
    case NORMAL_ID:
      if (node.identifier().symbolTableEntry->nestingLevel == 0) {
        Gen::inst(Inst::Lw, Reg::t0, "_g_" + node.name());
        result = Reg::t0;
      } else {
        Gen::inst(Inst::Mv, Reg::t0,
                  getReg(bind_map[node.identifier().symbolTableEntry->varid]));
        result = Reg::t0;
      }
      break;
    case ARRAY_ID: {
      SmartReg location = genArrayLocation(node);
      Gen::inst(Inst::Lw, Reg::t0, 0, getReg(location));
      result = Reg::t0;
    } break;
    default:
      fprintf(stderr, "unknown in genIntExpr\n");
    }
    break;
  case STMT_NODE:
    if (node.stmt().kind == FUNCTION_CALL_STMT) {
      SmartReg tmp = genFunctionCall(node);
      forceI(tmp);
      result = tmp;
    } else {
      fprintf(stderr, "unknown in genIntExpr\n");
    }
    break;
  default:
    fprintf(stderr, "unknown in genIntExpr\n");
  }
  return result;
}

SmartReg genFloatExpr(Node node) {
  static int gen_float_expr_id = 0;
  SmartReg result;
  switch (node->nodeType) {
  case EXPR_NODE:
    if (node.expr().isConstEval) {
      Gen::segment("data");
      Gen::label("_CONSTANT_" + to_string(const_id), ".float",
                 getConstFValue(node));
      Gen::segment("text");
      Gen::inst(Inst::Flw_sym, FReg::ft0, "_CONSTANT_" + to_string(const_id),
                Reg::t2);
      result = FReg::ft0;
      const_id++;
    } else if (node.expr().kind == BINARY_OPERATION) {
      switch (node.expr().op.binaryOp) {
      case BINARY_OP_AND: {
        SmartReg res = genExpr(node.child()[0]);
        Gen::inst(Inst::Mv, Reg::t0, getReg(res));
        Gen::inst(Inst::Beqz, getReg(res),
                  "_short_float_" + to_string(gen_float_expr_id));
        SmartReg res2 = genExpr(node.child()[1]);
        Gen::inst(Inst::Mv, Reg::t0, getReg(res2));
        Gen::label("_short_float_" + to_string(gen_float_expr_id));
        I2F(Reg::t0, FReg::ft0);
        result = FReg::ft0;
        gen_float_expr_id++;
      } break;
      case BINARY_OP_OR: {
        SmartReg res = genExpr(node.child()[0]);
        Gen::inst(Inst::Mv, Reg::t0, getReg(res));
        Gen::inst(Inst::Bnez, getReg(res),
                  "_short_float_" + to_string(gen_float_expr_id));
        SmartReg res2 = genExpr(node.child()[1]);
        Gen::inst(Inst::Mv, Reg::t0, getReg(res2));
        Gen::label("_short_float_" + to_string(gen_float_expr_id));
        I2F(Reg::t0, FReg::ft0);
        result = FReg::ft0;
        gen_float_expr_id++;
      } break;
      default: {
        SmartReg res1 = genExpr(node.child()[0]);
        genSavedItF(res1);
        SmartReg res2 = genExpr(node.child()[1]);
        FReg r2 = getFReg(res2);
        FReg r1 = getFReg(res1, r2);
        result = genFloatBinOp(node.expr().op.binaryOp, r1, r2);
      }
      }
    } else {
      SmartReg res = genExpr(node.child());
      result = genFloatUniOp(node.expr().op.unaryOp, getFReg(res));
    }
    break;
  case CONST_VALUE_NODE:
    switch (node.cons()->const_type) {
    case FLOATC:
      Gen::segment("data");
      Gen::label("_CONSTANT_" + to_string(const_id), ".float",
                 node.cons()->const_u.fval);
      Gen::segment("text");
      Gen::inst(Inst::Flw_sym, FReg::ft0, "_CONSTANT_" + to_string(const_id),
                Reg::t2);
      result = FReg::ft0;
      const_id++;
      break;
    case STRINGC:
    case INTEGERC:
    default:
      fprintf(stderr, "unknown in genFloatExpr\n");
    }
    break;
  case IDENTIFIER_NODE:
    switch (node.identifier().kind) {
    case NORMAL_ID:
      if (node.identifier().symbolTableEntry->nestingLevel == 0) {
        Gen::inst(Inst::Flw_sym, FReg::ft0, "_g_" + node.name(), Reg::t2);
        result = FReg::ft0;
      } else {
        Gen::inst(Inst::Fmv, FReg::ft0,
                  getFReg(bind_map[node.identifier().symbolTableEntry->varid]));
        result = FReg::ft0;
      }
      break;
    case ARRAY_ID: {
      SmartReg location = genArrayLocation(node);
      Gen::inst(Inst::Flw, FReg::ft0, 0, getReg(location));
      result = FReg::ft0;
    } break;
    default:
      fprintf(stderr, "unknown in genFloatExpr\n");
    }
    break;
  case STMT_NODE:
    if (node.stmt().kind == FUNCTION_CALL_STMT) {
      SmartReg tmp = genFunctionCall(node);
      forceF(tmp);
      result = tmp;
    } else {
      fprintf(stderr, "unknown in genFloatExpr\n");
    }
    break;
  default:
    fprintf(stderr, "unknown in genFloatExpr\n");
  }
  return result;
}

SmartReg genExpr(Node node) {
  if (node->dataType == INT_TYPE) {
    return genIntExpr(node);
  } else if (node->dataType == FLOAT_TYPE) {
    return genFloatExpr(node);
  } else if (node->dataType == CONST_STRING_TYPE) {
    Gen::segment("data");
    Gen::label("_CONSTANT_" + to_string(const_id), ".string",
               escape(node.cons()->const_u.sc));
    Gen::segment("text");
    Gen::inst(Inst::La, Reg::t0, "_CONSTANT_" + to_string(const_id));
    const_id++;
    return Reg::t0;
  } else {
    fprintf(stderr, "unknown in genExpr\n");
    return Reg::t0;
  }
}

SmartReg genArrayLocation(Node node) {
  auto sym = node.identifier().symbolTableEntry;
  auto &prop = sym->attribute->attr.typeDescriptor->properties.arrayProperties;
  int dim = 1;
  Node exprs = node.child();
  SmartReg location = allocSavedIt();
  setReg(location, Reg::x0);
  for (Node expr : exprs) {
    int factor = 4;
    if (dim < prop.dimension)
      factor = prop.sizeInEachDimension[dim];
    SmartReg res = genIntExpr(expr);
    Gen::inst(Inst::Add, Reg::t0, getReg(res), getReg(location));
    Gen::inst(Inst::Li, Reg::t1, factor);
    Gen::inst(Inst::Mul, Reg::t0, Reg::t0, Reg::t1);
    setReg(location, Reg::t0);
    dim++;
  }
  Reg addr;
  if (sym->nestingLevel == 0) {
    Gen::inst(Inst::La, Reg::t0, "_g_" + node.name());
    addr = Reg::t0;
  } else {
    addr = getAddr(bind_map[sym->varid]);
  }
  Gen::inst(Inst::Add, Reg::t0, getReg(location), addr);
  return Reg::t0;
}

SmartReg genAssign(Node node) {
  Node dst = node.child()[0];
  Node exp = node.child()[1];
  SmartReg result = genExpr(exp);
  switch (dst.identifier().kind) {
  case NORMAL_ID:
    if (dst->dataType == INT_TYPE) {
      if (dst.identifier().symbolTableEntry->nestingLevel == 0) {
        Reg tmp = getReg(result);
        Gen::inst(Inst::Sw_sym, tmp, "_g_" + dst.name(), Reg::t2);
        result = tmp;
      } else {
        setReg(bind_map[dst.identifier().symbolTableEntry->varid],
               getReg(result));
      }
    } else if (dst->dataType == FLOAT_TYPE) {
      if (dst.identifier().symbolTableEntry->nestingLevel == 0) {
        FReg tmp = getFReg(result);
        Gen::inst(Inst::Fsw_sym, tmp, "_g_" + dst.name(), Reg::t2);
        result = tmp;
      } else {
        setFReg(bind_map[dst.identifier().symbolTableEntry->varid],
                getFReg(result));
      }
    } else {
      fprintf(stderr, "unknown in genAssign\n");
    }
    break;
  case ARRAY_ID: {
    if (dst->dataType == INT_TYPE) {
      genSavedIt(result);
      SmartReg location = genArrayLocation(dst);
      Gen::inst(Inst::Sw, getReg(result), 0, getReg(location));
    } else if (dst->dataType == FLOAT_TYPE) {
      genSavedItF(result);
      SmartReg location = genArrayLocation(dst);
      Gen::inst(Inst::Fsw, getFReg(result), 0, getReg(location));
    } else {
      fprintf(stderr, "unknown in genAssign\n");
    }
  } break;
  default:
    fprintf(stderr, "unknown in genAssign\n");
  }
  return result;
}

void genIf(Node node) {
  static int if_id = 0;
  if_id++;
  int now_id = if_id;
  SmartReg result;
  Node exp = node.child()[0];
  if (exp->nodeType == STMT_NODE && exp.stmt().kind == ASSIGN_STMT) {
    result = genAssign(exp);
  } else {
    result = genExpr(exp);
  }
  Gen::inst(Inst::Beqz, getReg(result), "_if_end_" + to_string(now_id));
  Node run = node.child()[1];
  Node els = node.child()[2];
  if (run->nodeType == STMT_NODE) {
    genStmt(run);
  } else if (run->nodeType == BLOCK_NODE) {
    genBlock(run);
  } else {
    fprintf(stderr, "unknown in genIf\n");
  }
  if (els->nodeType != NUL_NODE)
    Gen::inst(Inst::J, "_else_end_" + to_string(now_id));
  Gen::label("_if_end_" + to_string(now_id));
  if (els->nodeType != NUL_NODE) {
    if (els->nodeType == STMT_NODE) {
      genStmt(els);
    } else if (els->nodeType == BLOCK_NODE) {
      genBlock(els);
    }
    Gen::label("_else_end_" + to_string(now_id));
  }
}
void genReturn(Node node) {
  if (node.child()->nodeType != NUL_NODE) {
    SmartReg result = genExpr(node.child());
    if (node.child()->dataType == INT_TYPE) {
      Gen::inst(Inst::Mv, Reg::a0, getReg(result));
    } else {
      Gen::inst(Inst::Fmv, FReg::fa0, getFReg(result));
    }
  }
  Gen::inst(Inst::J, "_end_" + cur_function_name);
}
SmartReg genFunctionCall(Node node) {
  // static int func_call_num = 0;
  // func_call_num++;
  // Gen::label("_func_call" + to_string(func_call_num));
  Node func = node.child()[0];
  Node param = node.child()[1];
  if (func.name() == "write") {
    SmartReg reg = genExpr(param.child());
    if (param.child()->dataType == CONST_STRING_TYPE) {
      Gen::inst(Inst::Mv, Reg::a0, getReg(reg));
      Gen::inst(Inst::Jal, "_write_str");
    } else if (param.child()->dataType == INT_TYPE) {
      Gen::inst(Inst::Mv, Reg::a0, getReg(reg));
      Gen::inst(Inst::Jal, "_write_int");
    } else {
      Gen::inst(Inst::Fmv, FReg::fa0, getFReg(reg));
      Gen::inst(Inst::Jal, "_write_float");
    }
    return Reg::x0;
  } else if (func.name() == "read") {
    Gen::inst(Inst::Jal, "_read_int");
    return Reg::a0;
  } else if (func.name() == "fread") {
    Gen::inst(Inst::Jal, "_read_float");
    return FReg::fa0;
  } else {
    if (param->nodeType != NUL_NODE) {
      // TODO pass parameters
    }
    Gen::inst(Inst::Jal, Reg::ra, "_start_" + func.name());
    auto returnType =
        func.identifier()
            .symbolTableEntry->attribute->attr.functionSignature->returnType;
    if (returnType == INT_TYPE)
      return Reg::a0;
    else if (returnType == FLOAT_TYPE)
      return FReg::fa0;
    else
      return Reg::x0;
  }
}

void genWhile(Node node) {
  static int while_id = 0;
  while_id++;
  int now_id = while_id;
  Node exp = node.child()[0];
  Node run = node.child()[1];
  SmartReg result;
  Gen::label("_while_begin_" + to_string(now_id));
  if (exp->nodeType == STMT_NODE && exp.stmt().kind == ASSIGN_STMT) {
    result = genAssign(exp);
  } else {
    result = genExpr(exp);
  }
  Gen::inst(Inst::Beqz, getReg(result), "_while_end_" + to_string(now_id));
  if (run->nodeType == STMT_NODE) {
    genStmt(run);
  } else if (run->nodeType == BLOCK_NODE) {
    genBlock(run);
  } else if (run->nodeType == NUL_NODE) {
    // Do nothing
  }
  Gen::inst(Inst::J, "_while_begin_" + to_string(now_id));
  Gen::label("_while_end_" + to_string(now_id));
}

void genStmt(Node node) {
  switch (node.stmt().kind) {
  case WHILE_STMT:
    genWhile(node);
    break;
  case FOR_STMT:
    break;
  case ASSIGN_STMT:
    genAssign(node);
    break;
  case IF_STMT:
    genIf(node);
    break;
  case FUNCTION_CALL_STMT:
    genFunctionCall(node);
    break;
  case RETURN_STMT:
    genReturn(node);
    break;
  default:;
  }
}
void genStmts(Node node) {
  Node childs = node.child();
  for (Node child : childs) {
    if (child->nodeType == STMT_NODE) {
      genStmt(child);
    } else if (child->nodeType == BLOCK_NODE) {
      genBlock(child);
    } else if (child->nodeType == NUL_NODE) {
      // Do nothing
    }
  }
}
void genBlock(Node node) {
  Node childs = node.child();
  for (Node child : childs) {
    if (child->nodeType == VARIABLE_DECL_LIST_NODE) {
      Node decls = child.child();
      for (Node decl : decls) {
        Node type = decl.child()[0];
        Node ids = decl.child()[1];
        auto sym = ids.identifier().symbolTableEntry;
        auto kind = sym->attribute->attr.typeDescriptor->kind;
        for (Node id : ids) {
          switch (id.identifier().kind) {
          case NORMAL_ID: {
            if (kind == SCALAR_TYPE_DESCRIPTOR) {
              if (type->dataType == INT_TYPE) {
                SmartReg reg = allocSavedIt();
                bindVar(id.identifier().symbolTableEntry->varid, reg);
              } else {
                SmartReg reg = allocSavedItF();
                bindVar(id.identifier().symbolTableEntry->varid, reg);
              }
            } else {
              int size = 4;
              auto &prop = sym->attribute->attr.typeDescriptor->properties
                               .arrayProperties;
              for (int i = 0; i < prop.dimension; i++) {
                size *= prop.sizeInEachDimension[i];
              }
              FrameVar var;
              function_frame_size += size;
              var.frame = function_frame_size;
              var.type = FrameType::Unknown;
              SmartReg reg = var;
              bindVar(id.identifier().symbolTableEntry->varid, reg);
            }
          } break;
          case WITH_INIT_ID: {
            SmartReg reg = genExpr(id.child());
            if (id.child()->dataType == INT_TYPE) {
              if (id.identifier()
                      .symbolTableEntry->attribute->attr.typeDescriptor
                      ->properties.dataType == INT_TYPE) {
                genSavedIt(reg);
                bindVar(id.identifier().symbolTableEntry->varid, reg);
              } else {
                I2F(getReg(reg), FReg::ft0);
                SmartReg tmp = FReg::ft0;
                genSavedItF(tmp);
                bindVar(id.identifier().symbolTableEntry->varid, tmp);
              }
            } else {
              if (id.identifier()
                      .symbolTableEntry->attribute->attr.typeDescriptor
                      ->properties.dataType == INT_TYPE) {
                F2I(getFReg(reg), Reg::t0);
                SmartReg tmp = Reg::t0;
                genSavedIt(reg);
                bindVar(id.identifier().symbolTableEntry->varid, tmp);
              } else {
                genSavedItF(reg);
                bindVar(id.identifier().symbolTableEntry->varid, reg);
              }
            }
          } break;
          case ARRAY_ID: {
            int size = 4;
            auto &prop = id.identifier()
                             .symbolTableEntry->attribute->attr.typeDescriptor
                             ->properties.arrayProperties;
            for (int i = 0; i < prop.dimension; i++) {
              size *= prop.sizeInEachDimension[i];
            }
            if (kind == SCALAR_TYPE_DESCRIPTOR) {
              auto &prop = sym->attribute->attr.typeDescriptor->properties
                               .arrayProperties;
              for (int i = 0; i < prop.dimension; i++) {
                size *= prop.sizeInEachDimension[i];
              }
            }
            FrameVar var;
            function_frame_size += size;
            var.frame = function_frame_size;
            var.type = FrameType::Unknown;
            SmartReg reg = var;
            bindVar(id.identifier().symbolTableEntry->varid, reg);
          } break;
          default:
            fprintf(stderr, "unkonwn in genBlock\n");
          }
        }
      }
    } else {
      genStmts(child);
    }
  }
}
void genGDeclFunction(Node node) {
  Node type = node.child()[0];
  Node id = node.child()[1];
  Node paramList = node.child()[2];
  Node block = node.child()[3];
  Gen::segment("text");
  Gen::label("_start_" + node.child()[1].name());
  Gen::inst(Inst::Sd, Reg::ra, 0, Reg::sp);
  Gen::inst(Inst::Sd, Reg::fp, -8, Reg::sp);
  Gen::inst(Inst::Add, Reg::fp, Reg::sp, -8);
  Gen::inst(Inst::Add, Reg::sp, Reg::sp, -16);
  Gen::inst(Inst::Lw, Reg::ra, "_frameSize_" + id.name());
  Gen::inst(Inst::Sub, Reg::sp, Reg::sp, Reg::ra);
  function_frame_size = 0;
  cur_function_name = id.name();
  // TODO call with parameters
  swap(codes, codes_tmp);
  genBlock(block);
  Gen::label("_end_" + id.name());
  swap(codes, codes_tmp);
  map<Reg, int> reg_map;
  map<FReg, int> freg_map;
  for (Reg reg : used_reg_pool) {
    function_frame_size += 4;
    reg_map[reg] = function_frame_size;
    Gen::inst(Inst::Sw, reg, -function_frame_size, Reg::fp);
  }
  for (FReg reg : f_used_reg_pool) {
    function_frame_size += 4;
    freg_map[reg] = function_frame_size;
    Gen::inst(Inst::Fsw, reg, -function_frame_size, Reg::fp);
  }
  for (auto &code : codes_tmp)
    codes.emplace_back(move(code));
  codes_tmp.clear();
  for (Reg reg : used_reg_pool) {
    Gen::inst(Inst::Lw, reg, -reg_map[reg], Reg::fp);
  }
  for (FReg reg : f_used_reg_pool) {
    Gen::inst(Inst::Flw, reg, -freg_map[reg], Reg::fp);
  }
  Gen::inst(Inst::Ld, Reg::ra, 8, Reg::fp);
  Gen::inst(Inst::Mv, Reg::sp, Reg::fp);
  Gen::inst(Inst::Add, Reg::sp, Reg::sp, 8);
  Gen::inst(Inst::Ld, Reg::fp, 0, Reg::fp);
  Gen::inst(Inst::Jr, Reg::ra);
  Gen::segment("data");
  Gen::label("_frameSize_" + id.name(), ".word", function_frame_size);
  resetFrame();
}
void genGDeclVariable(Node node) {
  Gen::segment("data");
  Node ids = node.child()[1];
  auto sym = ids.identifier().symbolTableEntry;
  if (!sym)
    return;
  auto kind = sym->attribute->attr.typeDescriptor->kind;
  auto type = sym->attribute->attr.typeDescriptor->properties.dataType;
  for (Node id : ids) {
    string name("_g_");
    auto &value = id.identifier();
    name += id.name();
    switch (value.kind) {
    case NORMAL_ID:
      if (kind == SCALAR_TYPE_DESCRIPTOR) {
        switch (type) {
        case INT_TYPE:
          Gen::label(name, ".word", 0);
          break;
        case FLOAT_TYPE:
          // https://forums.sifive.com/t/load-immediate-for-floats-doubles/2501/2
          Gen::label(name, ".float", 0.0);
          break;
        default:
          fprintf(stderr, "unknown in genGDeclVariable\n");
        }
      } else {
        int size = 4;
        auto &prop =
            sym->attribute->attr.typeDescriptor->properties.arrayProperties;
        for (int i = 0; i < prop.dimension; i++) {
          size *= prop.sizeInEachDimension[i];
        }
        Gen::label(name, ".zero", size);
      }
      break;
    case ARRAY_ID: {
      int size = 4;
      auto &prop = value.symbolTableEntry->attribute->attr.typeDescriptor
                       ->properties.arrayProperties;
      for (int i = 0; i < prop.dimension; i++) {
        size *= prop.sizeInEachDimension[i];
      }
      if (kind == SCALAR_TYPE_DESCRIPTOR) {
        auto &prop =
            sym->attribute->attr.typeDescriptor->properties.arrayProperties;
        for (int i = 0; i < prop.dimension; i++) {
          size *= prop.sizeInEachDimension[i];
        }
      }
      Gen::label(name, ".zero", size);
    } break;
    case WITH_INIT_ID:
      switch (type) {
      case INT_TYPE:
        Gen::label(name, ".word", getConstIValue(id.child()));
        break;
      case FLOAT_TYPE:
        Gen::label(name, ".float", getConstFValue(id.child()));
        break;
      default:
        fprintf(stderr, "unknown in genGDeclVariable\n");
      }
      break;
    default:
      fprintf(stderr, "unknown in genGDeclVariable\n");
      break;
    }
  }
}
void genGDecl(Node node) {
  if (node->nodeType == DECLARATION_NODE) {
    switch (node.decl().kind) {
    case VARIABLE_DECL:
      genGDeclVariable(node);
      break;
    case FUNCTION_DECL:
      genGDeclFunction(node);
      break;
    case TYPE_DECL:
      // Do nothing
      break;
    case FUNCTION_PARAMETER_DECL:
    default:
      fprintf(stderr, "unknown in genGDecl\n");
      break;
    }
  } else {
    fprintf(stderr, "unknown in genGDecl\n");
  }
}

void genGDecls(Node root) {
  auto childs = root.child();
  for (Node child : childs) {
    genGDecl(child);
  }
}
template <class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template <class... Ts> overloaded(Ts...)->overloaded<Ts...>;
void printArg(const InstArg &arg) {
  visit(overloaded{
            [](const Reg &arg) { printf(reg_name[static_cast<int>(arg)]); },
            [](const FReg &arg) { printf(freg_name[static_cast<int>(arg)]); },
            [](const Symbol &arg) { printf("%s", arg.c_str()); },
            [](const int &arg) { printf("%d", arg); },
            [](const double &arg) { printf("%f", arg); }},
        arg);
}
void printRemain(const vector<InstArg> &args, int i) {
  for (int i = 1; i < args.size(); i++)
    printf(" "), printArg(args[i]);
}
void printLoadStore(const vector<InstArg> &args) {
  printArg(args[0]);
  printf(", ");
  printArg(args[1]);
  if (args.size() == 3)
    printf("("), printArg(args[2]), printf(")");
}
void printArgs(const vector<InstArg> &args) {
  printArg(args[0]);
  for (int i = 1; i < args.size(); i++)
    printf(", "), printArg(args[i]);
}
void printCode() {
  for (auto &code : codes) {
    auto &args = code.second;
    switch (code.first) {
    case Inst::Segment:
      printf(".");
      printArg(args[0]);
      printRemain(args, 1);
      puts("");
      break;
    case Inst::Label:
      printArg(args[0]);
      printf(":");
      printRemain(args, 1);
      puts("");
      break;
    case Inst::Ld:
      printf("ld ");
      printLoadStore(args);
      puts("");
      break;
    case Inst::Lw:
      printf("lw ");
      printLoadStore(args);
      puts("");
      break;
    case Inst::Sd:
      printf("sd ");
      printLoadStore(args);
      puts("");
      break;
    case Inst::Sd_sym:
      printf("sd ");
      printArgs(args);
      puts("");
      break;
    case Inst::Sw:
      printf("sw ");
      printLoadStore(args);
      puts("");
      break;
    case Inst::Sw_sym:
      printf("sw ");
      printArgs(args);
      puts("");
      break;
    case Inst::Flw:
      printf("flw ");
      printLoadStore(args);
      puts("");
      break;
    case Inst::Flw_sym:
      printf("flw ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fsw:
      printf("fsw ");
      printLoadStore(args);
      puts("");
      break;
    case Inst::Fsw_sym:
      printf("fsw ");
      printArgs(args);
      puts("");
      break;
    case Inst::Mv:
      printf("mv ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fmv:
      printf("fmv.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Addi:
      printf("addi ");
      printArgs(args);
      puts("");
      break;
    case Inst::Add:
      printf("add ");
      printArgs(args);
      puts("");
      break;
    case Inst::Sub:
      printf("sub ");
      printArgs(args);
      puts("");
      break;
    case Inst::Mul:
      printf("mul ");
      printArgs(args);
      puts("");
      break;
    case Inst::Div:
      printf("div ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fadd:
      printf("fadd.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fsub:
      printf("fsub.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fmul:
      printf("fmul.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fdiv:
      printf("fdiv.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::And:
      printf("and ");
      printArgs(args);
      puts("");
      break;
    case Inst::Or:
      printf("or ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fneg:
      printf("fneg.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fcvt_w_s:
      printf("fcvt.w.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fcvt_s_w:
      printf("fcvt.s.w ");
      printArgs(args);
      puts("");
      break;
    case Inst::La:
      printf("la ");
      printArgs(args);
      puts("");
      break;
    case Inst::Li:
      printf("li ");
      printArgs(args);
      puts("");
      break;
    case Inst::Slt:
      printf("slt ");
      printArgs(args);
      puts("");
      break;
    case Inst::Sgt:
      printf("sgt ");
      printArgs(args);
      puts("");
      break;
    case Inst::Seqz:
      printf("seqz ");
      printArgs(args);
      puts("");
      break;
    case Inst::Snez:
      printf("snez ");
      printArgs(args);
      puts("");
      break;
    case Inst::Sltz:
      printf("sltz ");
      printArgs(args);
      puts("");
      break;
    case Inst::Sgtz:
      printf("sgtz ");
      printArgs(args);
      puts("");
      break;
    case Inst::Slez:
      printf("slez ");
      printArgs(args);
      puts("");
      break;
    case Inst::Sgez:
      printf("sgez ");
      printArgs(args);
      puts("");
      break;
    case Inst::Feq:
      printf("feq.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Flt:
      printf("flt.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fgt:
      printf("fgt.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fle:
      printf("fle.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fge:
      printf("fge.s ");
      printArgs(args);
      puts("");
      break;
    case Inst::Beqz:
      printf("beqz ");
      printArgs(args);
      puts("");
      break;
    case Inst::Bnez:
      printf("bnez ");
      printArgs(args);
      puts("");
      break;
    case Inst::Bltz:
      printf("bltz ");
      printArgs(args);
      puts("");
      break;
    case Inst::Bgtz:
      printf("bgtz ");
      printArgs(args);
      puts("");
      break;
    case Inst::Blez:
      printf("blez ");
      printArgs(args);
      puts("");
      break;
    case Inst::Bgez:
      printf("bgez ");
      printArgs(args);
      puts("");
      break;
    case Inst::Beq:
      printf("beq ");
      printArgs(args);
      puts("");
      break;
    case Inst::Bne:
      printf("bne ");
      printArgs(args);
      puts("");
      break;
    case Inst::Blt:
      printf("blt ");
      printArgs(args);
      puts("");
      break;
    case Inst::Bgt:
      printf("bgt ");
      printArgs(args);
      puts("");
      break;
    case Inst::Ble:
      printf("ble ");
      printArgs(args);
      puts("");
      break;
    case Inst::Bge:
      printf("bge ");
      printArgs(args);
      puts("");
      break;
    case Inst::J:
      printf("j ");
      printArgs(args);
      puts("");
      break;
    case Inst::Jr:
      printf("jr ");
      printArgs(args);
      puts("");
      break;
    case Inst::Jal:
      printf("jal ");
      printArgs(args);
      puts("");
      break;
    case Inst::Call:
      printf("call ");
      printArgs(args);
      puts("");
      break;
    default:
      fprintf(stderr, "unknown %d in printCode\n", code.first);
    }
  }
}

void init() {
  temp_queue.push(Reg::t3);
  temp_queue.push(Reg::t4);
  temp_queue.push(Reg::t5);
  temp_queue.push(Reg::t6);
  f_temp_queue.push(FReg::ft3);
  f_temp_queue.push(FReg::ft4);
  f_temp_queue.push(FReg::ft5);
  f_temp_queue.push(FReg::ft6);
}
void splitDataText() {
  CodeDec data, text;
  bool isData = true;
  for (auto &code : codes) {
    if (code.first == Inst::Segment) {
      if (get<Symbol>(code.second[0]) == "data")
        isData = true;
      else
        isData = false;
    } else {
      if (isData)
        data.emplace_back(move(code));
      else
        text.emplace_back(move(code));
    }
  }
  codes.clear();
  Gen::segment("data");
  for (auto &code : data)
    codes.emplace_back(move(code));
  Gen::segment("text");
  for (auto &code : text)
    codes.emplace_back(move(code));
}
void codeGen(AST_NODE *root) {
  init();
  auto childs = Node(root).child();
  for (Node child : childs) {
    switch (child->nodeType) {
    case VARIABLE_DECL_LIST_NODE:
      genGDecls(child);
      break;
    case DECLARATION_NODE:
      genGDecl(child);
      break;
    default:
      fprintf(stderr, "unknown in codeGen\n");
      break;
    }
  }
  splitDataText();
  freopen("output.S","w",stdout);
  printCode();
}
