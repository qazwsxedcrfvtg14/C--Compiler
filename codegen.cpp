#include "codegen.h"
#include <bitset>
#include <cstring>
#include <deque>
#include <functional>
#include <initializer_list>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
  Slli,    // Slli
  Addi,    // Addi
  Slti,    // Slti
  Andi,    // Andi
  Ori,     // Ori
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
  Nop,
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
  ArrayPtr,
};

template <typename T> FrameType reg2Type();

template <> FrameType reg2Type<Reg>() { return FrameType::Int; }
template <> FrameType reg2Type<FReg>() { return FrameType::Float; }

multiset<pair<int, int>> free_frame;

struct FrameVar {
  FrameVar(FrameType _type, int _size) : type(_type), size(_size) {
    auto it = free_frame.lower_bound(make_pair(_size, -1));
    if (it == free_frame.end()) {
      function_frame_size += size;
      frame = function_frame_size;
    } else {
      auto p = *it;
      frame = p.second;
      p.first -= size;
      p.second += size;
      free_frame.erase(it);
      if (p.first) {
        free_frame.insert(p);
      }
    }
  }
  ~FrameVar() { free_frame.insert(make_pair(size, frame)); }
  int frame;
  int size;
  FrameType type;
};

using Symbol = string;
using SReg = shared_ptr<Reg>;
using FSReg = shared_ptr<FReg>;
using SVar = shared_ptr<FrameVar>;
using SmartReg = variant<Reg, SReg, SVar, FReg, FSReg>;
using InstArg = variant<Reg, FReg, Symbol, int, double>;

using Codec = pair<Inst, vector<InstArg>>;
using CodeDec = vector<Codec>;
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

template <typename T> void setReg(SmartReg &reg, T src);

template <typename T> T getReg(const SmartReg &reg, T avoid);

template <typename T> T getReg(const SmartReg &reg);

SReg getSavedReg();
FSReg getSavedFReg();

unordered_map<int, SmartReg> bind_map;
unordered_map<int, bool> bind_use;
template <typename T> struct LRU {
  // T lru(){return pos.back();}
  // T mru(){return pos.front();}
  void touch(const T &reg) {
    auto it = iter.find(reg);
    if (it != iter.end()) {
      pos.erase(it->second);
      pos.push_front(reg);
      iter[reg] = pos.begin();
    }
  }
  void add(const T &reg, int varid) {
    pos.push_front(reg);
    iter[reg] = pos.begin();
    varid_map[reg] = varid;
  }
  void clear() {
    pos.clear();
    iter.clear();
    varid_map.clear();
  }
  optional<T> getSReg() {
    if (pos.empty())
      return optional<T>();
    T res = pos.back();
    pos.pop_back();
    iter.erase(res);
    int varid = varid_map[res];
    varid_map.erase(res);
    SmartReg old = bind_map[varid];
    SmartReg nw = make_shared<FrameVar>(reg2Type<T>(), 4);
    if (bind_use[varid])
      setReg<T>(nw, getReg<T>(old));
    bind_map[varid] = nw;
    return res;
  }
  list<T> pos;
  unordered_map<T, typename list<T>::iterator> iter;
  unordered_map<T, int> varid_map;
};

LRU<Reg> sreg_lru;
LRU<FReg> f_sreg_lru;

queue<Reg> temp_queue;
Reg getNextTemp(Reg avoid = Reg::x0) {
  Reg reg = temp_queue.front();
  temp_queue.pop();
  temp_queue.push(reg);
  if (reg == avoid) {
    reg = temp_queue.front();
    temp_queue.pop();
    temp_queue.push(reg);
  }
  return reg;
}

queue<FReg> f_temp_queue;
FReg getNextFTemp(FReg avoid = FReg::null) {
  FReg reg = f_temp_queue.front();
  f_temp_queue.pop();
  f_temp_queue.push(reg);
  if (reg == avoid) {
    reg = f_temp_queue.front();
    f_temp_queue.pop();
    f_temp_queue.push(reg);
  }
  return reg;
}

void F2I(FReg a, Reg b) { Gen::inst(Inst::Fcvt_w_s, b, a); }

void I2F(Reg a, FReg b) { Gen::inst(Inst::Fcvt_s_w, b, a); }

set<Reg> used_reg_pool;
set<FReg> f_used_reg_pool;

template <> Reg getReg<Reg>(const SmartReg &reg, Reg avoid) {
  if (auto r = get_if<SReg>(&reg)) {
    sreg_lru.touch(**r);
    // used_reg_pool.insert(**r);
    return **r;
  } else if (auto r = get_if<SVar>(&reg)) {
    Reg tmp = getNextTemp(avoid);
    if ((*r)->type == FrameType::Float) {
      FReg ftmp = getNextFTemp();
      Gen::inst(Inst::Flw, ftmp, -(*r)->frame, Reg::fp);
      F2I(ftmp, tmp);
    } else {
      Gen::inst(Inst::Lw, tmp, -(*r)->frame, Reg::fp);
    }
    return tmp;
  } else if (auto r = get_if<Reg>(&reg)) {
    return *r;
  } else if (auto r = get_if<FSReg>(&reg)) {
    Reg tmp = getNextTemp(avoid);
    // f_used_reg_pool.insert(**r);
    F2I(**r, tmp);
    return Reg::t0;
  } else if (auto r = get_if<FReg>(&reg)) {
    Reg tmp = getNextTemp(avoid);
    F2I(*r, tmp);
    return tmp;
  } else {
    fprintf(stderr, "empty getReg<Reg>\n");
    return Reg::x0;
  }
}

template <> Reg getReg<Reg>(const SmartReg &reg) {
  return getReg<Reg>(reg, Reg::x0);
}

template <> FReg getReg<FReg>(const SmartReg &reg, FReg avoid) {
  if (auto r = get_if<FSReg>(&reg)) {
    f_sreg_lru.touch(**r);
    // f_used_reg_pool.insert(**r);
    return **r;
  } else if (auto r = get_if<SVar>(&reg)) {
    FReg tmp = getNextFTemp(avoid);
    if ((*r)->type == FrameType::Int) {
      Reg itmp = getNextTemp();
      Gen::inst(Inst::Lw, itmp, -(*r)->frame, Reg::fp);
      I2F(itmp, tmp);
    } else {
      Gen::inst(Inst::Flw, tmp, -(*r)->frame, Reg::fp);
    }
    return tmp;
  } else if (auto r = get_if<FReg>(&reg)) {
    return *r;
  } else if (auto r = get_if<SReg>(&reg)) {
    FReg tmp = getNextFTemp(avoid);
    // used_reg_pool.insert(**r);
    I2F(**r, tmp);
    return tmp;
  } else if (auto r = get_if<Reg>(&reg)) {
    FReg tmp = getNextFTemp(avoid);
    I2F(*r, tmp);
    return tmp;
  } else {
    fprintf(stderr, "empty getReg<FReg>\n");
    return FReg::null;
  }
}

template <> FReg getReg<FReg>(const SmartReg &reg) {
  return getReg<FReg>(reg, FReg::null);
}

Reg getAddr(const SmartReg &reg, Reg avoid = Reg::x0) {
  if (auto r = get_if<SVar>(&reg)) {
    if ((*r)->type == FrameType::Unknown) {
      Reg tmp = getNextTemp(avoid);
      if (abs((*r)->frame) < (1 << 11)) {
        Gen::inst(Inst::Addi, tmp, Reg::fp, -(*r)->frame);
      } else {
        Gen::inst(Inst::Li, tmp, -(*r)->frame);
        Gen::inst(Inst::Add, tmp, Reg::fp, tmp);
      }
      return tmp;
    } else {
      return getReg<Reg>(reg);
    }
  } else if (auto r = get_if<SReg>(&reg)) {
    return **r;
  } else if (auto r = get_if<Reg>(&reg)) {
    return *r;
  } else {
    fprintf(stderr, "empty getAddr\n");
    return Reg::x0;
  }
}

bool ok_reg_redu = true;

template <> void setReg<Reg>(SmartReg &reg, Reg src) {
  if (auto r = get_if<Reg>(&reg)) {
    used_reg_pool.insert(*r);
    Gen::inst(Inst::Mv, *r, src);
  } else if (auto r = get_if<SReg>(&reg)) {
    used_reg_pool.insert(**r);
    Gen::inst(Inst::Mv, **r, src);
  } else if (auto r = get_if<SVar>(&reg)) {
    if ((*r)->type == FrameType::Float) {
      FReg ftmp = getNextFTemp();
      I2F(src, ftmp);
      Gen::inst(Inst::Fsw, ftmp, -(*r)->frame, Reg::fp);
    } else {
      auto sreg = getSavedReg();
      if (sreg && ok_reg_redu) {
        used_reg_pool.insert(*sreg);
        Gen::inst(Inst::Mv, *sreg, src);
        reg = *sreg;
      } else
        Gen::inst(Inst::Sw, src, -(*r)->frame, Reg::fp);
    }
  } else if (auto r = get_if<FSReg>(&reg)) {
    FReg ftmp = getNextFTemp();
    I2F(src, ftmp);
    f_used_reg_pool.insert(**r);
    Gen::inst(Inst::Fmv, **r, ftmp);
  } else if (auto r = get_if<FReg>(&reg)) {
    FReg ftmp = getNextFTemp();
    I2F(src, ftmp);
    Gen::inst(Inst::Fmv, *r, ftmp);
  } else {
    fprintf(stderr, "empty setReg<Reg>\n");
  }
}

template <> void setReg<FReg>(SmartReg &reg, FReg src) {
  if (auto r = get_if<FSReg>(&reg)) {
    f_used_reg_pool.insert(**r);
    Gen::inst(Inst::Fmv, **r, src);
  } else if (auto r = get_if<FReg>(&reg)) {
    f_used_reg_pool.insert(*r);
    Gen::inst(Inst::Fmv, *r, src);
  } else if (auto r = get_if<SVar>(&reg)) {
    if ((*r)->type == FrameType::Int) {
      Reg itmp = getNextTemp();
      F2I(src, itmp);
      Gen::inst(Inst::Sw, itmp, -(*r)->frame, Reg::fp);
    } else {
      auto sreg = getSavedFReg();
      if (sreg && ok_reg_redu) {
        f_used_reg_pool.insert(*sreg);
        Gen::inst(Inst::Fmv, *sreg, src);
      } else
        Gen::inst(Inst::Fsw, src, -(*r)->frame, Reg::fp);
    }
  } else if (auto r = get_if<SReg>(&reg)) {
    Reg itmp = getNextTemp();
    F2I(src, itmp);
    used_reg_pool.insert(**r);
    Gen::inst(Inst::Mv, **r, itmp);
  } else if (auto r = get_if<Reg>(&reg)) {
    Reg itmp = getNextTemp();
    F2I(src, itmp);
    Gen::inst(Inst::Mv, *r, itmp);
  } else {
    fprintf(stderr, "empty setReg<FReg>\n");
  }
}

vector<Reg> free_reg_pool = {Reg::s11, Reg::s10, Reg::s9, Reg::s8,
                             Reg::s7,  Reg::s6,  Reg::s5, Reg::s4,
                             Reg::s3,  Reg::s2,  Reg::s1};
vector<FReg> f_free_reg_pool = {FReg::fs11, FReg::fs10, FReg::fs9, FReg::fs8,
                                FReg::fs7,  FReg::fs6,  FReg::fs5, FReg::fs4,
                                FReg::fs3,  FReg::fs2,  FReg::fs1, FReg::fs0};

SReg getSavedReg() {
  if (free_reg_pool.empty()) {
    if (ok_reg_redu) {
      auto sreg = sreg_lru.getSReg();
      if (!sreg.has_value())
        return nullptr;
      if (free_reg_pool.empty()) {
        fprintf(stderr, "empty pool in getSavedReg\n");
        return nullptr;
      }
    } else {
      return nullptr;
    }
  }
  Reg reg = free_reg_pool.back();
  free_reg_pool.pop_back();
  // used_reg_pool.insert(reg);
  return SReg(new Reg(reg), [reg](Reg *r) {
    delete r;
    free_reg_pool.push_back(reg);
  });
}

FSReg getSavedFReg() {
  if (f_free_reg_pool.empty()) {
    if (ok_reg_redu) {
      auto sreg = f_sreg_lru.getSReg();
      if (!sreg.has_value())
        return nullptr;
      if (f_free_reg_pool.empty()) {
        fprintf(stderr, "empty pool in getSavedFReg\n");
        return nullptr;
      }
    } else {
      return nullptr;
    }
  }
  FReg reg = f_free_reg_pool.back();
  f_free_reg_pool.pop_back();
  // f_used_reg_pool.insert(reg);
  return FSReg(new FReg(reg), [reg](FReg *r) {
    delete r;
    f_free_reg_pool.push_back(reg);
  });
}

SmartReg allocSavedIt() {
  auto sreg = getSavedReg();
  if (sreg)
    return sreg;
  return make_shared<FrameVar>(FrameType::Int, 4);
}

SmartReg allocSavedItF() {
  auto sreg = getSavedFReg();
  if (sreg)
    return sreg;
  return make_shared<FrameVar>(FrameType::Float, 4);
}

bool forceI(SmartReg &reg) {
  if (get_if<SReg>(&reg))
    return true;
  if (get_if<Reg>(&reg))
    return false;
  if (get_if<FSReg>(&reg) || get_if<FReg>(&reg)) {
    reg = getReg<Reg>(reg);
  }
  if (auto r = get_if<SVar>(&reg)) {
    if ((*r)->type != FrameType::Float)
      return true;
    reg = getReg<Reg>(reg);
  }
  return false;
}

bool forceF(SmartReg &reg) {
  if (get_if<FSReg>(&reg))
    return true;
  if (get_if<FReg>(&reg))
    return false;
  if (get_if<SReg>(&reg) || get_if<Reg>(&reg)) {
    reg = getReg<FReg>(reg);
  }
  if (auto r = get_if<SVar>(&reg)) {
    if ((*r)->type != FrameType::Int)
      return true;
    reg = getReg<FReg>(reg);
  }
  return false;
}

void genSavedIt(SmartReg &reg) {
  if (forceI(reg))
    return;
  auto sreg = getSavedReg();
  if (sreg) {
    Gen::inst(Inst::Mv, *sreg, getReg<Reg>(reg));
    used_reg_pool.insert(*sreg);
    reg = sreg;
    return;
  }
  auto var = make_shared<FrameVar>(FrameType::Int, 4);
  Gen::inst(Inst::Sw, getReg<Reg>(reg), -var->frame, Reg::fp);
  reg = var;
}

void genSavedItF(SmartReg &reg) {
  if (forceF(reg))
    return;
  auto sreg = getSavedFReg();
  if (sreg) {
    Gen::inst(Inst::Fmv, *sreg, getReg<FReg>(reg));
    f_used_reg_pool.insert(*sreg);
    reg = sreg;
    return;
  }
  auto var = make_shared<FrameVar>(FrameType::Float, 4);
  Gen::inst(Inst::Fsw, getReg<FReg>(reg), -var->frame, Reg::fp);
  reg = var;
}

void bindVar(int &varid, SmartReg &reg) {
  static int varid_num = 1;
  if (!varid)
    varid = varid_num++;
  bind_map[varid] = reg;
  bind_use[varid] = false;
  if (auto r = get_if<SReg>(&reg))
    sreg_lru.add(**r, varid);
  else if (auto r = get_if<FSReg>(&reg))
    f_sreg_lru.add(**r, varid);
}

void resetFrame() {
  bind_map.clear();
  bind_use.clear();
  free_reg_pool = {Reg::s11, Reg::s10, Reg::s9, Reg::s8, Reg::s7, Reg::s6,
                   Reg::s5,  Reg::s4,  Reg::s3, Reg::s2, Reg::s1};
  f_free_reg_pool = {FReg::fs11, FReg::fs10, FReg::fs9, FReg::fs8,
                     FReg::fs7,  FReg::fs6,  FReg::fs5, FReg::fs4,
                     FReg::fs3,  FReg::fs2,  FReg::fs1, FReg::fs0};
  sreg_lru.clear();
  f_sreg_lru.clear();
  free_frame.clear();
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
    fprintf(stderr, "unknown in genFloatBinOp\n");
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
  int now_id = gen_int_expr_id++;
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
      if (getConstIValue(node)) {
        Gen::inst(Inst::Li, Reg::t0, getConstIValue(node));
        result = Reg::t0;
      } else {
        result = Reg::x0;
      }
    } else if (node.expr().kind == BINARY_OPERATION) {
      switch (node.expr().op.binaryOp) {
      case BINARY_OP_AND: {
        SmartReg res = genExpr(node.child()[0]);
        bool old_ok_reg_redu = ok_reg_redu;
        Gen::inst(Inst::Mv, Reg::t2, getReg<Reg>(res));
        Gen::inst(Inst::Beqz, Reg::t2, "_short_int_" + to_string(now_id));
        SmartReg res2 = genExpr(node.child()[1]);
        Gen::inst(Inst::Mv, Reg::t2, getReg<Reg>(res2));
        Gen::label("_short_int_" + to_string(now_id));
        ok_reg_redu = old_ok_reg_redu;
        Gen::inst(Inst::Snez, Reg::t2, Reg::t2);
        result = Reg::t2;
      } break;
      case BINARY_OP_OR: {
        SmartReg res = genExpr(node.child()[0]);
        bool old_ok_reg_redu = ok_reg_redu;
        Gen::inst(Inst::Mv, Reg::t2, getReg<Reg>(res));
        Gen::inst(Inst::Bnez, Reg::t2, "_short_int_" + to_string(now_id));
        SmartReg res2 = genExpr(node.child()[1]);
        Gen::inst(Inst::Mv, Reg::t2, getReg<Reg>(res2));
        Gen::label("_short_int_" + to_string(now_id));
        ok_reg_redu = old_ok_reg_redu;
        Gen::inst(Inst::Snez, Reg::t2, Reg::t2);
        result = Reg::t2;
      } break;
      default: {
        SmartReg res1 = genExpr(node.child()[0]);
        genSavedIt(res1);
        SmartReg res2 = genExpr(node.child()[1]);
        Reg r2 = getReg<Reg>(res2);
        Reg r1 = getReg<Reg>(res1, r2);
        result = genIntBinOp(node.expr().op.binaryOp, r1, r2);
      }
      }
    } else {
      SmartReg res = genExpr(node.child());
      result = genIntUniOp(node.expr().op.unaryOp, getReg<Reg>(res));
    }
    break;
  case CONST_VALUE_NODE:
    switch (node.cons()->const_type) {
    case INTEGERC:
      if (node.cons()->const_u.intval) {
        Gen::inst(Inst::Li, Reg::t0, node.cons()->const_u.intval);
        result = Reg::t0;
      } else {
        result = Reg::x0;
      }
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
        if (node->dataType == INT_PTR_TYPE) {
          Gen::inst(Inst::La, Reg::t0, "_g_" + node.name());
          result = Reg::t0;
        } else {
          Gen::inst(Inst::Lw, Reg::t0, "_g_" + node.name());
          result = Reg::t0;
        }
      } else {
        if (node->dataType == INT_PTR_TYPE) {
          result = getAddr(bind_map[node.identifier().symbolTableEntry->varid]);
        } else {
          result =
              getReg<Reg>(bind_map[node.identifier().symbolTableEntry->varid]);
        }
      }
      break;
    case ARRAY_ID: {
      if (node->dataType == INT_PTR_TYPE) {
        result = genArrayLocation(node);
      } else {
        SmartReg location = genArrayLocation(node);
        Gen::inst(Inst::Lw, Reg::t0, 0, getReg<Reg>(location));
        result = Reg::t0;
      }
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
  int now_id = gen_float_expr_id++;
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
        bool old_ok_reg_redu = ok_reg_redu;
        Gen::inst(Inst::Mv, Reg::t2, getReg<Reg>(res));
        Gen::inst(Inst::Beqz, Reg::t2, "_short_float_" + to_string(now_id));
        SmartReg res2 = genExpr(node.child()[1]);
        Gen::inst(Inst::Mv, Reg::t2, getReg<Reg>(res2));
        Gen::label("_short_float_" + to_string(now_id));
        ok_reg_redu = old_ok_reg_redu;
        Gen::inst(Inst::Snez, Reg::t2, Reg::t2);
        I2F(Reg::t2, FReg::ft0);
        result = FReg::ft0;
      } break;
      case BINARY_OP_OR: {
        SmartReg res = genExpr(node.child()[0]);
        bool old_ok_reg_redu = ok_reg_redu;
        Gen::inst(Inst::Mv, Reg::t2, getReg<Reg>(res));
        Gen::inst(Inst::Bnez, Reg::t2, "_short_float_" + to_string(now_id));
        SmartReg res2 = genExpr(node.child()[1]);
        Gen::inst(Inst::Mv, Reg::t2, getReg<Reg>(res2));
        Gen::label("_short_float_" + to_string(now_id));
        ok_reg_redu = old_ok_reg_redu;
        Gen::inst(Inst::Snez, Reg::t2, Reg::t2);
        I2F(Reg::t2, FReg::ft0);
        result = FReg::ft0;
      } break;
      default: {
        SmartReg res1 = genExpr(node.child()[0]);
        genSavedItF(res1);
        SmartReg res2 = genExpr(node.child()[1]);
        FReg r2 = getReg<FReg>(res2);
        FReg r1 = getReg<FReg>(res1, r2);
        result = genFloatBinOp(node.expr().op.binaryOp, r1, r2);
      }
      }
    } else {
      SmartReg res = genExpr(node.child());
      result = genFloatUniOp(node.expr().op.unaryOp, getReg<FReg>(res));
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
        if (node->dataType == FLOAT_PTR_TYPE) {
          Gen::inst(Inst::La, Reg::t0, "_g_" + node.name());
          result = Reg::t0;
        } else {
          Gen::inst(Inst::Flw_sym, FReg::ft0, "_g_" + node.name(), Reg::t2);
          result = FReg::ft0;
        }
      } else {
        if (node->dataType == FLOAT_PTR_TYPE) {
          result = getAddr(bind_map[node.identifier().symbolTableEntry->varid]);
        } else {
          result =
              getReg<FReg>(bind_map[node.identifier().symbolTableEntry->varid]);
        }
      }
      break;
    case ARRAY_ID: {
      if (node->dataType == FLOAT_PTR_TYPE) {
        result = genArrayLocation(node);
      } else {
        SmartReg location = genArrayLocation(node);
        Gen::inst(Inst::Flw, FReg::ft0, 0, getReg<Reg>(location));
        result = FReg::ft0;
      }
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
  } else if (node->dataType == INT_PTR_TYPE) {
    return genIntExpr(node);
  } else if (node->dataType == FLOAT_PTR_TYPE) {
    return genFloatExpr(node);
  } else {
    fprintf(stderr, "unknown in genExpr\n");
    return Reg::t0;
  }
}

SmartReg genArrayLocation(Node node) {
  auto sym = node.identifier().symbolTableEntry;
  auto &prop = sym->attribute->attr.typeDescriptor->properties.arrayProperties;
  Node expr = node.child();
  SmartReg location = allocSavedIt();
  setReg<Reg>(location, Reg::x0);
  for (int dim = 1; dim <= prop.dimension; dim++) {
    int factor = 4;
    if (dim < prop.dimension)
      factor = prop.sizeInEachDimension[dim];
    if (expr) {
      SmartReg res = genIntExpr(expr);
      Gen::inst(Inst::Add, Reg::t0, getReg<Reg>(res), getReg<Reg>(location));
    } else {
      Gen::inst(Inst::Mv, Reg::t0, getReg<Reg>(location));
    }
    Gen::inst(Inst::Li, Reg::t1, factor);
    Gen::inst(Inst::Mul, Reg::t0, Reg::t0, Reg::t1);
    setReg<Reg>(location, Reg::t0);
    if (expr)
      ++expr;
  }
  Reg addr;
  if (sym->nestingLevel == 0) {
    Gen::inst(Inst::La, Reg::t0, "_g_" + node.name());
    addr = Reg::t0;
  } else {
    addr = getAddr(bind_map[sym->varid]);
  }
  Gen::inst(Inst::Add, Reg::t0, getReg<Reg>(location), addr);
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
        Reg tmp = getReg<Reg>(result);
        Gen::inst(Inst::Sw_sym, tmp, "_g_" + dst.name(), Reg::t2);
        result = tmp;
      } else {
        setReg<Reg>(bind_map[dst.identifier().symbolTableEntry->varid],
                    getReg<Reg>(result));
        bind_use[dst.identifier().symbolTableEntry->varid] = true;
      }
    } else if (dst->dataType == FLOAT_TYPE) {
      if (dst.identifier().symbolTableEntry->nestingLevel == 0) {
        FReg tmp = getReg<FReg>(result);
        Gen::inst(Inst::Fsw_sym, tmp, "_g_" + dst.name(), Reg::t2);
        result = tmp;
      } else {
        setReg<FReg>(bind_map[dst.identifier().symbolTableEntry->varid],
                     getReg<FReg>(result));
        bind_use[dst.identifier().symbolTableEntry->varid] = true;
      }
    } else {
      fprintf(stderr, "unknown in genAssign\n");
    }
    break;
  case ARRAY_ID: {
    if (dst->dataType == INT_TYPE) {
      genSavedIt(result);
      SmartReg location = genArrayLocation(dst);
      Gen::inst(Inst::Sw, getReg<Reg>(result), 0, getReg<Reg>(location));
    } else if (dst->dataType == FLOAT_TYPE) {
      genSavedItF(result);
      SmartReg location = genArrayLocation(dst);
      Gen::inst(Inst::Fsw, getReg<FReg>(result), 0, getReg<Reg>(location));
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
  bool old_ok_reg_redu = ok_reg_redu;
  ok_reg_redu = false;
  Gen::inst(Inst::Beqz, getReg<Reg>(result), "_if_end_" + to_string(now_id));
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
  ok_reg_redu = old_ok_reg_redu;
}
void genReturn(Node node) {
  Node parent = node->parent;
  DATA_TYPE returnType = NONE_TYPE;
  while (parent) {
    if (parent->nodeType == DECLARATION_NODE) {
      if (parent->semantic_value.declSemanticValue.kind == FUNCTION_DECL)
        returnType = parent.child()->dataType;
      break;
    }
    parent = parent->parent;
  }
  if (node.child()->nodeType != NUL_NODE) {
    SmartReg result = genExpr(node.child());
    if (returnType == INT_TYPE) {
      Gen::inst(Inst::Mv, Reg::a0, getReg<Reg>(result));
    } else if (returnType == FLOAT_TYPE) {
      Gen::inst(Inst::Fmv, FReg::fa0, getReg<FReg>(result));
    } else {
      fprintf(stderr, "unknown in genReturn\n");
    }
  }
  Gen::inst(Inst::J, "_end_" + cur_function_name);
}
SmartReg genFunctionCall(Node node) {
  // static int func_call_num = 0;
  // func_call_num++;
  // Gen::label("_func_call" + to_string(func_call_num));
  Node func = node.child()[0];
  Node paramList = node.child()[1];
  if (func.name() == "write") {
    SmartReg reg = genExpr(paramList.child());
    if (paramList.child()->dataType == CONST_STRING_TYPE) {
      Gen::inst(Inst::Mv, Reg::a0, getReg<Reg>(reg));
      Gen::inst(Inst::Jal, "_write_str");
    } else if (paramList.child()->dataType == INT_TYPE) {
      Gen::inst(Inst::Mv, Reg::a0, getReg<Reg>(reg));
      Gen::inst(Inst::Jal, "_write_int");
    } else {
      Gen::inst(Inst::Fmv, FReg::fa0, getReg<FReg>(reg));
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
    if (paramList->nodeType != NUL_NODE) {
      deque<Reg> a_reg = {Reg::a0, Reg::a1, Reg::a2, Reg::a3,
                          Reg::a4, Reg::a5, Reg::a6, Reg::a7};
      deque<FReg> a_freg = {FReg::fa0, FReg::fa1, FReg::fa2, FReg::fa3,
                            FReg::fa4, FReg::fa5, FReg::fa6, FReg::fa7};
      Node params = paramList.child();
      Parameter *signParam = func.identifier()
                                 .symbolTableEntry->attribute->attr
                                 .functionSignature->parameterList;
      int stk_cnt = 0;
      for (Node param : params) {
        SmartReg res = genExpr(param);
        if (signParam->type->kind == SCALAR_TYPE_DESCRIPTOR) {
          if (signParam->type->properties.dataType == INT_TYPE) {
            if (a_reg.empty()) {
              Gen::inst(Inst::Sw, getReg<Reg>(res), -8 - 4 * ++stk_cnt,
                        Reg::sp);
            } else {
              Gen::inst(Inst::Mv, a_reg.front(), getReg<Reg>(res));
              a_reg.pop_front();
            }
          } else if (signParam->type->properties.dataType == FLOAT_TYPE) {
            if (a_freg.empty()) {
              Gen::inst(Inst::Fsw, getReg<FReg>(res), -8 - 4 * ++stk_cnt,
                        Reg::sp);
            } else {
              Gen::inst(Inst::Fmv, a_freg.front(), getReg<FReg>(res));
              a_freg.pop_front();
            }
          }
        } else {
          if (a_reg.empty()) {
            Gen::inst(Inst::Sw, getReg<Reg>(res), -8 - 4 * ++stk_cnt, Reg::sp);
          } else {
            Gen::inst(Inst::Mv, a_reg.front(), getReg<Reg>(res));
            a_reg.pop_front();
          }
        }
        signParam = signParam->next;
      }
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
  bool old_ok_reg_redu = ok_reg_redu;
  ok_reg_redu = false;
  Gen::label("_while_begin_" + to_string(now_id));
  if (exp->nodeType == STMT_NODE && exp.stmt().kind == ASSIGN_STMT) {
    result = genAssign(exp);
  } else {
    result = genExpr(exp);
  }
  Gen::inst(Inst::Beqz, getReg<Reg>(result), "_while_end_" + to_string(now_id));
  if (run->nodeType == STMT_NODE) {
    genStmt(run);
  } else if (run->nodeType == BLOCK_NODE) {
    genBlock(run);
  } else if (run->nodeType == NUL_NODE) {
    // Do nothing
  }
  Gen::inst(Inst::J, "_while_begin_" + to_string(now_id));
  Gen::label("_while_end_" + to_string(now_id));
  ok_reg_redu = old_ok_reg_redu;
}

void genFor(Node node) {
  static int for_id = 0;
  for_id++;
  int now_id = for_id;
  Node init = node.child()[0];
  Node cond = node.child()[1];
  Node fini = node.child()[2];
  Node run = node.child()[3];
  if (init->nodeType == NONEMPTY_ASSIGN_EXPR_LIST_NODE) {
    Node childs = init.child();
    for (Node child : childs)
      genAssign(child);
  }
  bool old_ok_reg_redu = ok_reg_redu;
  ok_reg_redu = false;
  Gen::label("_for_begin_" + to_string(now_id));
  if (cond->nodeType == NONEMPTY_RELOP_EXPR_LIST_NODE) {
    SmartReg result;
    Node exps = cond.child();
    for (Node exp : exps) {
      result = genExpr(exp);
    }
    Gen::inst(Inst::Beqz, getReg<Reg>(result), "_for_end_" + to_string(now_id));
  } else {
    // Do nothing
  }
  if (run->nodeType == STMT_NODE) {
    genStmt(run);
  } else if (run->nodeType == BLOCK_NODE) {
    genBlock(run);
  } else if (run->nodeType == NUL_NODE) {
    // Do nothing
  }
  if (fini->nodeType == NONEMPTY_ASSIGN_EXPR_LIST_NODE) {
    Node ends = fini.child();
    for (Node end : ends) {
      if (end->nodeType == STMT_NODE) {
        genAssign(end);
      } else if (end->nodeType == EXPR_NODE) {
        genExpr(end);
      }
    }
  }
  Gen::inst(Inst::J, "_for_begin_" + to_string(now_id));
  Gen::label("_for_end_" + to_string(now_id));
  ok_reg_redu = old_ok_reg_redu;
}

void genStmt(Node node) {
  switch (node.stmt().kind) {
  case WHILE_STMT:
    genWhile(node);
    break;
  case FOR_STMT:
    genFor(node);
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
            // if (kind == SCALAR_TYPE_DESCRIPTOR) {
            if (type->dataType == INT_TYPE) {
              SmartReg reg = allocSavedIt();
              bindVar(id.identifier().symbolTableEntry->varid, reg);
            } else {
              SmartReg reg = allocSavedItF();
              bindVar(id.identifier().symbolTableEntry->varid, reg);
            }
            // } else {
            //   int size = 4;
            //   auto &prop = sym->attribute->attr.typeDescriptor->properties
            //                    .arrayProperties;
            //   for (int i = 0; i < prop.dimension; i++) {
            //     size *= prop.sizeInEachDimension[i];
            //   }
            //   SmartReg reg = make_shared<FrameVar>(FrameType::Unknown, size);
            //   bindVar(id.identifier().symbolTableEntry->varid, reg);
            // }
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
                I2F(getReg<Reg>(reg), FReg::ft0);
                SmartReg tmp = FReg::ft0;
                genSavedItF(tmp);
                bindVar(id.identifier().symbolTableEntry->varid, tmp);
              }
            } else {
              if (id.identifier()
                      .symbolTableEntry->attribute->attr.typeDescriptor
                      ->properties.dataType == INT_TYPE) {
                F2I(getReg<FReg>(reg), Reg::t0);
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
            SmartReg reg = make_shared<FrameVar>(FrameType::Unknown, size);
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

void genFuncParam(Node node) {
  deque<Reg> a_reg = {Reg::a0, Reg::a1, Reg::a2, Reg::a3,
                      Reg::a4, Reg::a5, Reg::a6, Reg::a7};
  deque<FReg> a_freg = {FReg::fa0, FReg::fa1, FReg::fa2, FReg::fa3,
                        FReg::fa4, FReg::fa5, FReg::fa6, FReg::fa7};
  Node decls = node.child();
  for (Node decl : decls) {
    Node type = decl.child()[0];
    Node id = decl.child()[1];
    auto sym = id.identifier().symbolTableEntry;
    auto kind = sym->attribute->attr.typeDescriptor->kind;
    switch (id.identifier().kind) {
    case NORMAL_ID: {
      if (kind == SCALAR_TYPE_DESCRIPTOR) {
        if (type->dataType == INT_TYPE) {
          if (a_reg.empty()) {
            SmartReg reg = make_shared<FrameVar>(FrameType::Int, 4);
            bindVar(id.identifier().symbolTableEntry->varid, reg);
          } else {
            SmartReg reg = allocSavedIt();
            setReg<Reg>(reg, a_reg.front());
            a_reg.pop_front();
            bindVar(id.identifier().symbolTableEntry->varid, reg);
          }
        } else {
          if (a_freg.empty()) {
            SmartReg reg = make_shared<FrameVar>(FrameType::Float, 4);
            bindVar(id.identifier().symbolTableEntry->varid, reg);
          } else {
            SmartReg reg = allocSavedItF();
            setReg<FReg>(reg, a_freg.front());
            a_freg.pop_front();
            bindVar(id.identifier().symbolTableEntry->varid, reg);
          }
        }
      } else {
        if (a_reg.empty()) {
          SmartReg reg = make_shared<FrameVar>(FrameType::ArrayPtr, 4);
          bindVar(id.identifier().symbolTableEntry->varid, reg);
        } else {
          SmartReg reg = allocSavedIt();
          setReg<Reg>(reg, a_reg.front());
          a_reg.pop_front();
          bindVar(id.identifier().symbolTableEntry->varid, reg);
        }
      }
    } break;
    case ARRAY_ID: {
      if (a_reg.empty()) {
        SmartReg reg = make_shared<FrameVar>(FrameType::ArrayPtr, 4);
        bindVar(id.identifier().symbolTableEntry->varid, reg);
      } else {
        SmartReg reg = allocSavedIt();
        setReg<Reg>(reg, a_reg.front());
        a_reg.pop_front();
        bindVar(id.identifier().symbolTableEntry->varid, reg);
      }
    } break;
    default:
      fprintf(stderr, "unkonwn in genFuncParam\n");
    }
  }
}

void genGDeclFunction(Node node) {
  Node type = node.child()[0];
  Node id = node.child()[1];
  Node param = node.child()[2];
  Node block = node.child()[3];
  Gen::segment("text");
  Gen::label("_start_" + node.child()[1].name());
  Gen::inst(Inst::Sd, Reg::ra, 0, Reg::sp);
  Gen::inst(Inst::Sd, Reg::fp, -8, Reg::sp);
  Gen::inst(Inst::Add, Reg::fp, Reg::sp, -8);
  Gen::inst(Inst::Lw, Reg::ra, "_frameSize_" + id.name());
  Gen::inst(Inst::Sub, Reg::sp, Reg::sp, Reg::ra);
  function_frame_size = 0;
  cur_function_name = id.name();
  std::swap(codes, codes_tmp);
  genFuncParam(param);
  genBlock(block);
  Gen::label("_end_" + id.name());
  std::swap(codes, codes_tmp);
  map<Reg, int> reg_map;
  map<FReg, int> freg_map;
  Gen::label("_storeStart_" + node.child()[1].name());
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
  Gen::label("_storeEnd_" + node.child()[1].name());
  for (auto &code : codes_tmp)
    codes.emplace_back(move(code));
  codes_tmp.clear();
  Gen::label("_loadStart_" + node.child()[1].name());
  for (Reg reg : used_reg_pool) {
    Gen::inst(Inst::Lw, reg, -reg_map[reg], Reg::fp);
  }
  for (FReg reg : f_used_reg_pool) {
    Gen::inst(Inst::Flw, reg, -freg_map[reg], Reg::fp);
  }
  Gen::label("_loadEnd_" + node.child()[1].name());
  Gen::inst(Inst::Ld, Reg::ra, 8, Reg::fp);
  Gen::inst(Inst::Mv, Reg::sp, Reg::fp);
  Gen::inst(Inst::Add, Reg::sp, Reg::sp, 8);
  Gen::inst(Inst::Ld, Reg::fp, 0, Reg::fp);
  Gen::inst(Inst::Jr, Reg::ra);
  Gen::segment("data");
  Gen::label("_frameSize_" + id.name(), ".word", function_frame_size + 16);
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
string stringArg(const InstArg &arg) {
  return visit(
      overloaded{[](const Reg &arg) -> string {
                   return reg_name[static_cast<int>(arg)];
                 },
                 [](const FReg &arg) -> string {
                   return freg_name[static_cast<int>(arg)];
                 },
                 [](const Symbol &arg) -> string { return arg.c_str(); },
                 [](const int &arg) -> string { return to_string(arg); },
                 [](const double &arg) -> string { return to_string(arg); }},
      arg);
}
string stringArgs(const vector<InstArg> &args, int i = 0) {
  string s;
  for (; i < args.size(); i++) {
    s += stringArg(args[i]) + ",";
  }
  return s;
}
void printRemain(const vector<InstArg> &args, int i) {
  for (; i < args.size(); i++)
    printf(" "), printArg(args[i]);
}
void printLoadStore(const string &ins, const vector<InstArg> &args) {
  if (args.size() == 3) {
    int val = get<int>(args[1]);
    auto arg = args[2];
    if (abs(val) >= (1 << 11)) {
      printf("li t2, %d\n", val);
      printf("add t2, t2, ");
      printArg(args[2]);
      puts("");
      val = 0;
      arg = Reg::t2;
    }
    printf("%s", ins.c_str());
    printArg(args[0]);
    printf(", %d", val);
    printf("("), printArg(arg), printf(")");
  } else {
    printf("%s", ins.c_str());
    printArg(args[0]);
    printf(", ");
    printArg(args[1]);
  }
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
      printLoadStore("ld ", args);
      puts("");
      break;
    case Inst::Lw:
      printLoadStore("lw ", args);
      puts("");
      break;
    case Inst::Sd:
      printLoadStore("sd ", args);
      puts("");
      break;
    case Inst::Sd_sym:
      printf("sd ");
      printArgs(args);
      puts("");
      break;
    case Inst::Sw:
      printLoadStore("sw ", args);
      puts("");
      break;
    case Inst::Sw_sym:
      printf("sw ");
      printArgs(args);
      puts("");
      break;
    case Inst::Flw:
      printLoadStore("flw ", args);
      puts("");
      break;
    case Inst::Flw_sym:
      printf("flw ");
      printArgs(args);
      puts("");
      break;
    case Inst::Fsw:
      printLoadStore("fsw ", args);
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
    case Inst::Slli:
      printf("slli ");
      printArgs(args);
      puts("");
      break;
    case Inst::Addi:
      printf("addi ");
      printArgs(args);
      puts("");
      break;
    case Inst::Slti:
      printf("slti ");
      printArgs(args);
      puts("");
      break;
    case Inst::Andi:
      printf("andi ");
      printArgs(args);
      puts("");
      break;
    case Inst::Ori:
      printf("ori ");
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
      printf(", rtz");
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
    case Inst::Nop:
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
  unordered_map<string, string> data_map;
  unordered_map<string, string> label_map;
  for (auto &code : data) {
    if (code.first == Inst::Label) {
      if (get<Symbol>(code.second[0]).find("_CONSTANT_") == 0) {
        string s = stringArgs(code.second, 1);
        if (data_map.find(s) != data_map.end()) {
          label_map[get<Symbol>(code.second[0])] = data_map[s];
          continue;
        } else {
          data_map[s] = get<Symbol>(code.second[0]);
        }
      }
    }
    codes.emplace_back(move(code));
  }
  Gen::segment("text");
  for (auto &code : text) {
    for (auto &arg : code.second) {
      if (auto a = get_if<Symbol>(&arg)) {
        auto it = label_map.find(*a);
        if (it != label_map.end())
          arg = it->second;
      }
    }
    codes.emplace_back(move(code));
  }
}
namespace Status {
bitset<64> r_reg, w_reg;
bitset<64> r_freg, w_freg;
bool branch, label;
auto getWriteReadPos(const Codec &code) {
  vector<int> wr;
  vector<int> re;
  switch (code.first) {
  case Inst::Nop:
    break;
  case Inst::Mv:
  case Inst::Fmv:
  case Inst::La:
  case Inst::Li:
  case Inst::Lw:
  case Inst::Ld:
  case Inst::Flw:
  case Inst::Slli:
  case Inst::Addi:
  case Inst::Slti:
  case Inst::Andi:
  case Inst::Ori:
  case Inst::Add:
  case Inst::Sub:
  case Inst::Mul:
  case Inst::Div:
  case Inst::Fadd:
  case Inst::Fsub:
  case Inst::Fmul:
  case Inst::Fdiv:
  case Inst::And:
  case Inst::Or:
  case Inst::Fneg:
  case Inst::Fcvt_w_s:
  case Inst::Fcvt_s_w:
  case Inst::Slt:
  case Inst::Sgt:
  case Inst::Feq:
  case Inst::Flt:
  case Inst::Fgt:
  case Inst::Fle:
  case Inst::Fge:
  case Inst::Seqz:
  case Inst::Snez:
  case Inst::Sltz:
  case Inst::Sgtz:
  case Inst::Slez:
  case Inst::Sgez:
    wr = {0};
    for (int i = 1; i < code.second.size(); i++)
      re.push_back(i);
    break;
  case Inst::Sw:
  case Inst::Sd:
  case Inst::Fsw:
    for (int i = 0; i < code.second.size(); i++)
      re.push_back(i);
    break;
  case Inst::Flw_sym:
    wr = {0, 2};
    re = {1};
    break;
  case Inst::Sw_sym:
  case Inst::Sd_sym:
  case Inst::Fsw_sym:
    wr = {2};
    re = {0, 1};
    break;
  case Inst::Beq:
  case Inst::Bne:
  case Inst::Blt:
  case Inst::Bgt:
  case Inst::Beqz:
  case Inst::Bnez:
  case Inst::Bltz:
  case Inst::Bgtz:
  case Inst::Blez:
  case Inst::Bgez:
  case Inst::J:
  case Inst::Jr:
    for (int i = 0; i < code.second.size(); i++)
      re.push_back(i);
    break;
  case Inst::Jal:
    if (code.second.size() == 1)
      re = {0};
    else if (code.second.size() == 2)
      wr = {0}, re = {0};
    break;
  case Inst::Label:
  case Inst::Segment:
    break;
  default:
    fprintf(stderr, "unknown in getWriteReadPos\n");
  }
  return tuple(wr, re);
}
void getInstStatus(const Codec &code) {
  r_reg.reset();
  w_reg.reset();
  r_freg.reset();
  w_freg.reset();
  branch = false;
  label = false;
  auto w_setRegStat = [&](int i) {
    if (auto r = std::get_if<Reg>(&code.second[i])) {
      w_reg[static_cast<int>(*r)] = 1;
    } else if (auto r = std::get_if<FReg>(&code.second[i])) {
      w_freg[static_cast<int>(*r)] = 1;
    }
  };
  auto r_setRegStat = [&](int i) {
    if (auto r = std::get_if<Reg>(&code.second[i])) {
      r_reg[static_cast<int>(*r)] = 1;
    } else if (auto r = std::get_if<FReg>(&code.second[i])) {
      r_freg[static_cast<int>(*r)] = 1;
    }
  };
  switch (code.first) {
  case Inst::Nop:
  case Inst::Mv:
  case Inst::Fmv:
  case Inst::La:
  case Inst::Li:
  case Inst::Lw:
  case Inst::Ld:
  case Inst::Flw:
  case Inst::Slli:
  case Inst::Addi:
  case Inst::Slti:
  case Inst::Andi:
  case Inst::Ori:
  case Inst::Add:
  case Inst::Sub:
  case Inst::Mul:
  case Inst::Div:
  case Inst::Fadd:
  case Inst::Fsub:
  case Inst::Fmul:
  case Inst::Fdiv:
  case Inst::And:
  case Inst::Or:
  case Inst::Fneg:
  case Inst::Fcvt_w_s:
  case Inst::Fcvt_s_w:
  case Inst::Slt:
  case Inst::Sgt:
  case Inst::Feq:
  case Inst::Flt:
  case Inst::Fgt:
  case Inst::Fle:
  case Inst::Fge:
  case Inst::Seqz:
  case Inst::Snez:
  case Inst::Sltz:
  case Inst::Sgtz:
  case Inst::Slez:
  case Inst::Sgez:
  case Inst::Sw:
  case Inst::Sd:
  case Inst::Fsw:
  case Inst::Flw_sym:
  case Inst::Sw_sym:
  case Inst::Sd_sym:
  case Inst::Fsw_sym: {
    auto [wr, re] = getWriteReadPos(code);
    for (int w : wr)
      w_setRegStat(w);
    for (int r : re)
      r_setRegStat(r);
  } break;
  case Inst::Beq:
  case Inst::Bne:
  case Inst::Blt:
  case Inst::Bgt:
  case Inst::Beqz:
  case Inst::Bnez:
  case Inst::Bltz:
  case Inst::Bgtz:
  case Inst::Blez:
  case Inst::Bgez:
  case Inst::J:
  case Inst::Jr:
    branch = true;
    for (int i = 0; i < code.second.size(); i++)
      r_setRegStat(i);
    w_reg.set();
    w_freg.set();
    w_reg[(int)Reg::sp] = 0;
    w_reg[(int)Reg::fp] = 0;
    w_reg[(int)Reg::s1] = 0;
    w_reg[(int)Reg::s2] = 0;
    w_reg[(int)Reg::s3] = 0;
    w_reg[(int)Reg::s4] = 0;
    w_reg[(int)Reg::s5] = 0;
    w_reg[(int)Reg::s6] = 0;
    w_reg[(int)Reg::s7] = 0;
    w_reg[(int)Reg::s8] = 0;
    w_reg[(int)Reg::s9] = 0;
    w_reg[(int)Reg::s10] = 0;
    w_reg[(int)Reg::s11] = 0;
    w_freg[(int)FReg::fs0] = 0;
    w_freg[(int)FReg::fs1] = 0;
    w_freg[(int)FReg::fs2] = 0;
    w_freg[(int)FReg::fs3] = 0;
    w_freg[(int)FReg::fs4] = 0;
    w_freg[(int)FReg::fs5] = 0;
    w_freg[(int)FReg::fs6] = 0;
    w_freg[(int)FReg::fs7] = 0;
    w_freg[(int)FReg::fs8] = 0;
    w_freg[(int)FReg::fs9] = 0;
    w_freg[(int)FReg::fs10] = 0;
    w_freg[(int)FReg::fs11] = 0;
    r_reg[(int)Reg::sp] = 1;
    r_reg[(int)Reg::fp] = 1;
    r_reg[(int)Reg::a0] = 1;
    r_freg[(int)FReg::fa0] = 1;
    break;
  case Inst::Jal:
    branch = true;
    if (code.second.size() == 1)
      r_setRegStat(0), w_reg[static_cast<int>(Reg::ra)] = 1;
    if (code.second.size() == 2)
      w_setRegStat(0), r_setRegStat(1);
    w_reg.set();
    w_freg.set();
    w_reg[(int)Reg::sp] = 0;
    w_reg[(int)Reg::fp] = 0;
    w_reg[(int)Reg::s1] = 0;
    w_reg[(int)Reg::s2] = 0;
    w_reg[(int)Reg::s3] = 0;
    w_reg[(int)Reg::s4] = 0;
    w_reg[(int)Reg::s5] = 0;
    w_reg[(int)Reg::s6] = 0;
    w_reg[(int)Reg::s7] = 0;
    w_reg[(int)Reg::s8] = 0;
    w_reg[(int)Reg::s9] = 0;
    w_reg[(int)Reg::s10] = 0;
    w_reg[(int)Reg::s11] = 0;
    w_freg[(int)FReg::fs0] = 0;
    w_freg[(int)FReg::fs1] = 0;
    w_freg[(int)FReg::fs2] = 0;
    w_freg[(int)FReg::fs3] = 0;
    w_freg[(int)FReg::fs4] = 0;
    w_freg[(int)FReg::fs5] = 0;
    w_freg[(int)FReg::fs6] = 0;
    w_freg[(int)FReg::fs7] = 0;
    w_freg[(int)FReg::fs8] = 0;
    w_freg[(int)FReg::fs9] = 0;
    w_freg[(int)FReg::fs10] = 0;
    w_freg[(int)FReg::fs11] = 0;
    r_reg[(int)Reg::sp] = 1;
    r_reg[(int)Reg::fp] = 1;
    r_reg[(int)Reg::a0] = 1;
    r_reg[(int)Reg::a1] = 1;
    r_reg[(int)Reg::a2] = 1;
    r_reg[(int)Reg::a3] = 1;
    r_reg[(int)Reg::a4] = 1;
    r_reg[(int)Reg::a5] = 1;
    r_reg[(int)Reg::a6] = 1;
    r_reg[(int)Reg::a7] = 1;
    r_freg[(int)FReg::fa0] = 1;
    r_freg[(int)FReg::fa1] = 1;
    r_freg[(int)FReg::fa2] = 1;
    r_freg[(int)FReg::fa3] = 1;
    r_freg[(int)FReg::fa4] = 1;
    r_freg[(int)FReg::fa5] = 1;
    r_freg[(int)FReg::fa6] = 1;
    r_freg[(int)FReg::fa7] = 1;
    break;
  case Inst::Label:
    if (get<Symbol>(code.second[0]).find("_end_") == 0) {
      w_reg[(int)Reg::t0] = 1;
      w_reg[(int)Reg::t1] = 1;
      w_reg[(int)Reg::t2] = 1;
      w_reg[(int)Reg::t3] = 1;
      w_reg[(int)Reg::t4] = 1;
      w_reg[(int)Reg::t5] = 1;
      w_reg[(int)Reg::t6] = 1;
      w_reg[(int)Reg::s1] = 1;
      w_reg[(int)Reg::s2] = 1;
      w_reg[(int)Reg::s3] = 1;
      w_reg[(int)Reg::s4] = 1;
      w_reg[(int)Reg::s5] = 1;
      w_reg[(int)Reg::s6] = 1;
      w_reg[(int)Reg::s7] = 1;
      w_reg[(int)Reg::s8] = 1;
      w_reg[(int)Reg::s9] = 1;
      w_reg[(int)Reg::s11] = 1;
      w_reg[(int)Reg::a1] = 1;
      w_reg[(int)Reg::a2] = 1;
      w_reg[(int)Reg::a3] = 1;
      w_reg[(int)Reg::a4] = 1;
      w_reg[(int)Reg::a5] = 1;
      w_reg[(int)Reg::a6] = 1;
      w_reg[(int)Reg::a7] = 1;
      w_freg[(int)FReg::ft0] = 1;
      w_freg[(int)FReg::ft1] = 1;
      w_freg[(int)FReg::ft2] = 1;
      w_freg[(int)FReg::ft3] = 1;
      w_freg[(int)FReg::ft4] = 1;
      w_freg[(int)FReg::ft5] = 1;
      w_freg[(int)FReg::ft6] = 1;
      w_freg[(int)FReg::ft7] = 1;
      w_freg[(int)FReg::ft8] = 1;
      w_freg[(int)FReg::ft9] = 1;
      w_freg[(int)FReg::ft10] = 1;
      w_freg[(int)FReg::ft11] = 1;
      w_freg[(int)FReg::fs0] = 1;
      w_freg[(int)FReg::fs1] = 1;
      w_freg[(int)FReg::fs2] = 1;
      w_freg[(int)FReg::fs3] = 1;
      w_freg[(int)FReg::fs4] = 1;
      w_freg[(int)FReg::fs5] = 1;
      w_freg[(int)FReg::fs6] = 1;
      w_freg[(int)FReg::fs7] = 1;
      w_freg[(int)FReg::fs8] = 1;
      w_freg[(int)FReg::fs9] = 1;
      w_freg[(int)FReg::fs10] = 1;
      w_freg[(int)FReg::fs11] = 1;
      w_freg[(int)FReg::fa1] = 1;
      w_freg[(int)FReg::fa2] = 1;
      w_freg[(int)FReg::fa3] = 1;
      w_freg[(int)FReg::fa4] = 1;
      w_freg[(int)FReg::fa5] = 1;
      w_freg[(int)FReg::fa6] = 1;
      w_freg[(int)FReg::fa7] = 1;
    }
  case Inst::Segment:
    label = true;
    break;
  default:
    r_reg.set();
    w_reg.set();
    r_freg.set();
    w_freg.set();
    branch = 1;
  }
  if (branch) {
    w_reg[(int)Reg::t0] = 1;
    w_reg[(int)Reg::t1] = 1;
    w_reg[(int)Reg::t3] = 1;
    w_reg[(int)Reg::t4] = 1;
    w_reg[(int)Reg::t5] = 1;
    w_reg[(int)Reg::t6] = 1;
    w_freg[(int)FReg::ft0] = 1;
    w_freg[(int)FReg::ft1] = 1;
    w_freg[(int)FReg::ft2] = 1;
    w_freg[(int)FReg::ft3] = 1;
    w_freg[(int)FReg::ft4] = 1;
    w_freg[(int)FReg::ft5] = 1;
    w_freg[(int)FReg::ft6] = 1;
    w_freg[(int)FReg::ft7] = 1;
    w_freg[(int)FReg::ft8] = 1;
    w_freg[(int)FReg::ft9] = 1;
    w_freg[(int)FReg::ft10] = 1;
    w_freg[(int)FReg::ft11] = 1;
  }
}
} // namespace Status
void extendLoadStore() {
  CodeDec newCodes;
  for (int i = 0; i < codes.size(); i++) {
    auto &code = codes[i];
    auto &args = code.second;
    switch (code.first) {
    case Inst::Ld:
    case Inst::Lw:
    case Inst::Sd:
    case Inst::Sw:
    case Inst::Flw:
    case Inst::Fsw:
      if (args.size() == 3) {
        int val = get<int>(args[1]);
        auto arg = args[2];
        if (abs(val) >= (1 << 11)) {
          newCodes.emplace_back(Codec(Inst::Li, {Reg::t2, val}));
          newCodes.emplace_back(Codec(Inst::Add, {Reg::t2, Reg::t2, args[2]}));
          val = 0;
          arg = Reg::t2;
        }
        newCodes.emplace_back(move(code));
        newCodes.back().second[1] = val;
        newCodes.back().second[2] = arg;
        continue;
      }
    default:
      break;
    }
    newCodes.emplace_back(move(code));
  }
  std::swap(newCodes, codes);
}
void optimizeMove() {
  for (int i = 0; i < codes.size(); i++) {
    auto &code = codes[i];
    if (code.first == Inst::Mv) {
      if (code.second[0] == code.second[1]) {
        codes[i].first = Inst::Nop;
        continue;
      }
      bool opt = false;
      int reg0 = (int)get<Reg>(code.second[0]);
      int reg = (int)get<Reg>(code.second[1]);
      for (int k = i - 1; k >= 0; k--) {
        Status::getInstStatus(codes[k]);
        if (codes[k].second.size() && code.second[1] == codes[k].second[0]) {
          if (Status::w_reg[reg]) {
            for (int j = i + 1; j < codes.size(); j++) {
              Status::getInstStatus(codes[j]);
              if (Status::w_reg[reg]) {
                auto [wr, re] = Status::getWriteReadPos(codes[j]);
                for (int r : re) {
                  if (codes[j].second[r] == code.second[1]) {
                    codes[j].second[r] = code.second[0];
                  }
                }
                opt = true;
                break;
              }
              if (Status::r_reg[reg]) {
                opt = false;
                break;
              }
              if (Status::branch)
                break;
            }
            if (opt)
              codes[k].second[0] = code.second[0];
            break;
          }
        }
        if (Status::w_reg[reg] || Status::r_reg[reg] || Status::w_reg[reg0] ||
            Status::r_reg[reg0] || Status::label || Status::branch)
          break;
      }
      if (opt) {
        codes[i].first = Inst::Nop;
        continue;
      }
      bool is_writed = false;
      for (int j = i + 1; j < codes.size(); j++) {
        Status::getInstStatus(codes[j]);
        if (Status::branch)
          break;
        if (!is_writed && codes[j].first == Inst::Mv &&
            codes[j].second[0] == code.second[1] &&
            code.second[0] == codes[j].second[1]) {
          codes[j].first = Inst::Nop;
          continue;
        }
        if (Status::w_reg[reg0]) {
          for (int k = i + 1; k <= j; k++) {
            auto [wr, re] = Status::getWriteReadPos(codes[k]);
            for (int r : re) {
              if (codes[k].second[r] == code.second[0]) {
                codes[k].second[r] = code.second[1];
              }
            }
          }
          opt = true;
          break;
        }
        if ((is_writed && Status::r_reg[reg0]) || Status::label)
          break;
        if (Status::w_reg[reg])
          is_writed = true;
      }
      if (opt) {
        codes[i].first = Inst::Nop;
        continue;
      }
    } else if (code.first == Inst::Fmv) {
      if (code.second[0] == code.second[1]) {
        codes[i].first = Inst::Nop;
        continue;
      }
      bool opt = false;
      int reg0 = (int)get<FReg>(code.second[0]);
      int reg = (int)get<FReg>(code.second[1]);
      for (int k = i - 1; k >= 0; k--) {
        if (codes[k].second.size() && code.second[1] == codes[k].second[0]) {
          Status::getInstStatus(codes[k]);
          if (Status::w_freg[reg]) {
            for (int j = i + 1; j < codes.size(); j++) {
              Status::getInstStatus(codes[j]);
              if (Status::r_freg[reg]) {
                opt = false;
                break;
              }
              if (Status::w_freg[reg]) {
                opt = true;
                break;
              }
              if (Status::branch)
                break;
            }
            if (opt)
              codes[k].second[0] = code.second[0];
          }
          break;
        }
        Status::getInstStatus(codes[k]);
        if (Status::w_freg[reg] || Status::r_freg[reg] ||
            Status::w_freg[reg0] || Status::r_freg[reg0] || Status::label ||
            Status::branch)
          break;
      }
      if (opt) {
        codes[i].first = Inst::Nop;
        continue;
      }
      bool is_writed = false;
      for (int j = i + 1; j < codes.size(); j++) {
        Status::getInstStatus(codes[j]);
        if (Status::branch)
          break;
        if (!is_writed && codes[j].first == Inst::Fmv &&
            codes[j].second[0] == code.second[1] &&
            code.second[0] == codes[j].second[1]) {
          codes[j].first = Inst::Nop;
          continue;
        }
        if (Status::w_freg[reg0]) {
          for (int k = i + 1; k <= j; k++) {
            auto [wr, re] = Status::getWriteReadPos(codes[k]);
            for (int r : re) {
              if (codes[k].second[r] == code.second[0]) {
                codes[k].second[r] = code.second[1];
              }
            }
          }
          opt = true;
          break;
        }
        if ((is_writed && Status::r_freg[reg0]) || Status::label)
          break;
        if (Status::w_reg[reg])
          is_writed = true;
      }
      if (opt) {
        codes[i].first = Inst::Nop;
        continue;
      }
    }
  }
}
void optimizeLi() {
  for (int i = 0; i < codes.size(); i++) {
    auto &code = codes[i];
    if (code.first == Inst::Li) {
      if (auto r = get_if<int>(&code.second[1])) {
        if (abs(*r) < (1 << 11)) {
          int reg = static_cast<int>(get<Reg>(code.second.front()));
          for (int k = i + 1; k < codes.size(); k++) {
            Status::getInstStatus(codes[k]);
            if (codes[k].second.size() == 3 &&
                code.second.front() == codes[k].second[1]) {
              switch (codes[k].first) {
              case Inst::Add:
                codes[k].first = Inst::Addi;
                codes[k].second[1] = codes[k].second[2];
                codes[k].second.back() = code.second.back();
                break;
              case Inst::Mul:
                if (*r == 0) {
                  codes[k].first = Inst::Mv;
                  codes[k].second = {codes[k].second[0], Reg::x0};
                } else if (*r == 1) {
                  codes[k].first = Inst::Mv;
                  codes[k].second = {codes[k].second[0], codes[k].second[2]};
                } else if (*r > 0 && (1 << __lg(*r)) == *r) {
                  codes[k].first = Inst::Slli;
                  codes[k].second[1] = codes[k].second[2];
                  codes[k].second.back() = __lg(*r);
                }
                break;
              }
            }
            if (code.second.front() == codes[k].second.back()) {
              switch (codes[k].first) {
              case Inst::Add:
                codes[k].first = Inst::Addi;
                codes[k].second.back() = code.second.back();
                break;
              case Inst::Sub:
                codes[k].first = Inst::Addi;
                codes[k].second.back() = -get<int>(code.second.back());
                break;
              case Inst::Slt:
                codes[k].first = Inst::Slti;
                codes[k].second.back() = code.second.back();
                break;
              case Inst::And:
                codes[k].first = Inst::Andi;
                codes[k].second.back() = code.second.back();
                break;
              case Inst::Or:
                codes[k].first = Inst::Ori;
                codes[k].second.back() = code.second.back();
                break;
              case Inst::Mul:
                if (*r == 0) {
                  codes[k].first = Inst::Mv;
                  codes[k].second = {codes[k].second[0], Reg::x0};
                } else if (*r == 1) {
                  codes[k].first = Inst::Mv;
                  codes[k].second = {codes[k].second[0], codes[k].second[1]};
                } else if (*r > 0 && (1 << __lg(*r)) == *r) {
                  codes[k].first = Inst::Slli;
                  codes[k].second.back() = __lg(*r);
                }
                break;
              }
            }
            if (Status::w_reg[reg] || Status::branch || Status::label)
              break;
          }
        }
      }
    } else if (code.first == Inst::Add) {
      if (auto r = get_if<Reg>(&code.second[1])) {
        if (*r == Reg::x0) {
          code.first = Inst::Mv;
          code.second[1] = code.second[2];
          code.second.pop_back();
          continue;
        }
      }
      if (auto r = get_if<Reg>(&code.second[2])) {
        if (*r == Reg::x0) {
          code.first = Inst::Mv;
          code.second.pop_back();
          continue;
        }
      }
    } else if (code.first == Inst::Sub) {
      if (auto r = get_if<Reg>(&code.second[2])) {
        if (*r == Reg::x0) {
          code.first = Inst::Mv;
          code.second.pop_back();
          continue;
        }
      }
    } else if (code.first == Inst::Mul) {
      if (auto r = get_if<Reg>(&code.second[1])) {
        if (*r == Reg::x0) {
          code.first = Inst::Mv;
          code.second[1] = Reg::x0;
          code.second.pop_back();
          continue;
        }
      }
      if (auto r = get_if<Reg>(&code.second[2])) {
        if (*r == Reg::x0) {
          code.first = Inst::Mv;
          code.second[1] = Reg::x0;
          code.second.pop_back();
          continue;
        }
      }
    }
  }
}
void optimizeAddi() {
  for (int i = 0; i < codes.size(); i++) {
    auto &code = codes[i];
    if (code.first == Inst::Addi) {
      if (auto r = get_if<Reg>(&code.second[1])) {
        if (*r == Reg::x0) {
          code.first = Inst::Li;
          code.second[1] = code.second[2];
          code.second.resize(2);
        }
      }
    }
  }
}
void optimizeConst() {
  static map<Inst, function<int(int, int)>> simp_op = {
      {Inst::Add, std::plus<int>()},
      {Inst::Sub, std::minus<int>()},
      {Inst::Mul, std::multiplies<int>()},
      {Inst::Div, std::divides<int>()},
      {Inst::Slt, [](int a, int b) { return a < b; }},
      {Inst::Sgt, [](int a, int b) { return a > b; }},
      {Inst::And, std::logical_and<int>()},
      {Inst::Or, std::logical_or<int>()}};
  static map<Inst, function<int(int, int)>> simp_opi = {
      {Inst::Slli, [](int a, int b) { return a << b; }},
      {Inst::Addi, std::plus<int>()},
      {Inst::Slti, [](int a, int b) { return a < b; }},
      {Inst::Andi, std::logical_and<int>()},
      {Inst::Ori, std::logical_or<int>()},
  };
  for (int i = 0; i < codes.size(); i++) {
    auto &code = codes[i];
    if (simp_op.find(code.first) != simp_op.end()) {
      if (get_if<Reg>(&code.second[1]) && get_if<Reg>(&code.second[2])) {
        int reg1 = static_cast<int>(get<Reg>(code.second[1]));
        int reg2 = static_cast<int>(get<Reg>(code.second[2]));
        int val1, val2;
        bool ok1 = false, ok2 = false;
        for (int k = i - 1; k >= 0 && !(ok1 && ok2); k--) {
          Status::getInstStatus(codes[k]);
          if (codes[k].first == Inst::Li &&
              codes[k].second[0] == code.second[1]) {
            val1 = get<int>(codes[k].second[1]);
            ok1 = true;
          } else if (codes[k].first == Inst::Li &&
                     codes[k].second[0] == code.second[2]) {
            val2 = get<int>(codes[k].second[1]);
            ok2 = true;
          } else if (Status::w_reg[reg1] || Status::w_reg[reg2] ||
                     Status::label) {
            break;
          }
        }
        if (ok1 && ok2) {
          int val = simp_op[code.first](val1, val2);
          code.first = Inst::Li;
          code.second = {code.second[0], val};
        }
      }
    } else if (simp_opi.find(code.first) != simp_opi.end()) {
      if (get_if<Reg>(&code.second[1])) {
        int reg1 = static_cast<int>(get<Reg>(code.second[1]));
        int val1;
        bool ok1 = false;
        for (int k = i - 1; k >= 0 && !(ok1); k--) {
          Status::getInstStatus(codes[k]);
          if (codes[k].first == Inst::Li &&
              codes[k].second[0] == code.second[1]) {
            val1 = get<int>(codes[k].second[1]);
            ok1 = true;
          } else if (Status::w_reg[reg1] || Status::label) {
            break;
          }
        }
        if (ok1) {
          int val = simp_opi[code.first](val1, get<int>(code.second[2]));
          code.first = Inst::Li;
          code.second = {code.second[0], val};
        }
      }
    }
  }
}

void optimizeSnez() {
  for (int i = 0; i < codes.size(); i++) {
    auto &code = codes[i];
    bool opt = false;
    if (code.first == Inst::Snez && code.second[0] == code.second[1]) {
      int reg = static_cast<int>(get<Reg>(code.second[0]));
      for (int j = i + 1; j < codes.size(); j++) {
        Status::getInstStatus(codes[j]);
        if (Status::branch) {
          if ((codes[j].first == Inst::Bnez || codes[j].first == Inst::Beqz) &&
              (Status::w_reg[reg] || reg == (int)Reg::t2))
            opt = true;
          break;
        }
        if (Status::r_reg[reg])
          break;
        if (Status::w_reg[reg]) {
          opt = true;
          break;
        }
      }
    } else if (code.first == Inst::Seqz) {
      auto r = get_if<Reg>(&code.second[1]);
      if (code.second[0] == code.second[1] || (r && *r == Reg::t0)) {
        int reg = static_cast<int>(*r);
        for (int j = i + 1; j < codes.size(); j++) {
          Status::getInstStatus(codes[j]);
          if (Status::branch) {
            if ((codes[j].first == Inst::Bnez) &&
                codes[j].second[0] == code.second[1] &&
                (Status::w_reg[reg] || reg == (int)Reg::t2))
              opt = true, codes[j].first = Inst::Beqz,
              codes[j].second[0] = code.second[0];
            if ((codes[j].first == Inst::Beqz) &&
                codes[j].second[0] == code.second[1] &&
                (Status::w_reg[reg] || reg == (int)Reg::t2))
              opt = true, codes[j].first = Inst::Bnez,
              codes[j].second[0] = code.second[0];
            break;
          }
          if (Status::r_reg[reg] || Status::w_reg[reg])
            break;
        }
      }
    }
    if (opt) {
      codes[i].first = Inst::Nop;
      continue;
    }
  }
}
void optimizeUselessWrite() {
  for (int i = 0; i < codes.size(); i++) {
    auto &code = codes[i];
    Status::getInstStatus(code);
    if (Status::branch || Status::label)
      continue;
    auto [wr, re] = Status::getWriteReadPos(code);
    if (wr.size() == 1 && wr[0] == 0) {
      bool opt = false;
      if (auto r = get_if<Reg>(&code.second[0])) {
        int reg = static_cast<int>(*r);
        for (int j = i + 1; j < codes.size(); j++) {
          Status::getInstStatus(codes[j]);
          if (Status::r_reg[reg])
            break;
          if (Status::w_reg[reg]) {
            opt = true;
            break;
          }
          if (Status::branch && codes[j].first != Inst::Jal)
            break;
        }
      } else if (auto r = get_if<FReg>(&code.second[0])) {
        int reg = static_cast<int>(*r);
        for (int j = i + 1; j < codes.size(); j++) {
          Status::getInstStatus(codes[j]);
          if (Status::r_freg[reg])
            break;
          if (Status::w_freg[reg]) {
            opt = true;
            break;
          }
          if (Status::branch && codes[j].first != Inst::Jal)
            break;
        }
      }
      if (opt) {
        code.first = Inst::Nop;
      }
    }
  }
}
void optimizeHistory() {
  unordered_multimap<string, Reg> reg_history;
  unordered_multimap<string, FReg> freg_history;
  vector<string> reg_status(64);
  vector<string> freg_status(64);
  set<Reg> reg_trigger[128];
  set<FReg> freg_trigger[128];
  for (int i = 0; i < codes.size() - 1; i++) {
    auto &code = codes[i];
    Status::getInstStatus(code);
    if (Status::branch || Status::label) {
      reg_history.clear();
      freg_history.clear();
      reg_status = vector<string>(64);
      freg_status = vector<string>(64);
      for (int i = 0; i < 128; i++)
        reg_trigger[i].clear(), freg_trigger[i].clear();
      continue;
    }
    auto [wr, re] = Status::getWriteReadPos(code);
    bool ok = false;
    if (wr.size() == 1 && wr[0] == 0) {
      ok = code.first != Inst::Lw && code.first != Inst::Flw;
      for (auto &arg : code.second) {
        if (auto r = get_if<Symbol>(&arg)) {
          ok = false;
        }
      }
      if (ok) {
        if (auto r = get_if<Reg>(&code.second[0])) {
          auto s =
              to_string((int)code.first) + "_" + stringArgs(code.second, 1);
          if (reg_status[(int)*r] != s) {
            auto rng = reg_history.equal_range(reg_status[(int)*r]);
            for (auto it = rng.first; it != rng.second; ++it) {
              if ((int)it->second == (int)*r) {
                reg_history.erase(it);
                break;
              }
            }
            for (int i = 0; i < 128; i++)
              reg_trigger[i].erase(*r);
          }
          auto it = reg_history.find(s);
          if (it != reg_history.end()) {
            if (*r == it->second) {
              code.first = Inst::Nop;
            } else {
              code.first = Inst::Mv;
              code.second = {*r, it->second};
              reg_history.insert(make_pair(s, *r));
            }
          } else {
            reg_history.insert(make_pair(s, *r));
          }
          for (int i = 0; i < 64; i++) {
            if (Status::r_reg[i])
              reg_trigger[i].insert(*r);
            if (Status::r_freg[i])
              reg_trigger[i + 64].insert(*r);
          }
          reg_status[(int)*r] = s;
        } else if (auto r = get_if<FReg>(&code.second[0])) {
          auto s =
              to_string((int)code.first) + "_" + stringArgs(code.second, 1);
          if (freg_status[(int)*r] != s) {
            auto rng = freg_history.equal_range(freg_status[(int)*r]);
            for (auto it = rng.first; it != rng.second; ++it) {
              if ((int)it->second == (int)*r) {
                freg_history.erase(it);
                break;
              }
            }
            for (int i = 0; i < 128; i++)
              freg_trigger[i].erase(*r);
          }
          auto it = freg_history.find(s);
          if (it != freg_history.end()) {
            if (*r == it->second) {
              code.first = Inst::Nop;
            } else {
              code.first = Inst::Mv;
              code.second = {*r, it->second};
              freg_history.insert(make_pair(s, *r));
            }
          } else {
            freg_history.insert(make_pair(s, *r));
          }
          for (int i = 0; i < 64; i++) {
            if (Status::r_reg[i])
              freg_trigger[i].insert(*r);
            if (Status::r_freg[i])
              freg_trigger[i + 64].insert(*r);
          }
          freg_status[(int)*r] = s;
        }
      }
      vector<Reg> junk_reg;
      vector<FReg> junk_freg;
      for (int i = 0; i < 64; i++) {
        if (Status::w_reg[i]) {
          for (Reg r : reg_trigger[i])
            junk_reg.push_back(r);
          for (FReg r : freg_trigger[i])
            junk_freg.push_back(r);
          reg_trigger[i].clear();
          freg_trigger[i].clear();
        }
        if (Status::w_freg[i]) {
          for (Reg r : reg_trigger[i + 64])
            junk_reg.push_back(r);
          for (FReg r : freg_trigger[i + 64])
            junk_freg.push_back(r);
          reg_trigger[i + 64].clear();
          freg_trigger[i + 64].clear();
        }
      }
      if (!ok) {
        for (int i = 0; i < 64; i++) {
          if (Status::w_reg[i]) {
            junk_reg.push_back((Reg)i);
          }
          if (Status::w_freg[i]) {
            junk_freg.push_back((FReg)i);
          }
        }
      }
      sort(junk_reg.begin(), junk_reg.end());
      junk_reg.resize(unique(junk_reg.begin(), junk_reg.end()) -
                      junk_reg.begin());
      sort(junk_freg.begin(), junk_freg.end());
      junk_freg.resize(unique(junk_freg.begin(), junk_freg.end()) -
                       junk_freg.begin());
      for (Reg r : junk_reg) {
        int i = static_cast<int>(r);
        if (reg_status[i] != "") {
          auto rng = reg_history.equal_range(reg_status[i]);
          for (auto it = rng.first; it != rng.second; ++it) {
            if ((int)it->second == i) {
              reg_history.erase(it);
              break;
            }
          }
          reg_status[i] = "";
        }
      }
      for (FReg r : junk_freg) {
        int i = static_cast<int>(r);
        if (reg_status[i] != "") {
          auto rng = freg_history.equal_range(freg_status[i]);
          for (auto it = rng.first; it != rng.second; ++it) {
            if ((int)it->second == i) {
              freg_history.erase(it);
              break;
            }
          }
          freg_status[i] = "";
        }
      }
    }
  }
}
void optimizeJ() {
  for (int i = 0; i < codes.size() - 1; i++) {
    if (codes[i].first == Inst::J && codes[i + 1].first == Inst::Label &&
        codes[i].second[0] == codes[i + 1].second[0]) {
      codes[i].first = Inst::Nop;
      continue;
    }
  }
}
void optimizeNop() {
  CodeDec newCodes;
  for (int i = 0; i < codes.size(); i++) {
    auto &code = codes[i];
    if (code.first == Inst::Nop)
      continue;
    newCodes.emplace_back(move(code));
  }
  std::swap(newCodes, codes);
}
void optimizeFrame() {
  unordered_map<string, int> frame_size;
  int len_fr = strlen("_frameSize_");
  for (auto &code : codes)
    if (code.first == Inst::Label &&
        get<Symbol>(code.second[0]).find("_frameSize_") == 0)
      frame_size[get<Symbol>(code.second[0]).substr(len_fr)] =
          get<int>(code.second[2]);
  for (int i = 0; i < codes.size(); i++) {
    auto &code = codes[i];
    if (code.first == Inst::Label &&
        get<Symbol>(code.second[0]).find("_start_") == 0) {
      int pos = i + 1;
      int psts = 0, pste = 0, plds = 0, plde = 0;
      for (; pos < codes.size(); pos++) {
        if (codes[pos].first == Inst::Label &&
            get<Symbol>(codes[pos].second[0]).find("_storeStart_") == 0) {
          psts = pos;
          break;
        }
      }
      for (; pos < codes.size(); pos++)
        if (codes[pos].first == Inst::Label &&
            get<Symbol>(codes[pos].second[0]).find("_storeEnd_") == 0) {
          pste = pos;
          break;
        }
      for (; pos < codes.size(); pos++)
        if (codes[pos].first == Inst::Label &&
            get<Symbol>(codes[pos].second[0]).find("_loadStart_") == 0) {
          plds = pos;
          break;
        }
      for (; pos < codes.size(); pos++)
        if (codes[pos].first == Inst::Label &&
            get<Symbol>(codes[pos].second[0]).find("_loadEnd_") == 0) {
          plde = pos;
          break;
        }
      set<Reg> used_reg;
      set<FReg> used_freg;
      for (int k = pste; k < plds; k++) {
        for (auto &arg : codes[k].second) {
          if (auto r = get_if<Reg>(&arg))
            used_reg.insert(*r);
          if (auto r = get_if<FReg>(&arg))
            used_freg.insert(*r);
        }
      }
      for (int k = psts; k < pste; k++) {
        if (codes[k].first == Inst::Sw) {
          if (used_reg.find(get<Reg>(codes[k].second[0])) == used_reg.end()) {
            codes[k].first = Inst::Nop;
          }
        } else if (codes[k].first == Inst::Fsw) {
          if (used_freg.find(get<FReg>(codes[k].second[0])) ==
              used_freg.end()) {
            codes[k].first = Inst::Nop;
          }
        }
      }
      for (int k = plds; k < plde; k++) {
        if (codes[k].first == Inst::Lw) {
          if (used_reg.find(get<Reg>(codes[k].second[0])) == used_reg.end()) {
            codes[k].first = Inst::Nop;
          }
        } else if (codes[k].first == Inst::Flw) {
          if (used_freg.find(get<FReg>(codes[k].second[0])) ==
              used_freg.end()) {
            codes[k].first = Inst::Nop;
          }
        }
      }
      i = pos;
    }
  }
}
void optimizeLabel() {
  unordered_set<string> used_label;
  for (auto &code : codes)
    for (auto &arg : code.second)
      if (code.first != Inst::Label && code.first != Inst::Nop)
        if (auto r = get_if<Symbol>(&arg))
          used_label.insert(*r);
  for (auto &code : codes)
    if (code.first == Inst::Label) {
      auto &s = get<Symbol>(code.second[0]);
      if (s.find("_start_") == 0)
        continue;
      if (used_label.find(s) == used_label.end())
        code.first = Inst::Nop;
    }
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
  freopen("output0.S", "w", stdout);
  printCode();
  extendLoadStore();
  while (true) {
    int sz = codes.size();
    optimizeMove();
    optimizeLi();
    optimizeAddi();
    optimizeConst();
    optimizeSnez();
    optimizeUselessWrite();
    optimizeHistory();
    optimizeJ();
    optimizeFrame();
    optimizeNop();
    if (codes.size() >= sz)
      break;
  }
  optimizeLabel();
  optimizeNop();
  freopen("output.S", "w", stdout);
  printCode();
}
