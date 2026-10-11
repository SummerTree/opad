// Parasolid transmit files: header, lexers (text and binary) and node stream. See xt_parse.hpp.
#include "xt_parse.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>

#include "opad/core.hpp"

namespace opad::xt {
namespace {

constexpr int kNullInt = -32764;       // Parasolid's null integer
constexpr double kNullReal = -3.14158e13;

// Node types visible at the PK interface carry the user field when a file has one (XT Format Reference, "Node Types").
bool visible(int type) {
  static const std::set<int> v = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 29, 30, 31, 32, 38, 50, 51, 52, 53, 54, 56,
                                  60, 67, 68, 70, 80, 81, 90, 100, 120, 124, 130, 133, 134, 137};
  return v.count(type) > 0;
}

// ---- text: records of ~80 characters whose line breaks and trailing spaces mean nothing; numbers end in one space,
// characters and logicals in none, a null is a lone '?'.
class Text {
 public:
  explicit Text(std::string_view body, bool escapes = true) : escapes_(escapes) {
    s_.reserve(body.size());
    size_t i = 0;
    while (i < body.size()) {
      size_t end = body.find_first_of("\r\n", i);
      if (end == std::string_view::npos) end = body.size();
      size_t stop = end;
      while (stop > i && body[stop - 1] == ' ') --stop;
      s_.append(body.substr(i, stop - i));
      i = end;
      while (i < body.size() && (body[i] == '\r' || body[i] == '\n')) ++i;
    }
  }
  size_t pos() const { return p_; }
  void seek(size_t p) { p_ = p, spaces_ = 0; }
  bool done() const { return p_ >= s_.size(); }
  char peek() const { return p_ < s_.size() ? s_[p_] : '\0'; }
  [[noreturn]] void fail(const std::string& what) const {
    throw Error("Parasolid text: " + what + " at character " + std::to_string(p_));
  }
  char raw() {
    if (p_ >= s_.size()) fail("unexpected end of file");
    return s_[p_++];
  }
  // A character of a text field, with the escapes of V12.1 and later: \0 \n \r \\ and \9 for nine spaces.
  char character() {
    if (spaces_ > 0) {
      --spaces_;
      return ' ';
    }
    const char c = raw();
    if (c != '\\' || !escapes_) return c;
    switch (raw()) {
      case '0': return '\0';
      case 'n': return '\r';
      case 'r': return '\n';
      case '\\': return '\\';
      case '9': spaces_ = 8; return ' ';
      default: --p_; return '\\';  // not an escape after all: a backslash, then that character
    }
  }
  std::string token() {
    const size_t start = p_;
    while (p_ < s_.size() && s_[p_] != ' ') ++p_;
    std::string t = s_.substr(start, p_ - start);
    if (p_ < s_.size()) ++p_;  // the space
    if (t.empty()) fail("empty number");
    return t;
  }
  // An integer; null ('?') as kNullInt.
  long long integer() {
    if (peek() == '?') {
      ++p_;
      return kNullInt;
    }
    const std::string t = token();
    char* end = nullptr;
    const long long v = std::strtoll(t.c_str(), &end, 10);
    if (*end) fail("not an integer: '" + t + "'");
    return v;
  }
  double real() {
    if (peek() == '?') {
      ++p_;
      return kNull;
    }
    std::string t = token();
    for (char& c : t)
      if (c == ',') c = '.';  // some writers print reals with their locale's decimal comma
    char* end = nullptr;
    const double v = std::strtod(t.c_str(), &end);
    if (*end) fail("not a number: '" + t + "'");
    return v;
  }
  bool logical() {
    const char c = raw();
    if (c != 'T' && c != 'F') fail(std::string("not a logical: '") + c + "'");
    return c == 'T';
  }
  std::string string(size_t n) {
    std::string out;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) out.push_back(character());
    return out;
  }

 private:
  std::string s_;
  size_t p_ = 0;
  int spaces_ = 0;
  bool escapes_ = true;
};

// ---- binary: big-endian (neutral), or the writer's byte order (typed); indices and counts as one or two shorts.
class Binary {
 public:
  Binary(std::string_view data, size_t start, bool little) : d_(data), p_(start), little_(little) {}
  size_t pos() const { return p_; }
  void seek(size_t p) { p_ = p; }
  bool done() const { return p_ >= d_.size(); }
  [[noreturn]] void fail(const std::string& what) const {
    throw Error("Parasolid binary: " + what + " at byte " + std::to_string(p_));
  }
  const unsigned char* take(size_t n) {
    if (p_ + n > d_.size()) fail("unexpected end of file");
    const auto* b = reinterpret_cast<const unsigned char*>(d_.data() + p_);
    p_ += n;
    return b;
  }
  uint64_t raw(size_t n) {
    const unsigned char* b = take(n);
    uint64_t v = 0;
    for (size_t i = 0; i < n; ++i) v = (v << 8) | b[little_ ? n - 1 - i : i];
    return v;
  }
  uint8_t u8() { return uint8_t(raw(1)); }
  int16_t i16() { return int16_t(uint16_t(raw(2))); }
  uint16_t u16() { return uint16_t(raw(2)); }
  int32_t i32() { return int32_t(uint32_t(raw(4))); }
  double f64() {
    const uint64_t bits = raw(8);
    double v;
    std::memcpy(&v, &bits, 8);
    return v;
  }
  uint32_t index() {
    int16_t r = i16();
    int32_t q = 0;
    if (r < 0) {
      q = i16();
      r = int16_t(-r);
    }
    return uint32_t(q) * 32767u + uint32_t(r) - 1u;
  }
  std::string string(size_t n) {
    const unsigned char* b = take(n);
    return std::string(reinterpret_cast<const char*>(b), n);
  }

 private:
  std::string_view d_;
  size_t p_;
  bool little_;
};

int schema_number(const std::string& key, int which) {
  // SCH_<modeller>_<schema>[_<base>]
  std::vector<std::string> parts;
  size_t start = 0;
  for (size_t i = 0; i <= key.size(); ++i)
    if (i == key.size() || key[i] == '_') {
      parts.push_back(key.substr(start, i - start));
      start = i + 1;
    }
  if (parts.size() < 3 || parts[0] != "SCH") return 0;
  const size_t i = size_t(which) + 1;
  return i < parts.size() ? std::atoi(parts[i].c_str()) : 0;
}

// The stream's reader, generic over the two lexers.
template <class In>
struct Reader {
  In& in;
  File& file;
  bool text;
  const Schema* fixed = nullptr;   // a file without embedded layouts: the layout for its version
  std::set<int> seen;              // node types whose first node was read (embedded layouts come with it)
  int max_type = 4096;             // the highest node type, as the header of a file with embedded layouts says
  std::map<int, bool> user_word;   // per node type: whether its nodes carry the user field

  bool plausible_type(int type) const {
    if (type == 1) return true;
    if (type < 10 || type > max_type) return false;
    return seen.count(type) || base_schema().count(type) || (fixed && fixed->count(type)) || file.base_version;
  }
  // Whether nodes of `type` carry the file's user field. The reference lists the node types visible at the PK interface;
  // files show more (value nodes too), so on a type's first node both readings are tried against the node that follows.
  bool carries_user_field(int type) {
    if (auto it = user_word.find(type); it != user_word.end()) return it->second;
    const size_t at = in.pos();
    auto next_ok = [&](int skip) {
      bool ok = false;
      try {
        for (int k = 0; k < skip; ++k) (void)integer();
        int t;
        if constexpr (std::is_same_v<In, Text>) t = int(in.integer());
        else t = in.i16();
        ok = plausible_type(t);
      } catch (const Error&) {
      }
      in.seek(at);
      return ok;
    };
    const bool with = next_ok(file.user_fields), without = next_ok(0);
    return user_word[type] = with != without ? with : visible(type);
  }

  long long integer() {
    if constexpr (std::is_same_v<In, Text>) return in.integer();
    else return in.i32();
  }

  // A field definition embedded in the stream: name, pointer class, element count, type (absent for pointers) and, for
  // variable-length fields, whether it is transmitted.
  FieldDef field_def() {
    FieldDef f;
    if constexpr (std::is_same_v<In, Text>) {
      f.name = in.string(size_t(in.integer()));
      const long long cls = in.integer();
      f.n = int(in.integer());
      f.type = 'p';
      if (cls == 0) {
        const std::string t = in.string(size_t(in.integer()));
        f.type = t.empty() ? 'd' : t[0];
      }
      f.xmt = f.n == 1 ? in.logical() : true;
    } else {
      f.name = in.string(in.u8());
      const int cls = in.u16();
      f.n = int(in.index());
      f.type = 'p';
      if (cls == 0) {
        const std::string t = in.string(in.u8());
        f.type = t.empty() ? 'd' : t[0];
      }
      f.xmt = f.n == 1 ? in.u8() != 0 : true;
    }
    return f;
  }

  int u8() {
    if constexpr (std::is_same_v<In, Text>) return int(in.integer());
    else return in.u8();
  }
  char opcode() {
    if constexpr (std::is_same_v<In, Text>) return in.raw();
    else return char(in.u8());
  }

  // The layout of `type`, read from the stream on its first node when the file embeds layouts.
  const TypeDef& layout(int type) {
    if (auto it = file.layouts.find(type); it != file.layouts.end() && seen.count(type)) return it->second;
    seen.insert(type);
    if (!file.base_version) {
      auto it = fixed->find(type);
      if (it == fixed->end())
        throw Error("Parasolid file uses node type " + std::to_string(type) + ", unknown in schema " + std::to_string(file.schema_version));
      return file.layouts[type] = it->second;
    }
    const Schema& base = base_schema();
    const auto b = base.find(type);
    const int marker = u8();
    TypeDef def;
    if (b == base.end()) {  // a node type the base does not have: written out in full
      def.name = [&] {
        if constexpr (std::is_same_v<In, Text>) return in.string(size_t(in.integer()));
        else return in.string(in.u8());
      }();
      [&] {
        if constexpr (std::is_same_v<In, Text>) (void)in.string(size_t(in.integer()));
        else (void)in.string(in.u8());
      }();  // description
      for (int i = 0; i < marker; ++i) def.fields.push_back(field_def());
    } else if (marker == 255) {
      def = b->second;
    } else {
      def.name = b->second.name;
      const auto& bf = b->second.fields;
      size_t bi = 0;
      for (;;) {
        const char op = opcode();
        if (op == 'C') {
          if (bi >= bf.size()) throw Error("Parasolid layout of node type " + std::to_string(type) + ": copy past its base fields");
          def.fields.push_back(bf[bi++]);
        } else if (op == 'D') {
          ++bi;
        } else if (op == 'I' || op == 'A') {
          def.fields.push_back(field_def());
        } else if (op == 'Z') {
          break;
        } else {
          throw Error(std::string("Parasolid layout of node type ") + std::to_string(type) + ": unknown edit '" + op + "'");
        }
      }
      if (int(def.fields.size()) != marker)
        file.warnings.push_back("layout of node type " + std::to_string(type) + " has " + std::to_string(def.fields.size()) +
                                " fields, the file announced " + std::to_string(marker));
    }
    return file.layouts[type] = std::move(def);
  }

  void value(Value& v, char type) {
    switch (type) {
      case 'c': {
        char c;
        if constexpr (std::is_same_v<In, Text>) c = in.character();
        else c = char(in.u8());
        v.text.push_back(c);
        v.num.push_back(double(static_cast<unsigned char>(c)));
        break;
      }
      case 'l': {
        bool b;
        if constexpr (std::is_same_v<In, Text>) b = in.logical();
        else b = in.u8() != 0;
        v.num.push_back(b ? 1 : 0);
        break;
      }
      case 'u': {
        if constexpr (std::is_same_v<In, Text>) v.num.push_back(double(in.integer()));
        else v.num.push_back(in.u8());
        break;
      }
      case 'n':
      case 'w': {
        long long x;
        if constexpr (std::is_same_v<In, Text>) x = in.integer();
        else x = in.i16();
        v.num.push_back(x == kNullInt ? kNull : double(x));
        break;
      }
      case 'd':
      case 't': {
        long long x;
        if constexpr (std::is_same_v<In, Text>) x = in.integer();
        else x = in.i32();
        v.num.push_back(x == kNullInt ? kNull : double(x));
        break;
      }
      case 'p': {
        uint32_t x;
        if constexpr (std::is_same_v<In, Text>) {
          const long long t = in.integer();
          x = t < 0 ? 0 : uint32_t(t);
        } else {
          x = in.index();
        }
        v.ptr.push_back(x);
        break;
      }
      case 'f': {
        double x;
        if constexpr (std::is_same_v<In, Text>) x = in.real();
        else x = in.f64();
        v.num.push_back(x == kNullReal ? kNull : x);
        break;
      }
      case 'i':
      case 'v':
      case 'h':
      case 'b': {
        const int k = type == 'i' ? 2 : type == 'b' ? 6 : 3;
        if constexpr (std::is_same_v<In, Text>) {
          if (in.peek() == '?') {  // a null composite is one '?'
            in.raw();
            for (int i = 0; i < k; ++i) v.num.push_back(kNull);
            break;
          }
          for (int i = 0; i < k; ++i) v.num.push_back(in.real());
        } else {
          for (int i = 0; i < k; ++i) {
            const double x = in.f64();
            v.num.push_back(x == kNullReal ? kNull : x);
          }
        }
        break;
      }
      default:
        throw Error(std::string("Parasolid field type '") + type + "' is not supported");
    }
  }

  void run() {
    int last_type = 0;
    uint32_t last_index = 0;
    try {
      step(last_type, last_index);
    } catch (const Error& e) {
      if (!last_type) throw;
      const auto at = file.layouts.find(last_type);
      throw Error(std::string(e.what()) + " [after " + (at != file.layouts.end() ? at->second.name : std::to_string(last_type)) +
                  " " + std::to_string(last_index) + "]");
    }
  }

  void step(int& last_type, uint32_t& last_index) {
    for (;;) {
      int type;
      if constexpr (std::is_same_v<In, Text>) type = int(in.integer());
      else type = in.i16();
      if (type == 1) break;  // terminator (then index 0)
      if (type <= 1 || type > 4096) throw Error("Parasolid stream: bad node type " + std::to_string(type));
      const TypeDef& def = layout(type);
      long long length = 0;
      if (def.variable()) length = integer();
      uint32_t index;
      if constexpr (std::is_same_v<In, Text>) index = uint32_t(in.integer());
      else index = in.index();
      Node node;
      node.type = type;
      node.index = index;
      node.def = &file.layouts[type];
      node.values.resize(def.fields.size());
      for (size_t i = 0; i < def.fields.size(); ++i) {
        const FieldDef& f = def.fields[i];
        if (!f.xmt) continue;
        const long long count = f.n == 0 ? 1 : f.n == 1 ? length : f.n;
        if (count < 0 || count > 100000000) throw Error("Parasolid stream: bad field length " + std::to_string(count));
        Value& v = node.values[i];
        try {
          for (long long k = 0; k < count; ++k) value(v, f.type);
        } catch (const Error& e) {
          throw Error(std::string(e.what()) + " (" + def.name + " " + std::to_string(index) + ", field " + f.name + ")");
        }
      }
      if (file.user_fields > 0 && carries_user_field(type))
        for (int k = 0; k < file.user_fields; ++k) (void)integer();
      file.nodes[index] = std::move(node);
      last_type = type, last_index = index;
    }
  }
};

}  // namespace

File parse_text(File file, std::string_view body, bool escapes);

File parse(const std::string& bytes) {
  File file;
  size_t start = 0;
  // The human-readable header ("**PART1;...**END_OF_HEADER***") written by applications; the stream follows its line.
  if (const size_t h = bytes.find("**END_OF_HEADER"); h != std::string::npos && h < 1 << 20) {
    file.header = bytes.substr(0, h);
    size_t nl = bytes.find_first_of("\r\n", h);  // some writers end lines with CR alone
    start = nl == std::string::npos ? bytes.size() : nl + 1;
  }
  while (start < bytes.size() && (bytes[start] == '\r' || bytes[start] == '\n')) ++start;
  std::string_view body(bytes.data() + start, bytes.size() - start);
  if (body.empty()) throw Error("not a Parasolid transmit file (empty)");

  if (body.size() >= 4 && body[0] == 'P' && body[1] == 'S' && body[2] == 0 && (body[3] == 0 || body[3] == 1)) {
    const bool typed = body[3] == 1;
    bool little = false;
    size_t p = start + 4;
    if (typed) {
      if (body.size() < 7) throw Error("Parasolid binary: truncated header");
      little = body[4] == 1;
      if (body[5] != 0 || body[6] != 0) throw Error("Parasolid binary: only IEEE / ASCII typed files are supported");
      p += 3;
    }
    Binary in(bytes, p, little);
    file.format = typed ? 'B' : 'N';
    file.modeller = in.string(in.u16());
    file.schema = in.string(size_t(in.i32()));
    file.schema_version = schema_number(file.schema, 1);
    file.base_version = schema_number(file.schema, 2);
    const int max_type = file.base_version ? in.u16() : 4096;  // the highest node type number
    file.user_fields = in.i32();
    Schema fixed;
    Reader<Binary> r{in, file, false};
    r.max_type = max_type;
    if (!file.base_version) fixed = schema_for(file.schema_version), r.fixed = &fixed;
    r.run();
    return file;
  }
  if (body[0] == 'B') throw Error("Parasolid bare binary files (machine dependent) are not supported; save as text or neutral binary");

  return parse_text(file, body, true);
}

File parse_text(File file, std::string_view body, bool escapes) {
  file.nodes.clear(), file.layouts.clear(), file.warnings.clear();  // a second reading starts over
  Text in(body, escapes);
  if (in.raw() != 'T') throw Error("not a Parasolid transmit file");
  file.format = 'T';
  file.modeller = in.string(size_t(in.integer()));
  file.schema = in.string(size_t(in.integer()));
  file.schema_version = schema_number(file.schema, 1);
  file.base_version = schema_number(file.schema, 2);
  const int max_type = file.base_version ? int(in.integer()) : 4096;
  file.user_fields = int(in.integer());
  if (file.schema_version <= 0) throw Error("Parasolid file names no schema");
  Schema fixed;
  Reader<Text> r{in, file, true};
  r.max_type = max_type;
  if (!file.base_version) fixed = schema_for(file.schema_version), r.fixed = &fixed;
  try {
    r.run();
  } catch (const Error&) {
    if (!escapes) throw;
    try {  // backslashes a writer did not escape (Windows paths in attribute strings) shift every character after them
      return parse_text(file, body, false);
    } catch (const Error&) {
    }
    throw;
  }
  return file;
}

}  // namespace opad::xt
