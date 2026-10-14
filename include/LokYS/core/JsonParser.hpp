#pragma once

#include "LokYS/core/Types.hpp"
#include <string>
#include <vector>
#include <unordered_map>
#include <string_view>
#include <sstream>
#include <cctype>
#include <charconv>
#include <algorithm>

namespace LokYS::Core {

class JsonValue {
public:
    enum class Type {
        Null,
        Boolean,
        Number,
        String,
        Array,
        Object
    };

    JsonValue() noexcept : m_type(Type::Null) {}
    JsonValue(bool val) noexcept : m_type(Type::Boolean), m_boolVal(val) {}
    JsonValue(double val) noexcept : m_type(Type::Number), m_numVal(val) {}
    JsonValue(float val) noexcept : m_type(Type::Number), m_numVal(static_cast<double>(val)) {}
    JsonValue(int val) noexcept : m_type(Type::Number), m_numVal(static_cast<double>(val)) {}
    JsonValue(u32 val) noexcept : m_type(Type::Number), m_numVal(static_cast<double>(val)) {}
    JsonValue(u64 val) noexcept : m_type(Type::Number), m_numVal(static_cast<double>(val)) {}
    JsonValue(std::string_view val) : m_type(Type::String), m_strVal(val) {}
    JsonValue(const char* val) : m_type(Type::String), m_strVal(val ? val : "") {}
    JsonValue(std::string val) : m_type(Type::String), m_strVal(std::move(val)) {}
    JsonValue(std::vector<JsonValue> arr) : m_type(Type::Array), m_arrVal(std::move(arr)) {}
    JsonValue(std::unordered_map<std::string, JsonValue> obj) : m_type(Type::Object), m_objVal(std::move(obj)) {}

    [[nodiscard]] Type GetType() const noexcept { return m_type; }
    [[nodiscard]] bool IsNull() const noexcept { return m_type == Type::Null; }
    [[nodiscard]] bool IsBool() const noexcept { return m_type == Type::Boolean; }
    [[nodiscard]] bool IsNumber() const noexcept { return m_type == Type::Number; }
    [[nodiscard]] bool IsString() const noexcept { return m_type == Type::String; }
    [[nodiscard]] bool IsArray() const noexcept { return m_type == Type::Array; }
    [[nodiscard]] bool IsObject() const noexcept { return m_type == Type::Object; }

    [[nodiscard]] bool AsBool(bool def = false) const noexcept {
        return (m_type == Type::Boolean) ? m_boolVal : def;
    }

    [[nodiscard]] double AsNumber(double def = 0.0) const noexcept {
        return (m_type == Type::Number) ? m_numVal : def;
    }

    [[nodiscard]] float AsFloat(float def = 0.0f) const noexcept {
        return static_cast<float>(AsNumber(static_cast<double>(def)));
    }

    [[nodiscard]] int AsInt(int def = 0) const noexcept {
        return static_cast<int>(AsNumber(static_cast<double>(def)));
    }

    [[nodiscard]] u32 AsU32(u32 def = 0) const noexcept {
        return static_cast<u32>(std::max(0.0, AsNumber(static_cast<double>(def))));
    }

    [[nodiscard]] size_t AsSize(size_t def = 0) const noexcept {
        return static_cast<size_t>(std::max(0.0, AsNumber(static_cast<double>(def))));
    }

    static const std::string& EmptyString() noexcept {
        static const std::string s_empty{};
        return s_empty;
    }

    static const std::vector<JsonValue>& EmptyArray() noexcept {
        static const std::vector<JsonValue> s_empty{};
        return s_empty;
    }

    static const std::unordered_map<std::string, JsonValue>& EmptyObject() noexcept {
        static const std::unordered_map<std::string, JsonValue> s_empty{};
        return s_empty;
    }

    static const JsonValue& Null() noexcept {
        static const JsonValue s_null{};
        return s_null;
    }

    [[nodiscard]] const std::string& AsString() const noexcept {
        return (m_type == Type::String) ? m_strVal : EmptyString();
    }

    [[nodiscard]] const std::string& AsString(const std::string& def) const noexcept {
        return (m_type == Type::String) ? m_strVal : def;
    }

    [[nodiscard]] const std::vector<JsonValue>& AsArray() const noexcept {
        return (m_type == Type::Array) ? m_arrVal : EmptyArray();
    }

    [[nodiscard]] std::vector<JsonValue>& AsArray() noexcept {
        if (m_type != Type::Array) {
            m_type = Type::Array;
            m_arrVal.clear();
        }
        return m_arrVal;
    }

    [[nodiscard]] const std::unordered_map<std::string, JsonValue>& AsObject() const noexcept {
        return (m_type == Type::Object) ? m_objVal : EmptyObject();
    }

    [[nodiscard]] std::unordered_map<std::string, JsonValue>& AsObject() noexcept {
        if (m_type != Type::Object) {
            m_type = Type::Object;
            m_objVal.clear();
        }
        return m_objVal;
    }

    [[nodiscard]] size_t Size() const noexcept {
        if (m_type == Type::Array) return m_arrVal.size();
        if (m_type == Type::Object) return m_objVal.size();
        return 0;
    }

    [[nodiscard]] bool HasKey(const std::string& key) const noexcept {
        if (m_type != Type::Object) return false;
        return m_objVal.find(key) != m_objVal.end();
    }

    [[nodiscard]] const JsonValue& operator[](const std::string& key) const noexcept {
        if (m_type == Type::Object) {
            auto it = m_objVal.find(key);
            if (it != m_objVal.end()) {
                return it->second;
            }
        }
        return Null();
    }

    [[nodiscard]] JsonValue& operator[](const std::string& key) {
        if (m_type != Type::Object) {
            m_type = Type::Object;
            m_objVal.clear();
        }
        return m_objVal[key];
    }

    [[nodiscard]] const JsonValue& operator[](size_t index) const noexcept {
        if (m_type == Type::Array && index < m_arrVal.size()) {
            return m_arrVal[index];
        }
        return Null();
    }

    [[nodiscard]] JsonValue& operator[](size_t index) {
        if (m_type != Type::Array) {
            m_type = Type::Array;
            m_arrVal.clear();
        }
        if (index >= m_arrVal.size()) {
            m_arrVal.resize(index + 1);
        }
        return m_arrVal[index];
    }

    void PushBack(JsonValue val) {
        if (m_type != Type::Array) {
            m_type = Type::Array;
            m_arrVal.clear();
        }
        m_arrVal.push_back(std::move(val));
    }

    // Static parser
    [[nodiscard]] static JsonValue Parse(std::string_view text, std::string* outError = nullptr) {
        size_t pos = 0;
        SkipWhitespace(text, pos);
        if (pos >= text.length()) {
            if (outError) *outError = "Empty JSON input";
            return Null();
        }

        std::string err;
        JsonValue val = ParseValue(text, pos, err);
        if (!err.empty()) {
            if (outError) *outError = err;
            return Null();
        }
        return val;
    }

    // Static serializer
    [[nodiscard]] std::string Serialize(bool pretty = false, int indentLevel = 0) const {
        std::ostringstream ss;
        SerializeInternal(ss, pretty, indentLevel);
        return ss.str();
    }

private:
    static void SkipWhitespace(std::string_view text, size_t& pos) noexcept {
        while (pos < text.length() && (text[pos] == ' ' || text[pos] == '\t' || text[pos] == '\r' || text[pos] == '\n')) {
            ++pos;
        }
    }

    static JsonValue ParseValue(std::string_view text, size_t& pos, std::string& outErr) {
        SkipWhitespace(text, pos);
        if (pos >= text.length()) {
            outErr = "Unexpected end of input";
            return Null();
        }

        char c = text[pos];
        if (c == '{') {
            return ParseObject(text, pos, outErr);
        } else if (c == '[') {
            return ParseArray(text, pos, outErr);
        } else if (c == '"') {
            return ParseString(text, pos, outErr);
        } else if (c == 't' || c == 'f') {
            return ParseBoolean(text, pos, outErr);
        } else if (c == 'n') {
            return ParseNull(text, pos, outErr);
        } else if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
            return ParseNumber(text, pos, outErr);
        }

        outErr = std::string("Unexpected character: ") + c;
        return Null();
    }

    static JsonValue ParseObject(std::string_view text, size_t& pos, std::string& outErr) {
        ++pos; // skip '{'
        std::unordered_map<std::string, JsonValue> obj;
        SkipWhitespace(text, pos);

        if (pos < text.length() && text[pos] == '}') {
            ++pos;
            return JsonValue(std::move(obj));
        }

        while (pos < text.length()) {
            SkipWhitespace(text, pos);
            if (pos >= text.length() || text[pos] != '"') {
                outErr = "Expected string key in object";
                return Null();
            }

            JsonValue keyVal = ParseString(text, pos, outErr);
            if (!outErr.empty()) return Null();

            SkipWhitespace(text, pos);
            if (pos >= text.length() || text[pos] != ':') {
                outErr = "Expected ':' after key";
                return Null();
            }
            ++pos; // skip ':'

            JsonValue val = ParseValue(text, pos, outErr);
            if (!outErr.empty()) return Null();

            obj[keyVal.AsString()] = std::move(val);

            SkipWhitespace(text, pos);
            if (pos >= text.length()) {
                outErr = "Unterminated object";
                return Null();
            }

            if (text[pos] == '}') {
                ++pos;
                return JsonValue(std::move(obj));
            } else if (text[pos] == ',') {
                ++pos;
            } else {
                outErr = std::string("Expected ',' or '}' in object, got: ") + text[pos];
                return Null();
            }
        }

        outErr = "Unterminated object";
        return Null();
    }

    static JsonValue ParseArray(std::string_view text, size_t& pos, std::string& outErr) {
        ++pos; // skip '['
        std::vector<JsonValue> arr;
        SkipWhitespace(text, pos);

        if (pos < text.length() && text[pos] == ']') {
            ++pos;
            return JsonValue(std::move(arr));
        }

        while (pos < text.length()) {
            JsonValue val = ParseValue(text, pos, outErr);
            if (!outErr.empty()) return Null();

            arr.push_back(std::move(val));

            SkipWhitespace(text, pos);
            if (pos >= text.length()) {
                outErr = "Unterminated array";
                return Null();
            }

            if (text[pos] == ']') {
                ++pos;
                return JsonValue(std::move(arr));
            } else if (text[pos] == ',') {
                ++pos;
            } else {
                outErr = std::string("Expected ',' or ']' in array, got: ") + text[pos];
                return Null();
            }
        }

        outErr = "Unterminated array";
        return Null();
    }

    static JsonValue ParseString(std::string_view text, size_t& pos, std::string& outErr) {
        ++pos; // skip opening '"'
        std::string str;
        while (pos < text.length()) {
            char c = text[pos++];
            if (c == '"') {
                return JsonValue(std::move(str));
            } else if (c == '\\') {
                if (pos >= text.length()) {
                    outErr = "Unexpected end of escaped string";
                    return Null();
                }
                char esc = text[pos++];
                switch (esc) {
                    case '"': str.push_back('"'); break;
                    case '\\': str.push_back('\\'); break;
                    case '/': str.push_back('/'); break;
                    case 'b': str.push_back('\b'); break;
                    case 'f': str.push_back('\f'); break;
                    case 'n': str.push_back('\n'); break;
                    case 'r': str.push_back('\r'); break;
                    case 't': str.push_back('\t'); break;
                    default: str.push_back(esc); break;
                }
            } else {
                str.push_back(c);
            }
        }

        outErr = "Unterminated string";
        return Null();
    }

    static JsonValue ParseNumber(std::string_view text, size_t& pos, std::string& outErr) {
        size_t start = pos;
        if (text[pos] == '-') ++pos;
        while (pos < text.length() && std::isdigit(static_cast<unsigned char>(text[pos]))) {
            ++pos;
        }
        if (pos < text.length() && text[pos] == '.') {
            ++pos;
            while (pos < text.length() && std::isdigit(static_cast<unsigned char>(text[pos]))) {
                ++pos;
            }
        }
        if (pos < text.length() && (text[pos] == 'e' || text[pos] == 'E')) {
            ++pos;
            if (pos < text.length() && (text[pos] == '+' || text[pos] == '-')) {
                ++pos;
            }
            while (pos < text.length() && std::isdigit(static_cast<unsigned char>(text[pos]))) {
                ++pos;
            }
        }

        std::string numStr(text.substr(start, pos - start));
        try {
            double val = std::stod(numStr);
            return JsonValue(val);
        } catch (...) {
            outErr = "Failed to parse number: " + numStr;
            return Null();
        }
    }

    static JsonValue ParseBoolean(std::string_view text, size_t& pos, std::string& outErr) {
        if (text.substr(pos, 4) == "true") {
            pos += 4;
            return JsonValue(true);
        } else if (text.substr(pos, 5) == "false") {
            pos += 5;
            return JsonValue(false);
        }
        outErr = "Invalid boolean literal";
        return Null();
    }

    static JsonValue ParseNull(std::string_view text, size_t& pos, std::string& outErr) {
        if (text.substr(pos, 4) == "null") {
            pos += 4;
            return JsonValue();
        }
        outErr = "Invalid null literal";
        return Null();
    }

    void SerializeInternal(std::ostringstream& ss, bool pretty, int indent) const {
        std::string indentStr(pretty ? indent * 2 : 0, ' ');
        std::string nextIndentStr(pretty ? (indent + 1) * 2 : 0, ' ');
        std::string nl = pretty ? "\n" : "";
        std::string sp = pretty ? " " : "";

        switch (m_type) {
            case Type::Null:
                ss << "null";
                break;
            case Type::Boolean:
                ss << (m_boolVal ? "true" : "false");
                break;
            case Type::Number: {
                if (m_numVal == static_cast<double>(static_cast<i64>(m_numVal))) {
                    ss << static_cast<i64>(m_numVal);
                } else {
                    ss << m_numVal;
                }
                break;
            }
            case Type::String: {
                ss << '"';
                for (char c : m_strVal) {
                    if (c == '"') ss << "\\\"";
                    else if (c == '\\') ss << "\\\\";
                    else if (c == '\n') ss << "\\n";
                    else if (c == '\r') ss << "\\r";
                    else if (c == '\t') ss << "\\t";
                    else ss << c;
                }
                ss << '"';
                break;
            }
            case Type::Array: {
                if (m_arrVal.empty()) {
                    ss << "[]";
                    break;
                }
                ss << "[" << nl;
                for (size_t i = 0; i < m_arrVal.size(); ++i) {
                    ss << nextIndentStr;
                    m_arrVal[i].SerializeInternal(ss, pretty, indent + 1);
                    if (i + 1 < m_arrVal.size()) ss << ",";
                    ss << nl;
                }
                ss << indentStr << "]";
                break;
            }
            case Type::Object: {
                if (m_objVal.empty()) {
                    ss << "{}";
                    break;
                }
                ss << "{" << nl;
                size_t count = 0;
                for (const auto& [k, v] : m_objVal) {
                    ss << nextIndentStr << '"' << k << "\":" << sp;
                    v.SerializeInternal(ss, pretty, indent + 1);
                    if (++count < m_objVal.size()) ss << ",";
                    ss << nl;
                }
                ss << indentStr << "}";
                break;
            }
        }
    }

    Type m_type{ Type::Null };
    bool m_boolVal{ false };
    double m_numVal{ 0.0 };
    std::string m_strVal{};
    std::vector<JsonValue> m_arrVal{};
    std::unordered_map<std::string, JsonValue> m_objVal{};
};

} // namespace LokYS::Core
