#pragma once

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <cmath>

namespace simple_json {

class Value {
public:
    enum Type { NULL_TYPE, BOOL, NUMBER, STRING, ARRAY, OBJECT };

    Value() : type_(NULL_TYPE) {}
    Value(bool b) : type_(BOOL), bool_val_(b) {}
    Value(double d) : type_(NUMBER), number_val_(d) {}
    Value(const std::string& s) : type_(STRING), string_val_(s) {}
    Value(const char* s) : type_(STRING), string_val_(s) {}

    // 拷贝构造和赋值
    Value(const Value&) = default;
    Value& operator=(const Value&) = default;
    Value(Value&&) = default;
    Value& operator=(Value&&) = default;
    
    Type type() const { return type_; }
    bool is_null() const { return type_ == NULL_TYPE; }
    bool is_bool() const { return type_ == BOOL; }
    bool is_number() const { return type_ == NUMBER; }
    bool is_string() const { return type_ == STRING; }
    bool is_array() const { return type_ == ARRAY; }
    bool is_object() const { return type_ == OBJECT; }
    
    bool as_bool() const { return bool_val_; }
    double as_double() const { return number_val_; }
    const std::string& as_string() const { return string_val_; }
    
    const std::vector<Value>& as_array() const { return array_val_; }
    std::vector<Value>& as_array() { return array_val_; }
    
    const std::map<std::string, Value>& as_object() const { return object_val_; }
    std::map<std::string, Value>& as_object() { return object_val_; }
    
    const Value& operator[](size_t index) const {
        if (!is_array()) throw std::runtime_error("Not an array");
        return array_val_[index];
    }
    
    Value operator[](const std::string& key) const {
        if (!is_object()) throw std::runtime_error("Not an object");
        auto it = object_val_.find(key);
        if (it == object_val_.end()) {
            return Value();  // 返回新的空值对象
        }
        return it->second;
    }
    
    Value& operator[](const std::string& key) {
        if (type_ == NULL_TYPE) {
            type_ = OBJECT;
        }
        if (!is_object()) throw std::runtime_error("Not an object");
        return object_val_[key];
    }
    
    void push_back(const Value& v) {
        if (type_ == NULL_TYPE) {
            type_ = ARRAY;
        }
        if (!is_array()) throw std::runtime_error("Not an array");
        array_val_.push_back(v);
    }
    
    size_t size() const {
        if (is_array()) return array_val_.size();
        if (is_object()) return object_val_.size();
        return 0;
    }
    
    bool contains(const std::string& key) const {
        if (!is_object()) return false;
        return object_val_.find(key) != object_val_.end();
    }

private:
    Type type_;
    bool bool_val_ = false;
    double number_val_ = 0.0;
    std::string string_val_;
    std::vector<Value> array_val_;
    std::map<std::string, Value> object_val_;
};

class Parser {
public:
    static Value parse(const std::string& json_str) {
        Parser p(json_str);
        return p.parse_value();
    }
    
    static Value parse_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename);
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        return parse(buffer.str());
    }

private:
    Parser(const std::string& str) : str_(str), pos_(0) {}
    
    void skip_whitespace() {
        while (pos_ < str_.size() && std::isspace(str_[pos_])) {
            ++pos_;
        }
    }
    
    char peek() {
        skip_whitespace();
        return pos_ < str_.size() ? str_[pos_] : '\0';
    }
    
    char get() {
        skip_whitespace();
        return pos_ < str_.size() ? str_[pos_++] : '\0';
    }
    
    void expect(char c) {
        if (get() != c) {
            throw std::runtime_error(std::string("Expected '") + c + "'");
        }
    }
    
    Value parse_value() {
        char c = peek();
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return parse_string();
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') return parse_null();
        if (c == '-' || std::isdigit(c)) return parse_number();
        throw std::runtime_error("Unexpected character");
    }
    
    Value parse_object() {
        Value obj;
        obj.as_object(); // Initialize as object
        expect('{');
        
        if (peek() == '}') {
            get();
            return obj;
        }
        
        while (true) {
            std::string key = parse_string().as_string();
            expect(':');
            Value value = parse_value();
            obj[key] = value;
            
            char c = get();
            if (c == '}') break;
            if (c != ',') throw std::runtime_error("Expected ',' or '}'");
        }
        
        return obj;
    }
    
    Value parse_array() {
        Value arr;
        arr.as_array(); // Initialize as array
        expect('[');
        
        if (peek() == ']') {
            get();
            return arr;
        }
        
        while (true) {
            arr.push_back(parse_value());
            char c = get();
            if (c == ']') break;
            if (c != ',') throw std::runtime_error("Expected ',' or ']'");
        }
        
        return arr;
    }
    
    Value parse_string() {
        expect('"');
        std::string result;
        while (pos_ < str_.size() && str_[pos_] != '"') {
            if (str_[pos_] == '\\') {
                ++pos_;
                if (pos_ >= str_.size()) throw std::runtime_error("Unexpected end of string");
                char c = str_[pos_++];
                if (c == 'n') result += '\n';
                else if (c == 't') result += '\t';
                else if (c == 'r') result += '\r';
                else if (c == '\\') result += '\\';
                else if (c == '"') result += '"';
                else result += c;
            } else {
                result += str_[pos_++];
            }
        }
        expect('"');
        return Value(result);
    }
    
    Value parse_number() {
        size_t start = pos_;
        if (str_[pos_] == '-') ++pos_;
        while (pos_ < str_.size() && (std::isdigit(str_[pos_]) || str_[pos_] == '.' || 
                                       str_[pos_] == 'e' || str_[pos_] == 'E' || 
                                       str_[pos_] == '+' || str_[pos_] == '-')) {
            ++pos_;
        }
        std::string num_str = str_.substr(start, pos_ - start);
        return Value(std::stod(num_str));
    }
    
    Value parse_bool() {
        if (str_.substr(pos_, 4) == "true") {
            pos_ += 4;
            return Value(true);
        }
        if (str_.substr(pos_, 5) == "false") {
            pos_ += 5;
            return Value(false);
        }
        throw std::runtime_error("Invalid boolean");
    }
    
    Value parse_null() {
        if (str_.substr(pos_, 4) == "null") {
            pos_ += 4;
            return Value();
        }
        throw std::runtime_error("Invalid null");
    }
    
    std::string str_;
    size_t pos_;
};

} // namespace simple_json

