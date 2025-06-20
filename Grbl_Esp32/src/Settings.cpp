#include "Grbl.h"
#include "WebUI/JSONEncoder.h"
#include <map>
#include <nvs.h>

bool anyState() {
    return false;
}
bool idleOrJog() {
    return sys.state != State::Idle && sys.state != State::Jog;
}
bool idleOrAlarm() {
    return sys.state != State::Idle && sys.state != State::Alarm;
}
bool notCycleOrHold() {
    return sys.state == State::Cycle && sys.state == State::Hold;
}
bool isAxisAsda(int axis) {
    if (axis == POSITIONABLE_SPINDLE_AXIS && static_cast<SpindleType>(spindle_type->get()) == SpindleType::ASDA_CN1) {
        return true;
    }
    return false;
}
bool isAxisRpm(int axis) {
#ifdef POSITIONABLE_AXIS_CONVERT
    if (axis == POSITIONABLE_SPINDLE_AXIS) {
        return true;
    }
#endif
    return false;
}
bool isAxisMovable(int axis) {
    // Disabled for automatic ATC connection detection. Kept for potential future use.

    if ((axis == REMOVABLE_AXIS_LIMIT && static_cast<SpindleType>(spindle_type->get()) != SpindleType::ASDA_CN1) || (axis == REMOVABLE_AXIS_LIMIT && !atc_connected->get())) {
        return false;
    }
    return true;
}

Error setATCConnection(bool isConnected) {
    protocol_buffer_synchronize();
    limits_disable();
    Error temp = atc_connected->setBoolValue(isConnected);
    limits_init();
    protocol_buffer_synchronize();
    return temp;
}

Word::Word(type_t type, permissions_t permissions, const char* description, const char* grblName, const char* fullName) : _description(description), _grblName(grblName), _fullName(fullName), _type(type), _permissions(permissions) {}

Command* Command::List = NULL;

Command::Command(const char* description, type_t type, permissions_t permissions, const char* grblName, const char* fullName, bool (*cmdChecker)()) : Word(type, permissions, description, grblName, fullName), _cmdChecker(cmdChecker) {
    link = List;
    List = this;
}

Setting* Setting::List = NULL;

Setting::Setting(const char* description, type_t type, permissions_t permissions, const char* grblName, const char* fullName, bool (*checker)(char*)) : Word(type, permissions, description, grblName, fullName), _checker(checker) {
    link = List;
    List = this;

    // NVS keys are limited to 15 characters, so if the setting name is longer
    // than that, we derive a 15-character name from a hash function
    size_t len = strlen(fullName);
    if (len <= 15) {
        _keyName = _fullName;
    } else {
        // This is Donald Knuth's hash function from Vol 3, chapter 6.4
        char*    hashName = (char*)malloc(16);
        uint32_t hash     = len;
        for (const char* s = fullName; *s; s++) {
            hash = ((hash << 5) ^ (hash >> 27)) ^ (*s);
        }
        sprintf(hashName, "%.7s%08x", fullName, hash);
        _keyName = hashName;
    }
}

Error Setting::check(char* s) {
    if (sys.state != State::Idle && sys.state != State::Alarm) {
        return Error::IdleError;
    }
    if (!_checker) {
        return Error::Ok;
    }
    bool test = _checker(s);
    if (_checkError != Error::Ok) {
        return _checkError;
    }
    return test ? Error::Ok : Error::InvalidValue;
}

nvs_handle Setting::_handle = 0;

void Setting::init() {
    if (!_handle) {
        if (esp_err_t err = nvs_open("Grbl_ESP32", NVS_READWRITE, &_handle)) {
            grbl_sendf(CLIENT_SERIAL, "nvs_open failed with error %d\r\n", err);
        }
    }
}

IntSetting::IntSetting(const char* description, type_t type, permissions_t permissions, const char* grblName, const char* name, int32_t defVal, int32_t minVal, int32_t maxVal, bool (*checker)(char*) = NULL, bool currentIsNvm) :
    Setting(description, type, permissions, grblName, name, checker), _defaultValue(defVal), _currentValue(defVal), _minValue(minVal), _maxValue(maxVal), _currentIsNvm(currentIsNvm) {
    _storedValue = std::numeric_limits<int32_t>::min();
}

void IntSetting::load() {
    esp_err_t err = nvs_get_i32(_handle, _keyName, &_storedValue);
    if (err) {
        _storedValue  = std::numeric_limits<int32_t>::min();
        _currentValue = _defaultValue;
    } else {
        _currentValue = _storedValue;
    }
}

void IntSetting::setDefault() {
    if (_currentIsNvm) {
        nvs_erase_key(_handle, _keyName);
    } else {
        _currentValue = _defaultValue;
        if (_storedValue != _currentValue) {
            nvs_erase_key(_handle, _keyName);
        }
    }
}

Error IntSetting::setStringValue(char* s) {
    s         = trim(s);
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }
    char*   endptr;
    int32_t convertedValue = strtol(s, &endptr, 10);
    if (endptr == s || *endptr != '\0') {
        return Error::BadNumberFormat;
    }
    if (convertedValue < _minValue || convertedValue > _maxValue) {
        return Error::NumberRange;
    }

    // If we don't see the NVM state, we have to make this the live value:
    if (!_currentIsNvm) {
        _currentValue = convertedValue;
    }

    if (_storedValue != convertedValue) {
        if (convertedValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            if (nvs_set_i32(_handle, _keyName, convertedValue)) {
                return Error::NvsSetFailed;
            }
            _storedValue = convertedValue;
        }
    }
    check(NULL);
    return Error::Ok;
}

Error IntSetting::setValue(int32_t value) {
    char s[16];
    snprintf(s, sizeof(s), "%d", value);
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }

    int32_t convertedValue = value;

    if (convertedValue < _minValue || convertedValue > _maxValue) {
        return Error::NumberRange;
    }

    // If we don't see the NVM state, we have to make this the live value:
    if (!_currentIsNvm) {
        _currentValue = convertedValue;
    }

    if (_storedValue != convertedValue) {
        if (convertedValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            if (nvs_set_i32(_handle, _keyName, convertedValue)) {
                return Error::NvsSetFailed;
            }
            _storedValue = convertedValue;
        }
    }
    check(NULL);
    return Error::Ok;
}

const char* IntSetting::getDefaultString() {
    static char strval[32];
    sprintf(strval, "%d", _defaultValue);
    return strval;
}

const char* IntSetting::getStringValue() {
    static char strval[32];

    int currentSettingValue;
    if (_currentIsNvm) {
        if (std::numeric_limits<int32_t>::min() == _storedValue) {
            currentSettingValue = _defaultValue;
        } else {
            currentSettingValue = _storedValue;
        }
    } else {
        currentSettingValue = get();
    }

    sprintf(strval, "%d", currentSettingValue);
    return strval;
}

void IntSetting::addWebui(WebUI::JSONencoder* j) {
    if (getDescription()) {
        j->begin_webui(getName(), getDescription(), "I", getStringValue(), _minValue, _maxValue);
        j->end_object();
    }
}

AxisMaskSetting::AxisMaskSetting(const char* description, type_t type, permissions_t permissions, const char* grblName, const char* name, int32_t defVal, bool (*checker)(char*) = NULL) :
    Setting(description, type, permissions, grblName, name, checker), _defaultValue(defVal), _currentValue(defVal) {}

void AxisMaskSetting::load() {
    esp_err_t err = nvs_get_i32(_handle, _keyName, &_storedValue);
    if (err) {
        _storedValue  = -1;
        _currentValue = _defaultValue;
    } else {
        _currentValue = _storedValue;
    }
}

void AxisMaskSetting::setDefault() {
    _currentValue = _defaultValue;
    if (_storedValue != _currentValue) {
        nvs_erase_key(_handle, _keyName);
    }
}

Error AxisMaskSetting::saveValue() {
    if (_storedValue != _currentValue) {
        if (_currentValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            if (nvs_set_i32(_handle, _keyName, _currentValue)) {
                return Error::NvsSetFailed;
            }
            _storedValue = _currentValue;
        }
    }
    return Error::Ok;
}

Error AxisMaskSetting::setAxis(uint8_t axis, bool val) {
    if (axis < 0 && axis >= number_axis->get()) {
        return Error::NvsSetFailed;
    }
    if (val) {
        bitnum_true(_currentValue, axis);
    } else {
        bit_false(_currentValue, bit(axis));
    }
    return Error::Ok;
}

Error AxisMaskSetting::setStringValue(char* s) {
    s         = trim(s);
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }
    int32_t convertedValue;
    char*   endptr;
    if (*s == '\0') {
        convertedValue = 0;
    } else {
        convertedValue = strtol(s, &endptr, 10);
        if (endptr == s || *endptr != '\0') {
            // Try to convert as an axis list
            convertedValue = 0;
            auto axisNames = String("XYZABC");
            while (*s) {
                int index = axisNames.indexOf(toupper(*s++));
                if (index < 0) {
                    return Error::BadNumberFormat;
                }
                convertedValue |= bit(index);
            }
        }
    }
    _currentValue = convertedValue;
    if (_storedValue != _currentValue) {
        if (_currentValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            if (nvs_set_i32(_handle, _keyName, _currentValue)) {
                return Error::NvsSetFailed;
            }
            _storedValue = _currentValue;
        }
    }
    check(NULL);
    return Error::Ok;
}

const char* AxisMaskSetting::getCompatibleValue() {
    static char strval[32];
    sprintf(strval, "%d", get());
    return strval;
}

static char* maskToString(uint32_t mask, char* strval) {
    char* s = strval;
    for (int i = 0; i < MAX_N_AXIS; i++) {
        if (mask & bit(i)) {
            *s++ = "XYZABC"[i];
        }
    }
    *s = '\0';
    return strval;
}

const char* AxisMaskSetting::getDefaultString() {
    static char strval[32];
    return maskToString(_defaultValue, strval);
}

const char* AxisMaskSetting::getStringValue() {
    static char strval[32];
    return maskToString(get(), strval);
}

void AxisMaskSetting::addWebui(WebUI::JSONencoder* j) {
    if (getDescription()) {
        j->begin_webui(getName(), getDescription(), "I", getStringValue(), 0, (1 << MAX_N_AXIS) - 1);
        j->end_object();
    }
}

FloatSetting::FloatSetting(const char* description, type_t type, permissions_t permissions, const char* grblName, const char* name, float defVal, float minVal, float maxVal, bool (*checker)(char*) = NULL) :
    Setting(description, type, permissions, grblName, name, checker), _defaultValue(defVal), _currentValue(defVal), _minValue(minVal), _maxValue(maxVal) {}

void FloatSetting::load() {
    union {
        int32_t ival;
        float   fval;
    } v;
    if (nvs_get_i32(_handle, _keyName, &v.ival)) {
        _currentValue = _defaultValue;
    } else {
        _currentValue = v.fval;
    }
}

void FloatSetting::setDefault() {
    _currentValue = _defaultValue;
    if (_storedValue != _currentValue) {
        nvs_erase_key(_handle, _keyName);
    }
}

Error FloatSetting::setStringValue(char* s) {
    s         = trim(s);
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }

    float   convertedValue;
    uint8_t len    = strlen(s);
    uint8_t retlen = 0;
    if (!read_float(s, &retlen, &convertedValue) || retlen != len) {
        return Error::BadNumberFormat;
    }
    if (convertedValue < _minValue || convertedValue > _maxValue) {
        return Error::NumberRange;
    }
    _currentValue = convertedValue;
    if (_storedValue != _currentValue) {
        if (_currentValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            union {
                int32_t ival;
                float   fval;
            } v;
            v.fval = _currentValue;
            if (nvs_set_i32(_handle, _keyName, v.ival)) {
                return Error::NvsSetFailed;
            }
            _storedValue = _currentValue;
        }
    }
    check(NULL);
    return Error::Ok;
}

Error FloatSetting::setValue(float value) {
    char s[32];
    snprintf(s, sizeof(s), "%.6f", value);
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }

    float convertedValue = value;

    if (convertedValue < _minValue || convertedValue > _maxValue) {
        return Error::NumberRange;
    }
    _currentValue = convertedValue;
    if (_storedValue != _currentValue) {
        if (_currentValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            union {
                int32_t ival;
                float   fval;
            } v;
            v.fval = _currentValue;
            if (nvs_set_i32(_handle, _keyName, v.ival)) {
                return Error::NvsSetFailed;
            }
            _storedValue = _currentValue;
        }
    }
    check(NULL);
    return Error::Ok;
}

const char* FloatSetting::getDefaultString() {
    static char strval[32];
    (void)sprintf(strval, "%.3f", _defaultValue);
    return strval;
}

const char* FloatSetting::getStringValue() {
    static char strval[32];
    (void)sprintf(strval, "%.3f", get());
#if 0
    // With the goal of representing both large and small floating point
    // numbers compactly while showing clearly that the are floating point,
    // remove trailing zeros leaving at least one post-decimal digit.
    // The loop is guaranteed to terminate because the string contains
    // a decimal point which is not a '0'.
    for (char *p = strval + strlen(strval) - 1; *p == '0'; --p) {
        if (*(p-1) != '.' && *(p-1) != ',') {
            *p = '\0';
        }
    }
#endif
    return strval;
}

StringSetting::StringSetting(const char* description, type_t type, permissions_t permissions, const char* grblName, const char* name, const char* defVal, int min, int max, bool (*checker)(char*)) :
    Setting(description, type, permissions, grblName, name, checker) {
    _defaultValue = defVal;
    _currentValue = defVal;
    _minLength    = min;
    _maxLength    = max;
};

void StringSetting::load() {
    size_t    len = 0;
    esp_err_t err = nvs_get_str(_handle, _keyName, NULL, &len);
    if (err) {
        _storedValue  = _defaultValue;
        _currentValue = _defaultValue;
        return;
    }
    char buf[len];
    err = nvs_get_str(_handle, _keyName, buf, &len);
    if (err) {
        _storedValue  = _defaultValue;
        _currentValue = _defaultValue;
        return;
    }
    _storedValue  = String(buf);
    _currentValue = _storedValue;
}

void StringSetting::setDefault() {
    _currentValue = _defaultValue;
    if (_storedValue != _currentValue) {
        nvs_erase_key(_handle, _keyName);
    }
}

Error StringSetting::setStringValue(char* s) {
    if (_minLength && _maxLength && (strlen(s) < _minLength || strlen(s) > _maxLength)) {
        return Error::BadNumberFormat;
    }
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }
    _currentValue = s;
    if (_storedValue != _currentValue) {
        if (_currentValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
            _storedValue = _defaultValue;
        } else {
            if (nvs_set_str(_handle, _keyName, _currentValue.c_str())) {
                return Error::NvsSetFailed;
            }
            _storedValue = _currentValue;
        }
    }
    check(NULL);
    return Error::Ok;
}

static bool isPassword(bool (*_checker)(char*)) {
#ifdef ENABLE_WIFI
    if (_checker == (bool (*)(char*))WebUI::WiFiConfig::isPasswordValid) {
        return true;
    }
#endif
    return _checker == (bool (*)(char*))WebUI::COMMANDS::isLocalPasswordValid;
}

const char* StringSetting::getDefaultString() {
    // If the string is a password do not display it
    return (_checker && isPassword(_checker)) ? "******" : _defaultValue.c_str();
}
const char* StringSetting::getStringValue() {
    return (_checker && isPassword(_checker)) ? "******" : get();
}

void StringSetting::addWebui(WebUI::JSONencoder* j) {
    if (!getDescription()) {
        return;
    }
    j->begin_webui(getName(), getDescription(), "S", getStringValue(), _minLength, _maxLength);
    j->end_object();
}

typedef std::map<const char*, int8_t, cmp_str> enum_opt_t;

EnumSetting::EnumSetting(const char* description, type_t type, permissions_t permissions, const char* grblName, const char* name, int8_t defVal, enum_opt_t* opts, bool (*checker)(char*) = NULL) :
    Setting(description, type, permissions, grblName, name, checker), _defaultValue(defVal), _options(opts) {}

void EnumSetting::load() {
    esp_err_t err = nvs_get_i8(_handle, _keyName, &_storedValue);
    if (err) {
        _storedValue  = -1;
        _currentValue = _defaultValue;
    } else {
        _currentValue = _storedValue;
    }
}

void EnumSetting::setDefault() {
    _currentValue = _defaultValue;
    if (_storedValue != _currentValue) {
        nvs_erase_key(_handle, _keyName);
    }
}

uint8_t EnumSetting::stringToEnum(char* s) {
    enum_opt_t::iterator it = _options->find(s);
    if (it == _options->end()) {
        // If we don't find the value in keys, look for it in the numeric values

        // Disallow empty string
        if (!s || !*s) {
            return UINT8_MAX;
        }
        char*   endptr;
        uint8_t num = strtol(s, &endptr, 10);
        // Disallow non-numeric characters in string
        if (*endptr) {
            return UINT8_MAX;
        }
        for (it = _options->begin(); it != _options->end(); it++) {
            if (it->second == num) {
                break;
            }
        }
        if (it == _options->end()) {
            return UINT8_MAX;
        }
    }
    return it->second;
}

// For enumerations, we allow the value to be set
// either with the string name or the numeric value.
// This is necessary for WebUI, which uses the number
// for setting.
Error EnumSetting::setStringValue(char* s) {
    s         = trim(s);
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }
    enum_opt_t::iterator it = _options->find(s);
    if (it == _options->end()) {
        // If we don't find the value in keys, look for it in the numeric values

        // Disallow empty string
        if (!s || !*s) {
            return Error::BadNumberFormat;
        }
        char*   endptr;
        uint8_t num = strtol(s, &endptr, 10);
        // Disallow non-numeric characters in string
        if (*endptr) {
            return Error::BadNumberFormat;
        }
        for (it = _options->begin(); it != _options->end(); it++) {
            if (it->second == num) {
                break;
            }
        }
        if (it == _options->end()) {
            return Error::BadNumberFormat;
        }
    }
    _currentValue = it->second;
    if (_storedValue != _currentValue) {
        if (_currentValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            if (nvs_set_i8(_handle, _keyName, _currentValue)) {
                return Error::NvsSetFailed;
            }
            _storedValue = _currentValue;
        }
    }
    check(NULL);
    return Error::Ok;
}

Error EnumSetting::setEnumValue(int8_t value) {
    const char* result = enumToString(value);
    char        s[256];  // Make sure the size is sufficient
    std::strncpy(s, result, sizeof(s) - 1);
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }
    // Look for the value in the _options map using the numeric value
    auto it = std::find_if(_options->begin(), _options->end(), [value](const std::pair<const char*, int8_t>& option) { return option.second == value; });

    if (it != _options->end()) {
        // If we find the value in the map
        _currentValue = it->second;
    } else {
        // If the value is not found in the enum map that means there is something wrong in the application we need to fix the code
        return Error::BadNumberFormat;
    }
    if (_storedValue != _currentValue) {
        if (_currentValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            if (nvs_set_i8(_handle, _keyName, _currentValue)) {
                return Error::NvsSetFailed;
            }
            _storedValue = _currentValue;
        }
    }
    check(NULL);
    return Error::Ok;
}

const char* EnumSetting::enumToString(int8_t value) {
    for (enum_opt_t::iterator it = _options->begin(); it != _options->end(); it++) {
        if (it->second == value) {
            return it->first;
        }
    }
    return "???";
}
const char* EnumSetting::getDefaultString() {
    return enumToString(_defaultValue);
}
const char* EnumSetting::getStringValue() {
    return enumToString(get());
}

void EnumSetting::addWebui(WebUI::JSONencoder* j) {
    if (!getDescription()) {
        return;
    }
    j->begin_webui(getName(), getDescription(), "B", String(get()).c_str());
    j->begin_array("O");
    for (enum_opt_t::iterator it = _options->begin(); it != _options->end(); it++) {
        j->begin_object();
        j->member(it->first, it->second);
        j->end_object();
    }
    j->end_array();
    j->end_object();
}

FlagSetting::FlagSetting(const char* description, type_t type, permissions_t permissions, const char* grblName, const char* name, bool defVal, bool (*checker)(char*) = NULL) :
    Setting(description, type, permissions, grblName, name, checker), _defaultValue(defVal) {}

void FlagSetting::load() {
    esp_err_t err = nvs_get_i8(_handle, _keyName, &_storedValue);
    if (err) {
        _storedValue  = -1;  // Neither well-formed false (0) nor true (1)
        _currentValue = _defaultValue;
    } else {
        _currentValue = !!_storedValue;
    }
}
void FlagSetting::setDefault() {
    _currentValue = _defaultValue;
    if (_storedValue != _currentValue) {
        nvs_erase_key(_handle, _keyName);
    }
}

Error FlagSetting::setBoolValue(bool value) {
    char s[4];
    strcpy(s, value ? "on" : "off");
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }

    _currentValue = value;

    // _storedValue is -1, 0, or 1
    // _currentValue is 0 or 1
    if (_storedValue != (int8_t)_currentValue) {
        if (_currentValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            if (nvs_set_i8(_handle, _keyName, _currentValue)) {
                return Error::NvsSetFailed;
            }
            _storedValue = _currentValue;
        }
    }
    check(NULL);
    return Error::Ok;
}

Error FlagSetting::setStringValue(char* s) {
    s         = trim(s);
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }
    _currentValue = (strcasecmp(s, "on") == 0) || (strcasecmp(s, "true") == 0) || (strcasecmp(s, "enabled") == 0) || (strcasecmp(s, "yes") == 0) || (strcasecmp(s, "1") == 0);
    // _storedValue is -1, 0, or 1
    // _currentValue is 0 or 1
    if (_storedValue != (int8_t)_currentValue) {
        if (_currentValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            if (nvs_set_i8(_handle, _keyName, _currentValue)) {
                return Error::NvsSetFailed;
            }
            _storedValue = _currentValue;
        }
    }
    check(NULL);
    return Error::Ok;
}
const char* FlagSetting::getDefaultString() {
    return _defaultValue ? "On" : "Off";
}
const char* FlagSetting::getStringValue() {
    return get() ? "On" : "Off";
}
const char* FlagSetting::getCompatibleValue() {
    return get() ? "1" : "0";
}

#include <WiFi.h>

IPaddrSetting::IPaddrSetting(const char* description, type_t type, permissions_t permissions, const char* grblName, const char* name, uint32_t defVal, bool (*checker)(char*) = NULL) :
    Setting(description, type, permissions, grblName, name, checker)  // There are no GRBL IP settings.
    ,
    _defaultValue(defVal), _currentValue(defVal) {}

IPaddrSetting::IPaddrSetting(const char* description, type_t type, permissions_t permissions, const char* grblName, const char* name, const char* defVal, bool (*checker)(char*) = NULL) :
    Setting(description, type, permissions, grblName, name, checker) {
    IPAddress ipaddr;
    if (ipaddr.fromString(defVal)) {
        _defaultValue = ipaddr;
        _currentValue = _defaultValue;
    } else {
        throw std::runtime_error("Bad IPaddr default");
    }
}

void IPaddrSetting::load() {
    esp_err_t err = nvs_get_i32(_handle, _keyName, (int32_t*)&_storedValue);
    if (err) {
        _storedValue  = 0x000000ff;  // Unreasonable value for any IP thing
        _currentValue = _defaultValue;
    } else {
        _currentValue = _storedValue;
    }
}

void IPaddrSetting::setDefault() {
    _currentValue = _defaultValue;
    if (_storedValue != _currentValue) {
        nvs_erase_key(_handle, _keyName);
    }
}

Error IPaddrSetting::setStringValue(char* s) {
    s         = trim(s);
    Error err = check(s);
    if (err != Error::Ok) {
        return err;
    }
    IPAddress ipaddr;
    if (!ipaddr.fromString(s)) {
        return Error::InvalidValue;
    }
    _currentValue = ipaddr;
    if (_storedValue != _currentValue) {
        if (_currentValue == _defaultValue) {
            nvs_erase_key(_handle, _keyName);
        } else {
            if (nvs_set_i32(_handle, _keyName, (int32_t)_currentValue)) {
                return Error::NvsSetFailed;
            }
            _storedValue = _currentValue;
        }
    }
    check(NULL);
    return Error::Ok;
}

const char* IPaddrSetting::getDefaultString() {
    static String s;
    s = IPAddress(_defaultValue).toString();
    return s.c_str();
}
const char* IPaddrSetting::getStringValue() {
    static String s;
    s = IPAddress(get()).toString();
    return s.c_str();
}

void IPaddrSetting::addWebui(WebUI::JSONencoder* j) {
    if (getDescription()) {
        j->begin_webui(getName(), getDescription(), "A", getStringValue());
        j->end_object();
    }
}

AxisSettings::AxisSettings(const char* axisName) : name(axisName) {}

Error GrblCommand::action(char* value, WebUI::AuthenticationLevel auth_level, WebUI::ESPResponseStream* out) {
    if (_cmdChecker && _cmdChecker()) {
        return Error::IdleError;
    }
    return _action((const char*)value, auth_level, out);
};

Coordinates* coords[CoordIndex::End];

bool Coordinates::load() {
    size_t    len;
    esp_err_t test = nvs_get_blob(Setting::_handle, _name, _currentValue, &len);
    switch (test) {
        case ESP_OK:
            return true;
        case ESP_ERR_NVS_INVALID_LENGTH:
            // This could happen if the stored value is longer than the buffer.
            // That is highly unlikely since we always store MAX_N_AXIS coordinates.
            // It would indicate that we have decreased MAX_N_AXIS since the
            // value was stored.  We don't flag it as an error, but rather
            // accept the initial coordinates and ignore the residue.
            // We could issue a warning message if we were so inclined.
            return true;
        case ESP_ERR_NVS_INVALID_NAME:
        case ESP_ERR_NVS_INVALID_HANDLE:
        default:
            return false;
    }
};

void Coordinates::set(float value[MAX_N_AXIS]) {
    memcpy(&_currentValue, value, sizeof(_currentValue));
#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
    protocol_buffer_synchronize();
#endif
    nvs_set_blob(Setting::_handle, _name, _currentValue, sizeof(_currentValue));
}

ToolTable_t* ToolTable;  //= new ToolTable_t();

bool ToolTable_t::load() {
    uint8_t load_mask = 0;
    size_t  len;

    // load active
    switch (nvs_get_blob(Setting::_handle, name_act, &_tool_active, &len)) {
        case ESP_OK:
        case ESP_ERR_NVS_INVALID_LENGTH:
            // This could happen if the stored value is longer than the buffer.
            // That is highly unlikely since we always store MAX_N_AXIS coordinates.
            // It would indicate that we have decreased MAX_N_AXIS since the
            // value was stored.  We don't flag it as an error, but rather
            // accept the initial coordinates and ignore the residue.
            // We could issue a warning message if we were so inclined.
            bit_true(load_mask, bit(0));
            break;
        case ESP_ERR_NVS_INVALID_NAME:
        case ESP_ERR_NVS_INVALID_HANDLE:
        default:
            bit_false(load_mask, bit(0));
            break;
    }

    // load selected
    switch (nvs_get_blob(Setting::_handle, name_sel, &_tool_selected, &len)) {
        case ESP_OK:
        case ESP_ERR_NVS_INVALID_LENGTH:
            // This could happen if the stored value is longer than the buffer.
            // That is highly unlikely since we always store MAX_N_AXIS coordinates.
            // It would indicate that we have decreased MAX_N_AXIS since the
            // value was stored.  We don't flag it as an error, but rather
            // accept the initial coordinates and ignore the residue.
            // We could issue a warning message if we were so inclined.
            bit_true(load_mask, bit(1));
            break;
        case ESP_ERR_NVS_INVALID_NAME:
        case ESP_ERR_NVS_INVALID_HANDLE:
        default:
            bit_false(load_mask, bit(1));
            break;
    }

    // load tools
    for (int idx = 0; idx < MAX_TOOL_NUMBER; idx++) {
        if (_tools[idx]->load())
            bit_true(load_mask, bit(idx + 2));
        else
            bit_false(load_mask, bit(idx + 2));
    }

    _isinit = true;

    if (bit_isfalse(load_mask, B00111111)) {
        setDefault(B00111111);
        return false;
    }
    return true;
};

void ToolTable_t::set_tool_active(uint8_t value) {
    if (_isinit) {
        _tool_active = value;
#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
        protocol_buffer_synchronize();
#endif
        nvs_set_blob(Setting::_handle, name_act, &_tool_active, sizeof(_tool_active));
    }
}

void ToolTable_t::set_tool_selected(uint8_t value) {
    if (_isinit) {
        _tool_selected = value;
#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
        protocol_buffer_synchronize();
#endif
        nvs_set_blob(Setting::_handle, name_sel, &_tool_selected, sizeof(_tool_selected));
    }
}

bool ToolNfo::load() {
    uint8_t load_mask = 0;
    uint8_t load_xyz  = 0;
    size_t  len;

    // load xyz
    snprintf(_temp, sizeof(_temp), "%s_xyz", _name);
    switch (nvs_get_blob(Setting::_handle, _temp, _xyz, &len)) {
        case ESP_OK:
            bit_true(load_mask, bit(0));
            break;
        case ESP_ERR_NVS_INVALID_LENGTH:
            // This could happen if the stored value is longer than the buffer.
            // That is highly unlikely since we always store MAX_N_AXIS coordinates.
            // It would indicate that we have decreased MAX_N_AXIS since the
            // value was stored.  We don't flag it as an error, but rather
            // accept the initial coordinates and ignore the residue.
            // We could issue a warning message if we were so inclined.
        case ESP_ERR_NVS_INVALID_NAME:
        case ESP_ERR_NVS_INVALID_HANDLE:
        default:
            bit_false(load_mask, bit(0));
            break;
    }

    // load xyz alternative
    if (bit_isfalse(load_mask, bit(0))) {
        for (int idx = 0; idx < MAX_N_AXIS; idx++) {
            snprintf(_temp, sizeof(_temp), "%s_xyz_%d", _name, idx);
            switch (nvs_get_blob(Setting::_handle, _temp, &_xyz[idx], &len)) {
                case ESP_OK:
                case ESP_ERR_NVS_INVALID_LENGTH:
                    // This could happen if the stored value is longer than the buffer.
                    // That is highly unlikely since we always store MAX_N_AXIS coordinates.
                    // It would indicate that we have decreased MAX_N_AXIS since the
                    // value was stored.  We don't flag it as an error, but rather
                    // accept the initial coordinates and ignore the residue.
                    // We could issue a warning message if we were so inclined.
                    bit_true(load_xyz, bit(idx));
                    break;
                case ESP_ERR_NVS_INVALID_NAME:
                case ESP_ERR_NVS_INVALID_HANDLE:
                default:
                    bit_false(load_xyz, bit(idx));
                    break;
            }
        }
        if (bit_istrue(load_xyz, B00111111)) {
            bit_true(load_mask, bit(0));
        }
    }

    // load i
    snprintf(_temp, sizeof(_temp), "%s_i", _name);
    switch (nvs_get_blob(Setting::_handle, _temp, &_i, &len)) {
        case ESP_OK:
        case ESP_ERR_NVS_INVALID_LENGTH:
            // This could happen if the stored value is longer than the buffer.
            // That is highly unlikely since we always store MAX_N_AXIS coordinates.
            // It would indicate that we have decreased MAX_N_AXIS since the
            // value was stored.  We don't flag it as an error, but rather
            // accept the initial coordinates and ignore the residue.
            // We could issue a warning message if we were so inclined.
            bit_true(load_mask, bit(1));
            break;
        case ESP_ERR_NVS_INVALID_NAME:
        case ESP_ERR_NVS_INVALID_HANDLE:
        default:
            bit_false(load_mask, bit(1));
            break;
    }

    // load j
    snprintf(_temp, sizeof(_temp), "%s_j", _name);
    switch (nvs_get_blob(Setting::_handle, _temp, &_j, &len)) {
        case ESP_OK:
        case ESP_ERR_NVS_INVALID_LENGTH:
            // This could happen if the stored value is longer than the buffer.
            // That is highly unlikely since we always store MAX_N_AXIS coordinates.
            // It would indicate that we have decreased MAX_N_AXIS since the
            // value was stored.  We don't flag it as an error, but rather
            // accept the initial coordinates and ignore the residue.
            // We could issue a warning message if we were so inclined.
            bit_true(load_mask, bit(2));
            break;
        case ESP_ERR_NVS_INVALID_NAME:
        case ESP_ERR_NVS_INVALID_HANDLE:
        default:
            bit_false(load_mask, bit(2));
            break;
    }

    // load p
    snprintf(_temp, sizeof(_temp), "%s_p", _name);
    switch (nvs_get_blob(Setting::_handle, _temp, &_p, &len)) {
        case ESP_OK:
        case ESP_ERR_NVS_INVALID_LENGTH:
            // This could happen if the stored value is longer than the buffer.
            // That is highly unlikely since we always store MAX_N_AXIS coordinates.
            // It would indicate that we have decreased MAX_N_AXIS since the
            // value was stored.  We don't flag it as an error, but rather
            // accept the initial coordinates and ignore the residue.
            // We could issue a warning message if we were so inclined.
            bit_true(load_mask, bit(3));
            break;
        case ESP_ERR_NVS_INVALID_NAME:
        case ESP_ERR_NVS_INVALID_HANDLE:
        default:
            bit_false(load_mask, bit(3));
            break;
    }

    // load r
    snprintf(_temp, sizeof(_temp), "%s_r", _name);
    switch (nvs_get_blob(Setting::_handle, _temp, &_r, &len)) {
        case ESP_OK:
        case ESP_ERR_NVS_INVALID_LENGTH:
            // This could happen if the stored value is longer than the buffer.
            // That is highly unlikely since we always store MAX_N_AXIS coordinates.
            // It would indicate that we have decreased MAX_N_AXIS since the
            // value was stored.  We don't flag it as an error, but rather
            // accept the initial coordinates and ignore the residue.
            // We could issue a warning message if we were so inclined.
            bit_true(load_mask, bit(4));
            break;
        case ESP_ERR_NVS_INVALID_NAME:
        case ESP_ERR_NVS_INVALID_HANDLE:
        default:
            bit_false(load_mask, bit(4));
            break;
    }

    // load q
    snprintf(_temp, sizeof(_temp), "%s_q", _name);
    switch (nvs_get_blob(Setting::_handle, _temp, &_q, &len)) {
        case ESP_OK:
        case ESP_ERR_NVS_INVALID_LENGTH:
            // This could happen if the stored value is longer than the buffer.
            // That is highly unlikely since we always store MAX_N_AXIS coordinates.
            // It would indicate that we have decreased MAX_N_AXIS since the
            // value was stored.  We don't flag it as an error, but rather
            // accept the initial coordinates and ignore the residue.
            // We could issue a warning message if we were so inclined.
            bit_true(load_mask, bit(5));
            break;
        case ESP_ERR_NVS_INVALID_NAME:
        case ESP_ERR_NVS_INVALID_HANDLE:
        default:
            bit_false(load_mask, bit(5));
            break;
    }

    _isinit = true;

    if (bit_isfalse(load_mask, B00111111)) {
        // setDefault(-1);
        return false;
    }
    return true;
};

void ToolNfo::set_i(float value) {
    if (_isinit) {
        _i = value;
#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
        protocol_buffer_synchronize();
#endif
        snprintf(_temp, sizeof(_temp), "%s_i", _name);
        nvs_set_blob(Setting::_handle, _temp, &_i, sizeof(_i));
    }
}

void ToolNfo::set_j(float value) {
    if (_isinit) {
        _j = value;
#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
        protocol_buffer_synchronize();
#endif
        snprintf(_temp, sizeof(_temp), "%s_j", _name);
        nvs_set_blob(Setting::_handle, _temp, &_j, sizeof(_j));
    }
}

void ToolNfo::set_p(float value) {
    if (_isinit) {
        _p = value;
#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
        protocol_buffer_synchronize();
#endif
        snprintf(_temp, sizeof(_temp), "%s_p", _name);
        nvs_set_blob(Setting::_handle, _temp, &_p, sizeof(_p));
    }
}

void ToolNfo::set_q(float value) {
    if (_isinit) {
        _q = value;
#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
        protocol_buffer_synchronize();
#endif
        snprintf(_temp, sizeof(_temp), "%s_q", _name);
        nvs_set_blob(Setting::_handle, _temp, &_q, sizeof(_q));
    }
}

void ToolNfo::set_r(float value) {
    if (_isinit) {
        _r = value;
#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
        protocol_buffer_synchronize();
#endif
        snprintf(_temp, sizeof(_temp), "%s_r", _name);
        nvs_set_blob(Setting::_handle, _temp, &_r, sizeof(_r));
    }
}

void ToolNfo::set_xyz(float value[MAX_N_AXIS]) {
    if (_isinit) {
        memcpy(&_xyz, value, sizeof(_xyz));
#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
        protocol_buffer_synchronize();
#endif
        snprintf(_temp, sizeof(_temp), "%s_xyz", _name);

        nvs_set_blob(Setting::_handle, _temp, _xyz, sizeof(_xyz));

        for (int idx = 0; idx < MAX_N_AXIS; idx++) {
            snprintf(_temp, sizeof(_temp), "%s_xyz_%d", _name, idx);
            nvs_set_blob(Setting::_handle, _temp, &_xyz[idx], sizeof(float));
        }
    }
}
