#include <node_api.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <inttypes.h>
#include <limits.h>
#include <time.h>

napi_ref Int64Ref;

// TODO -----Creators-----

napi_value createTrue(napi_env env){
        napi_value result;
        napi_get_boolean(env, true, &result);
        return result;
}

napi_value createFalse(napi_env env){
        napi_value result;
        napi_get_boolean(env, false, &result);
        return result;
}

napi_value createTrueOrFalse(napi_env env, bool boolean){
        napi_value result;
        napi_get_boolean(env, boolean, &result);
        return result;
}

// TODO -----Functions-----

uint64_t randomUInt64(){
        return (rand() & 0x0000000000000FFFull) |
        (((uint64_t)rand() << 12) & 0x0000000000FFF000ull) |
        (((uint64_t)rand() << 24) & 0x0000000FFF000000ull) |
        (((uint64_t)rand() << 36) & 0x0000FFF000000000ull) |
        (((uint64_t)rand() << 48) & 0x0FFF000000000000ull) |
        (((uint64_t)rand() << 60) & 0xF000000000000000ull);
}

int64_t randomInt64(int64_t min, int64_t max){
        uint64_t size = max - min + 1;
        if(size == 0){
                size = UINT_MAX;
        }
        uint64_t rnd = randomUInt64() % size;
        return (int64_t)(rnd + min);
}

void getRawData(napi_env env, napi_value me, int64_t** raw){
        napi_value _raw;
        napi_get_element(env, me, 0, &_raw);
        uint64_t rawLength;
        napi_get_buffer_info(env, _raw, (void**)raw, &rawLength);
}

int64_t argsToInt64(napi_env env, napi_value* args, size_t argsLength){
        if(argsLength == 0) {
                return 0;
        }else if(argsLength == 1) {
                napi_valuetype type;
                napi_typeof(env, args[0], &type);
                if (type == napi_string) {
                        size_t sourceDataSize;
                        napi_get_value_string_latin1(env, args[0], NULL, 0, &sourceDataSize);
                        ++sourceDataSize;
                        char* data = (char*)malloc(sizeof(char) * sourceDataSize);
                        napi_get_value_string_latin1(env, args[0], data, sourceDataSize, &sourceDataSize);
                        int64_t sum = 0;
                        if(sourceDataSize > 2 && sourceDataSize <= 18 && data[0] == '0' && (data[1] == 'x' || data[1] == 'X' || data[1] == 'b' || data[1] == 'B')) {
                                sum = strtoul(data, (char**)0, 0);
                        }else{
                                size_t i;
                                char c;
                                for(i = 0; i < sourceDataSize; ++i) {
                                        sum *= 10;
                                        c = data[i];
                                        if(c >= '0' && c <= '9') {
                                                sum += c - '0';
                                        }
                                }
                        }
                        free(data);
                        return sum;
                }else if(type == napi_object) {
                        bool isInt64;
                        napi_value Int64;
                        napi_get_reference_value(env, Int64Ref, &Int64);
                        napi_instanceof(env, args[0], Int64, &isInt64);
                        if(isInt64) {
                                int64_t* raw;
                                getRawData(env, args[0], &raw);
                                return *raw;
                        }else{
                                bool isBuffer;
                                napi_is_buffer(env, args[0], &isBuffer);
                                if(isBuffer) {
                                        uint8_t* data;
                                        size_t bufferLength;
                                        napi_get_buffer_info(env, args[0], (void*)(&data),&bufferLength);
                                        if(bufferLength == 8) {
                                                return (int64_t)(*((int64_t*)data));
                                        }
                                }
                        }
                }else if(type == napi_number) {
                        int64_t initialValue;
                        napi_get_value_int64(env, args[0], &initialValue);
                        return initialValue;
                }
                return 0;
        }else{
                int32_t initialLowValue, initialHighValue;
                napi_get_value_int32(env, args[0], &initialLowValue);
                napi_get_value_int32(env, args[1], &initialHighValue);
                return ((int64_t)initialHighValue << 32) | ((int64_t)initialLowValue & 0x00000000FFFFFFFF);
        }
}

void argsPairToInt64s(napi_env env, napi_value* args, size_t argsLength, int64_t* result, int64_t defaultValue){
        if(argsLength == 0) {
                result[0] = 0;
                result[1] = defaultValue;
        }else if(argsLength == 1) {
                result[0] = argsToInt64(env, args, 1);
                result[1] = defaultValue;
        }else if(argsLength == 2) {
                bool isANumber, isBNumber;
                napi_valuetype typeA;
                napi_typeof(env, args[0], &typeA);
                isANumber = typeA == napi_number;

                napi_valuetype typeB;
                napi_typeof(env, args[1], &typeB);
                isBNumber = typeB == napi_number;
                if(isANumber) {
                        if(isBNumber){
                                result[0] = argsToInt64(env, args, 2);
                                result[1] = 0;
                        }else{
                                result[0] = argsToInt64(env, args, 1);
                                result[1] = argsToInt64(env, args + 1, 1);
                        }
                }else{
                        if(isBNumber) {
                                result[0] = argsToInt64(env, args, 1);
                                result[1] = argsToInt64(env, args + 1, 1);
                        }else{
                                result[0] = argsToInt64(env, args, 1);
                                result[1] = argsToInt64(env, args + 1, 1);
                        }
                }
                result[0] = argsToInt64(env, args, 1);
                result[1] = argsToInt64(env, args + 1, 1);
        }else if(argsLength == 3) {
                bool isANumber;
                napi_valuetype typeA;
                napi_typeof(env, args[0], &typeA);
                isANumber = typeA == napi_number;
                if(isANumber) {
                        bool isBNumber;
                        napi_valuetype typeB;
                        napi_typeof(env, args[1], &typeB);
                        isBNumber = typeB == napi_number;
                        if(isBNumber) {
                                result[0] = argsToInt64(env, args, 2);
                                result[1] = argsToInt64(env, args + 2, 1);
                        }else{
                                result[0] = argsToInt64(env, args, 1);
                                result[1] = argsToInt64(env, args + 1, 2);
                        }
                }else{
                        result[0] = argsToInt64(env, args, 1);
                        result[1] = argsToInt64(env, args + 1, 2);
                }
        }else{
                bool isANumber;
                napi_valuetype typeA;
                napi_typeof(env, args[0], &typeA);
                isANumber = typeA == napi_number;
                if(isANumber) {
                        bool isBNumber;
                        napi_valuetype typeB;
                        napi_typeof(env, args[1], &typeB);
                        isBNumber = typeB == napi_number;
                        if(isBNumber) {
                                result[0] = argsToInt64(env, args, 2);
                                result[1] = argsToInt64(env, args + 2, 2);
                        }else{
                                result[0] = argsToInt64(env, args, 1);
                                result[1] = argsToInt64(env, args + 2, 2);
                        }
                }else{
                        result[0] = argsToInt64(env, args, 1);
                        result[1] = argsToInt64(env, args + 1, 2);
                }
        }
}

// TODO -----Static-----

napi_value from(napi_env env, napi_callback_info info){
        napi_value me, newMe;
        size_t argsLength = 2;
        napi_value args[2];
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_new_instance(env, Int64, argsLength, args, &newMe);
        return newMe;
}

napi_value rnd(napi_env env, napi_callback_info info){
        napi_value me, newMe;
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        int64_t result[2];
        if(argsLength == 0){
                result[0] = LONG_MIN;
                result[1] = LONG_MAX;
        }else{
                argsPairToInt64s(env, args, argsLength, result, LONG_MAX);
        }

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_new_instance(env, Int64, argsLength, args, &newMe);

        int64_t* raw;
        getRawData(env, newMe, &raw);

        *raw = randomInt64(result[0], result[1]);
        return newMe;
}

napi_value add(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = result[0] + result[1];
        return newObj;
}

napi_value subtract(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = result[0] - result[1];
        return newObj;
}

napi_value multiply(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 1);

        *raw = result[0] * result[1];
        return newObj;
}

napi_value divide(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 1);

        *raw = result[0] / result[1];
        return newObj;
}

napi_value mod(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = result[0] % result[1];
        return newObj;
}

napi_value negative(napi_env env, napi_callback_info info){
        napi_value me, newMe;
        size_t argsLength = 2;
        napi_value args[2];
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        int64_t* raw;
        napi_new_instance(env, Int64, argsLength, args, &newMe);
        getRawData(env, newMe, &raw);

        *raw = -(*raw);
        return newMe;
}

napi_value shiftLeft(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = result[0] << result[1];
        return newObj;
}

napi_value shiftRight(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = result[0] >> result[1];
        return newObj;
}

napi_value shiftRightUnsigned(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = (int64_t)((uint64_t)result[0] >> result[1]);
        return newObj;
}

napi_value rotateRight(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = (int64_t)((result[0] >> result[1]) | (result[0] << (64 - result[1])));
        return newObj;
}

napi_value rotateLeft(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = (int64_t)((result[0] << result[1]) | ((uint64_t)result[0] >> (64 - result[1])));
        return newObj;
}

napi_value and(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = result[0] & result[1];
        return newObj;
}

napi_value or(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = result[0] | result[1];
        return newObj;
}

napi_value xor (napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = result[0] ^ result[1];
        return newObj;
}

napi_value nand(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = ~(result[0] & result[1]);
        return newObj;
}

napi_value nor(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = ~(result[0] | result[1]);
        return newObj;
}

napi_value xnor(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        napi_value newObj;
        int64_t* raw;
        napi_new_instance(env, Int64, 0, NULL, &newObj);
        getRawData(env, newObj, &raw);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        *raw = ~(result[0] ^ result[1]);
        return newObj;
}

napi_value not (napi_env env, napi_callback_info info){
        napi_value me, newMe;
        size_t argsLength = 2;
        napi_value args[2];
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        napi_value Int64;
        napi_get_reference_value(env, Int64Ref, &Int64);

        int64_t* raw;
        napi_new_instance(env, Int64, argsLength, args, &newMe);
        getRawData(env, newMe, &raw);

        *raw = ~(*raw);
        return newMe;
}

napi_value eq(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        return createTrueOrFalse(env, result[0] == result[1]);
}

napi_value ne(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        return createTrueOrFalse(env, result[0] != result[1]);
}

napi_value gt(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        return createTrueOrFalse(env, result[0] > result[1]);
}

napi_value gte(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        return createTrueOrFalse(env, result[0] >= result[1]);
}

napi_value lt(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        return createTrueOrFalse(env, result[0] < result[1]);
}

napi_value lte(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        return createTrueOrFalse(env, result[0] <= result[1]);
}

napi_value comp(napi_env env, napi_callback_info info){
        size_t argsLength = 4;
        napi_value args[4];
        napi_get_cb_info(env, info, &argsLength, args, 0, 0);

        int64_t result[2];
        argsPairToInt64s(env, args, argsLength, result, 0);

        int32_t v;
        if(result[0] > result[1]) {
                v = 1;
        }else if(result[0] == result[1]) {
                v = 0;
        }else{
                v = -1;
        }
        napi_value value;
        napi_create_int32(env, v, &value);
        return value;
}

// TODO -----Object-----

napi_value AddMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw += v;
        return me;
}

napi_value SubtractMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw -= v;
        return me;
}

napi_value MultiplyMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw *= v;
        return me;
}

napi_value DivideMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw /= v;
        return me;
}

napi_value ModMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw %= v;
        return me;
}

napi_value NegativeMethod(napi_env env, napi_callback_info info){
        napi_value me;
        napi_get_cb_info(env, info, 0, NULL, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        *raw = -(*raw);
        return me;
}

napi_value ShiftLeftMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw <<= v;
        return me;
}

napi_value ShiftRightMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw >>= v;
        return me;
}

napi_value ShiftRightUnsignedMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw = (int64_t)((uint64_t)*raw >> v);
        return me;
}

napi_value RotateRightMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw = (int64_t)((*raw >> v) | (*raw << (64 - v)));
        return me;
}

napi_value RotateLeftMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw = (int64_t)((*raw << v) | ((uint64_t)*raw >> (64 - v)));
        return me;
}

napi_value SetMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw = v;
        return me;
}

napi_value AndMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw = *raw & v;
        return me;
}

napi_value OrMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw = *raw | v;
        return me;
}

napi_value XorMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw = *raw ^ v;
        return me;
}

napi_value NandMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw = ~(*raw & v);
        return me;
}

napi_value NorMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw = ~(*raw | v);
        return me;
}

napi_value XnorMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);
        if(argsLength == 0) {
                return me;
        }

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        *raw = ~(*raw ^ v);
        return me;
}

napi_value NotMethod(napi_env env, napi_callback_info info){
        napi_value me;
        napi_get_cb_info(env, info, 0, NULL, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        *raw = ~(*raw);
        return me;
}

napi_value EqMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        return createTrueOrFalse(env, *raw == v);
}

napi_value NeMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        return createTrueOrFalse(env, *raw != v);
}

napi_value GtMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        return createTrueOrFalse(env, *raw > v);
}

napi_value GteMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        return createTrueOrFalse(env, *raw >= v);
}

napi_value LtMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        return createTrueOrFalse(env, *raw < v);
}

napi_value LteMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        return createTrueOrFalse(env, *raw <= v);
}

napi_value CompMethod(napi_env env, napi_callback_info info){
        size_t argsLength = 2;
        napi_value args[2];
        napi_value me;
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = argsToInt64(env, args, argsLength);
        if(*raw > v) {
                v = 1;
        }else if(*raw == v) {
                v = 0;
        }else{
                v = -1;
        }
        napi_value value;
        napi_create_int32(env, v, &value);
        return value;
}

napi_value GetText(napi_env env, napi_callback_info info){
        napi_value me;
        napi_get_cb_info(env, info, 0, 0, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        const int n = snprintf(NULL, 0, "%" PRIx64, *raw);
        char* str = (char*)malloc(sizeof(char) * (19));
        int pointer = 0;
        str[pointer++] = '0';
        str[pointer++] = 'x';
        int i;
        int k = 16 - n;
        for(i = 0; i < k; ++i) {
                str[pointer++] = '0';
        }
        snprintf(str + pointer, n + 1, "%" PRIx64, *raw);
        for(i = 0; i < n; ++i) {
                char c = str[pointer];
                if(c >= 'a' && c <= 'z') {
                        str[pointer] = c - 'a' + 'A';
                }
                ++pointer;
        }

        napi_value _str;
        napi_create_string_latin1(env, str, NAPI_AUTO_LENGTH, &_str);
        free(str);
        return _str;
}

napi_value GetHex(napi_env env, napi_callback_info info){
        napi_value me;
        napi_get_cb_info(env, info, 0, 0, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        const int n = snprintf(NULL, 0, "%" PRIx64, *raw);
        char* str = (char*)malloc(sizeof(char) * (16));
        int pointer = 0;
        int i;
        int k = 16 - n;
        for(i = 0; i < k; ++i) {
                str[pointer++] = '0';
        }
        snprintf(str + pointer, n + 1, "%" PRIx64, *raw);

        napi_value _str;
        napi_create_string_latin1(env, str, NAPI_AUTO_LENGTH, &_str);
        free(str);
        return _str;
}

napi_value GetString(napi_env env, napi_callback_info info){
        napi_value me;
        napi_get_cb_info(env, info, 0, 0, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        const int n = snprintf(NULL, 0, "%" PRId64, *raw);
        char* str = (char*)malloc(sizeof(char) * (n + 1));
        snprintf(str, n+1, "%" PRId64, *raw);

        napi_value _str;
        napi_create_string_latin1(env, str, NAPI_AUTO_LENGTH, &_str);
        free(str);
        return _str;
}

napi_value GetValue(napi_env env, napi_callback_info info){
        napi_value me;
        napi_get_cb_info(env, info, 0, 0, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        napi_value _value;
        int64_t* value;
        napi_create_buffer(env, 8, (void**)(&value), &_value);
        memcpy(value, raw, 8);
        return _value;
}

napi_value GetIntValues(napi_env env, napi_callback_info info){
        napi_value me;
        napi_get_cb_info(env, info, 0, 0, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = *raw;
        int32_t low = (int32_t)(((uint64_t)(v << 32)) >> 32);
        int32_t high = (int32_t)((uint64_t)v >> 32);
        napi_value result, lowValue, highValue;
        napi_create_int32(env, low, &lowValue);
        napi_create_int32(env, high, &highValue);
        napi_create_object(env, &result);
        napi_set_named_property(env, result, "low", lowValue);
        napi_set_named_property(env, result, "high", highValue);
        return result;
}

napi_value GetNumber(napi_env env, napi_callback_info info){
        napi_value me;
        napi_get_cb_info(env, info, 0, 0, &me, 0);

        int64_t* raw;
        getRawData(env, me, &raw);

        int64_t v = *raw;
        if(v > 9007199254740991l || v < -9007199254740991) {
                return createFalse(env);
        }

        napi_value value;
        napi_create_int64(env, v, &value);
        return value;
}

napi_value constructor(napi_env env, napi_callback_info info){
        napi_value me;

        size_t argsLength = 2;
        napi_value args[2];
        napi_get_cb_info(env, info, &argsLength, args, &me, 0);

        napi_value _raw;
        int64_t* raw;
        napi_create_buffer(env, 8, (void**)(&raw), &_raw);
        *raw = argsToInt64(env, args, argsLength);
        napi_set_element(env, me, 0, _raw);
        return me;
}

napi_value Init (napi_env env, napi_value exports) {
        napi_property_descriptor allDesc[] = {
                {"from", 0, from, 0, 0, 0, napi_default, 0},
                {"random", 0, rnd, 0, 0, 0, napi_default, 0},
                {"add", 0, add, 0, 0, 0, napi_default, 0},
                {"subtract", 0, subtract, 0, 0, 0, napi_default, 0},
                {"multiply", 0, multiply, 0, 0, 0, napi_default, 0},
                {"divide", 0, divide, 0, 0, 0, napi_default, 0},
                {"mod", 0, mod, 0, 0, 0, napi_default, 0},
                {"negative", 0, negative, 0, 0, 0, napi_default, 0},
                {"shiftLeft", 0, shiftLeft, 0, 0, 0, napi_default, 0},
                {"shiftRight", 0, shiftRight, 0, 0, 0, napi_default, 0},
                {"shiftRightUnsigned", 0, shiftRightUnsigned, 0, 0, 0, napi_default, 0},
                {"rotateRight", 0, rotateRight, 0, 0, 0, napi_default, 0},
                {"rotateLeft", 0, rotateLeft, 0, 0, 0, napi_default, 0},
                {"and", 0, and, 0, 0, 0, napi_default, 0},
                {"or", 0, or, 0, 0, 0, napi_default, 0},
                {"xor", 0, xor, 0, 0, 0, napi_default, 0},
                {"nand", 0, nand, 0, 0, 0, napi_default, 0},
                {"nor", 0, nor, 0, 0, 0, napi_default, 0},
                {"xnor", 0, xnor, 0, 0, 0, napi_default, 0},
                {"not", 0, not, 0, 0, 0, napi_default, 0},
                {"eq", 0, eq, 0, 0, 0, napi_default, 0},
                {"ne", 0, ne, 0, 0, 0, napi_default, 0},
                {"gt", 0, gt, 0, 0, 0, napi_default, 0},
                {"gte", 0, gte, 0, 0, 0, napi_default, 0},
                {"lt", 0, lt, 0, 0, 0, napi_default, 0},
                {"lte", 0, lte, 0, 0, 0, napi_default, 0},
                {"comp", 0, comp, 0, 0, 0, napi_default, 0}
        };
        napi_define_properties(env, exports, 27, allDesc);

        napi_property_descriptor int64AllDesc[] = {
                {"from", 0, from, 0, 0, 0, napi_static, 0},
                {"random", 0, rnd, 0, 0, 0, napi_static, 0},
                {"set", 0, SetMethod, 0, 0, 0, napi_default, 0},
                {"add", 0, AddMethod, 0, 0, 0, napi_default, 0},
                {"subtract", 0, SubtractMethod, 0, 0, 0, napi_default, 0},
                {"multiply", 0, MultiplyMethod, 0, 0, 0, napi_default, 0},
                {"divide", 0, DivideMethod, 0, 0, 0, napi_default, 0},
                {"mod", 0, ModMethod, 0, 0, 0, napi_default, 0},
                {"negative", 0, NegativeMethod, 0, 0, 0, napi_default, 0},
                {"shiftLeft", 0, ShiftLeftMethod, 0, 0, 0, napi_default, 0},
                {"shiftRight", 0, ShiftRightMethod, 0, 0, 0, napi_default, 0},
                {"shiftRightUnsigned", 0, ShiftRightUnsignedMethod, 0, 0, 0, napi_default, 0},
                {"rotateRight", 0, RotateRightMethod, 0, 0, 0, napi_default, 0},
                {"rotateLeft", 0, RotateLeftMethod, 0, 0, 0, napi_default, 0},
                {"and", 0, AndMethod, 0, 0, 0, napi_default, 0},
                {"or", 0, OrMethod, 0, 0, 0, napi_default, 0},
                {"xor", 0, XorMethod, 0, 0, 0, napi_default, 0},
                {"nand", 0, NandMethod, 0, 0, 0, napi_default, 0},
                {"nor", 0, NorMethod, 0, 0, 0, napi_default, 0},
                {"xnor", 0, XnorMethod, 0, 0, 0, napi_default, 0},
                {"not", 0, NotMethod, 0, 0, 0, napi_default, 0},
                {"eq", 0, EqMethod, 0, 0, 0, napi_default, 0},
                {"ne", 0, NeMethod, 0, 0, 0, napi_default, 0},
                {"gt", 0, GtMethod, 0, 0, 0, napi_default, 0},
                {"gte", 0, GteMethod, 0, 0, 0, napi_default, 0},
                {"lt", 0, LtMethod, 0, 0, 0, napi_default, 0},
                {"lte", 0, LteMethod, 0, 0, 0, napi_default, 0},
                {"comp", 0, CompMethod, 0, 0, 0, napi_default, 0},
                {"getValue", 0, GetValue, 0, 0, 0, napi_default, 0},
                {"getIntValues", 0, GetIntValues, 0, 0, 0, napi_default, 0},
                {"inspect", 0, GetString, 0, 0, 0, napi_default, 0},
                {"toString", 0, GetString, 0, 0, 0, napi_default, 0},
                {"toHex", 0, GetHex, 0, 0, 0, napi_default, 0},
                {"toText", 0, GetText, 0, 0, 0, napi_default, 0},
                {"toNumber", 0, GetNumber, 0, 0, 0, napi_default, 0}
        };
        napi_value cons;
        napi_define_class(env, "Int64", -1, constructor, 0, 35, int64AllDesc, &cons);
        napi_set_named_property(env, exports, "Int64", cons);
        napi_create_reference(env, cons, 1, &Int64Ref);

        srand(time(NULL));
        return exports;
}

NAPI_MODULE(int64, Init);
