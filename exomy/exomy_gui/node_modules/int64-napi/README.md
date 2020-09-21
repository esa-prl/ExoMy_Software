Int64
=================================

## Introduction

This module uses N-API to compute 64 bits integers, which allows you can do some 64-bit integer computing (such as hashing) in Node.js.

In JavaScript, the length of type `number` is 64 bits. A `number` can be used to represent an IEEE-754 double-precision number or a 53-bit integer.

## Installation

Run `npm i` or `npm install` to install.

```bash
npm install int64-napi
```

If you want to save this module to package.json, please add `--save` option.

```bash
npm install int64-napi --save
```

## Initialization

Import this module by using `require` function.

```javascript
const int64 = require('int64-napi'); // static functions
const Int64 = int64.Int64; // instance methods
```

## Usage

First of all, an int64(long) value can be represented by

1. An Int64 object(instance).
1. Two integer values, lowBits and highBits respectively.
1. A 53-bit integer number.
1. A string of a decimal number.
1. A string of a hexadecimal number, starting with `0x`.
1. A string of a binary number, starting with `0b`.
1. A buffer with 8 bytes


### Static

#### Random

```javascript
var n = int64.random('0x1234567800000000', '0x123456780000FFFF'); // 0x123456780000CE74
```

#### Add

```javascript
var n = int64.add('1', 0x00000002, 0x00000000); // 1 + 2 = 3
```

#### Subtract

```javascript
var n = int64.subtract(1, 2); // 1 - 2 = -1
```

#### multiply

```javascript
var n = int64.subtract(2, 6); // 12
```

#### divide

```javascript
var n = int64.divide(6, 4); // 1
```

#### mod

```javascript
var n = int64.mod(6, 4); // 2
```

#### shiftLeft

```javascript
var n = int64.shiftLeft(5, 2); // 20
```

#### shiftRight

```javascript
var n1 = int64.shiftRight(5, 2); // 1
var n2 = int64.shiftRight(6, 1); // 3
var n3 = int64.shiftRight(-5, 1); // -3
```

#### shiftRightUnsigned

```javascript
var n = int64.shiftRightUnsigned(-5, 1); // 9223372036854775805
```

#### rotateRight

```javascript
var n = int64.rotateRight('0x0000000080000100', 16); // 0x0100000000008000
```

#### rotateLeft

```javascript
var n = int64.rotateLeft('0x0080000000000100', 16); // 0x0000000001000080
```

#### and

```javascript
var n = int64.and('0x000000000000FFFF', '0x0123456789ABCDEF'); // 0x000000000000CDEF
```

#### or

```javascript
var n = int64.or('0x0000FFFF0000FFFF', '0xFFFFFFFFFFFF0000'); // 0xFFFFFFFFFFFFFFFF
```

#### xor

```javascript
var n = int64.xor('0x0000FFFF0000FFFF', '0xFFFFFFFFFFFF0000'); // 0xFFFF0000FFFFFFFF
```

#### nand

```javascript
var n = int64.nand('0x000000000000FFFF', '0x0123456789ABCDEF'); // 0xFFFFFFFFFFFF3210
```

#### nor

```javascript
var n = int64.nor('0x0000FFFF0000FFFF', '0xFFFFFFFFFFFF0000'); // 0x0000000000000000
```

#### xnor

```javascript
var n = int64.xnor('0x0000FFFF0000FFFF', '0xFFFFFFFFFFFF0000'); // 0x0000FFFF00000000
```

#### not

```javascript
var n = int64.nor('0x0000FFFF0000FFFF', '0xFFFFFFFFFFFF0000'); // 0x0000000000000000
```

#### eq(Equal)

```javascript
var n = int64.eq('0x0000FFFF0000FFFF', '281470681808895'); // true
```

#### ne(Not Equal)

```javascript
var n = int64.ne('0x0000FFFF0000FFFF', '0x0000FFFF00000000'); // true
```

#### gt(Greater Than)

```javascript
var n = int64.gt('0x0000FFFF0000FFFF', '0x0000FFFF00000000'); // true
```

#### gte(Greater Than or Equal)

```javascript
var n = int64.gte('0x0000FFFF0000FFFF', '0x0000FFFF00000000'); // true
```

#### lt(Less Than)

```javascript
var n = int64.lt('0x0000FFFF0000FFFF', '0x0000FFFF0000FFFF'); // false
```

#### lte(Less Than or Equal)

```javascript
var n = int64.lte('0x0000FFFF0000FFFF', '0x0000FFFF0000FFFF'); // true
```

#### comp(Compare)

If the first one is bigger than the second one, returns `1`.

If the first one is smaller than the second one, returns `-1`.

If the first one is equal to the second one, returns `0`.

```javascript
var a = int64.comp('0x0000FFFF0000FFFF', '0x0000FFFF0000FFFF'); // 0
var b = int64.comp('0x0000FFFF0000FFFF', '0x0000FFFF00000000'); // 1
var c = int64.comp('0x0000FFFF00000000', '0x0000FFFF0000FFFF'); // -1
```

### Instance / Object

#### Create an Instance

```javascript
var i64 = new Int64(1);
```

or

```javascript
var i64 = int64.from(1);
```

or

```javascript
var i64 = Int64.from(1);
```

#### Methods

```javascript
var n1 = i64.add(1).multiply(3).subtract(3).divide(3).toText(); // 0x0000000000000001
i64.set(0, 0xFFFF0000);
var n2 = i64.shiftLeft(8).shiftRight(56).toText(); // 0xFFFFFFFFFFFFFFFF
i64.set(0, 0xFFFF0000);
var n3 = i64.shiftLeft(8).shiftRightUnsigned(56).toText(); // 0x00000000000000FF
i64.set('0x000000010001');
var n4 = i64.rotateRight(8).toText(); // 0x0100000000000100
i64.set(0xFFFF0000, 0x0000FFFF);
var n5 = i64.getIntValues();  // { low: -65536, high: 65535 }
var n51 = i64.toText(); // 0x0000FFFFFFFF0000
var n52 = i64.toHex(); // 0000ffffffff0000
var n53 = i64.toString() + 1; // 2814749766451201
var n54 = i64.toNumber() + 1; // 281474976645121
```

## Tests

To run the test suite, first install the dependencies, then run `npm test`:

```bash
npm install
npm test
```

## License

[MIT](LICENSE)
