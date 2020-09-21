'use strict';

const expect = require('chai').expect;
const int64 = require('../index');

describe('Add', function() {
  it('should add', function() {
    let n = int64.from(0);
    for (let i = 0; i < 10000; ++i) {
      n = int64.add(n, 98766455);
    }
    expect(n.toString()).to.equal('987664550000');
    let m = int64.from(0);
    for (let i = 0; i < 10; ++i) {
      m.add(n);
    }
    expect(m.toString()).to.equal('9876645500000');
  });
});

describe('Subtract', function() {
  it('should subtract', function() {
    let n = int64.from(0);
    for (let i = 0; i < 10000; ++i) {
      n = int64.subtract(n, 98766455);
    }
    expect(n.toString()).to.equal('-987664550000');
    let m = int64.from(0);
    for (let i = 0; i < 10; ++i) {
      m.subtract(n);
    }
    expect(m.toString()).to.equal('9876645500000');
  });
});

describe('Multiply', function() {
  it('should multiply', function() {
    let n = int64.from(1);
    for (let i = 1; i < 20; ++i) {
      n = int64.multiply(n, i);
    }
    expect(n.toString()).to.equal('121645100408832000');
    n.multiply(2);
    expect(n.toString()).to.equal('243290200817664000');
  });
});

describe('Divide', function() {
  it('should divide', function() {
    let n = int64.from('243290200817664000');
    n = int64.divide(n, 20);
    expect(n.toString()).to.equal('12164510040883200');
    n.divide(2);
    expect(n.toString()).to.equal('6082255020441600');
  });
});

describe('Mod', function() {
  it('should mod', function() {
    let n = int64.from('894453210654871');
    n = int64.mod(n, 8);
    expect(n.toNumber()).to.equal(7);
    n.mod(2);
    expect(n.toNumber()).to.equal(1);
  });
});

describe('Negative', function() {
  it('should negative', function() {
    let n = int64.from('894453210654871');
    n = int64.negative(n);
    expect(n.toString()).to.equal('-894453210654871');
    n.negative();
    expect(n.toString()).to.equal('894453210654871');
  });
});

describe('Gt', function() {
  it('should greater than', function() {
    let a = int64.from('894453210654871');
    let b = int64.from('894453210654870');
    let n = int64.gt(a, b);
    expect(n).to.equal(true);
    n = a.gt(b);
    expect(n).to.equal(true);
  });
});
