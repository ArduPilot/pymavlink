// ignore_for_file: non_constant_identifier_names

abstract class _MAVTypeNative<T> {
  late final T _nativeValue;
  T get value => _nativeValue;

  final int bits;
  int get bytes => bits ~/ 8;

  abstract final T MAX_VALUE;
  abstract final T MIN_VALUE;

  _MAVTypeNative({required T value, required int this.bits}) : _nativeValue = value {
    if (!validate()) {
      throw ArgumentError.value(value, 'value', 'Value is out of range');
    }
  }

  bool validate<T extends num>() {
    return ((_nativeValue as num) >= (MIN_VALUE as num) && (_nativeValue as num) <= (MAX_VALUE as num));
  }
}

abstract class _MAVTypeBigInt<T extends BigInt> extends _MAVTypeNative<T> {
  _MAVTypeBigInt({required T value, required int bits}) : super(value: value, bits: bits);

  @override
  bool validate<T extends num>() {
    return (_nativeValue >= MIN_VALUE && _nativeValue <= MAX_VALUE);
  }
}

class MAVFloat extends _MAVTypeNative<double> {
  @override
  final double MAX_VALUE = 3.40282346638528859811704183484516925439e38;

  @override
  final double MIN_VALUE = -3.40282346638528859811704183484516925440e38;

  MAVFloat(double value): super(value: value, bits: 32);
}

class MAVDouble extends _MAVTypeNative<double> {
  @override
  final double MAX_VALUE = double.maxFinite;

  @override
  final double MIN_VALUE = -double.maxFinite;

  MAVDouble(double value): super(value: value, bits: 64);
}

class MAVInt8 extends _MAVTypeNative<int> {
  @override
  final int MAX_VALUE = 127;

  @override
  final int MIN_VALUE = -128;

  MAVInt8(int value): super(value: value, bits: 8);
}

class MAVChar extends _MAVTypeNative<int> {
  @override
  final int MAX_VALUE = 127;

  @override
  final int MIN_VALUE = 0;

  late final String _representation;

  @override
  String toString() => _representation;

  MAVChar(int value): super(value: value, bits: 8) {
    _representation = String.fromCharCode(value);
  }

  MAVChar.fromString(String value): super(value: value.codeUnitAt(0), bits: 8) {
    if (value.length != 1) {
      throw ArgumentError.value(value, 'value', 'String must be exactly 1 character long');
    }
    _representation = value;
  }

  @override
  bool validate<T extends num>() {
    return super.validate() && _representation.length == 1;
  }
}

class MAVUint8 extends _MAVTypeNative<int> {
  @override
  final int MAX_VALUE = 255;

  @override
  final int MIN_VALUE = 0;

  MAVUint8(int value): super(value: value, bits: 8);
}

class MAVUint16 extends _MAVTypeNative<int> {
  @override
  final int MAX_VALUE = 65535;

  @override
  final int MIN_VALUE = 0;

  MAVUint16(int value): super(value: value, bits: 16);
}

class MAVInt16 extends _MAVTypeNative<int> {
  @override
  final int MAX_VALUE = 32767;

  @override
  final int MIN_VALUE = -32768;

  MAVInt16(int value): super(value: value, bits: 16);
}

class MAVUint32 extends _MAVTypeNative<int> {
  @override
  final int MAX_VALUE = 4294967295;

  @override
  final int MIN_VALUE = 0;

  MAVUint32(int value): super(value: value, bits: 32);
}

class MAVInt32 extends _MAVTypeNative<int> {
  @override
  final int MAX_VALUE = 2147483647;

  @override
  final int MIN_VALUE = -2147483648;

  MAVInt32(int value): super(value: value, bits: 32);
}

class MAVUint64 extends _MAVTypeBigInt<BigInt> {
  @override
  final BigInt MAX_VALUE = BigInt.parse("18446744073709551615");

  @override
  final BigInt MIN_VALUE = BigInt.zero;

  MAVUint64(BigInt value): super(value: value, bits: 64);

  @override
  bool validate<T extends num>() {
    return super.validate() && _nativeValue.sign != -1;
  }
}

class MAVInt64 extends _MAVTypeBigInt<BigInt> {
  @override
  final BigInt MAX_VALUE = BigInt.parse("9223372036854775807");

  @override
  final BigInt MIN_VALUE = -BigInt.parse("9223372036854775808");

  MAVInt64(BigInt value): super(value: value, bits: 64);
}
