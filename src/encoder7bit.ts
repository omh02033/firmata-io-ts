export default {
  to7BitArray(data: number[]): number[] {
    let shift = 0;
    let previous = 0;
    const output: number[] = [];

    for (const byte of data) {
      if (shift === 0) {
        output.push(byte & 0x7f);
        shift++;
        previous = byte >> 7;
      } else {
        output.push(((byte << shift) & 0x7f) | previous);
        if (shift === 6) {
          output.push(byte >> 1);
          shift = 0;
        } else {
          shift++;
          previous = byte >> (8 - shift);
        }
      }
    }

    if (shift > 0) {
      output.push(previous);
    }

    return output;
  },

  from7BitArray(encoded: number[]): number[] {
    const expectedBytes: number = (encoded.length * 7) >> 3;
    const decoded: number[] = [];

    for (let i = 0; i < expectedBytes; i++) {
      const j: number = i << 3;
      const pos: number = (j / 7) >>> 0;
      const shift: number = j % 7;
      decoded[i] = (encoded[pos] >> shift) | ((encoded[pos + 1] << (7 - shift)) & 0xff);
    }

    return decoded;
  },
};
