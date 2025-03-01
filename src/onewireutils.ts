import Encoder7Bit from './encoder7bit';

interface OneWireUtils {
  crc8(data: number[]): number;
  readDevices(data: number[]): number[][];
}

const OneWireUtils: OneWireUtils = {
  crc8(data) {
    let crc = 0;

    for (let inbyte of data) {
      for (let n = 8; n; n--) {
        const mix: boolean = !!((crc ^ inbyte) & 0x01);
        crc >>= 1;

        if (mix) {
          crc ^= 0x8c;
        }

        inbyte >>= 1;
      }
    }

    return crc;
  },
  readDevices(data) {
    const deviceBytes: number[] = Encoder7Bit.from7BitArray(data);
    const devices: number[][] = [];

    for (let i = 0; i < deviceBytes.length; i += 8) {
      const device: number[] = deviceBytes.slice(i, i + 8);

      if (device.length !== 8) {
        continue;
      }

      const check: number = OneWireUtils.crc8(device.slice(0, 7));

      if (check !== device[7]) {
        console.error('ROM invalid!');
      }

      devices.push(device);
    }

    return devices;
  },

  // readDevices(data: number[]): number[][] => {
  // }
};

export default OneWireUtils;
