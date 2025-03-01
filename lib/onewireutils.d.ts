interface OneWireUtils {
    crc8(data: number[]): number;
    readDevices(data: number[]): number[][];
}
declare const OneWireUtils: OneWireUtils;
export default OneWireUtils;
