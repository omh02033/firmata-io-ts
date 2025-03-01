import Emitter from "events";
declare const SYM_sendOneWireSearch: unique symbol;
declare const SYM_sendOneWireRequest: unique symbol;
interface Board {
    version: {
        major?: number;
        minor?: number;
    };
    buffer: number[];
    pins: Pin[];
    analogPins: number[];
    ports: number[];
    firmware?: {
        name?: string;
        version?: {
            major: number;
            minor: number;
        };
    };
    RESOLUTION: {
        ADC: number | null;
        PWM: number | null;
        DAC: number | null;
    };
    MODES: {
        [key: string]: number;
    };
    emit(event: string, ...args: any[]): void;
}
interface Pin {
    supportedModes: number[];
    mode?: number;
    value?: number;
    report?: number;
    analogChannel?: number;
    state?: number;
}
/**
 * @class The Board object represents an arduino board.
 * @augments EventEmitter
 * @param {String} port This is the serial port the arduino is connected to.
 * @param {function} function A function to be called when the arduino is ready to communicate.
 * @property MODES All the modes available for pins on this arduino board.
 * @property I2C_MODES All the I2C modes available.
 * @property SERIAL_MODES All the Serial modes available.
 * @property SERIAL_PORT_ID ID values to pass as the portId parameter when calling serialConfig.
 * @property HIGH A constant to set a pins value to HIGH when the pin is set to an output.
 * @property LOW A constant to set a pins value to LOW when the pin is set to an output.
 * @property pins An array of pin object literals.
 * @property analogPins An array of analog pins and their corresponding indexes in the pins array.
 * @property version An object indicating the major and minor version of the firmware currently running.
 * @property firmware An object indicating the name, major and minor version of the firmware currently running.
 * @property buffer An array holding the current bytes received from the arduino.
 * @property {SerialPort} sp The serial port object used to communicate with the arduino.
 */
export declare class Firmata extends Emitter implements Board {
    isReady: boolean;
    MODES: {
        INPUT: number;
        OUTPUT: number;
        ANALOG: number;
        PWM: number;
        SERVO: number;
        SHIFT: number;
        I2C: number;
        ONEWIRE: number;
        STEPPER: number;
        SERIAL: number;
        PULLUP: number;
        IGNORE: number;
        PING_READ: number;
        UNKOWN: number;
    };
    I2C_MODES: {
        WRITE: number;
        READ: number;
        CONTINUOUS_READ: number;
        STOP_READING: number;
    };
    STEPPER: {
        TYPE: {
            DRIVER: number;
            TWO_WIRE: number;
            THREE_WIRE: number;
            FOUR_WIRE: number;
        };
        STEP_SIZE: {
            WHOLE: number;
            HALF: number;
        };
        RUN_STATE: {
            STOP: number;
            ACCEL: number;
            DECEL: number;
            RUN: number;
        };
        DIRECTION: {
            CCW: number;
            CW: number;
        };
    };
    SERIAL_MODES: {
        CONTINUOUS_READ: number;
        STOP_READING: number;
    };
    SERIAL_PORT_IDs: {
        HW_SERIAL0: number;
        HW_SERIAL1: number;
        HW_SERIAL2: number;
        HW_SERIAL3: number;
        SW_SERIAL0: number;
        SW_SERIAL1: number;
        SW_SERIAL2: number;
        SW_SERIAL3: number;
        DEFAULT: number;
    };
    SERIAL_PIN_TYPES: {
        RES_RX0: number;
        RES_TX0: number;
        RES_RX1: number;
        RES_TX1: number;
        RES_RX2: number;
        RES_TX2: number;
        RES_RX3: number;
        RES_TX3: number;
    };
    RESOLUTION: {
        ADC: null | number;
        DAC: null | number;
        PWM: null | number;
    };
    HIGH: number;
    LOW: number;
    pins: Pin[];
    ports: any[];
    analogPins: number[];
    version: {
        major: number;
        minor: number;
    };
    firmware: {};
    buffer: number[];
    versionReceived: boolean;
    name: string;
    settings: {
        reportVersionTimeout: number;
        samplingInterval: number;
        serialport: {
            baudRate: number;
            highWaterMark: number;
        };
    } & {
        samplingInterval?: any;
    };
    pending: number;
    digitalPortQueue: number;
    transport: any;
    reportVersionTimeoutId: NodeJS.Timeout;
    static SYSEX_RESPONSE: any;
    constructor(port: any, options: {
        samplingInterval?: any;
        pinCount?: number;
        skipCapabilities?: boolean;
        analogPins?: number[];
        reportVersionTimeout?: number;
        pins?: {
            supportedModes: number[];
            analogChannel: number;
        }[];
    }, callback?: (arg0: Error | undefined) => void);
    /**
     * Asks the arduino to tell us its version.
     * @param {function} callback A function to be called when the arduino has reported its version.
     */
    reportVersion(callback: () => void): void;
    /**
     * Asks the arduino to tell us its firmware version.
     * @param {function} callback A function to be called when the arduino has reported its firmware version.
     */
    queryFirmware(callback: () => void): void;
    /**
     * Asks the arduino to read analog data. Turn on reporting for this pin.
     * @param {number} pin The pin to read analog data
     * @param {function} callback A function to call when we have the analag data.
     */
    analogRead(pin: number, callback: () => void): void;
    /**
     * Write a PWM value Asks the arduino to write an analog message.
     * @param {number} pin The pin to write analog data to.
     * @param {number} value The data to write to the pin between 0 and this.RESOLUTION.PWM.
     */
    pwmWrite(pin: number, value: number): void;
    /**
     * Set a pin to SERVO mode with an explicit PWM range.
     *
     * @param {number} pin The pin the servo is connected to
     * @param {number} min A 14-bit signed int.
     * @param {number} max A 14-bit signed int.
     */
    servoConfig(pin: number | {
        pin: number;
        max: number;
        min: number;
    }, min: number, max: number): void;
    /**
     * Asks the arduino to move a servo
     * @param {number} pin The pin the servo is connected to
     * @param {number} value The degrees to move the servo to.
     */
    servoWrite(...args: any[]): void;
    analogWrite(...arg: any): void;
    /**
     * Asks the arduino to set the pin to a certain mode.
     * @param {number} pin The pin you want to change the mode of.
     * @param {number} mode The mode you want to set. Must be one of board.MODES
     */
    pinMode(pin: number, mode: number): void;
    /**
     * Asks the arduino to write a value to a digital pin
     * @param {number} pin The pin you want to write a value to.
     * @param {number} value The value you want to write. Must be board.HIGH or board.LOW
     * @param {boolean} enqueue When true, the local state is updated but the command is not sent to the Arduino
     */
    digitalWrite(pin: number, value: number, enqueue?: number): void;
    /**
     * Update local store of digital port state
     * @param {number} pin The pin you want to write a value to.
     * @param {number} value The value you want to write. Must be board.HIGH or board.LOW
     */
    updateDigitalPort(pin: number, value: number): number;
    /**
     * Write queued digital ports
     */
    flushDigitalPorts(): void;
    /**
     * Update a digital port (group of 8 digital pins) on the Arduino
     * @param {number} port The port you want to update.
     */
    writeDigitalPort(port: number): void;
    /**
     * Asks the arduino to read digital data. Turn on reporting for this pin's port.
     *
     * @param {number} pin The pin to read data from
     * @param {function} callback The function to call when data has been received
     */
    digitalRead(pin: number, callback: () => void): void;
    /**
     * Asks the arduino to tell us its capabilities
     * @param {function} callback A function to call when we receive the capabilities
     */
    queryCapabilities(callback: () => void): void;
    /**
     * Asks the arduino to tell us its analog pin mapping
     * @param {function} callback A function to call when we receive the pin mappings.
     */
    queryAnalogMapping(callback: () => void): void;
    /**
     * Asks the arduino to tell us the current state of a pin
     * @param {number} pin The pin we want to the know the state of
     * @param {function} callback A function to call when we receive the pin state.
     */
    queryPinState(pin: number, callback: () => void): void;
    /**
     * Sends a string to the arduino
     * @param {String} string to send to the device
     */
    sendString(string: string): void;
    /**
     * Sends a I2C config request to the arduino board with an optional
     * value in microseconds to delay an I2C Read.  Must be called before
     * an I2C Read or Write
     * @param {number} delay in microseconds to set for I2C Read
     */
    sendI2CConfig(delay: number): this;
    /**
     * Enable I2C with an optional read delay. Must be called before
     * an I2C Read or Write
     *
     * Supersedes sendI2CConfig
     *
     * @param {number} delay in microseconds to set for I2C Read
     *
     * or
     *
     * @param {object} with a single property `delay`
     */
    i2cConfig(options: number | {
        settings: any;
        delay: any;
        address: any;
    }): this;
    /**
     * Asks the arduino to send an I2C request to a device
     * @param {number} slaveAddress The address of the I2C device
     * @param {Array} bytes The bytes to send to the device
     */
    sendI2CWriteRequest(slaveAddress: number, bytes: number[]): void;
    /**
     * Write data to a register
     *
     * @param {number} address      The address of the I2C device.
     * @param {Array} cmdRegOrData  An array of bytes
     *
     * Write a command to a register
     *
     * @param {number} address      The address of the I2C device.
     * @param {number} cmdRegOrData The register
     * @param {Array} inBytes       An array of bytes
     *
     */
    i2cWrite(address: number, registerOrData: any, inBytes: number[]): this;
    /**
     * Write data to a register
     *
     * @param {number} address    The address of the I2C device.
     * @param {number} register   The register.
     * @param {number} byte       The byte value to write.
     *
     */
    i2cWriteReg(address: number, register: number, byte: number): this;
    /**
     * Asks the arduino to request bytes from an I2C device
     * @param {number} slaveAddress The address of the I2C device
     * @param {number} numBytes The number of bytes to receive.
     * @param {function} callback A function to call when we have received the bytes.
     */
    sendI2CReadRequest(address: number, numBytes: number, callback: () => void): void;
    /**
     * Initialize a continuous I2C read.
     *
     * @param {number} address    The address of the I2C device
     * @param {number} register   Optionally set the register to read from.
     * @param {number} numBytes   The number of bytes to receive.
     * @param {function} callback A function to call when we have received the bytes.
     */
    i2cRead(address: number, register: number | null, bytesToRead: number, callback: () => void): this;
    /**
     * Stop continuous reading of the specified I2C address or register.
     *
     * @param {object} options Options:
     *   bus {number} The I2C bus (on supported platforms)
     *   address {number} The I2C peripheral address to stop reading.
     *
     * @param {number} address The I2C peripheral address to stop reading.
     */
    i2cStop(options: {
        address: number;
    }): void;
    _events(_events: any): void;
    /**
     * Perform a single I2C read
     *
     * Supersedes sendI2CReadRequest
     *
     * Read bytes from address
     *
     * @param {number} address    The address of the I2C device
     * @param {number} register   Optionally set the register to read from.
     * @param {number} numBytes   The number of bytes to receive.
     * @param {function} callback A function to call when we have received the bytes.
     *
     */
    i2cReadOnce(address: number, register: number | null, bytesToRead: number, callback: () => void): this;
    /**
     * Configure the passed pin as the controller in a 1-wire bus.
     * Pass as enableParasiticPower true if you want the data pin to power the bus.
     * @param pin
     * @param enableParasiticPower
     */
    sendOneWireConfig(pin: number, enableParasiticPower: any): void;
    /**
     * Searches for 1-wire devices on the bus.  The passed callback should accept
     * and error argument and an array of device identifiers.
     * @param pin
     * @param callback
     */
    sendOneWireSearch(pin: number, callback: () => void): void;
    /**
     * Searches for 1-wire devices on the bus in an alarmed state.  The passed callback
     * should accept and error argument and an array of device identifiers.
     * @param pin
     * @param callback
     */
    sendOneWireAlarmsSearch(pin: number, callback: () => void): void;
    [SYM_sendOneWireSearch](type: any, event: string | symbol, pin: any, callback: (arg0: Error | null, arg1?: undefined) => void): void;
    /**
     * Reads data from a device on the bus and invokes the passed callback.
     *
     * N.b. ConfigurableFirmata will issue the 1-wire select command internally.
     * @param pin
     * @param device
     * @param numBytesToRead
     * @param callback
     */
    sendOneWireRead(pin: any, device: any, numBytesToRead: any, callback: (arg0: Error | null, arg1?: undefined) => void): void;
    /**
     * Resets all devices on the bus.
     * @param pin
     */
    sendOneWireReset(pin: number): void;
    /**
     * Writes data to the bus to be received by the passed device.  The device
     * should be obtained from a previous call to sendOneWireSearch.
     *
     * N.b. ConfigurableFirmata will issue the 1-wire select command internally.
     * @param pin
     * @param device
     * @param data
     */
    sendOneWireWrite(pin: number, device: any, data: any): void;
    /**
     * Tells firmata to not do anything for the passed amount of ms.  For when you
     * need to give a device attached to the bus time to do a calculation.
     * @param pin
     */
    sendOneWireDelay(pin: number, delay: number): void;
    /**
     * Sends the passed data to the passed device on the bus, reads the specified
     * number of bytes and invokes the passed callback.
     *
     * N.b. ConfigurableFirmata will issue the 1-wire select command internally.
     * @param pin
     * @param device
     * @param data
     * @param numBytesToRead
     * @param callback
     */
    sendOneWireWriteAndRead(pin: any, device: any, data: any, numBytesToRead: any, callback: (arg0: Error | null, arg1?: undefined) => void): void;
    [SYM_sendOneWireRequest](pin: any, subcommand: number, device?: ConcatArray<number>, numBytesToRead?: number, correlationId?: number, delay?: number, dataToWrite?: any, event?: string | symbol, callback?: (...args: any[]) => void): void;
    /**
     * Set sampling interval in millis. Default is 19 ms
     * @param {number} interval The sampling interval in ms > 10
     */
    setSamplingInterval(interval: number): void;
    /**
     * Get sampling interval in millis. Default is 19 ms
     *
     * @return {number} samplingInterval
     */
    getSamplingInterval(): any;
    /**
     * Set reporting on pin
     * @param {number} pin The pin to turn on/off reporting
     * @param {number} value Binary value to turn reporting on/off
     */
    reportAnalogPin(pin: number, value: number): void;
    /**
     * Set reporting on pin
     * @param {number} pin The pin to turn on/off reporting
     * @param {number} value Binary value to turn reporting on/off
     */
    reportDigitalPin(pin: number, value: number): void;
    /**
     *
     *
     */
    pingRead(options: any, callback: () => void): void;
    /**
     * Stepper functions to support version 2 of ConfigurableFirmata's asynchronous control of stepper motors
     * https://github.com/soundanalogous/ConfigurableFirmata
     */
    /**
     * Asks the arduino to configure a stepper motor with the given config to allow asynchronous control of the stepper
     * @param {object} opts Options:
     *    {number} deviceNum: Device number for the stepper (range 0-9)
     *    {number} type: One of this.STEPPER.TYPE.*
     *    {number} stepSize: One of this.STEPPER.STEP_SIZE.*
     *    {number} stepPin: Only used if STEPPER.TYPE.DRIVER
     *    {number} directionPin: Only used if STEPPER.TYPE.DRIVER
     *    {number} motorPin1: motor pin 1
     *    {number} motorPin2:  motor pin 2
     *    {number} [motorPin3]: Only required if type == this.STEPPER.TYPE.THREE_WIRE || this.STEPPER.TYPE.FOUR_WIRE
     *    {number} [motorPin4]: Only required if type == this.STEPPER.TYPE.FOUR_WIRE
     *    {number} [enablePin]: Enable pin
     *    {array} [invertPins]: Array of pins to invert
     */
    accelStepperConfig(options: any): void;
    /**
     * Asks the arduino to set the stepper position to 0
     * Note: This is not a move. We are setting the current position equal to zero
     * @param {number} deviceNum Device number for the stepper (range 0-9)
     */
    accelStepperZero(deviceNum: number): void;
    /**
     * Asks the arduino to move a stepper a number of steps
     * (and optionally with and acceleration and deceleration)
     * speed is in units of steps/sec
     * @param {number} deviceNum Device number for the stepper (range 0-5)
     * @param {number} steps Number of steps to make
     */
    accelStepperStep(deviceNum: number, steps: number, callback: () => void): void;
    /**
     * Asks the arduino to move a stepper to a specific location
     * @param {number} deviceNum Device number for the stepper (range 0-5)
     * @param {number} position Desired position
     */
    accelStepperTo(deviceNum: number, position: number, callback: () => void): void;
    /**
     * Asks the arduino to enable/disable a stepper
     * @param {number} deviceNum Device number for the stepper (range 0-9)
     * @param {boolean} [enabled]
     */
    accelStepperEnable(deviceNum: number, enabled?: boolean): void;
    /**
     * Asks the arduino to stop a stepper
     * @param {number} deviceNum Device number for the stepper (range 0-9)
     */
    accelStepperStop(deviceNum: number): void;
    /**
     * Asks the arduino to report the position of a stepper
     * @param {number} deviceNum Device number for the stepper (range 0-9)
     */
    accelStepperReportPosition(deviceNum: number, callback: () => void): void;
    /**
     * Asks the arduino to set the acceleration for a stepper
     * @param {number} deviceNum Device number for the stepper (range 0-9)
     * @param {number} acceleration Desired acceleration in steps per sec^2
     */
    accelStepperAcceleration(deviceNum: number, acceleration: number): void;
    /**
     * Asks the arduino to set the max speed for a stepper
     * @param {number} deviceNum Device number for the stepper (range 0-9)
     * @param {number} speed Desired speed or maxSpeed in steps per second
     * @param {function} [callback]
     */
    accelStepperSpeed(deviceNum: number, speed: number): void;
    /**
     * Asks the arduino to configure a multiStepper group
     * @param {object} options Options:
     *    {number} groupNum: Group number for the multiSteppers (range 0-5)
     *    {number} devices: array of accelStepper device numbers in group
     **/
    multiStepperConfig(options: any): void;
    /**
     * Asks the arduino to move a multiStepper group
     * @param {number} groupNum Group number for the multiSteppers (range 0-5)
     * @param {number} positions array of absolute stepper positions
     **/
    multiStepperTo(groupNum: number, positions: number[], callback: () => void): void;
    /**
     * Asks the arduino to stop a multiStepper group
     * @param {number} groupNum: Group number for the multiSteppers (range 0-5)
     **/
    multiStepperStop(groupNum: number): void;
    /**
     * Stepper functions to support AdvancedFirmata's asynchronous control of stepper motors
     * https://github.com/soundanalogous/AdvancedFirmata
     */
    /**
     * Asks the arduino to configure a stepper motor with the given config to allow asynchronous control of the stepper
     * @param {number} deviceNum Device number for the stepper (range 0-5, expects steppers to be setup in order from 0 to 5)
     * @param {number} type One of this.STEPPER.TYPE.*
     * @param {number} stepsPerRev Number of steps motor takes to make one revolution
     * @param {number} stepOrMotor1Pin If using EasyDriver type stepper driver, this is direction pin, otherwise it is motor 1 pin
     * @param {number} dirOrMotor2Pin If using EasyDriver type stepper driver, this is step pin, otherwise it is motor 2 pin
     * @param {number} [motorPin3] Only required if type == this.STEPPER.TYPE.FOUR_WIRE
     * @param {number} [motorPin4] Only required if type == this.STEPPER.TYPE.FOUR_WIRE
     */
    stepperConfig(deviceNum: number, type: number, stepsPerRev: number, dirOrMotor1Pin: number, dirOrMotor2Pin: number, motorPin3: number, motorPin4: number): void;
    /**
     * Asks the arduino to move a stepper a number of steps at a specific speed
     * (and optionally with and acceleration and deceleration)
     * speed is in units of .01 rad/sec
     * accel and decel are in units of .01 rad/sec^2
     * TODO: verify the units of speed, accel, and decel
     * @param {number} deviceNum Device number for the stepper (range 0-5)
     * @param {number} direction One of this.STEPPER.DIRECTION.*
     * @param {number} steps Number of steps to make
     * @param {number} speed
     * @param {number|function} accel Acceleration or if accel and decel are not used, then it can be the callback
     * @param {number} [decel]
     * @param {function} [callback]
     */
    stepperStep(deviceNum: number, direction: number, steps: number, speed: number, accel: number, decel: number, callback: (...args: any[]) => void): void;
    /**
     * Asks the Arduino to configure a hardware or serial port.
     * @param {object} options Options:
     *   portId {number} The serial port to use (HW_SERIAL1, HW_SERIAL2, HW_SERIAL3, SW_SERIAL0,
     *   SW_SERIAL1, SW_SERIAL2, SW_SERIAL3)
     *   baud {number} The baud rate of the serial port
     *   rxPin {number} [SW Serial only] The RX pin of the SoftwareSerial instance
     *   txPin {number} [SW Serial only] The TX pin of the SoftwareSerial instance
     */
    serialConfig(options: {
        portId: any;
        baud: any;
        rxPin: any;
        txPin: any;
    } | null): void;
    /**
     * Write an array of bytes to the specified serial port.
     * @param {number} portId The serial port to write to.
     * @param {Array} inBytes An array of bytes to write to the serial port.
     */
    serialWrite(portId: number, bytes: number[]): void;
    /**
     * Start continuous reading of the specified serial port. The port is checked for data each
     * iteration of the main Arduino loop.
     * @param {number} portId The serial port to start reading continuously.
     * @param {number} maxBytesToRead [Optional] The maximum number of bytes to read per iteration.
     * If there are less bytes in the buffer, the lesser number of bytes will be returned. A value of 0
     * indicates that all available bytes in the buffer should be read.
     * @param {function} callback A function to call when we have received the bytes.
     */
    serialRead(portId: number, maxBytesToRead: number, callback: () => void): void;
    /**
     * Stop continuous reading of the specified serial port. This does not close the port, it stops
     * reading it but keeps the port open.
     * @param {number} portId The serial port to stop reading.
     */
    serialStop(portId: number): void;
    /**
     * Close the specified serial port.
     * @param {number} portId The serial port to close.
     */
    serialClose(portId: number): void;
    /**
     * Flush the specified serial port. For hardware serial, this waits for the transmission of
     * outgoing serial data to complete. For software serial, this removed any buffered incoming serial
     * data.
     * @param {number} portId The serial port to flush.
     */
    serialFlush(portId: number): void;
    /**
     * For SoftwareSerial only. Only a single SoftwareSerial instance can read data at a time.
     * Call this method to set this port to be the reading port in the case there are multiple
     * SoftwareSerial instances.
     * @param {number} portId The serial port to listen on.
     */
    serialListen(portId: number): void;
    /**
     * Allow user code to handle arbitrary sysex responses
     *
     * @param {number} commandByte The commandByte must be associated with some message
     *                             that's expected from the slave device. The handler is
     *                             called with an array of _raw_ data from the slave. Data
     *                             decoding must be done within the handler itself.
     *
     *                             Use Firmata.decode(data) to extract useful values from
     *                             the incoming response data.
     *
     *  @param {function} handler Function which handles receipt of responses matching
     *                            commandByte.
     */
    sysexResponse(commandByte: number, handler: {
        call: (arg0: any, arg1: any) => any;
    }): this;
    clearSysexResponse(commandByte: number): void;
    /**
     * Allow user code to send arbitrary sysex messages
     *
     * @param {Array} message The message array is expected to be all necessary bytes
     *                        between START_SYSEX and END_SYSEX (non-inclusive). It will
     *                        be assumed that the data in the message array is
     *                        already encoded as 2 7-bit bytes LSB first.
     *
     *
     */
    sysexCommand(message: any): this;
    /**
     * Send SYSTEM_RESET to arduino
     */
    reset(): void;
    /**
     * Firmata.isAcceptablePort Determines if a `port` object (from SerialPort.list())
     * is a valid Arduino (or similar) device.
     * @return {Boolean} true if port can be connected to by Firmata
     */
    static isAcceptablePort(port: {
        path: string;
    }): boolean;
    /**
     * Firmata.requestPort(callback) Request an acceptable port to connect to.
     * callback(error, port)
     */
    static requestPort(callback: (arg0: Error | null, arg1: null) => void): void;
    static encode(data: string | any[]): number[];
    static decode(data: any[]): number[];
}
/**
 * writeToTransport
 *
 * Due to the non-blocking behaviour of transport write operations, dependent programs
 * need a way to know when all writes are complete. Every write increments a `pending` value;
 * when the write operation has completed, the `pending` value is decremented.
 *
 * @param board An active Firmata instance.
 * @param data  An array of 8- and 7-bit values that will be wrapped in a Buffer and written.
 */
export declare function writeToTransport(board: Firmata, data: number[]): void;
/**
 * i2cRequest
 *
 * Send an I2C request. If I2C is not enabled on the board, an Error is thrown.
 *
 * @param board An active Firmata instance.
 * @param bytes An array of bytes representing the I2C request.
 */
export declare function i2cRequest(board: Firmata, bytes: number[]): void;
/**
 * encode32BitSignedInteger
 *
 * Encodes a signed 32-bit integer into an array of 5 7-bit numbers.
 *
 * @param data The number to encode.
 * @returns An array of encoded numbers.
 */
export declare function encode32BitSignedInteger(data: number): number[];
/**
 * decode32BitSignedInteger
 *
 * Decodes an array of 5 7-bit numbers into a signed 32-bit integer.
 *
 * @param bytes The array of encoded bytes.
 * @returns The decoded integer.
 */
export declare function decode32BitSignedInteger(bytes: number[]): number;
/**
 * encodeCustomFloat
 *
 * Encodes a floating-point number into an array of 4 7-bit values.
 *
 * @param input The number to encode.
 * @returns An array of encoded numbers.
 */
export declare function encodeCustomFloat(input: number): number[];
/**
 * decodeCustomFloat
 *
 * Decodes an array of 4 7-bit numbers into a floating-point number.
 *
 * @param input The encoded array.
 * @returns The decoded number.
 */
export declare function decodeCustomFloat(input: number[]): number;
/**
 * bindTransport
 *
 * Binds a transport to the Firmata implementation.
 *
 * @param transport The transport to bind.
 * @returns The Firmata class.
 */
declare const bindTransport: (transport: any) => typeof Firmata;
export default bindTransport;
