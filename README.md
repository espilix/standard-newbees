## Electronics/PCB Design (30 Problems)

### Basic Level (Problems 1-10)
1. **Ohm's Law Applications**: Design a voltage divider circuit to convert 12V to 3.3V with 100mA load current. Calculate resistor values and power dissipation.

2. **RC Time Constants**: Design an RC circuit with τ = 1ms. Calculate component values and sketch voltage/current waveforms during charging/discharging.

3. **Diode Circuits**: Design a full-wave bridge rectifier for 12V DC output from 120V AC input. Include filter capacitor calculations.

4. **Transistor Biasing**: Design a common-emitter amplifier with BJT biased at VCE = 6V, IC = 2mA using voltage divider biasing.

5. **Op-Amp Basics**: Design a non-inverting amplifier with gain = 10 using LM741. Include input/output impedance calculations.

6. **Filter Design**: Design a 2nd-order Sallen-Key low-pass filter with fc = 1kHz and Q = 0.707.

7. **Power Supply Design**: Design a linear power supply converting 24V AC to regulated 5V DC, 1A output with ripple < 1%.

8. **LED Driver Circuit**: Design a constant current LED driver for 3W LED (If = 700mA, Vf = 3.4V) from 12V supply.

9. **Comparator Circuit**: Design a window comparator using LM339 to detect voltage between 2V and 8V with hysteresis.

10. **Oscillator Design**: Design a Wien bridge oscillator for 1kHz sine wave output with amplitude control.

### Intermediate Level (Problems 11-20)
11. **Switching Regulator**: Design a buck converter to convert 12V to 3.3V at 2A with efficiency > 85%.

12. **PCB Layout Rules**: Design a 4-layer PCB layout for a microcontroller circuit including power planes, via stitching, and EMI considerations.

13. **Impedance Matching**: Design a 50Ω microstrip transmission line on FR4 substrate (εr = 4.4, h = 1.6mm).

14. **EMI/EMC Design**: Analyze and solve EMI issues in a switching power supply PCB layout.

15. **Thermal Management**: Design thermal solution for a 10W power MOSFET in TO-220 package with ambient temperature 50°C.

16. **Signal Integrity**: Analyze crosstalk between parallel traces and design guard traces/differential pairs solution.

17. **Power Distribution Network**: Design PDN for FPGA requiring 1.2V core (5A) and 3.3V I/O (2A) supplies.

18. **High-Speed Design**: Design PCB layout for 100MHz digital signals including length matching and via optimization.

19. **Analog Front-End**: Design instrumentation amplifier circuit for sensor signal conditioning (0-10mV to 0-5V).

20. **Motor Driver**: Design H-bridge motor driver circuit for 24V, 5A brushed DC motor with current sensing.

### Advanced Level (Problems 21-30)
21. **RF Circuit Design**: Design a 2.4GHz antenna matching network with Smith chart analysis.

22. **Mixed-Signal PCB**: Design PCB layout separating analog/digital domains for ADC application with PSRR > 60dB.

23. **Power Integrity**: Perform PI analysis for high-current FPGA application including decoupling capacitor placement.

24. **DDR Memory Interface**: Design PCB layout for DDR3 memory interface including length matching and termination.

25. **Isolated Power Supply**: Design flyback converter with galvanic isolation (input: 85-265V AC, output: 12V/2A).

26. **Battery Management**: Design Li-ion battery charging circuit with protection (overcurrent, overvoltage, temperature).

27. **Precision Analog**: Design 16-bit ADC front-end with noise analysis and layout considerations.

28. **High-Frequency Switching**: Design GaN-based switching converter operating at 1MHz with minimal EMI.

29. **Sensor Interface**: Design complete sensor interface for strain gauge bridge with temperature compensation.

30. **System Integration**: Design power management system for portable device with multiple voltage rails and power sequencing.

## FPGA Design (30 Problems)

### Basic Level (Problems 31-40)
31. **Basic Logic Gates**: Implement all basic logic gates (AND, OR, NOT, XOR) in Verilog and verify with testbench.

32. **Combinational Circuits**: Design 4-bit binary to BCD converter using Verilog.

33. **Sequential Logic**: Implement D flip-flop, JK flip-flop, and T flip-flop with reset functionality.

34. **Counter Design**: Design 8-bit up/down counter with load, enable, and reset controls.

35. **Shift Registers**: Implement SISO, SIPO, PISO, and PIPO shift registers.

36. **Multiplexer/Demultiplexer**: Design 8:1 multiplexer and 1:8 demultiplexer with enable signals.

37. **Encoder/Decoder**: Implement 8:3 priority encoder and 3:8 decoder with enable.

38. **State Machine Basics**: Design Moore and Mealy state machines for sequence detector (1011).

39. **Clock Dividers**: Implement programmable clock divider with 50% duty cycle for any division ratio.

40. **Basic Arithmetic**: Design 8-bit adder/subtractor with overflow detection.

### Intermediate Level (Problems 41-50)
41. **UART Implementation**: Design complete UART transmitter and receiver with configurable baud rate.

42. **SPI Master/Slave**: Implement SPI master and slave controllers with configurable data width.

43. **I2C Controller**: Design I2C master controller supporting multi-byte read/write operations.

44. **Memory Controller**: Implement SRAM controller with read/write operations and wait states.

45. **FIFO Design**: Design synchronous and asynchronous FIFO with full/empty flags and programmable thresholds.

46. **Digital Filters**: Implement FIR and IIR digital filters with configurable coefficients.

47. **PWM Generator**: Design multi-channel PWM generator with deadtime control for motor applications.

48. **Frequency Synthesizer**: Implement PLL-based frequency synthesizer using FPGA primitives.

49. **Video Controller**: Design VGA controller generating standard video timing signals.

50. **Ethernet MAC**: Implement basic Ethernet MAC layer with frame transmission/reception.

### Advanced Level (Problems 51-60)
51. **DDR Controller**: Design DDR3 memory controller with command scheduling and refresh management.

52. **PCIe Interface**: Implement PCIe endpoint with DMA capabilities for data transfer.

53. **DSP Applications**: Design FFT processor using butterfly operations and bit-reversal addressing.

54. **Image Processing**: Implement real-time image processing pipeline with filtering and edge detection.

55. **Soft Processor**: Design simple RISC processor with instruction fetch, decode, and execute stages.

56. **High-Speed SerDes**: Implement high-speed serial interface with 8b/10b encoding/decoding.

57. **Cryptographic Engine**: Design AES encryption/decryption engine with key scheduling.

58. **Network Packet Processing**: Implement packet classifier and traffic shaper for network applications.

59. **Radar Signal Processing**: Design pulse compression and Doppler processing for radar applications.

60. **Machine Learning Accelerator**: Implement CNN accelerator for neural network inference.

## Microcontroller/Embedded Systems (30 Problems)

### Basic Level (Problems 61-70)
61. **GPIO Control**: Program LED patterns and button debouncing using GPIO interrupts.

62. **Timer Applications**: Implement precise timing functions, delays, and periodic interrupts.

63. **UART Communication**: Develop serial communication protocol with error handling and buffering.

64. **ADC Interface**: Read analog sensors and implement digital filtering and calibration.

65. **PWM Applications**: Control servo motors and LED brightness using PWM signals.

66. **Interrupt Handling**: Design interrupt-driven system with priority management and nested interrupts.

67. **I2C Sensor Interface**: Interface with multiple I2C sensors (temperature, pressure, accelerometer).

68. **SPI Flash Memory**: Implement file system for external SPI flash memory storage.

69. **Watchdog Timer**: Implement robust watchdog system with fault recovery mechanisms.

70. **Low Power Design**: Design ultra-low power application with sleep modes and wake-up sources.

### Intermediate Level (Problems 71-80)
71. **RTOS Implementation**: Port FreeRTOS and implement multi-task application with synchronization.

72. **USB Device**: Develop USB HID device for custom hardware interface.

73. **Ethernet Communication**: Implement TCP/IP stack for web server application.

74. **CAN Bus Interface**: Design CAN bus communication system for automotive applications.

75. **Motor Control**: Implement BLDC motor control with hall sensor feedback and PID control.

76. **Data Logging System**: Design high-speed data acquisition system with SD card storage.

77. **Wireless Communication**: Implement wireless sensor network using RF modules.

78. **Touch Interface**: Develop capacitive touch sensing with gesture recognition.

79. **Audio Processing**: Implement digital audio effects and codec interface.

80. **Bootloader Design**: Develop secure bootloader with firmware update capability over various interfaces.

### Advanced Level (Problems 81-90)
81. **Multi-Core Programming**: Implement parallel processing application using dual-core microcontroller.

82. **Security Implementation**: Design secure communication with encryption and authentication.

83. **Machine Learning**: Implement TinyML model for edge AI applications.

84. **Industrial Protocol**: Develop Modbus RTU/TCP implementation for industrial automation.

85. **High-Speed DAQ**: Design high-speed data acquisition system with DMA and circular buffers.

86. **Sensor Fusion**: Implement IMU sensor fusion using Kalman filtering for orientation estimation.

87. **Power Management**: Design intelligent power management system with load balancing.

88. **Fault-Tolerant System**: Implement safety-critical system with redundancy and error detection.

89. **Edge Computing**: Develop IoT edge gateway with local processing and cloud connectivity.

90. **System Integration**: Design complete embedded system integrating FPGA, MCU, and analog circuits for specific application (e.g., software-defined radio, industrial controller, medical device).

## Evaluation Criteria for Monthly Tests

### Month 1 Test: Problems 1-30 (Electronics/PCB)
- Circuit analysis and design fundamentals
- Component selection and calculations
- Basic PCB layout principles
- Simulation and verification skills

### Month 2 Test: Problems 31-60 (FPGA)
- HDL coding proficiency
- Digital design concepts
- Timing analysis and constraints
- Verification and testing methodologies

### Month 3 Test: Problems 61-90 (Microcontroller/Embedded)
- Embedded programming skills
- Real-time system design
- Hardware-software integration
- System optimization and debugging
