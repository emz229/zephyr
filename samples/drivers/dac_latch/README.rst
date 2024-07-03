.. zephyr:code-sample:: dac
   :name: Digital-to-Analog Converter (DAC)
   :relevant-api: dac_interface

   Generate an analog sawtooth signal using the DAC driver API with
   gpio latch to show synchronized DAC channel updates.

Overview
********

This sample demonstrates how to use the :ref:`DAC driver API <dac_api>`.
This sample has been extended to allow synchronized DAC channel updates.

Building and Running
********************

The DAC output is defined in the src/main.c file.

The board must have ``dac`` device tree alias defined.
See the predefined overlays in :zephyr_file:`samples/drivers/daclatch
/boards` for examples.

Building and Running for Teensy4.1
=========================================
The sample can be built and executed for the
:ref:`teensy41_board` as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/dac
   :board: teensy41
   :goals: build flash
   :compact:

.. note:: This expects a MCP48xx device to be attached to SPI bus 4
   which corresponds to Teensy4.1 pins as follows: CS - 10, MOSI - 11, 
   MISO - 12, SCK - 13

Sample output
=============

You should see a sawtooth signal with an amplitude of the DAC reference
voltage and a period of approx. 20 seconds at the DAC output pin specified
by the board.

The following output is printed:

.. note:: If the DAC is not supported, the output will be an error message.
